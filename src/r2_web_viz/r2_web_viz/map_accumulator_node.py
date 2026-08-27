#!/usr/bin/env python3
"""
Fuses /r2/points (the camera-frame RGB point cloud from depth_image_proc,
see web_viz.launch.py) into a persistent, map-frame, voxel-downsampled
reconstruction on /r2/map_points, and separately republishes a clean
map-frame robot pose on /r2/robot_pose_map -- both via real tf2_ros
lookups, not the browser hand-composing a 2-hop transform itself.

WHY THIS EXISTS, not a browser-side fix: this project's actual frame
chain (checked against ekf.yaml and slam_toolbox.yaml/
slam_toolbox_localization.yaml before writing this, not assumed) is
map -> odom (published by slam_toolbox, both mapping and localization
modes, drifts and is corrected by scan-matching/loop-closure) -> base_link
(published by the EKF, world_frame: odom). /odometry/filtered's pose is
in the ODOM frame, not map -- treating it as map-frame directly (r2_web_
viz's original v1 behaviour) is wrong whenever map->odom is non-identity,
which is the normal case once SLAM has corrected any drift at all. A
real tf2_ros.Buffer does this composition correctly, including the time-
indexed lookup a hand-rolled 2-hop JS composer would not get right for
free.

ACCUMULATION, not per-frame replace: v1 replaced the rendered point cloud
with whatever /r2/points last published -- a live, camera-relative view,
not a growing reconstruction. This node instead keeps a dict keyed by
voxel_key() (geometry.py) -> (r,g,b), overwritten on each new observation
of that cell (most-recent-observation-wins; no running average -- see
geometry.py's docstring for why that's a deliberate simplification, not
an oversight). Bounded by max_voxels, not by run duration: see
_add_points() below for what happens at the cap.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import read_points, create_cloud
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from r2_web_viz.geometry import quat_to_rotmat, transform_points, voxel_key, pack_rgb_float, unpack_rgb_uint32

FIELDS_XYZRGB = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]

WARN_THROTTLE_SEC = 5.0


class MapAccumulatorNode(Node):
    def __init__(self):
        super().__init__('map_accumulator_node')
        d = self.declare_parameter
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        d('points_topic', '/r2/points')
        d('output_points_topic', '/r2/map_points')
        d('robot_pose_topic', '/r2/robot_pose_map')
        d('voxel_size_m', 0.03)
        d('max_voxels', 500000)
        d('map_publish_rate_hz', 1.0)
        d('pose_publish_rate_hz', 10.0)
        d('tf_timeout_sec', 0.2)

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.voxel_size = float(g('voxel_size_m'))
        self.max_voxels = int(g('max_voxels'))
        self.tf_timeout = Duration(seconds=float(g('tf_timeout_sec')))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.voxels = {}  # (ix,iy,iz) -> (r,g,b), most-recent-observation-wins
        self._cap_warned = False
        self._last_points_tf_warn = 0.0
        self._last_pose_tf_warn = 0.0

        self.map_points_pub = self.create_publisher(PointCloud2, g('output_points_topic'), 1)
        self.robot_pose_pub = self.create_publisher(PoseStamped, g('robot_pose_topic'), 10)
        self.create_subscription(PointCloud2, g('points_topic'), self._points_cb, 1)

        map_period = 1.0 / max(1e-3, float(g('map_publish_rate_hz')))
        pose_period = 1.0 / max(1e-3, float(g('pose_publish_rate_hz')))
        self.create_timer(map_period, self._publish_map_points)
        self.create_timer(pose_period, self._publish_robot_pose)

        self.get_logger().info(
            f'map_accumulator_node ready: voxel_size={self.voxel_size}m, '
            f'max_voxels={self.max_voxels}, map_frame={self.map_frame}')

    # ------------------------------------------------------------ point cloud accumulation
    def _points_cb(self, msg: PointCloud2):
        try:
            # msg.header.stamp, not "latest" -- same discipline already
            # applied to semantic_mapper.cb() and target_manager.cb() this
            # project (both fixed this exact class of bug this session):
            # this is projecting a PAST observation into the map frame, so
            # it needs the transform AS OF when the observation was made.
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                timeout=self.tf_timeout)
        except TransformException as e:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_points_tf_warn > WARN_THROTTLE_SEC:
                self.get_logger().warn(
                    f'No transform {self.map_frame}<-{msg.header.frame_id} at cloud '
                    f'timestamp ({e}) -- skipping this frame\'s accumulation. Normal at '
                    f'startup before SLAM publishes map->odom; persistent means '
                    f'something is actually wrong.')
                self._last_points_tf_warn = now
            return

        pts = read_points(msg, field_names=['x', 'y', 'z', 'rgb'], skip_nans=True)
        if pts.size == 0:
            return
        xyz_local = np.column_stack([
            np.asarray(pts['x'], dtype=np.float64),
            np.asarray(pts['y'], dtype=np.float64),
            np.asarray(pts['z'], dtype=np.float64),
        ])
        r, g, b = unpack_rgb_uint32(np.asarray(pts['rgb'], dtype=np.float32))

        t = tf.transform.translation
        q = tf.transform.rotation
        rotmat = quat_to_rotmat(q.x, q.y, q.z, q.w)
        xyz_map = transform_points(xyz_local, rotmat, (t.x, t.y, t.z))

        self._add_points(xyz_map, r, g, b)

    def _add_points(self, xyz_map, r, g, b):
        # Vectorized key computation, then a plain-Python dict-update loop
        # -- the dict insert itself can't be vectorized away. If this
        # becomes a measured bottleneck on the Orin with a denser cloud
        # than this project currently produces, a proper spatial-hash
        # library (e.g. open3d's voxel_down_sample) is the next step, not
        # a cleverer loop here.
        ix = np.round(xyz_map[:, 0] / self.voxel_size).astype(np.int64)
        iy = np.round(xyz_map[:, 1] / self.voxel_size).astype(np.int64)
        iz = np.round(xyz_map[:, 2] / self.voxel_size).astype(np.int64)

        voxels = self.voxels
        at_cap_skipped = 0
        for i in range(len(ix)):
            key = (int(ix[i]), int(iy[i]), int(iz[i]))
            if key not in voxels and len(voxels) >= self.max_voxels:
                at_cap_skipped += 1
                continue
            voxels[key] = (int(r[i]), int(g[i]), int(b[i]))

        if at_cap_skipped and not self._cap_warned:
            self.get_logger().warn(
                f'max_voxels ({self.max_voxels}) reached -- new spatial regions are no '
                f'longer being added to the accumulated map (existing voxels still '
                f'refine). Raise max_voxels or voxel_size_m if this is unexpected. This '
                f'warning is logged once, not every frame.')
            self._cap_warned = True

    def _publish_map_points(self):
        if not self.voxels:
            return
        points = [
            (key[0] * self.voxel_size, key[1] * self.voxel_size, key[2] * self.voxel_size,
             pack_rgb_float(*color))
            for key, color in self.voxels.items()
        ]
        header = Header()
        header.frame_id = self.map_frame
        header.stamp = self.get_clock().now().to_msg()
        cloud = create_cloud(header, FIELDS_XYZRGB, points)
        self.map_points_pub.publish(cloud)

    # ------------------------------------------------------------ robot pose
    def _publish_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=self.tf_timeout)
        except TransformException as e:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_pose_tf_warn > WARN_THROTTLE_SEC:
                self.get_logger().warn(
                    f'No transform {self.map_frame}<-{self.base_frame} available ({e}) '
                    f'-- /r2/robot_pose_map not published this cycle.')
                self._last_pose_tf_warn = now
            return

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self.robot_pose_pub.publish(pose)


def main():
    rclpy.init()
    node = MapAccumulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
