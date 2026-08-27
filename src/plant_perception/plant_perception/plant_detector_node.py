#!/usr/bin/env python3
"""
Strawberry (ripe-fruit) detector for TABLETOP tunnels using the Orbbec Astra.

Classical HSV segmentation (NO training) retargeted from green foliage to RED
ripe fruit. On tabletop the fruit hangs from gutters ABOVE the robot, so the
camera is tilted up and an optional ROI keeps only the upper image region.

Fixes vs. original:
  * camera intrinsics were garbage (`fx=k`, `cy=k[6][7]`) -> fx=k[0], fy=k[4],
    cx=k[2], cy=k[5]  (CameraInfo.k is row-major 3x3)
  * depth bounds check `v>=shape or u>=shape:[4]` -> proper [0]/[1] indexing
  * RGB and depth were paired by "latest" -> now time-synced with
    message_filters.ApproximateTimeSynchronizer
  * sensor-data QoS on image streams (best-effort) so callbacks actually fire

Publishes /plant_targets (geometry_msgs/PoseArray) in the depth optical frame.

ACCURACY NOTE: for correct 3D, enable depth->color registration in the Astra
DRIVER itself (aligned depth) so colour pixels and depth pixels share
intrinsics, then point the `depth_topic` parameter at that aligned stream.
This node has no `use_aligned_depth` parameter of its own -- alignment is
not something it can do after the fact (it has no depth-colour extrinsics),
only the upstream driver can produce an already-aligned stream. The only
lever here is which topic `depth_topic` subscribes to; a previous revision
of this docstring referenced a `use_aligned_depth: true` parameter that was
never actually declared below, which would have silently done nothing if
set in a launch/yaml config.

Both the HSV path here and the zero-shot path
(plant_detector_zeroshot_node.py) back-project colour-pixel coordinates
into depth-image space by width/height RATIO alone (see
zeroshot_postprocess.backproject_mask_pixels' own docstring), which is only
geometrically correct when depth and colour genuinely share an optical
axis -- i.e. when depth_topic really is pointed at an aligned stream.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters


class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        d = self.declare_parameter
        d('color_topic', '/camera/color/image_raw')
        d('depth_topic', '/camera/depth/image_raw')
        d('info_topic', '/camera/color/camera_info')
        d('depth_optical_frame', 'camera_depth_optical_frame')
        d('min_contour_area', 600)
        d('roi_top_fraction', 0.0)   # 0=full image; e.g. 0.6 keeps top 60%
        d('depth_min_m', 0.15)
        d('depth_max_m', 3.0)
        # red HSV (OpenCV H in 0..179, red wraps around 0/180)
        d('h1_lo', 0)
        d('h1_hi', 10)
        d('h2_lo', 170)
        d('h2_hi', 179)
        d('s_lo', 90)
        d('v_lo', 60)
        d('publish_debug', True)     # annotated overlay for HSV tuning

        g = lambda n: self.get_parameter(n).value
        self.publish_debug = g('publish_debug')
        self.frame = g('depth_optical_frame')
        self.area_min = g('min_contour_area')
        self.roi_frac = g('roi_top_fraction')
        self.dmin, self.dmax = g('depth_min_m'), g('depth_max_m')
        self.h1lo, self.h1hi = g('h1_lo'), g('h1_hi')
        self.h2lo, self.h2hi = g('h2_lo'), g('h2_hi')
        self.slo, self.vlo = g('s_lo'), g('v_lo')

        self.bridge = CvBridge()
        self.K = None
        self.pose_pub = self.create_publisher(PoseArray, '/plant_targets', 10)
        self.status_pub = self.create_publisher(String, '/plant_detector/status', 10)
        self.debug_pub = self.create_publisher(Image, '/plant_detector/debug_image', 5)
        self.create_subscription(CameraInfo, g('info_topic'), self.info_cb, 10)

        rgb = message_filters.Subscriber(self, Image, g('color_topic'),
                                         qos_profile=qos_profile_sensor_data)
        dep = message_filters.Subscriber(self, Image, g('depth_topic'),
                                         qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, dep], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.rgbd_cb)
        self.get_logger().info('plant_detector_node ready (HSV red, RGB-D synced).')

    def info_cb(self, msg):
        self.K = msg.k   # [fx,0,cx, 0,fy,cy, 0,0,1]

    def rgbd_cb(self, rgb_msg, dep_msg):
        if self.K is None:
            return
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')
        H, W = img.shape[:2]
        dH, dW = depth.shape[:2]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, (self.h1lo, self.slo, self.vlo), (self.h1hi, 255, 255))
        m2 = cv2.inRange(hsv, (self.h2lo, self.slo, self.vlo), (self.h2hi, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
        if self.roi_frac > 0.0:
            keep = int(self.roi_frac * H)
            mask[keep:, :] = 0           # keep only top region (fruit overhead)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        poses = PoseArray()
        poses.header = dep_msg.header
        poses.header.frame_id = self.frame
        is_mm = depth.dtype != np.float32
        dbg = img.copy() if self.publish_debug else None
        if dbg is not None and self.roi_frac > 0.0:
            cv2.line(dbg, (0, keep), (W, keep), (255, 255, 0), 1)  # ROI cutoff

        for c in cnts:
            if cv2.contourArea(c) < self.area_min:
                continue
            x, y, w, h = cv2.boundingRect(c)
            u, v = x + w // 2, y + h // 2
            du, dv = int(u * dW / W), int(v * dH / H)   # scale to depth res
            if dv < 0 or du < 0 or dv >= dH or du >= dW:
                continue
            z = float(depth[dv, du])
            if z <= 0:
                continue
            z_m = z / 1000.0 if is_mm else z
            if z_m < self.dmin or z_m > self.dmax:
                continue
            p = Pose()
            p.position.x = (u - cx) * z_m / fx
            p.position.y = (v - cy) * z_m / fy
            p.position.z = z_m
            p.orientation.w = 1.0
            poses.poses.append(p)
            if dbg is not None:
                cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(dbg, (u, v), 4, (0, 0, 255), -1)
                cv2.putText(dbg, f'{z_m:.2f}m', (x, max(0, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if poses.poses:
            self.pose_pub.publish(poses)
            self.status_pub.publish(String(data=f'detected:{len(poses.poses)}'))
        if dbg is not None:
            cv2.putText(dbg, f'fruit: {len(poses.poses)}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            out = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
            out.header = rgb_msg.header
            self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = PlantDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
