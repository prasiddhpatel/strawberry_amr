#!/usr/bin/env python3
"""
Projects /plant_targets into the map frame via TF and logs a de-duplicated
semantic_targets.csv.

Fix vs. original: docstring claimed map projection but coordinates were passed
straight through in the camera frame. Now does a real tf2 transform
map <- depth_optical_frame and de-duplicates on a grid so the CSV stays small.
"""
import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class SemanticMapperNode(Node):
    def __init__(self):
        super().__init__('semantic_mapper_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('dedup_grid_m', 0.10)
        self.declare_parameter('csv_path',
                               os.path.expanduser('~/ros2_ws/maps/semantic_targets.csv'))
        self.map_frame = self.get_parameter('map_frame').value
        self.grid = self.get_parameter('dedup_grid_m').value
        # expanduser on the VALUE, not just the declare_parameter default.
        # This was a real bug: semantic_mapper_params.yaml sets
        # csv_path: "~/ros2_ws/maps/semantic_targets.csv", which OVERRIDES
        # the expanded default with a literal '~' string. Without this the
        # node created a directory literally named '~' relative to the
        # process CWD, so Phase 2 launched from a different directory would
        # silently not find Phase 1's output.
        self.csv_path = os.path.expanduser(self.get_parameter('csv_path').value)

        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.cells = set()
        self.targets = []
        self._load_existing_targets()
        # Snapshot of self.targets as of the last successful save() -- see
        # save() below. Initialised post-load so an unchanged reload doesn't
        # even trigger the first scheduled save's write.
        self._last_saved_targets = list(self.targets)

        self.create_subscription(PoseArray, '/plant_targets', self.cb, 10)
        self.create_timer(5.0, self.save)
        self.get_logger().info('semantic_mapper_node ready (tf2 map projection).')

    def _load_existing_targets(self):
        # Without this, a node restart mid-mapping (crash, manual restart)
        # silently loses every plant mapped before the restart: self.cells/
        # self.targets always started empty, and save() unconditionally
        # overwrites self.csv_path (open in 'w' mode) every 5.0s regardless
        # of whether anything changed -- so the very next save cycle after a
        # restart would permanently destroy the prior run's data, replacing
        # it with only what's been re-detected since. Mirrors the same
        # class of problem target_manager's persisted-visited-state
        # mechanism already exists to prevent, applied here to the plant
        # map itself, which is the more valuable data of the two.
        if not os.path.exists(self.csv_path):
            return
        try:
            with open(self.csv_path, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x, y = float(row['x']), float(row['y'])
                    key = (round(x / self.grid), round(y / self.grid))
                    if key in self.cells:
                        continue
                    self.cells.add(key)
                    self.targets.append({
                        'class': row.get('class', 'strawberry'),
                        'x': x, 'y': y, 'z': float(row.get('z', 0.0)),
                        'status': row.get('status', 'unharvested'),
                    })
            self.get_logger().info(
                f'Loaded {len(self.targets)} existing target(s) from {self.csv_path}.')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f'Failed to reload existing {self.csv_path}: {e} -- starting with '
                f'an empty map. If this file has real prior-run data, back it up '
                f'before the next save cycle overwrites it.')

    def cb(self, msg):
        try:
            tr = self.buf.lookup_transform(
                self.map_frame, msg.header.frame_id,
                msg.header.stamp, timeout=Duration(seconds=0.1))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'TF {self.map_frame}<-{msg.header.frame_id} unavailable: {e}')
            return
        for p in msg.poses:
            tp = do_transform_pose(p, tr)
            key = (round(tp.position.x / self.grid), round(tp.position.y / self.grid))
            if key in self.cells:
                continue
            self.cells.add(key)
            self.targets.append({
                'class': 'strawberry',
                'x': round(tp.position.x, 4),
                'y': round(tp.position.y, 4),
                'z': round(tp.position.z, 4),
                'status': 'unharvested',
            })

    def save(self):
        # Skip the write entirely if nothing has changed since the last
        # save -- this used to unconditionally rewrite self.csv_path every
        # 5.0s regardless of content, which changes the file's mtime on
        # every cycle even when nothing new was detected. That silently
        # defeated target_manager's own visited-state staleness safeguard
        # (csv_fingerprint() is (mtime, size)-based): with both nodes
        # running concurrently during Phase 2 (the actual deployed
        # configuration, not an edge case -- see
        # pi_missionbrain_phase2_nav.launch.py), this file's mtime changed
        # every 5s regardless of real content, so a visited-state sidecar
        # written at any point almost never matched the CSV's fingerprint
        # by the time of a restart, and the entire visited set was
        # discarded -- close to guaranteed, not a rare race. cb() only ever
        # appends new dicts to self.targets, never mutates an existing
        # one in place, so a shallow list-equality check against a
        # snapshot correctly detects "nothing new" without needing to
        # re-read or hash the file.
        if self.targets == self._last_saved_targets:
            return
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['class', 'x', 'y', 'z', 'status'])
            w.writeheader()
            for r in self.targets:
                w.writerow(r)
        self._last_saved_targets = list(self.targets)
        self.get_logger().info(f'Saved {len(self.targets)} targets -> {self.csv_path}')


def main():
    rclpy.init()
    node = SemanticMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
