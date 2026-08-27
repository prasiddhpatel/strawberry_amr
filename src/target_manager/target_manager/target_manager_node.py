#!/usr/bin/env python3
"""
Sequences the mapped strawberry plants into a "pre-mapped optimized
trajectory" and publishes the next goal on /selected_plant_goal.

WHY THIS NODE NEEDS TO LOAD A FILE (this was a real gap, not a style choice):
semantic_mapper saves a de-duplicated landmark CSV throughout Phase 1
(mapping) -- see semantic_mapper_node.py's periodic save(). But in the
two-phase deployment, Phase 2 (harvest) starts a FRESH process tree: a
brand-new target_manager_node has no memory of anything Phase 1 found
unless it explicitly loads that file. Without this, Phase 2 would have no
goals to offer Nav2 until plant_perception happened to re-detect something
live -- there would be no "pre-mapped" trajectory at all, just opportunistic
live detection, which is not what was asked for. This node now loads the
CSV ONCE at startup and treats it as the primary candidate set; live
/plant_targets detections (from Phase 2's plant_perception, kept running
for re-verification -- see pi_missionbrain_phase2_harvest.launch.py) are
still consumed too, both to refine the position of an already-loaded plant
and to pick up any genuinely new one Phase 1 missed.

ORDERING ("optimized trajectory"): a full TSP solve is overkill for a
row-structured tunnel and would be harder to verify/defend than a simple,
explainable heuristic. Targets are bucketed into rows by
`round(y / row_spacing)` and sorted along each row by x, with the
along-row direction ALTERNATING per row (ascending x on even rows,
descending on odd) -- this deliberately matches coverage_planner's own
boustrophedon serpentine (see coverage_planner_node.py) so the sequence
target_manager hands to Nav2 visits plants in the same order the vehicle
is already passing them in during row-following, rather than an
independent order that would send Nav2 zig-zagging across the map.
`row_spacing` MUST match coverage_planner's and row_navigation's value --
same cross-file constraint documented in docs/HEADLAND_TURN_GEOMETRY.md.

PERSISTED VISITED-STATE -- previously a documented, unfixed limitation:
if Phase 2 was interrupted (e.g. flat battery) and restarted, the
freshly-reloaded CSV had no record of which plants were already visited
in the interrupted run (visited state lived only in this node's
in-memory `self.visited` set, lost on restart), so a restart would
re-offer already-visited plants. Fixed: visited keys are now written to
a small JSON sidecar file next to the CSV (`<csv_path>.visited.json`)
every time a new one is marked, and reloaded at startup -- see
`_visited_state_path()`, `_load_visited_state()`, `_save_visited_state()`.

STALENESS SAFEGUARD, not just "trust the file": the sidecar also stores
a fingerprint (the CSV's mtime + byte size) of the semantic_targets.csv
it was written against. On load, if the CURRENT CSV's fingerprint
doesn't match, the sidecar is treated as stale and discarded with a
logged warning, rather than risk silently marking real, unvisited plants
in a genuinely NEW Phase 1 run as already visited just because an old
sidecar file happened to still be sitting next to the new CSV.
"""
import csv
import json
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from target_manager.visited_state import (
    csv_fingerprint, restore_visited_keys, build_sidecar_data, visited_state_path,
)


class TargetManagerNode(Node):
    def __init__(self):
        super().__init__('target_manager_node')
        d = self.declare_parameter
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        d('dedup_grid_m', 0.15)
        d('reach_radius_m', 0.10)        # mark visited within this
        d('row_spacing', 0.50)           # MUST match coverage_planner / row_navigation
        d('initial_csv_path',
          os.path.expanduser('~/ros2_ws/maps/semantic_targets.csv'))  # Phase 1's output

        g = lambda n: self.get_parameter(n).value
        # expanduser applied to the VALUE below (see initial_csv_path) --
        # the yaml overrides the expanded default with a literal '~'
        # string, same bug class fixed in semantic_mapper_node.py.
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.grid = g('dedup_grid_m')
        self.reach = g('reach_radius_m')
        self.row_spacing = float(g('row_spacing'))
        self.csv_path = os.path.expanduser(g('initial_csv_path'))

        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.seen = {}          # cell -> (x,y,z) -- both loaded AND live-detected
        self.visited = set()
        self.order = []         # ordered list of cell keys -- THE pre-mapped trajectory
        self._order_dirty = False

        self._load_initial_targets()

        self.create_subscription(PoseArray, '/plant_targets', self.cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/selected_plant_goal', 10)
        self.create_timer(1.0, self.pick)
        self.get_logger().info(
            f'target_manager_node ready: {len(self.seen)} plant(s) loaded from '
            f'{self.csv_path if self.seen else "(no file / empty -- live-detection only)"}.')

    # ------------------------------------------------------------ startup load
    def _load_initial_targets(self):
        if not os.path.exists(self.csv_path):
            self.get_logger().info(
                f'{self.csv_path} not found -- starting with an EMPTY pre-mapped '
                f'list. Expected and normal if you are exploring a genuinely '
                f'unknown tunnel (no Phase 1 to have produced this file yet -- '
                f'this node is not even launched by the Orin explore-mode '
                f'launch file, see docs/ORIN_PI_SPLIT_ARCHITECTURE.md). If '
                f'you DID expect a pre-mapped list here (e.g. running Phase 2 '
                f'of the known-layout workflow), this means Phase 1 either '
                f'was not run or saved to a different path -- check before '
                f'proceeding.')
            return
        try:
            with open(self.csv_path, newline='') as f:
                for row in csv.DictReader(f):
                    x, y, z = float(row['x']), float(row['y']), float(row['z'])
                    key = (round(x / self.grid), round(y / self.grid))
                    self.seen[key] = (x, y, z)
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().error(
                f'Failed to load {self.csv_path}: {e}. Starting with an EMPTY '
                f'pre-mapped list -- check the file was written by a completed '
                f'Phase 1 run (see DEPLOYMENT_GUIDE.md, "End of Phase 1").')
            return
        self._order_dirty = True
        self._rebuild_order()
        self._load_visited_state()

    # ------------------------------------------------------------ visited-state persistence
    def _load_visited_state(self):
        path = visited_state_path(self.csv_path)
        if not os.path.exists(path):
            return
        try:
            with open(path) as f:
                data = json.load(f)
            self.visited, was_stale = restore_visited_keys(
                data, csv_fingerprint(self.csv_path), set(self.seen.keys()))
            if was_stale:
                self.get_logger().warn(
                    f'{path} exists but its recorded CSV fingerprint does not '
                    f'match the currently-loaded {self.csv_path} -- treating as '
                    f'STALE (likely from a different/earlier Phase 1 run) and '
                    f'ignoring it, rather than risk marking real unvisited '
                    f'plants as already visited. Starting with a clean visited '
                    f'set.')
            elif self.visited:
                self.get_logger().info(
                    f'Restored {len(self.visited)} previously-visited plant(s) '
                    f'from {path} (interrupted-run resume).')
        except (OSError, ValueError, TypeError) as e:
            self.get_logger().warn(
                f'Failed to load visited-state from {path}: {e}. Starting with '
                f'a clean visited set rather than failing startup over this.')

    def _save_visited_state(self):
        path = visited_state_path(self.csv_path)
        try:
            with open(path, 'w') as f:
                json.dump(build_sidecar_data(self.csv_path, self.visited), f)
        except OSError as e:
            self.get_logger().warn(
                f'Failed to write visited-state to {path}: {e} -- a restart '
                f'from here would re-offer this plant. Not fatal, continuing.')

    # ------------------------------------------------------------ ordering
    def _rebuild_order(self):
        """Row-bucket + alternating-direction sort -- see module docstring.
        Re-run whenever a new key is added; O(n log n) on a plant COUNT
        (tens to low hundreds), trivial cost even on a 2GB Pi."""
        def row_of(key):
            _, y, _ = self.seen[key]
            return round(y / self.row_spacing)

        rows = {}
        for key in self.seen:
            rows.setdefault(row_of(key), []).append(key)

        ordered = []
        for row_idx in sorted(rows):
            row_keys = rows[row_idx]
            descending = (row_idx % 2 == 1)   # alternate, matching coverage_planner
            row_keys.sort(key=lambda k: self.seen[k][0], reverse=descending)
            ordered.extend(row_keys)
        self.order = ordered
        self._order_dirty = False

    # ------------------------------------------------------------ live callback
    def cb(self, msg):
        try:
            # msg.header.stamp, not rclpy.time.Time() ("latest") -- this is
            # projecting a PAST detection into the map frame, so it needs
            # the transform AS OF when the detection was actually made, not
            # the robot's current pose. Using "latest" here introduces a
            # positional error proportional to robot speed x the latency
            # between detection and this callback running, worst while the
            # robot is actively moving. _robot_xy() below is intentionally
            # different -- it wants the CURRENT pose, not a historical one.
            tr = self.buf.lookup_transform(
                self.map_frame, msg.header.frame_id,
                msg.header.stamp, timeout=Duration(seconds=0.1))
        except Exception:  # noqa: BLE001
            return
        for p in msg.poses:
            tp = do_transform_pose(p, tr)
            key = (round(tp.position.x / self.grid), round(tp.position.y / self.grid))
            is_new = key not in self.seen
            self.seen[key] = (tp.position.x, tp.position.y, tp.position.z)
            if is_new:
                # genuinely new (not in the Phase-1 CSV) -- append to the END of
                # the pre-mapped order rather than re-sorting everything, so a
                # late discovery doesn't reshuffle a trajectory Nav2/the operator
                # may already be relying on.
                self.order.append(key)

    def _robot_xy(self):
        try:
            tr = self.buf.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            return tr.transform.translation.x, tr.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    # ------------------------------------------------------------ main pick loop
    def pick(self):
        if self._order_dirty:
            self._rebuild_order()
        rob = self._robot_xy()
        if rob is None or not self.order:
            return
        rx, ry = rob

        # mark-visited pass: anything within reach_radius of the CURRENT pose,
        # regardless of position in the order (handles the case where the robot
        # is already sitting on the next target when this fires).
        for key in self.order:
            if key in self.visited or key not in self.seen:
                continue
            x, y, _ = self.seen[key]
            if math.hypot(x - rx, y - ry) < self.reach:
                self.visited.add(key)
                self._save_visited_state()

        # advance strictly through the pre-mapped order -- see module docstring
        # for why this is preferred over nearest-unvisited for this application.
        for key in self.order:
            if key in self.visited:
                continue
            if key not in self.seen:
                continue
            x, y, z = self.seen[key]
            g = PoseStamped()
            g.header.frame_id = self.map_frame
            g.header.stamp = self.get_clock().now().to_msg()
            g.pose.position.x = x
            g.pose.position.y = y
            g.pose.position.z = z
            g.pose.orientation.w = 1.0
            self.goal_pub.publish(g)
            return
        # exhausted -- nothing left to publish (mission_control's own state
        # machine, not this node, is responsible for declaring mission complete)


def main():
    rclpy.init()
    node = TargetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
