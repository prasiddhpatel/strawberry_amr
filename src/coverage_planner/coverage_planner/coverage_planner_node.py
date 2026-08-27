#!/usr/bin/env python3
"""
Boustrophedon ("ox-plough") coverage planner for the strawberry AMR.

This node makes the GLOBAL mission route an EXPLICIT, inspectable plan rather
than an implicit side-effect of the row follower's "count headland turns" loop.
It computes a serpentine route that visits every tabletop aisle in turn and
publishes:

  /coverage_plan   std_msgs/String  (JSON, latched)  -- the ordered list of
                   row segments and the headland turn after each one. Consumed
                   by mission_control to know how many rows to cover and to
                   track progress.

  /coverage_path   nav_msgs/Path    (latched)         -- the geometric route in
                   the map frame for RViz visualisation (and optional use as a
                   Nav2 NavigateThroughPoses goal).

Geometry (map frame): aisles run along +x with length `row_length`; aisle i has
its centreline at  y = first_row_y + i * row_spacing  and is entered at x =
`origin_x`. Even rows are driven forward (+x), odd rows reverse (-x); the
robot U-turns in the headland between rows. The headland turn handedness
alternates left, right, left, ... which matches the row follower's alternating
point-turn, so plan and execution stay consistent.

NOTE: the row follower (row_navigation) executes the rows reactively from LiDAR;
this plan does not micro-command the wheels. It defines the row ORDER, the row
COUNT, and the turn sequence, and provides a visual route. Path tracking inside
a row is still closed-loop on the posts.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def _latched_qos():
    # transient_local so late subscribers (FSM, RViz) still receive the plan.
    return QoSProfile(depth=1,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner_node')
        d = self.declare_parameter
        d('num_rows', 6)
        d('row_spacing', 0.50)        # lateral spacing between aisle centrelines (m)
        d('row_length', 8.0)          # drivable length of each aisle (m)
        d('origin_x', 0.0)            # x of the row-entry end (m, map frame)
        d('first_row_y', 0.0)         # y of aisle 0 centreline (m, map frame)
        d('headland_margin', 0.6)     # SCHEMATIC route-preview only -- NOT the real
                                       # required clearance; see docs/HEADLAND_TURN_GEOMETRY.md
                                       # (~1.7 m actual) and the note in coverage_params.yaml
        d('map_frame', 'map')
        d('republish_period', 2.0)    # re-publish latched plan at this period (s)

        g = lambda n: self.get_parameter(n).value
        self.n = max(1, int(g('num_rows')))
        self.spacing = float(g('row_spacing'))
        self.length = float(g('row_length'))
        self.x0 = float(g('origin_x'))
        self.y0 = float(g('first_row_y'))
        self.hl = float(g('headland_margin'))
        self.frame = g('map_frame')

        qos = _latched_qos()
        self.plan_pub = self.create_publisher(String, '/coverage_plan', qos)
        self.path_pub = self.create_publisher(Path, '/coverage_path', qos)

        self.plan_msg, self.path_msg = self._build()
        self.plan_pub.publish(self.plan_msg)
        self.path_pub.publish(self.path_msg)
        # Periodic re-publish keeps RViz / late nodes in sync (cheap, latched).
        self.create_timer(float(g('republish_period')), self._tick)

        self.get_logger().info(
            f'coverage_planner_node ready: {self.n} rows, '
            f'spacing={self.spacing:.2f} m, length={self.length:.1f} m '
            f'(boustrophedon serpentine).')

    # ------------------------------------------------------------------
    def _build(self):
        """Return (String plan, Path route)."""
        segments = []
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()

        x_near = self.x0
        x_far = self.x0 + self.length

        for i in range(self.n):
            y = self.y0 + i * self.spacing
            forward = (i % 2 == 0)
            if forward:
                entry, exit_, heading = (x_near, y), (x_far, y), 0.0
            else:
                entry, exit_, heading = (x_far, y), (x_near, y), math.pi

            if i < self.n - 1:
                turn_after = 'left' if (i % 2 == 0) else 'right'
            else:
                turn_after = 'none'

            segments.append({
                'row': i,
                'direction': 'forward' if forward else 'reverse',
                'entry': [round(entry[0], 3), round(entry[1], 3)],
                'exit': [round(exit_[0], 3), round(exit_[1], 3)],
                'turn_after': turn_after,
            })

            # --- geometric route waypoints (map frame) ---
            self._add_pose(path, entry[0], entry[1], heading)
            self._add_pose(path, exit_[0], exit_[1], heading)
            if turn_after != 'none':
                # drive out into the headland, then shift laterally to the next aisle
                sign = 1.0 if forward else -1.0
                hx = exit_[0] + sign * self.hl
                self._add_pose(path, hx, y, heading)
                self._add_pose(path, hx, y + self.spacing, heading)

        plan = {
            'frame': self.frame,
            'pattern': 'boustrophedon',
            'num_rows': self.n,
            'row_spacing': self.spacing,
            'row_length': self.length,
            'segments': segments,
        }
        s = String()
        s.data = json.dumps(plan)
        return s, path

    def _add_pose(self, path, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.frame
        p.header.stamp = path.header.stamp
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        path.poses.append(p)

    def _tick(self):
        # refresh stamp and re-publish (latched topics keep last sample)
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        self.plan_pub.publish(self.plan_msg)
        self.path_pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
