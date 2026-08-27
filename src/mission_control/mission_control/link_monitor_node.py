#!/usr/bin/env python3
"""
Publishes /link_status so the operator can SEE whether the Orin side of
the Pi<->Orin link is actually alive -- inspired by a link-watchdog idea
found while reviewing an external AI-generated bundle for this same robot,
rewritten here with an important, deliberate difference: this node is NOT
a required safety mechanism, and its docstring says so explicitly rather
than implying otherwise.

UPDATED AGAIN: an earlier revision of this docstring (written right after
row_navigation/safety_supervisor/twist_mux/ekf_node moved to the Orin)
correctly said a link drop now means a blunt hard stop with no graceful
fallback, since at that point EVERYTHING between the LiDAR and the motors
was Orin-side, including teleop's arbitration. That is no longer the
full picture: teleop now has its own Pi-local twist_mux instance
(twist_mux_pi_local, see pi_hardware_and_control.launch.py) that
arbitrates human override independently of the Orin and the network
link entirely. So there are now genuinely two different outcomes on a
link drop, not one:

  - If the operator is NOT actively driving (deadman released) when the
    link drops: unchanged from the previous correction -- base_controller's
    local command watchdog produces a hard stop once /cmd_vel goes stale
    (default 0.5s). Autonomous behaviour (row_navigation, Nav2) cannot
    reach the Pi at all during the drop, regardless of what either
    decides upstream.
  - If the operator IS actively driving (deadman held) when the link
    drops: control continues, completely unaffected, because
    twist_mux_pi_local's arbitration between /cmd_vel_teleop and
    /cmd_vel_remote happens entirely on this Pi -- /cmd_vel_remote simply
    times out and stops competing, exactly like any twist_mux input
    timeout, and teleop keeps winning as it already was.

The physical hardware e-stop remains independent of all of this in both
cases, by design. The PS2 pad's own SOFTWARE e-stop button also now locks
out motion with zero network dependency (it feeds twist_mux_pi_local's
lock directly) -- a genuine improvement over the previous revision, where
even that had to round-trip through the Orin.

ONE LIMIT THAT DID NOT CHANGE AND CANNOT, BY THE NATURE OF THE PROBLEM:
safety_supervisor's obstacle-triggered e-stop is Orin-side, so it cannot
reach the Pi during a link drop either way. Driving manually during a
drop means driving without that specific protection -- your own judgement,
the deadman requirement, and the physical e-stop are what's left in that
window. State this to whoever is operating the pad; don't let the "teleop
now works during a drop" improvement above read as "nothing to think
about here."

What THIS node adds, on top of all of the above, is unchanged: purely
informational, an explicit monitorable signal so the operator finds out
the link is down and can go investigate, rather than only noticing
indirectly because the robot has gone quiet.

Signal used: /mission/status, published continuously by mission_control's
Orin-side instance (see mission_control_node.py) as a natural heartbeat
-- if it stops updating for longer than max_silence_sec, the Orin side
is presumed unreachable (network drop, crashed process, or anything else
that stops it publishing -- this node can't and doesn't try to distinguish
which).
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class LinkMonitorNode(Node):
    def __init__(self):
        super().__init__('link_monitor_node')
        d = self.declare_parameter
        d('max_silence_sec', 3.0)   # generous relative to mission_control's own
                                    # 0.5s publish loop -- this is an awareness
                                    # signal, not a fast-reacting safety timeout,
                                    # so a wider margin avoids false alarms from
                                    # an ordinary GC pause or a brief signal dip
                                    # on the Pi<->Orin WiFi link itself (see
                                    # scripts/setup_network_pi_orin.sh -- this
                                    # link is WiFi, not wired, so some jitter
                                    # here is expected and not itself a fault)
        d('check_rate_hz', 2.0)
        self.max_silence = float(self.get_parameter('max_silence_sec').value)

        self._last_seen = None
        self._link_up = False   # starts False -- "unknown/not yet seen" is the
                                 # honest initial state, not an assumed-good one

        self.create_subscription(String, '/mission/status', self._on_status, 10)
        self.pub = self.create_publisher(Bool, '/link_status', 10)
        self.create_timer(1.0 / float(self.get_parameter('check_rate_hz').value),
                          self._check)
        self.get_logger().info(
            f'link_monitor_node ready (informational only -- see module '
            f'docstring for why this is not a required safety mechanism). '
            f'max_silence={self.max_silence}s')

    def _on_status(self, _msg):
        self._last_seen = time.monotonic()
        if not self._link_up:
            self._link_up = True
            self.pub.publish(Bool(data=True))
            self.get_logger().info('Orin link: UP')

    def _check(self):
        if self._last_seen is None:
            return   # haven't heard from the Orin even once yet -- stay silent
                     # rather than falsely claim "down" before a first contact
        elapsed = time.monotonic() - self._last_seen
        if elapsed > self.max_silence and self._link_up:
            self._link_up = False
            self.pub.publish(Bool(data=False))
            self.get_logger().warn(
                f'Orin link: DOWN ({elapsed:.1f}s since last /mission/status). '
                f'Row-following continues locally regardless -- see this node\'s '
                f'own docstring. Check the Orin and the network link.')


def main():
    rclpy.init()
    node = LinkMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
