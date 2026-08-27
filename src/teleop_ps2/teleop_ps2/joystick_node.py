#!/usr/bin/env python3
"""
PS2 / PS-style joypad teleop with a safety-first interaction model.

Publishes:
  /cmd_vel_teleop   geometry_msgs/Twist   (manual driving -- see "CRITICAL"
                                           note below on exactly when this
                                           is and is not published)
  /autonomy_enable  std_msgs/Bool         (master autonomy switch for row_nav)
  /e_stop           std_msgs/Bool         (latched software stop -> feeds
                                           twist_mux_pi_local's lock, see
                                           docs/ORIN_PI_SPLIT_ARCHITECTURE.md.
                                           TRANSIENT_LOCAL QoS, published only
                                           on change -- see _estop_qos() --
                                           NOT on the 0.2s heartbeat below,
                                           because safety_supervisor_node.cpp
                                           independently publishes to this
                                           same topic and continuous
                                           republishing from both sides was
                                           clobbering each other's latch)

Interaction model:
  * DEADMAN (hold, e.g. L1): you only drive while holding it. Holding it also
    forces /autonomy_enable=False, so grabbing the stick always takes control
    away from autonomy (human override).
  * AUTONOMY TOGGLE (press, e.g. X/cross): flips /autonomy_enable, but ONLY
    when the deadman is released. Hand control to autonomy by letting go of
    the deadman and tapping this.
  * E-STOP (press, e.g. Circle): latches /e_stop=True (everything stops).
  * RESET (press, e.g. Start): clears the e-stop latch.
  * TURBO (hold, e.g. R1): raises max speed.

CRITICAL -- /cmd_vel_teleop is published ONLY while the deadman is held,
never as an unconditional per-cycle publish (not even an explicit zero
when released). This is not a style preference; it's required for
correctness. twist_mux and twist_mux_pi_local (see
docs/ORIN_PI_SPLIT_ARCHITECTURE.md) select the highest-priority topic
that has received ANY message within its configured timeout, entirely
independent of that message's actual value. An earlier revision of this
file published an explicit Twist() every single /joy callback regardless
of deadman state -- meaning /cmd_vel_teleop, at priority 100, would NEVER
time out, and would therefore PERMANENTLY outrank every autonomous
behaviour underneath it (row-following at 10, Nav2 approach at 20),
regardless of whether the deadman was actually held, for the entire time
this node happens to be running -- which is always, since every launch
file in this workspace includes it as the ever-present human override.
That is a serious, previously-undiscovered bug predating any of today's
Pi/Orin-split work: it would have meant autonomous driving could never
actually produce motion, on any hardware, at any point, as long as this
node was also running (which is every real deployment configuration this
workspace has). Fixed by simply not calling publish() on /cmd_vel_teleop
when the deadman is released, letting the mux's own timeout correctly
detect "teleop has nothing to contribute" and fall through to autonomy.

WHY THIS NODE ALSO TRACKS /joy'S OWN STALENESS (joy_stale_timeout_s): if
the PS2 dongle disconnects while the deadman was held, joy_cb() simply
stops being called -- which already, on its own, stops /cmd_vel_teleop
from refreshing (see above), so no extra handling is needed for THAT
specific property. The staleness check here exists for a narrower,
secondary purpose: keeping this node's own internal deadman_live state
(used only for this node's own logging clarity) from silently claiming
"still held" forever after the controller has gone physically quiet.

Axis/button indices vary by adapter -- confirm with `ros2 topic echo /joy`
and edit config/ps2_mapping.yaml.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def _estop_qos():
    # TRANSIENT_LOCAL (latched) rather than the previous plain depth-10
    # QoS: /e_stop is a LOCK signal for twist_mux_pi_local (timeout: 0.0 --
    # it holds the last received value indefinitely, it does not require
    # continuous republishing), and this node now only publishes on this
    # topic when self.estop actually CHANGES (see _publish_latches below),
    # not on every 0.2s heartbeat / every /joy callback. Without
    # TRANSIENT_LOCAL, a subscriber that starts after the last publish
    # would see nothing until the next change. safety_supervisor_node.cpp
    # uses the identical QoS on this same topic for the identical reason --
    # see that file's module header LATCHING section for the full context:
    # two independent, uncoordinated publishers continuously republishing
    # to one Bool topic meant whichever last-wrote determined the topic's
    # apparent value, silently clobbering the other's latch within
    # ~50-200ms regardless of either node's own latch logic being correct.
    return QoSProfile(depth=1,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        d = self.declare_parameter
        # axes
        d('axis_linear', 1)
        d('axis_angular', 2)
        d('invert_linear', False)
        d('invert_angular', False)
        # buttons
        d('deadman_button', 6)        # L1
        d('turbo_button', 7)          # R1
        d('autonomy_button', 0)       # X / cross
        d('estop_button', 1)          # circle
        d('reset_button', 9)          # start
        # speeds
        d('max_linear', 0.30)
        d('max_angular', 0.8)
        d('turbo_linear', 0.5)
        d('turbo_angular', 1.2)
        d('deadzone', 0.08)
        # staleness -- see module docstring
        d('joy_stale_timeout_s', 0.5)

        g = lambda n: self.get_parameter(n).value
        self.ax_lin, self.ax_ang = g('axis_linear'), g('axis_angular')
        self.inv_lin, self.inv_ang = g('invert_linear'), g('invert_angular')
        self.b_dead, self.b_turbo = g('deadman_button'), g('turbo_button')
        self.b_auto, self.b_estop = g('autonomy_button'), g('estop_button')
        self.b_reset = g('reset_button')
        self.v_max, self.w_max = g('max_linear'), g('max_angular')
        self.v_turbo, self.w_turbo = g('turbo_linear'), g('turbo_angular')
        self.dz = g('deadzone')
        self.joy_stale_timeout = float(g('joy_stale_timeout_s'))

        self.autonomy = False
        self.estop = False
        self._last_published_estop = None   # None = "never published yet",
                                              # forces an initial publish
                                              # below regardless of value
        self.deadman_live = False   # current, joy-stream-derived deadman state
        self.prev_auto = 0
        self.prev_estop = 0
        self.prev_reset = 0
        self._last_joy_time = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.auto_pub = self.create_publisher(Bool, '/autonomy_enable', 10)
        self.estop_pub = self.create_publisher(Bool, '/e_stop', _estop_qos())
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        # heartbeat so /autonomy_enable keeps publishing even without joy
        # events -- /e_stop is exempted from this heartbeat's continuous
        # republish, see _publish_latches() and _estop_qos() above
        self.create_timer(0.2, self._publish_latches)
        self._publish_latches()   # establish an explicit initial /e_stop
                                    # value on the TRANSIENT_LOCAL topic
        self.get_logger().info('joystick_node ready (deadman + e-stop + autonomy).')

    def _dz(self, v):
        return 0.0 if abs(v) < self.dz else v

    def _btn(self, buttons, idx):
        return buttons[idx] if 0 <= idx < len(buttons) else 0

    def joy_cb(self, msg: Joy):
        self._last_joy_time = self.get_clock().now()
        ax, bt = msg.axes, msg.buttons

        # --- latched e-stop ---
        es = self._btn(bt, self.b_estop)
        if es and not self.prev_estop:
            self.estop = True
            # Also force autonomy off here, not just gate the published value
            # below: without this, self.autonomy silently survives the whole
            # e-stop/reset cycle untouched, so clearing the e-stop (reset
            # button) republishes /autonomy_enable=True on its own, with no
            # fresh press of the autonomy toggle required. An e-stop reset
            # must always return to "autonomy off" and require a deliberate,
            # separate act to resume automation.
            self.autonomy = False
            self.get_logger().warn('E-STOP latched (joypad) -- autonomy forced off.')
        self.prev_estop = es
        rs = self._btn(bt, self.b_reset)
        if rs and not self.prev_reset:
            if self.estop:
                self.get_logger().info('E-stop cleared (joypad).')
            self.estop = False
        self.prev_reset = rs

        deadman = bool(self._btn(bt, self.b_dead))
        self.deadman_live = deadman

        # --- autonomy toggle (only when deadman released) ---
        au = self._btn(bt, self.b_auto)
        if au and not self.prev_auto and not deadman and not self.estop:
            self.autonomy = not self.autonomy
            self.get_logger().info(f'Autonomy -> {self.autonomy}')
        self.prev_auto = au

        # holding the deadman always overrides autonomy OFF
        if deadman:
            self.autonomy = False

        # --- manual velocity: ONLY PUBLISHED while deadman held -- see
        # module docstring's "CRITICAL" section for why this must not be
        # an unconditional per-cycle publish.
        if deadman and not self.estop:
            turbo = bool(self._btn(bt, self.b_turbo))
            vmax = self.v_turbo if turbo else self.v_max
            wmax = self.w_turbo if turbo else self.w_max
            lin = self._dz(ax[self.ax_lin] if self.ax_lin < len(ax) else 0.0)
            ang = self._dz(ax[self.ax_ang] if self.ax_ang < len(ax) else 0.0)
            if self.inv_lin:
                lin = -lin
            if self.inv_ang:
                ang = -ang
            cmd = Twist()
            cmd.linear.x = lin * vmax
            cmd.angular.z = ang * wmax
            self.cmd_pub.publish(cmd)
        # else: publish NOTHING on /cmd_vel_teleop this cycle. This is what
        # lets twist_mux_pi_local's own timeout correctly fall through to
        # /cmd_vel_remote (autonomy) once the deadman is released.

        self._publish_latches()

    def _publish_latches(self):
        # /joy staleness check -- see module docstring. Affects only this
        # node's own deadman_live bookkeeping/logging; /cmd_vel_teleop's
        # timeout-relevant behaviour already follows correctly from joy_cb()
        # simply not being called while /joy is stale (see above).
        if self._last_joy_time is not None:
            elapsed = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
            if elapsed > self.joy_stale_timeout:
                self.deadman_live = False

        self.auto_pub.publish(Bool(data=self.autonomy and not self.estop))
        # Edge-triggered only -- see _estop_qos() above for why: this used
        # to publish unconditionally every 0.2s (and on every /joy
        # callback), which on a shared, non-latched /e_stop topic meant
        # this node's own routine republish of False could clobber
        # safety_supervisor_node.cpp's independent latch, and vice versa,
        # within one cycle. TRANSIENT_LOCAL QoS means a late subscriber
        # still gets the current value even though this only publishes on
        # an actual change.
        if self.estop != self._last_published_estop:
            self.estop_pub.publish(Bool(data=self.estop))
            self._last_published_estop = self.estop


def main():
    rclpy.init()
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # deliberately do NOT publish a final Twist() here -- see module
        # docstring's "CRITICAL" section; publishing anything on shutdown,
        # even zero, would be exactly the mistake this fix corrects.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
