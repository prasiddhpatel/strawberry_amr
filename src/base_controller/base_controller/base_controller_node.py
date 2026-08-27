#!/usr/bin/env python3
"""
Hardware interface for the Yahboom ROSMASTER R2, wrapping Yahboom's own
Rosmaster_Lib (vendored in ./vendor/Rosmaster_Lib.py, v2.3.3, taken verbatim
from the official ROSMASTER_R2_robot_sample_codes package the operator
supplied) rather than a hand-rolled serial protocol.

Subscribes:
  /cmd_vel             geometry_msgs/Twist   (final, arbitrated command --
                                              from twist_mux_pi_local)

Publishes:
  /wheel_odom          nav_msgs/Odometry     -- consumed by the EKF
                                              (robot_localization, external)
  /imu/data_raw        sensor_msgs/Imu       -- consumed by imu_filter_madgwick
                                              and the EKF (both external)
  /battery_state        sensor_msgs/BatteryState  -- diagnostic/monitoring
                                              only; nothing in this workspace
                                              gates behaviour on it today
  /wheel_encoders_raw   std_msgs/Int32MultiArray  -- diagnostic only, for the
                                              bench-verification step in
                                              docs/PARAMETER_TUNING_CHECKLIST.md
                                              (confirming real encoder counts
                                              match the 836 CPR figure); not
                                              consumed by any node

WHY THIS REPLACES THE EARLIER CUSTOM-PROTOCOL DESIGN: reading the real
Rosmaster_Lib source settled several previously-disputed claims with
certainty, and revealed a materially better architecture than hand-rolling
an Ackermann bicycle-model command converter ourselves:

  * Rosmaster_Lib exposes set_car_motion(vx, vy, vz), which is CAR-TYPE-AWARE
    (car_type is baked into every FUNC_MOTION packet). For R2/R2L
    specifically: vx in [-1.8, 1.8] m/s, vy in [-0.045, 0.045] m/s
    (near-zero on purpose -- an Ackermann chassis cannot strafe), vz in
    [-3, 3] rad/s. The STM32 firmware performs its OWN internal Ackermann
    bicycle-model conversion into rear-motor speed and front steering
    angle. That means a separate ackermann_bridge package computing
    delta = atan(L*omega/v) itself was duplicating logic the vendor's own
    firmware already implements and has tested against the real hardware
    -- trusting tested vendor code over a from-scratch reimplementation is
    the more reliable choice, so ackermann_bridge was removed and its role
    folded into this node (see git history for the removed package).
  * car_type: CARTYPE_R2 = 0x05, NOT 2 -- confirmed directly from
    `self.CARTYPE_R2 = 0x05` in the vendored source. An earlier draft
    (content pasted from a different AI session into this project) claimed
    car_type=2; that was wrong and was never used in any shipped code here.
  * get_battery_voltage() returns `self.__battery_voltage / 10.0` --
    confirmed DIVIDE by 10.0, not "multiply by 0.01", directly from source.
    A different pasted claim asserted x0.01 with a worked example
    (1210 -> 12.10V) that is not even internally consistent with real
    /10.0 math (1210/10.0 = 121.0V) -- further evidence that source should
    not have been trusted without checking it against the real library.
  * get_imu_attitude_data() (a ready fused-quaternion getter) EXISTS in the
    source but is commented out and inactive in this library version -- so
    the claim that firmware-fused orientation is available and
    imu_filter_madgwick is redundant does not hold here. Only
    get_accelerometer_data()/get_gyroscope_data()/get_magnetometer_data()
    (raw axes) are active, so the external Madgwick filter this workspace
    already used is still the right design, not an unnecessary extra step.
  * get_motion_data() returns (vx, vy, vz) already in SI units (m/s, m/s,
    rad/s) -- confirmed from the receive-thread unpacking, which reverses
    the same *1000 int16 encoding set_car_motion uses to SEND commands.
    Used as the PRIMARY odometry source here rather than re-deriving
    Ackermann odometry from raw encoder ticks in Python a second time, for
    the same "don't duplicate the vendor's tested kinematic model" reason
    command conversion was delegated to the firmware. Raw 4-channel
    encoder ticks (get_motor_encoder()) are still read and published on a
    diagnostic topic for the encoder bench-check in the calibration guide,
    but are NOT the primary odometry input.

REMAINING GENUINE UNCERTAINTY (flagged, not hidden): accel_ratio's output
units are not stated in the vendored source (gyro_ratio's inline comment
says "+/-500dps" explicitly; accel_ratio has no equivalent comment). This
node assumes the conventional IMU-wrapper pattern of accelerometer output
already in g's, multiplying by G_TO_MS2 below to get m/s^2 for the ROS Imu
message. VERIFY this against docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md
Part 3.2's stationary check: a level, stationary reading should show ~9.8
on the vertical axis of /imu/data_raw -- if it instead shows ~1.0, the
raw ratio is already m/s^2 and G_TO_MS2 below should be set to 1.0.

HARDWARE THIS TALKS TO: Yahboom YB-ERF01-V2.0 board (STM32F103RCT6) over
USB-serial, default port name /dev/myserial (matching Rosmaster_Lib's own
default -- see scripts/setup_udev_rules.sh, which creates this symlink),
115200 baud (set inside Rosmaster_Lib itself, not configurable here).

WHY THIS NODE NEEDS NO SPECIAL HANDLING FOR THE NETWORK-INDEPENDENT-TELEOP
DESIGN: making manual override work during a Pi<->Orin link drop (see
docs/ORIN_PI_SPLIT_ARCHITECTURE.md) is handled entirely UPSTREAM of this
node, by a second, Pi-local twist_mux instance
(twist_mux_pi_local, see pi_hardware_and_control.launch.py and
twist_mux_pi_local.yaml) that arbitrates /cmd_vel_teleop against whatever
arrived from the Orin, and republishes the winner to /cmd_vel -- exactly
the same topic this node has always subscribed to. This node's own
subscription and watchdog below are therefore completely unchanged from
before the Pi/Orin split existed; an earlier attempt at this same
problem tried to duplicate that arbitration logic directly inside this
node (new subscriptions to /cmd_vel_teleop, /e_stop, a hand-rolled
priority timer) -- reverted, since it duplicated what twist_mux_pi_local
already does, correctly and more simply, using the standard, tested
twist_mux package rather than bespoke code here.

Every set_car_motion() call below always passes 0.0 for vy explicitly,
regardless of source: an Ackermann chassis cannot strafe, and
set_car_motion's own R2 range for vy is a near-zero +/-0.045 m/s
tolerance band, not a real capability -- forcing it here is a cheap
defensive guard against any upstream source ever setting linear.y on this
platform.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

from base_controller.vendor.Rosmaster_Lib import Rosmaster
from base_controller.command_safety import clamp_command

G_TO_MS2 = 9.80665   # see "REMAINING GENUINE UNCERTAINTY" above -- verify on bench
DEG_TO_RAD = math.pi / 180.0

# Confirmed from Rosmaster_Lib.py source (class attributes on Rosmaster.__init__)
CARTYPE_R2 = 0x05

# /wheel_odom covariance -- PLACEHOLDER VALUES, NOT BENCH-MEASURED.
# All-zero covariance (the previous state) is not "unknown" to
# robot_localization's EKF -- it means PERFECT CERTAINTY, and gets copied
# straight into the measurement-noise matrix R for whatever state
# dimensions ekf.yaml fuses from this topic (currently just twist.linear.x,
# per odom0_config), causing the filter to over-trust that raw measurement
# with essentially zero blending against its own prediction.
#
# ODOM_VX_VARIANCE and ODOM_YAWRATE_VARIANCE are the two values that
# actually matter given the CURRENT ekf.yaml fusion config; the rest exist
# so this message is honest about uncertainty on every channel regardless
# of what gets fused later, not because anything currently reads them.
#
# These are ROUGH, CONSERVATIVE STARTING NUMBERS pending real
# characterization, not measured drift data -- this project does not yet
# have that. get_motion_data() is a firmware-internal velocity estimate
# (Rosmaster_Lib), not something derived per-tick from the raw 836 CPR
# encoder ticks in this file, so its true noise characteristics cannot be
# derived from encoder quantization alone either -- only measured. Replace
# these with sample variance from repeated trials comparing commanded vs.
# ground-truth vx (tape-measure-and-stopwatch is a legitimate starting
# point; a surveyed space tightens it later) before trusting this for
# anything beyond "better than claiming perfect certainty".
ODOM_VX_VARIANCE = 0.0025        # (m/s)^2 -- sigma ~ 0.05 m/s. Ackermann
                                   # wheel-odometry-derived forward velocity;
                                   # this is the one channel ekf.yaml
                                   # currently fuses from this topic.
ODOM_VY_VARIANCE = 0.0025        # (m/s)^2 -- vy is not a meaningful DOF for
                                   # this Ackermann chassis (set_car_motion's
                                   # own R2 vy range is a near-zero +/-0.045
                                   # m/s tolerance band, not a real
                                   # capability); same order as vx rather
                                   # than a fixed-dimension near-zero value,
                                   # since it's still a live firmware
                                   # measurement, just of a quantity that
                                   # should stay near zero by construction.
ODOM_YAWRATE_VARIANCE = 0.01     # (rad/s)^2 -- sigma ~ 0.1 rad/s (~5.7
                                   # deg/s). Firmware differential-wheel-
                                   # speed-derived turn rate; not currently
                                   # fused (ekf.yaml takes vyaw from the IMU
                                   # instead), included for honesty in case
                                   # that ever changes.
ODOM_FIXED_DIM_VARIANCE = 1e-6   # for dimensions that are structurally
                                   # fixed at (near) zero for a 2D ground
                                   # vehicle -- linear.z, angular.x,
                                   # angular.y -- a small positive value
                                   # rather than literal 0.0, so the EKF
                                   # treats these as known-and-fixed rather
                                   # than hitting robot_localization's
                                   # zero-covariance special case. This
                                   # pattern does NOT apply to vx above:
                                   # vx is the genuinely uncertain quantity
                                   # this fix exists for, not a fixed
                                   # dimension, so it needs a real
                                   # (eventually measured) variance, not a
                                   # near-zero "treat as certain" value.
ODOM_POSE_XY_VARIANCE = 0.01     # m^2 -- sigma ~ 0.1 m per message. Pose is
                                   # open-loop Euler-integrated dead
                                   # reckoning here (see _poll_and_publish)
                                   # with no bound on accumulated error, so
                                   # a single constant covariance is a
                                   # compromise -- true uncertainty grows
                                   # with distance/time since the last
                                   # correction, which this message format
                                   # cannot express. Not currently fused
                                   # (ekf.yaml's odom0_config leaves all
                                   # pose dimensions false).
ODOM_POSE_YAW_VARIANCE = 0.02    # rad^2 -- sigma ~ 0.14 rad (~8 deg). Same
                                   # unbounded-growth caveat as above.


class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('base_controller_node')
        d = self.declare_parameter
        d('serial_port', '/dev/myserial')   # matches Rosmaster_Lib's own default name
        d('poll_rate_hz', 20.0)             # sensor/odometry poll + publish rate
        d('cmd_watchdog_timeout_s', 0.5)    # stop the robot if commands stop arriving
        d('max_linear_accel_mps2', 1.0)     # see command_safety.py's module docstring --
                                              # this NEVER limits deceleration/braking, only
                                              # increasing |velocity|. Generous relative to
                                              # this AMR's actual operating speeds (row-
                                              # following/headland are 0.06-0.35 m/s, well
                                              # under max_linear_mps=1.8), so it shouldn't be
                                              # felt in normal operation -- it exists to catch
                                              # a sudden, erroneous full-speed step command.
        d('max_linear_mps', 1.8)            # R2/R2L set_car_motion vx limit (firmware-enforced too)
        d('max_angular_radps', 3.0)         # R2/R2L set_car_motion vz limit (firmware-enforced too)
        d('publish_tf', False)              # EKF/robot_localization normally owns odom->base_link;
                                             # only enable this for a bench test with no EKF running
        d('base_frame', 'base_link')
        d('odom_frame', 'odom')
        # 3S LiPo thresholds (11.1V nominal, ~12.6V full, per the project's confirmed battery --
        # Yahboom colloquially calls this a "12V" pack in their own docs, consistent usage)
        d('battery_warn_v', 9.6)
        d('battery_critical_v', 9.0)

        g = lambda n: self.get_parameter(n).value
        self.max_v = float(g('max_linear_mps'))
        self.max_w = float(g('max_angular_radps'))
        self.watchdog_timeout = float(g('cmd_watchdog_timeout_s'))
        self.max_accel = float(g('max_linear_accel_mps2'))
        self.publish_tf = bool(g('publish_tf'))
        self.base_frame = g('base_frame')
        self.odom_frame = g('odom_frame')
        self.batt_warn = float(g('battery_warn_v'))
        self.batt_crit = float(g('battery_critical_v'))
        port = g('serial_port')
        poll_hz = float(g('poll_rate_hz'))

        self.bot = None
        try:
            self.bot = Rosmaster(car_type=CARTYPE_R2, com=port, debug=False)
            self.bot.create_receive_threading()
            self.get_logger().info(
                f'Rosmaster_Lib connected on {port}, car_type=CARTYPE_R2 (0x05).')
        except Exception as e:  # noqa: BLE001 -- genuinely want to catch anything from the vendor lib
            self.get_logger().error(
                f'Could not open {port} via Rosmaster_Lib: {e}. This node will keep running '
                f'(so the rest of the stack can start) but every hardware call below will be a '
                f'no-op until this is fixed -- check the port, check '
                f'scripts/setup_udev_rules.sh was run, check nothing else has the port open.')

        self.last_cmd_time = self.get_clock().now()
        self.last_vx_out = 0.0   # the actually-commanded vx from the previous
                                  # cycle -- clamp_command needs this, not the
                                  # requested vx, to correctly accumulate
                                  # acceleration limiting across calls
        self.x = self.y = self.theta = 0.0
        self._last_odom_time = None

        qos_cmd = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, qos_cmd)

        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.enc_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_timer(1.0 / poll_hz, self._poll_and_publish)
        self.create_timer(0.05, self._watchdog_check)   # 20 Hz watchdog, independent of poll rate

        self.get_logger().info(
            f'base_controller_node ready (Rosmaster_Lib wrapper). '
            f'poll_rate={poll_hz} Hz, watchdog={self.watchdog_timeout}s, '
            f'publish_tf={self.publish_tf} (leave False if an EKF is running).')

    # ------------------------------------------------------------ command path
    def _cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now()
        # dt computed from the OLD last_cmd_time, BEFORE overwriting it below --
        # this is genuinely "time since the previous command", which is what
        # clamp_command's acceleration limiting needs to compute a correct
        # max-delta-per-call bound.
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = now

        vx, vz = clamp_command(
            msg.linear.x, msg.angular.z,
            max_v=self.max_v, max_w=self.max_w,
            prev_vx=self.last_vx_out, dt=dt, max_accel=self.max_accel)
        self.last_vx_out = vx

        # vy is always forced to 0.0 here, not passed through from msg.linear.y:
        # an Ackermann chassis cannot strafe, and set_car_motion's own R2 range for
        # vy is a near-zero +/-0.045 m/s tolerance band, not a real capability --
        # callers should never be setting linear.y for this platform, and forcing
        # it here is a cheap defensive guard against a misconfigured upstream node.
        if self.bot is not None:
            self.bot.set_car_motion(vx, 0.0, vz)

    def _watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout and self.bot is not None:
            self.bot.set_car_motion(0.0, 0.0, 0.0)
            # Keep clamp_command's ramp state in sync with what was actually
            # commanded. Without this, last_vx_out stays at its pre-dropout
            # value while the motors are physically at 0 -- so when a real
            # command resumes at (or below) that same stale value,
            # clamp_command's `increasing_magnitude = abs(vx) > abs(prev_vx)`
            # check evaluates False and the acceleration limiter is skipped
            # entirely, producing an instant full-speed jump from the real
            # 0 m/s instead of a ramp-limited step -- exactly the "sudden,
            # erroneous full-speed step command" scenario command_safety.py's
            # own docstring says the limiter exists to catch.
            self.last_vx_out = 0.0

    # ------------------------------------------------------------ sensor/odometry path
    def _poll_and_publish(self):
        now = self.get_clock().now()
        if self.bot is None:
            return

        # ---- odometry: PRIMARY source is the firmware's own get_motion_data(),
        # see module docstring for why this is trusted over re-deriving Ackermann
        # kinematics from raw encoder ticks a second time in Python ----
        try:
            vx, vy, vz = self.bot.get_motion_data()
        except Exception:  # noqa: BLE001
            vx = vy = vz = 0.0

        if self._last_odom_time is not None:
            dt = (now - self._last_odom_time).nanoseconds / 1e9
            if 0.0 < dt < 1.0:  # guard against a huge dt on the very first/a delayed call
                # standard unicycle-style pose integration; vy stays ~0 by construction
                # for an Ackermann chassis, included for completeness/diagnostics only
                self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
                self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
                self.theta += vz * dt
        self._last_odom_time = now

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vz

        # Diagonal-only: no attempt to estimate cross-covariance terms
        # (e.g. x/y correlation) with placeholder data -- see the module-
        # level ODOM_* constants above for what these are and are not.
        # Row-major 6x6, diagonal at i*6+i for i in [x,y,z,roll,pitch,yaw].
        odom.pose.covariance[0] = ODOM_POSE_XY_VARIANCE     # x
        odom.pose.covariance[7] = ODOM_POSE_XY_VARIANCE     # y
        odom.pose.covariance[14] = ODOM_FIXED_DIM_VARIANCE  # z
        odom.pose.covariance[21] = ODOM_FIXED_DIM_VARIANCE  # roll
        odom.pose.covariance[28] = ODOM_FIXED_DIM_VARIANCE  # pitch
        odom.pose.covariance[35] = ODOM_POSE_YAW_VARIANCE   # yaw

        odom.twist.covariance[0] = ODOM_VX_VARIANCE         # vx -- the
                                                              # channel
                                                              # ekf.yaml
                                                              # actually
                                                              # fuses today
        odom.twist.covariance[7] = ODOM_VY_VARIANCE         # vy
        odom.twist.covariance[14] = ODOM_FIXED_DIM_VARIANCE  # vz
        odom.twist.covariance[21] = ODOM_FIXED_DIM_VARIANCE  # roll rate
        odom.twist.covariance[28] = ODOM_FIXED_DIM_VARIANCE  # pitch rate
        odom.twist.covariance[35] = ODOM_YAWRATE_VARIANCE   # yaw rate

        self.odom_pub.publish(odom)

        if self.publish_tf and self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = odom.pose.pose.orientation.z
            t.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(t)

        # ---- raw encoders: diagnostic only, for the calibration guide's
        # "confirm 836 ticks/rev" bench check -- not used for odometry above ----
        try:
            m1, m2, m3, m4 = self.bot.get_motor_encoder()
            enc_msg = Int32MultiArray()
            enc_msg.data = [int(m1), int(m2), int(m3), int(m4)]
            self.enc_pub.publish(enc_msg)
        except Exception:  # noqa: BLE001
            pass

        # ---- IMU: raw axes only (fused attitude getter is inactive in this
        # library version -- see module docstring) -- feeds imu_filter_madgwick ----
        try:
            ax, ay, az = self.bot.get_accelerometer_data()
            gx, gy, gz = self.bot.get_gyroscope_data()
            imu = Imu()
            imu.header.stamp = now.to_msg()
            imu.header.frame_id = 'imu_link'
            imu.linear_acceleration.x = ax * G_TO_MS2
            imu.linear_acceleration.y = ay * G_TO_MS2
            imu.linear_acceleration.z = az * G_TO_MS2
            # gyro_ratio's source comment states "+/-500dps" explicitly -> degrees/s,
            # REP-103 requires rad/s for sensor_msgs/Imu
            imu.angular_velocity.x = gx * DEG_TO_RAD
            imu.angular_velocity.y = gy * DEG_TO_RAD
            imu.angular_velocity.z = gz * DEG_TO_RAD
            imu.orientation_covariance[0] = -1.0  # no orientation in this raw message;
                                                    # imu_filter_madgwick produces /imu/data
            self.imu_pub.publish(imu)
        except Exception:  # noqa: BLE001
            pass

        # ---- battery ----
        try:
            v = self.bot.get_battery_voltage()   # confirmed /10.0 scale from source, see docstring
            batt = BatteryState()
            batt.header.stamp = now.to_msg()
            batt.voltage = float(v)
            batt.present = True
            if v <= self.batt_crit:
                batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                self.get_logger().error(f'BATTERY CRITICAL: {v:.2f}V <= {self.batt_crit}V')
            elif v <= self.batt_warn:
                self.get_logger().warn(f'Battery low: {v:.2f}V <= {self.batt_warn}V')
            self.batt_pub.publish(batt)
        except Exception:  # noqa: BLE001
            pass

    def destroy_node(self):
        if self.bot is not None:
            try:
                self.bot.set_car_motion(0.0, 0.0, 0.0)
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # GRACEFUL SHUTDOWN -- found missing during a robotics-agent-skills
        # review pass (robot-bringup skill: "graceful shutdown handlers...
        # zero velocity... on SIGINT/SIGTERM"). Without this, killing this
        # process (Ctrl+C, systemd stop, a crash in a supervising script)
        # left whatever velocity was last commanded in effect indefinitely
        # -- set_car_motion() is a "hold this" command, not something the
        # firmware re-asserts or expects a heartbeat for on its own, so
        # there was no other mechanism that would have stopped the robot.
        # This must be a genuine best-effort attempt, not a guarantee: if
        # the process is killed with SIGKILL (which cannot be caught) or
        # the Python interpreter itself is already unwinding badly, this
        # will not run. It is a real improvement over having nothing, not
        # a substitute for the physical e-stop, which remains the only
        # unconditional guarantee.
        if node.bot is not None:
            try:
                node.bot.set_car_motion(0.0, 0.0, 0.0)
            except Exception:  # noqa: BLE001 -- shutting down regardless
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
