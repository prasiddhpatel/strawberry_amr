// Safety supervisor: hard obstacle gate between the row follower and the mux.
//
// NOT COMPILED IN THE AUTHORING ENVIRONMENT -- no ROS 2 Humble C++
// toolchain (no rclcpp/sensor_msgs/geometry_msgs headers) has been
// available at any point this file was written or edited. Verified by
// careful manual review and brace/paren-balance checking, PLUS (for the
// two fixes below) a hand-mirrored Python reference model of the exact
// branching/classification logic, with real executable tests --
// src/safety_supervisor/test/test_safety_supervisor_reference_model.py
// -- proving the FIXED decision logic behaves as intended, including
// directly reproducing the ORIGINAL bug in a parallel "pre-fix" model to
// confirm the fix actually changes the reported failure case. This is
// real verification of the decision logic, not a substitute for
// `colcon build --packages-select safety_supervisor` and real hardware
// testing (still required, see docs/CLAUDE_CODE_VERIFICATION_GUIDE.md)
// before trusting this against a real obstacle.
//
//   subscribes  /scan         (sensor_msgs/LaserScan, SensorDataQoS)
//   subscribes  /cmd_vel_auto  (geometry_msgs/Twist, from row_nav)
//   subscribes  /e_stop_reset  (std_msgs/Empty) -- see LATCHING below
//   publishes   /cmd_vel_safe  (geometry_msgs/Twist, to twist_mux 'navigation')
//   publishes   /e_stop        (std_msgs/Bool)
//
// LATCHING (added after an independent code review found /e_stop was
// documented as latched here and in SKILLS_REVIEW_FINDINGS.md, but tick()
// actually recomputed it from scratch every 50ms tick with zero
// persistence -- the instant a fault condition cleared, the very next tick
// silently resumed full autonomy with no operator acknowledgement
// required): a persistent estop_latched_ member is now set true the
// moment ANY fault condition below first triggers, and is only cleared by
// an explicit /e_stop_reset message -- never automatically, regardless of
// whether the triggering condition has since cleared. KNOWN RELATED GAP,
// not fixed by this change: joystick_node.py (teleop_ps2) also publishes
// to this same /e_stop topic, independently, with its own genuine latch.
// Two uncoordinated publishers on one topic means whichever publishes
// last determines the topic's apparent value to any subscriber (e.g.
// twist_mux_pi_local's lock) -- this node's own latch does not protect
// against that cross-node race; see the two nodes' own comments for the
// current mitigation (edge-triggered, not continuous, publishing) and
// docs/ORIN_PI_SPLIT_ARCHITECTURE.md for why a single merged authority
// was rejected (this node is Orin-side and must not become a
// network-dependent single point of failure for the Pi-local, physically
// link-independent joypad e-stop).
//
// Behaviour:
//   * scans a forward angular sector; if the nearest return is closer than
//     stop_distance -> FULL STOP (zero both linear AND angular) and latch
//     /e_stop while blocked. Between slow_distance and stop_distance ->
//     scale BOTH linear and angular by the same factor.
//   * COMMS WATCHDOG: if no /cmd_vel_auto within cmd_timeout -> output zero.
//   * SENSOR WATCHDOG (added during a robotics-agent-skills review pass --
//     robotics-software-principles skill: "sensor failure -> stop, not use
//     last reading"): if no /scan within scan_timeout -> treat exactly like
//     an obstacle inside stop_distance (full stop + latch /e_stop), not a
//     silent continuation on a frozen `nearest_` value. Before this fix,
//     nearest_ simply held whatever it was last set to forever if the
//     LiDAR driver crashed or the sensor disconnected -- this node would
//     have kept gating commands against a stale, possibly wildly wrong
//     obstacle picture with no indication anything had failed.
//   * SECOND SENSOR WATCHDOG (found by an independent adversarial review,
//     confirmed by re-reading this file directly): a /scan that arrives
//     ON SCHEDULE but with every return in the forward sector flagged
//     erroneous (NaN, per REP-117) previously left nearest_ at its
//     initial +infinity, which is indistinguishable from "confirmed
//     clear, nothing within sensor range" -- full speed allowed. This is
//     a real failure signature (e.g. lens glare/contamination) the
//     staleness check above cannot see, since messages keep arriving on
//     time; they're just empty of usable information. Fixed in scan_cb():
//     tracks NaN returns specifically, separately from +inf ("nothing in
//     range, sensor is fine" per REP-117) -- a genuinely open corridor
//     legitimately reports +inf and must NOT trigger this fault; only an
//     all-NaN (or zero-coverage) sector does. Deliberately does not
//     extend obstacle-based /e_stop into the teleop-only path below
//     (stale /cmd_vel_auto) -- that would be a separate, larger design
//     decision (should the LiDAR be able to lock out a human operator's
//     manual driving?) that these two sensor-integrity fixes should not
//     silently bundle in.
//   * CRITICAL FIX: both sensor watchdogs above now run BEFORE the
//     /cmd_vel_auto staleness check, not after. Previously they ran
//     after, so a stale /cmd_vel_auto caused an early return before
//     either sensor check ever executed -- meaning during ANY launch
//     configuration without row_navigation running (e.g.
//     teleop.launch.py, which starts this node but never row_navigation,
//     so /cmd_vel_auto has no publisher for the entire session), the
//     LiDAR obstacle gate and both sensor watchdogs were silently,
//     permanently inert, and /e_stop stayed hard-wired to false no
//     matter what the LiDAR actually saw. Confirmed via careful control-
//     flow review (this file still cannot be compiled in this authoring
//     environment) cross-referenced against teleop.launch.py's actual
//     node list.
//
// ACKERMANN NOTE (fixed vs. the original skid-steer-era version): the
// original taper scaled linear.x only and left angular.z untouched. For a
// skid-steer base that is a reasonable "slow down, keep turning the same
// way" behaviour. For THIS Ackermann chassis it is wrong on two counts:
// (1) downstream ackermann_bridge computes steering angle as
// atan(L*omega/v) -- holding omega fixed while v shrinks makes the
// REQUIRED steering angle grow without bound as the robot slows, right up
// until it saturates at the mechanical limit, silently distorting the
// intended path into a much tighter turn than row_navigation commanded;
// (2) at full stop (v=0), a nonzero omega is not an "allow rotation"
// fallback the way it is for skid-steer -- an Ackermann vehicle physically
// cannot rotate in place, and ackermann_bridge's v~0 floor check would
// force it to a full stop anyway, so leaving omega nonzero here just
// papers over a now-meaningless intent in a different node. Fix: scale
// BOTH fields by the same factor s. Since the geometric path curvature is
// kappa = omega/v, scaling both by s leaves kappa -- and so the resulting
// steering angle -- UNCHANGED; the robot decelerates smoothly along the
// SAME intended path rather than the path shape changing as it slows.
//
// Fix vs. original: builds (real CMake target), uses SensorDataQoS so the
// best-effort LiDAR actually reaches the callback, adds the watchdog.

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

using std::placeholders::_1;

class SafetySupervisor : public rclcpp::Node
{
public:
  SafetySupervisor() : Node("safety_supervisor_node")
  {
    stop_distance_  = declare_parameter("stop_distance", 0.30);
    slow_distance_  = declare_parameter("slow_distance", 0.70);
    if (slow_distance_ <= stop_distance_) {
      // Refuse to start rather than silently run with the graduated
      // slow-down zone collapsed to nothing. This does NOT produce the
      // NaN/divide-by-zero the naive formula (d - stop_distance_) /
      // (slow_distance_ - stop_distance_) suggests -- tick()'s
      // `if (d < stop_distance_) ... else if (d < slow_distance_)`
      // structure makes that branch mathematically unreachable whenever
      // slow_distance_ <= stop_distance_, so the division is simply never
      // evaluated. The real consequence is narrower but still a genuine
      // safety regression from a bad param override: every command either
      // full-stops (d < stop_distance_) or passes through completely
      // unscaled (d >= stop_distance_, since the graduated branch can
      // never be entered), losing the intended smooth deceleration ramp
      // entirely.
      throw std::runtime_error(
        "safety_supervisor: slow_distance (" + std::to_string(slow_distance_) +
        ") must be greater than stop_distance (" + std::to_string(stop_distance_) + ")");
    }
    sector_half_deg_ = declare_parameter("sector_half_deg", 35.0);
    cmd_timeout_    = declare_parameter("cmd_timeout", 0.5);
    scan_timeout_   = declare_parameter("scan_timeout", 1.0);  // RPLIDAR A1M8 runs
                                                                 // ~5.5-10Hz; 1.0s is
                                                                 // generous margin
                                                                 // before treating it
                                                                 // as genuinely gone,
                                                                 // not just a slow tick

    auto sensor_qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos, std::bind(&SafetySupervisor::scan_cb, this, _1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_auto", 10, std::bind(&SafetySupervisor::cmd_cb, this, _1));
    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/e_stop_reset", 10, std::bind(&SafetySupervisor::reset_cb, this, _1));

    safe_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
    // TRANSIENT_LOCAL (latched) rather than the previous default VOLATILE
    // QoS: /e_stop is a LOCK signal for twist_mux_pi_local (timeout: 0.0 --
    // it holds the last received value indefinitely, it does not require
    // continuous republishing), and this node now only publishes on this
    // topic when its own decision actually CHANGES (see
    // publish_estop_if_changed() below), not every 50ms tick. Without
    // TRANSIENT_LOCAL, a subscriber that starts after the last publish
    // would see nothing until the next change. This is purely additive --
    // a VOLATILE-QoS subscriber remains fully compatible with a
    // TRANSIENT_LOCAL publisher. See the module header LATCHING section
    // for why this node also stopped continuously republishing false:
    // joystick_node.py publishes to this same topic independently, and
    // this node re-asserting false every tick was clobbering the
    // operator's own latch within one 50ms cycle regardless of this fix.
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/e_stop", rclcpp::QoS(1).transient_local().reliable());

    // Establish an explicit initial value on the TRANSIENT_LOCAL topic
    // rather than relying on a late subscriber's (e.g. twist_mux_pi_local
    // restarting) undocumented default behaviour for a lock topic it has
    // never received a message on.
    publish_estop_if_changed(false);

    last_cmd_time_ = now();
    // last_scan_time_ deliberately NOT initialised to now() -- see tick():
    // has_scan_ starts false, so we correctly treat "never received a scan
    // yet" as a fault condition too, not as "just received one at startup".
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SafetySupervisor::tick, this));

    RCLCPP_INFO(get_logger(), "safety_supervisor_node ready (obstacle gate + watchdog).");
  }

  // Graceful shutdown -- see module header comment; found missing during a
  // robotics-agent-skills review pass (robot-bringup skill). Best-effort
  // only: cannot run if this process is killed with SIGKILL.
  ~SafetySupervisor()
  {
    geometry_msgs::msg::Twist zero;
    safe_pub_->publish(zero);
  }

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double half = sector_half_deg_ * M_PI / 180.0;
    double nearest = std::numeric_limits<double>::infinity();
    size_t in_sector = 0;
    size_t erroneous = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double ang = msg->angle_min + i * msg->angle_increment;
      if (ang < -half || ang > half) continue;          // forward sector only
      ++in_sector;
      const double r = msg->ranges[i];
      if (std::isnan(r)) { ++erroneous; continue; }      // REP-117: NaN = erroneous
                                                           // detection -- deliberately
                                                           // tracked separately from
                                                           // +inf below, which means
                                                           // "nothing in range, sensor
                                                           // is fine" and must NOT
                                                           // count as a fault
      if (!std::isfinite(r) || r < msg->range_min) continue;  // +inf/out-of-range:
                                                                // legitimately clear
      nearest = std::min(nearest, static_cast<double>(r));
    }
    // Sensor fault: either this message has no coverage at all in the
    // configured sector (angle_min/angle_increment don't span
    // sector_half_deg_), or every single in-sector return was flagged
    // erroneous. See the module header for why this is NOT the same
    // condition as "sector legitimately clear" (which reports +inf, not
    // NaN, and is handled correctly above -- nearest_ ending up at
    // +infinity because nothing is closer than sensor range is a valid,
    // intentional reading, not a fault).
    sector_fault_ = (in_sector == 0) || (erroneous == in_sector);
    nearest_ = nearest;
    last_scan_time_ = now();
    has_scan_ = true;
  }

  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = now();
  }

  void reset_cb(const std_msgs::msg::Empty::SharedPtr)
  {
    if (estop_latched_) {
      RCLCPP_INFO(get_logger(), "/e_stop_reset received -- clearing latch.");
    }
    estop_latched_ = false;
  }

  // Publishes /e_stop only when this node's own decision actually changes,
  // not on every 50ms tick -- see the constructor's QoS comment and the
  // module header LATCHING section for why: continuous republishing was
  // clobbering joystick_node.py's independent latch on this same topic
  // within one tick, regardless of the C1 latch fix above.
  void publish_estop_if_changed(bool value)
  {
    if (!has_published_estop_ || value != last_published_estop_) {
      std_msgs::msg::Bool msg;
      msg.data = value;
      estop_pub_->publish(msg);
      last_published_estop_ = value;
      has_published_estop_ = true;
    }
  }

  void tick()
  {
    // SENSOR WATCHDOGS -- see module header comment. Evaluated FIRST,
    // before the /cmd_vel_auto staleness check below: this node's
    // obstacle-safety function must hold regardless of whether autonomous
    // driving happens to be active right now, not only while row_navigation
    // is publishing.
    const bool scan_stale = !has_scan_ ||
      (now() - last_scan_time_).seconds() > scan_timeout_;

    // Per-condition transition logging is independent of the latch below --
    // "the underlying condition cleared" is worth logging even while the
    // latch itself is still waiting on an explicit /e_stop_reset.
    if (scan_stale) {
      if (!scan_was_stale_) {
        RCLCPP_ERROR(get_logger(),
          "/scan stale (>%.1fs) or never received -- treating as obstacle, "
          "latching e-stop until /e_stop_reset.", scan_timeout_);
        scan_was_stale_ = true;
      }
    } else if (scan_was_stale_) {
      RCLCPP_INFO(get_logger(), "/scan recovered.");
      scan_was_stale_ = false;
    }

    if (sector_fault_) {
      if (!sector_was_faulted_) {
        RCLCPP_ERROR(get_logger(),
          "/scan arriving on schedule but the entire forward sector is "
          "erroneous (NaN) or has no coverage -- treating as obstacle, "
          "latching e-stop until /e_stop_reset.");
        sector_was_faulted_ = true;
      }
    } else if (sector_was_faulted_) {
      RCLCPP_INFO(get_logger(), "Forward sector returns recovered.");
      sector_was_faulted_ = false;
    }

    // Obstacle-distance fault only means anything once the sensor itself is
    // known-good this tick -- mirrors the original control-flow ordering,
    // where the distance gate was only ever reached after both sensor
    // checks passed.
    const bool obstacle_now = !scan_stale && !sector_fault_ && (nearest_ < stop_distance_);
    const bool fault_now = scan_stale || sector_fault_ || obstacle_now;

    if (fault_now && !estop_latched_) {
      // Rising edge only -- this fires once per fault episode, not every
      // tick while already latched, since estop_latched_ is checked here
      // before being set.
      estop_latched_ = true;
      RCLCPP_ERROR(get_logger(),
        "/e_stop LATCHED (%s) -- will remain stopped until an explicit "
        "/e_stop_reset message is received, regardless of whether the "
        "triggering condition clears. This is the fix for the previously "
        "undocumented behaviour where the very next good tick silently "
        "resumed motion with no operator acknowledgement.",
        scan_stale ? "scan stale" : sector_fault_ ? "sector fault" : "obstacle within stop_distance");
    }

    if (estop_latched_) {
      geometry_msgs::msg::Twist out;   // zero-initialised -> stop
      safe_pub_->publish(out);
      publish_estop_if_changed(true);
      return;
    }

    // Not latched: cmd-staleness check and graduated obstacle gate, exactly
    // as before. d >= stop_distance_ is now guaranteed here -- if it had
    // ever been < stop_distance_ this tick, fault_now/obstacle_now above
    // would already have latched and returned.
    const double age = (now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_) {
      geometry_msgs::msg::Twist out;   // stale command -> stop (estop stays
                                        // false -- deliberately not extended
                                        // into this teleop-only path, see
                                        // module header)
      safe_pub_->publish(out);
      publish_estop_if_changed(false);
      return;
    }

    geometry_msgs::msg::Twist out = last_cmd_;
    const double d = nearest_;
    if (d < slow_distance_) {
      const double s = std::clamp(
        (d - stop_distance_) / (slow_distance_ - stop_distance_), 0.0, 1.0);
      out.linear.x *= s;
      out.angular.z *= s;                      // scale together -> curvature (and so
                                                 // the resulting steering angle) is preserved
    }
    safe_pub_->publish(out);
    publish_estop_if_changed(false);
  }

  double stop_distance_, slow_distance_, sector_half_deg_, cmd_timeout_, scan_timeout_;
  double nearest_ = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_scan_time_;
  bool has_scan_ = false;
  bool estop_latched_ = false;
  bool scan_was_stale_ = false;
  bool sector_fault_ = false;
  bool sector_was_faulted_ = false;
  bool last_published_estop_ = false;
  bool has_published_estop_ = false;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}
