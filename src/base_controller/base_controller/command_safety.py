#!/usr/bin/env python3
"""
Pure, ROS-independent command safety math for base_controller_node.py --
deliberately extracted into its own module with zero rclpy dependency so
it can be unit-tested directly (see test/test_command_safety.py), matching
the robotics-testing skill's explicit "test pure functions separately from
ROS" pattern. base_controller_node.py imports and calls clamp_command();
nothing about the node's own interface, topics, or behaviour changes by
having this in its own file -- it's the same logic, just made independently
testable, per Principle 1 (Single Responsibility) from the
robotics-software-principles skill.

Added during a robotics-agent-skills review pass: acceleration limiting.
Velocity clamping already existed; rate-of-change (acceleration) limiting
did not, and the robotics-security skill flags this specifically
("Command Velocity Validation and Rate Limiting... enforce at the driver
level -- last line of defense before actuators").

CRITICAL SAFETY PROPERTY, load-bearing: this limits the rate of INCREASING
|velocity| only. Decreasing |velocity| (braking, including all the way to
zero, including a sign reversal that passes through zero) is NEVER rate-
limited -- it always takes effect immediately, in one step. This is
standard vehicle-safety practice for a reason: an e-stop or any other
"slow down now" command must never be smoothed or delayed. Getting this
backwards (limiting deceleration too) would mean the acceleration limiter
itself could delay an emergency stop -- exactly the kind of safety feature
that becomes a hazard if implemented carelessly. See
test_command_safety.py's test_deceleration_is_never_limited and
test_estop_zero_is_immediate for the tests that pin this property down.
"""


def clamp_command(vx, vz, max_v, max_w, prev_vx, dt, max_accel=None):
    """
    Clamp a (vx, vz) command to absolute limits, then apply acceleration
    limiting to vx only (see module docstring for why vz/angular is not
    rate-limited here: it's already bounded by the firmware's own vz range,
    and unlike linear speed, a sudden angular change doesn't carry the same
    "sudden jackrabbit start" risk this is specifically guarding against).

    Args:
        vx, vz: requested velocities (m/s, rad/s) -- already whatever the
            upstream source (teleop, autonomous, e-stop-zero) decided.
        max_v, max_w: absolute limits (m/s, rad/s).
        prev_vx: the vx actually output last cycle (not the requested one
            -- chain this function's own previous output for correct
            accumulation across cycles).
        dt: seconds since the last call (must be > 0 for acceleration
            limiting to apply; if <= 0 or max_accel is None/0, only
            absolute clamping is applied).
        max_accel: m/s^2, or None to disable acceleration limiting
            entirely (absolute clamping still always applies).

    Returns:
        (vx_out, vz_out) -- vz_out is always just the absolute-clamped vz;
        only vx_out is ever acceleration-limited.
    """
    vx = max(-max_v, min(max_v, vx))
    vz = max(-max_w, min(max_w, vz))

    if dt is not None and dt > 0 and max_accel:
        same_direction = (prev_vx == 0.0) or ((vx > 0) == (prev_vx > 0))
        increasing_magnitude = abs(vx) > abs(prev_vx)
        if same_direction and increasing_magnitude:
            max_delta = max_accel * dt
            if vx > prev_vx:
                vx = min(vx, prev_vx + max_delta)
            else:
                vx = max(vx, prev_vx - max_delta)
        # else: braking, holding, or a sign reversal (which passes through
        # zero) -- always applied immediately, unmodified, per the module
        # docstring's critical safety property.

    return vx, vz
