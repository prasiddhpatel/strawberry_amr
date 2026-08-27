"""
Real, executable unit tests for command_safety.py -- pure functions, no
ROS dependency, runs under plain pytest. Auto-discovered by `colcon test`
via this package's standard ament_python test conventions (package.xml's
ament_flake8/ament_pep257/python3-pytest test_depend entries) -- no extra
CMake wiring needed, that's specific to ament_cmake packages, not this one.

Run directly: cd src/base_controller && python3 -m pytest test/ -v
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import pytest  # noqa: E402 -- must follow sys.path.insert above, which is
                # itself required before the base_controller import below

from base_controller.command_safety import clamp_command


class TestAbsoluteClamping:
    def test_within_limits_unchanged(self):
        vx, vz = clamp_command(1.0, 1.0, max_v=1.8, max_w=3.0,
                                prev_vx=1.0, dt=0.05, max_accel=None)
        assert vx == 1.0
        assert vz == 1.0

    def test_vx_clamped_to_max(self):
        vx, vz = clamp_command(5.0, 0.0, max_v=1.8, max_w=3.0,
                                prev_vx=1.8, dt=0.05, max_accel=None)
        assert vx == 1.8

    def test_vx_clamped_to_min(self):
        vx, vz = clamp_command(-5.0, 0.0, max_v=1.8, max_w=3.0,
                                prev_vx=-1.8, dt=0.05, max_accel=None)
        assert vx == -1.8

    def test_vz_clamped_independently_of_vx(self):
        vx, vz = clamp_command(0.5, 10.0, max_v=1.8, max_w=3.0,
                                prev_vx=0.5, dt=0.05, max_accel=None)
        assert vz == 3.0


class TestAccelerationLimiting:
    def test_sudden_forward_jump_is_limited(self):
        """Requesting 0 -> 1.8 m/s in one 20Hz tick (0.05s) with
        max_accel=2.0 m/s^2 should only allow 0.1 m/s of change."""
        vx, _ = clamp_command(1.8, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=0.0, dt=0.05, max_accel=2.0)
        assert vx == pytest.approx(0.1, abs=1e-9)

    def test_gradual_increase_within_limit_passes_through(self):
        vx, _ = clamp_command(0.05, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=0.0, dt=0.05, max_accel=2.0)
        assert vx == pytest.approx(0.05, abs=1e-9)

    def test_no_limiting_when_max_accel_none(self):
        vx, _ = clamp_command(1.8, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=0.0, dt=0.05, max_accel=None)
        assert vx == 1.8

    def test_no_limiting_when_dt_zero(self):
        vx, _ = clamp_command(1.8, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=0.0, dt=0.0, max_accel=2.0)
        assert vx == 1.8


class TestDecelerationNeverLimited:
    """The single most important property in this whole module -- see
    command_safety.py's own docstring. Braking must NEVER be delayed."""

    def test_deceleration_is_never_limited(self):
        """Going from full speed to zero in one tick must be immediate,
        even with a very strict max_accel."""
        vx, _ = clamp_command(0.0, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=1.8, dt=0.05, max_accel=0.01)
        assert vx == 0.0

    def test_estop_zero_is_immediate(self):
        """The actual e-stop scenario: robot moving at max speed, e-stop
        latches, twist_mux_pi_local outputs zero. This must reach the
        motors as zero on the very next cycle, not ramp down."""
        vx, vz = clamp_command(0.0, 0.0, max_v=1.8, max_w=3.0,
                                prev_vx=1.8, dt=0.05, max_accel=0.5)
        assert vx == 0.0
        assert vz == 0.0

    def test_sign_reversal_passes_through_zero_immediately(self):
        """Forward at max speed, then a hard reverse command -- this
        passes through zero, which counts as deceleration the whole way,
        not acceleration in the new direction."""
        vx, _ = clamp_command(-1.8, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=1.8, dt=0.05, max_accel=0.5)
        assert vx == -1.8

    def test_reducing_magnitude_same_direction_is_immediate(self):
        vx, _ = clamp_command(0.2, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=1.8, dt=0.05, max_accel=0.1)
        assert vx == pytest.approx(0.2, abs=1e-9)


class TestWatchdogResumeAfterStop:
    """Regression coverage for the base_controller_node.py watchdog fix:
    after a /cmd_vel dropout triggers _watchdog_check()'s direct
    set_car_motion(0,0,0), self.last_vx_out must be reset to 0.0 so the
    next real command correctly ramps from the true physical velocity (0)
    rather than from a stale pre-dropout value. This module
    (command_safety.py) doesn't hold that state itself -- the node does --
    so these tests exercise the CONTRACT clamp_command relies on: whatever
    calls it must pass the vx that was actually last commanded, not what
    was last requested before an out-of-band stop."""

    def test_resume_with_correctly_reset_prev_vx_ramps(self):
        # Watchdog fired and correctly reset last_vx_out to 0.0 (the fix).
        # Upstream resumes at 1.5 m/s after a 0.6s dropout.
        vx, _ = clamp_command(1.5, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=0.0, dt=0.6, max_accel=1.0)
        assert vx == pytest.approx(0.6, abs=1e-9)   # ramp-limited: 1.0 * 0.6

    def test_resume_with_stale_prev_vx_would_jump(self):
        # Documents the bug this fix closes: if last_vx_out were left at
        # its stale pre-dropout value (1.5) instead of being reset to 0.0,
        # the acceleration limiter's `increasing_magnitude = abs(vx) >
        # abs(prev_vx)` check is False (1.5 is not > 1.5), so the limiter
        # is skipped entirely and the full 1.5 m/s passes through unramped
        # -- an instant jump from the real 0 m/s the robot was actually at.
        vx, _ = clamp_command(1.5, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=1.5, dt=0.6, max_accel=1.0)
        assert vx == 1.5   # unramped -- this IS the bug, kept here as a
                            # concrete demonstration of exactly what the fix
                            # (resetting last_vx_out in _watchdog_check)
                            # prevents in the real node


class TestFailureCases:
    """Per the robotics-testing skill's own anti-pattern warning: "Not
    Testing Failure Cases" -- these exercise edge/boundary conditions,
    not just the happy path."""

    def test_zero_max_accel_disables_limiting_not_freezes_velocity(self):
        vx, _ = clamp_command(1.8, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=0.0, dt=0.05, max_accel=0.0)
        assert vx == 1.8

    def test_negative_prev_vx_accelerating_further_negative(self):
        vx, _ = clamp_command(-1.8, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=-0.1, dt=0.05, max_accel=2.0)
        assert vx == pytest.approx(-0.2, abs=1e-9)

    def test_negative_dt_treated_as_no_limiting(self):
        vx, _ = clamp_command(1.8, 0.0, max_v=1.8, max_w=3.0,
                               prev_vx=0.0, dt=-0.01, max_accel=2.0)
        assert vx == 1.8

