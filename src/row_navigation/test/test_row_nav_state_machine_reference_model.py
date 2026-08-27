"""
Reference-model verification for two row_nav_node.py fixes:
  1. scan_cb()'s top-level mode dispatch order -- EXPLORE_HALT and the new
     ODOM_LOST_HALT are now checked BEFORE the autonomy/mission gate.
  2. _do_headland()'s odom-staleness halt, added so ARC1/ARC2 cannot loop
     forever on a frozen self.yaw if /odometry/filtered stops arriving.

WHY THIS EXISTS: no ROS 2 Humble toolchain has been available in this
authoring environment at any point -- row_nav_node.py cannot be imported or
run here (it subclasses rclpy.node.Node). This mirrors the exact
control-flow this fix changes as small, dependency-free models and tests
THAT -- the same pattern already used elsewhere in this project for
safety_supervisor_node.cpp's tick() and the Ackermann bulb-turn arc math.

Run: python3 -m pytest test/test_row_nav_state_machine_reference_model.py -v
"""


class DispatchModel:
    """Mirrors the FIXED scan_cb()'s top-of-function mode dispatch order."""

    def __init__(self):
        self.mode = 'IDLE'
        self.autonomy = True
        self.mission = True

    def scan_tick(self):
        """Returns 'stop_halted' (EXPLORE_HALT/ODOM_LOST_HALT branch),
        'stop_gated' (autonomy/mission gate branch), or 'proceed' (would
        fall through to normal FOLLOW/headland dispatch)."""
        if self.mode in ('EXPLORE_HALT', 'ODOM_LOST_HALT'):
            return 'stop_halted'
        if not (self.autonomy and self.mission):
            self.mode = 'IDLE'
            return 'stop_gated'
        return 'proceed'

    def auton_off(self):
        """Mirrors auton_cb's unconditional mode=IDLE on autonomy->False,
        independent of scan_tick's ordering -- the legitimate, documented
        way out of either halt state."""
        self.autonomy = False
        self.mode = 'IDLE'


class DispatchModel_PRE_FIX:
    """Mirrors the ORIGINAL ordering (the autonomy/mission gate checked
    before the halt states) -- kept only to demonstrate the fix actually
    changes behaviour for the reported scenario."""

    def __init__(self):
        self.mode = 'IDLE'
        self.autonomy = True
        self.mission = True

    def scan_tick(self):
        if not (self.autonomy and self.mission):
            self.mode = 'IDLE'
            return 'stop_gated'
        if self.mode in ('EXPLORE_HALT', 'ODOM_LOST_HALT'):
            return 'stop_halted'
        return 'proceed'


class TestHaltStateStickiness:
    def test_prefix_model_explore_halt_downgraded_to_idle_by_mission_flip(self):
        """The exact reported scenario: mission_control_node.py's
        row_found_cb reacts to EXPLORE_HALT's own /row_found=False by
        calling _set_mission_active(False) -- the normal, expected sequence
        every single time this state is entered, not a rare edge case."""
        m = DispatchModel_PRE_FIX()
        m.mode = 'EXPLORE_HALT'
        m.mission = False
        result = m.scan_tick()
        assert result == 'stop_gated'
        assert m.mode == 'IDLE'   # THE BUG: silently downgraded, losing the
                                    # documented "stays stopped" guarantee

    def test_fixed_model_explore_halt_survives_mission_flip(self):
        m = DispatchModel()
        m.mode = 'EXPLORE_HALT'
        m.mission = False
        result = m.scan_tick()
        assert result == 'stop_halted'
        assert m.mode == 'EXPLORE_HALT'   # still latched

    def test_fixed_model_explore_halt_survives_many_ticks(self):
        m = DispatchModel()
        m.mode = 'EXPLORE_HALT'
        m.mission = False
        for _ in range(50):
            assert m.scan_tick() == 'stop_halted'
        assert m.mode == 'EXPLORE_HALT'

    def test_fixed_model_explicit_autonomy_cycle_still_clears_it(self):
        """The legitimate, documented way out must still work."""
        m = DispatchModel()
        m.mode = 'EXPLORE_HALT'
        m.mission = False
        m.auton_off()
        assert m.mode == 'IDLE'
        m.autonomy = True
        m.mission = True
        assert m.scan_tick() == 'proceed'

    def test_odom_lost_halt_has_identical_stickiness(self):
        m = DispatchModel()
        m.mode = 'ODOM_LOST_HALT'
        m.mission = False
        assert m.scan_tick() == 'stop_halted'
        assert m.mode == 'ODOM_LOST_HALT'

    def test_normal_modes_still_get_the_mission_gate_reset(self):
        """The mission-active gate's mode=IDLE side effect is itself a
        legitimate safety reset for the non-halt states (e.g. resuming
        mid-ARC2 with a stale yaw_at_phase_start would be wrong) -- must be
        preserved for everything except the two halt states."""
        for mode in ('FOLLOW', 'EXIT_BUFFER', 'ARC1', 'ARC2', 'REACQUIRE'):
            m = DispatchModel()
            m.mode = mode
            m.mission = False
            result = m.scan_tick()
            assert result == 'stop_gated'
            assert m.mode == 'IDLE'


class OdomStalenessModel:
    """Mirrors _do_headland()'s staleness condition exactly:
    `self.last_odom_time is None or (now - self.last_odom_time) > self.odom_timeout`."""

    @staticmethod
    def is_stale(now, last_odom_time, odom_timeout):
        return last_odom_time is None or (now - last_odom_time) > odom_timeout


class TestOdomStalenessDetection:
    def test_fresh_odom_not_stale(self):
        assert not OdomStalenessModel.is_stale(now=10.0, last_odom_time=9.95, odom_timeout=1.0)

    def test_odom_never_received_is_stale(self):
        # self.yaw is None is checked separately and returns earlier in the
        # real function, but last_odom_time is also None in that state --
        # this is the belt-and-braces case where yaw was somehow set (should
        # not happen given odom_cb sets both together) without a timestamp.
        assert OdomStalenessModel.is_stale(now=10.0, last_odom_time=None, odom_timeout=1.0)

    def test_odom_older_than_timeout_is_stale(self):
        assert OdomStalenessModel.is_stale(now=10.0, last_odom_time=8.5, odom_timeout=1.0)

    def test_odom_exactly_at_timeout_boundary_not_yet_stale(self):
        # strictly greater-than in the real check
        assert not OdomStalenessModel.is_stale(now=10.0, last_odom_time=9.0, odom_timeout=1.0)

    def test_odom_just_past_timeout_boundary_is_stale(self):
        assert OdomStalenessModel.is_stale(now=10.0, last_odom_time=8.999, odom_timeout=1.0)


class TestArcStuckForeverScenario:
    """End-to-end reproduction of the reported failure: ARC1's
    phase-advance condition depends entirely on a yaw estimate that freezes
    the instant odometry goes stale."""

    @staticmethod
    def arc_would_advance(yaw, yaw_at_phase_start, turn_sign, phi1, turn_tol):
        import math
        target = math.atan2(math.sin(yaw_at_phase_start + turn_sign * phi1),
                            math.cos(yaw_at_phase_start + turn_sign * phi1))
        err = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
        return abs(err) < turn_tol

    def test_frozen_yaw_never_satisfies_phase_advance_without_the_fix(self):
        """Without the staleness halt, a frozen self.yaw mid-ARC1 (large
        remaining error at the instant odom died) would keep this False
        forever -- demonstrating the "loop indefinitely" claim numerically,
        not just asserting it."""
        import math
        yaw_at_phase_start = 0.0
        phi1 = math.radians(90.0)
        turn_sign = 1.0
        turn_tol = 0.05
        frozen_yaw = 0.10   # odom died early in the arc, far from target
        for _ in range(1000):   # simulate many ticks -- nothing changes self.yaw
            advanced = self.arc_would_advance(
                frozen_yaw, yaw_at_phase_start, turn_sign, phi1, turn_tol)
            assert advanced is False
        # With the fix, OdomStalenessModel.is_stale would have already
        # halted this well before 1000 ticks at any real odom_timeout/tick
        # rate -- this test documents the failure mode the halt prevents.
