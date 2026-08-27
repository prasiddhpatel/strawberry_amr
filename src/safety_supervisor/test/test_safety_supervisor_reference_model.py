"""
Reference-model verification for the safety_supervisor_node.cpp fixes
(tick() branching order + scan_cb() fault classification).

WHY THIS EXISTS: no ROS 2 Humble C++ toolchain has been available in the
environment these edits were made in, at any point -- see the file's own
header comment. This is not a substitute for `colcon build
--packages-select safety_supervisor` and real hardware testing (still
required, see docs/CLAUDE_CODE_VERIFICATION_GUIDE.md), but it IS a real,
executable proof that the *decision logic* the C++ was hand-edited to
implement has the intended observable behaviour -- by mirroring the exact
branching structure as a small Python reference model and testing THAT,
the same pattern used earlier in this project to verify the Ackermann
bulb-turn state machine's (v, omega) sequence without a ROS2 runtime.

Run: python3 -m pytest test/test_safety_supervisor_reference_model.py -v
"""
import math


def classify_sector(ranges, angle_min, angle_increment, sector_half_rad,
                    range_min):
    """
    Direct mirror of the fixed scan_cb()'s classification logic.
    Returns (nearest, sector_fault) exactly as the C++ now computes them.
    """
    nearest = math.inf
    in_sector = 0
    erroneous = 0
    for i, r in enumerate(ranges):
        ang = angle_min + i * angle_increment
        if ang < -sector_half_rad or ang > sector_half_rad:
            continue
        in_sector += 1
        if math.isnan(r):
            erroneous += 1
            continue
        if not math.isfinite(r) or r < range_min:
            continue
        nearest = min(nearest, r)
    sector_fault = (in_sector == 0) or (erroneous == in_sector)
    return nearest, sector_fault


def tick_decision(scan_stale, sector_fault, cmd_stale, nearest,
                  stop_distance, slow_distance):
    """
    Direct mirror of the FIXED tick()'s branching order: both sensor
    watchdogs (scan_stale, sector_fault) are checked BEFORE cmd_stale --
    this is precisely the property the bug fix establishes. Returns
    (estop: bool, scale: float) -- scale of 0.0 means full stop,
    1.0 means unscaled.
    """
    if scan_stale:
        return True, 0.0
    if sector_fault:
        return True, 0.0
    if cmd_stale:
        return False, 0.0   # matches original, unchanged behaviour:
                             # zero output, but NOT an obstacle e-stop --
                             # deliberately not extending obstacle-based
                             # e-stop into the teleop-only path, see the
                             # file's own header comment for why
    if nearest < stop_distance:
        return True, 0.0
    if nearest < slow_distance:
        s = max(0.0, min(1.0, (nearest - stop_distance) / (slow_distance - stop_distance)))
        return False, s
    return False, 1.0


def tick_decision_PRE_FIX(scan_stale, sector_fault, cmd_stale, nearest,
                          stop_distance, slow_distance):
    """
    Mirror of the ORIGINAL, buggy branching order (cmd_stale checked
    FIRST) -- kept only so the tests below can demonstrate the fix
    actually changes behaviour for the reported scenario, not just that
    the new function exists.
    """
    if cmd_stale:
        return False, 0.0   # THE BUG: returns here, sensor checks below
                             # never run, e-stop hard-wired false
    if scan_stale:
        return True, 0.0
    if sector_fault:
        return True, 0.0
    if nearest < stop_distance:
        return True, 0.0
    if nearest < slow_distance:
        s = max(0.0, min(1.0, (nearest - stop_distance) / (slow_distance - stop_distance)))
        return False, s
    return False, 1.0


class TickModel:
    """
    Stateful mirror of the FIXED tick()/reset_cb(), including the
    estop_latched_ persistence added to close the finding that /e_stop was
    documented as latched but was actually recomputed from scratch, with
    zero persistence, every 50ms tick. Call .tick(...) once per simulated
    cycle; call .reset() to simulate an /e_stop_reset message.
    """
    def __init__(self):
        self.estop_latched = False

    def tick(self, scan_stale, sector_fault, cmd_stale, nearest,
             stop_distance, slow_distance):
        obstacle_now = (not scan_stale) and (not sector_fault) and (nearest < stop_distance)
        fault_now = scan_stale or sector_fault or obstacle_now
        if fault_now:
            self.estop_latched = True
        if self.estop_latched:
            return True, 0.0
        if cmd_stale:
            return False, 0.0
        if nearest < slow_distance:
            s = max(0.0, min(1.0, (nearest - stop_distance) / (slow_distance - stop_distance)))
            return False, s
        return False, 1.0

    def reset(self):
        self.estop_latched = False


class TickModel_PRE_LATCH_FIX:
    """Mirror of the original, stateless tick_decision() function above --
    kept only to demonstrate the latch fix actually changes behaviour for
    the reported scenario: without it, the instant a fault condition
    clears, the very next tick silently resumes full autonomy with zero
    operator acknowledgement."""

    def tick(self, scan_stale, sector_fault, cmd_stale, nearest,
             stop_distance, slow_distance):
        return tick_decision(scan_stale, sector_fault, cmd_stale, nearest,
                             stop_distance, slow_distance)

    def reset(self):
        pass  # no-op: there was never any persistent state to reset


class TestEstopLatching:
    """Verifies the C1 fix: /e_stop must persist across ticks once
    triggered, and only clear on an explicit /e_stop_reset -- not
    automatically the instant the triggering condition clears."""

    def test_prefix_model_auto_resumes_the_instant_condition_clears(self):
        m = TickModel_PRE_LATCH_FIX()
        estop, _ = m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
                          nearest=0.10, stop_distance=0.3, slow_distance=0.7)
        assert estop is True   # obstacle triggers estop this tick
        # obstacle clears on the very next tick, no reset sent
        estop, scale = m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
                              nearest=5.0, stop_distance=0.3, slow_distance=0.7)
        assert estop is False   # THE BUG: instantly resumed, no ack required
        assert scale == 1.0

    def test_fixed_model_stays_latched_after_condition_clears(self):
        m = TickModel()
        estop, _ = m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
                          nearest=0.10, stop_distance=0.3, slow_distance=0.7)
        assert estop is True
        estop, scale = m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
                              nearest=5.0, stop_distance=0.3, slow_distance=0.7)
        assert estop is True    # still latched -- the obstacle clearing
        assert scale == 0.0     # alone is not enough to resume

    def test_explicit_reset_required_to_resume(self):
        m = TickModel()
        m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
              nearest=0.10, stop_distance=0.3, slow_distance=0.7)
        m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
              nearest=5.0, stop_distance=0.3, slow_distance=0.7)   # still latched
        m.reset()
        estop, scale = m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
                              nearest=5.0, stop_distance=0.3, slow_distance=0.7)
        assert estop is False
        assert scale == 1.0

    def test_scan_stale_latches_and_persists_past_recovery(self):
        m = TickModel()
        m.tick(scan_stale=True, sector_fault=False, cmd_stale=False,
              nearest=math.inf, stop_distance=0.3, slow_distance=0.7)
        estop, scale = m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
                              nearest=5.0, stop_distance=0.3, slow_distance=0.7)
        assert estop is True   # scan recovered, but still latched
        assert scale == 0.0

    def test_reset_while_fault_still_active_does_not_clear_latch(self):
        """A reset received while the fault condition is STILL live must
        not resume motion -- the same tick's fault_now check re-latches
        immediately. Mirrors the real tick()'s ordering: fault detection
        happens before the latch is consulted, every tick, so a reset can
        never "outrun" a still-active fault."""
        m = TickModel()
        m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
              nearest=0.10, stop_distance=0.3, slow_distance=0.7)   # latch on
        m.reset()   # operator resets while the obstacle is still there
        estop, scale = m.tick(scan_stale=False, sector_fault=False, cmd_stale=False,
                              nearest=0.10, stop_distance=0.3, slow_distance=0.7)
        assert estop is True   # re-latches immediately -- reset didn't help
        assert scale == 0.0


class TestSectorClassification:
    def test_all_valid_finite_returns_no_fault(self):
        nearest, fault = classify_sector(
            [1.5, 1.2, 0.9, 1.1], 0.0, 0.1, 0.3, 0.1)
        assert not fault
        assert nearest == 0.9

    def test_legitimately_clear_corridor_all_inf_is_NOT_a_fault(self):
        """The critical property distinguishing this fix from a naive
        'zero valid points = fault' check: a genuinely open corridor,
        where every in-sector point reports +inf (nothing within sensor
        range, per REP-117), must NOT be flagged as a sensor fault."""
        nearest, fault = classify_sector(
            [math.inf, math.inf, math.inf], 0.0, 0.1, 0.3, 0.1)
        assert not fault
        assert nearest == math.inf

    def test_all_nan_sector_IS_a_fault(self):
        nearest, fault = classify_sector(
            [math.nan, math.nan, math.nan], 0.0, 0.1, 0.3, 0.1)
        assert fault

    def test_zero_points_in_configured_sector_IS_a_fault(self):
        # an empty ranges list unambiguously means zero coverage --
        # a malformed/empty LaserScan message, a plausible real fault
        nearest, fault = classify_sector(
            [], 0.0, 0.1, 0.3, 0.1)
        assert fault

    def test_angle_min_entirely_outside_sector_IS_a_fault(self):
        # angle_min itself already outside the configured sector, so
        # every point in the message lands outside it too
        nearest, fault = classify_sector(
            [1.0, 1.0, 1.0], 1.0, 0.01, 0.05, 0.1)
        assert fault

    def test_mixed_nan_and_valid_is_NOT_a_fault(self):
        """Some erroneous points alongside real ones -- normal sensor
        noise, must not trip the fault the way an ALL-erroneous sector
        does."""
        nearest, fault = classify_sector(
            [math.nan, 0.8, math.nan, 1.2], 0.0, 0.1, 0.3, 0.1)
        assert not fault
        assert nearest == 0.8

    def test_below_range_min_excluded_like_original_behaviour(self):
        nearest, fault = classify_sector(
            [0.05, 1.0], 0.0, 0.1, 0.3, 0.1)   # first point below range_min
        assert not fault
        assert nearest == 1.0


class TestTickBranchingOrder:
    """Proves the fix's core property: both sensor watchdogs now take
    precedence over cmd staleness, where the original code had it
    backwards."""

    def test_reported_bug_scenario_now_fixed(self):
        """The exact scenario from the confirmed finding: teleop-only
        session (cmd_stale=True, since row_navigation never runs), scan
        genuinely stale/missing. Pre-fix: estop stays False forever.
        Post-fix: estop correctly becomes True."""
        pre = tick_decision_PRE_FIX(
            scan_stale=True, sector_fault=False, cmd_stale=True,
            nearest=math.inf, stop_distance=0.3, slow_distance=0.7)
        post = tick_decision(
            scan_stale=True, sector_fault=False, cmd_stale=True,
            nearest=math.inf, stop_distance=0.3, slow_distance=0.7)
        assert pre == (False, 0.0)     # demonstrates the bug existed
        assert post == (True, 0.0)     # demonstrates it's fixed

    def test_sector_fault_during_teleop_now_correctly_estops(self):
        pre = tick_decision_PRE_FIX(
            scan_stale=False, sector_fault=True, cmd_stale=True,
            nearest=math.inf, stop_distance=0.3, slow_distance=0.7)
        post = tick_decision(
            scan_stale=False, sector_fault=True, cmd_stale=True,
            nearest=math.inf, stop_distance=0.3, slow_distance=0.7)
        assert pre == (False, 0.0)
        assert post == (True, 0.0)

    def test_healthy_sensors_cmd_stale_unchanged_from_original(self):
        """When sensors are genuinely fine and only the command is
        stale, behaviour must be UNCHANGED from the original (zero
        output, no obstacle e-stop) -- the fix must not introduce new
        behaviour beyond what was actually reported."""
        pre = tick_decision_PRE_FIX(
            scan_stale=False, sector_fault=False, cmd_stale=True,
            nearest=math.inf, stop_distance=0.3, slow_distance=0.7)
        post = tick_decision(
            scan_stale=False, sector_fault=False, cmd_stale=True,
            nearest=math.inf, stop_distance=0.3, slow_distance=0.7)
        assert pre == post == (False, 0.0)

    def test_normal_autonomous_obstacle_gating_unaffected(self):
        """Everything healthy, cmd fresh, obstacle inside stop_distance
        -- must behave identically pre/post fix, since this path wasn't
        part of the bug."""
        pre = tick_decision_PRE_FIX(
            scan_stale=False, sector_fault=False, cmd_stale=False,
            nearest=0.15, stop_distance=0.3, slow_distance=0.7)
        post = tick_decision(
            scan_stale=False, sector_fault=False, cmd_stale=False,
            nearest=0.15, stop_distance=0.3, slow_distance=0.7)
        assert pre == post == (True, 0.0)

    def test_normal_autonomous_slowdown_unaffected(self):
        pre = tick_decision_PRE_FIX(
            scan_stale=False, sector_fault=False, cmd_stale=False,
            nearest=0.5, stop_distance=0.3, slow_distance=0.7)
        post = tick_decision(
            scan_stale=False, sector_fault=False, cmd_stale=False,
            nearest=0.5, stop_distance=0.3, slow_distance=0.7)
        assert pre == post
        estop, scale = post
        assert not estop
        assert 0.0 < scale < 1.0

    def test_scan_stale_always_wins_regardless_of_everything_else(self):
        estop, scale = tick_decision(
            scan_stale=True, sector_fault=True, cmd_stale=False,
            nearest=5.0, stop_distance=0.3, slow_distance=0.7)
        assert estop is True and scale == 0.0
