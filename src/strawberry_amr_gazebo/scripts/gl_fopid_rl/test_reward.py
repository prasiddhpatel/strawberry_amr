"""
Real, executable unit tests for reward.py -- pure functions, no ROS/
Gazebo dependency, runs under plain pytest.

Run directly: cd src/strawberry_amr_gazebo/scripts/gl_fopid_rl && python3 -m pytest test_reward.py -v
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
import pytest  # noqa: E402

from reward import (
    action_to_params, params_to_action, build_observation, compute_reward,
    PARAM_NAMES, PARAM_MIN, PARAM_MAX, ACTION_DIM, OBS_DIM,
)


class TestActionParamMapping:
    def test_action_zero_maps_to_range_midpoint(self):
        params = action_to_params(np.zeros(ACTION_DIM))
        for i, name in enumerate(PARAM_NAMES):
            expected_mid = (PARAM_MIN[i] + PARAM_MAX[i]) / 2.0
            assert params[name] == pytest.approx(expected_mid, abs=1e-4)

    def test_action_minus_one_maps_to_param_min(self):
        params = action_to_params(-np.ones(ACTION_DIM))
        for i, name in enumerate(PARAM_NAMES):
            assert params[name] == pytest.approx(PARAM_MIN[i], abs=1e-4)

    def test_action_plus_one_maps_to_param_max(self):
        params = action_to_params(np.ones(ACTION_DIM))
        for i, name in enumerate(PARAM_NAMES):
            assert params[name] == pytest.approx(PARAM_MAX[i], abs=1e-4)

    def test_out_of_range_action_is_clipped_not_extrapolated(self):
        """An RL algorithm SHOULD respect the declared action space, but
        this must not silently produce an out-of-bounds parameter value
        (e.g. a negative Kp) if it ever doesn't -- this gets applied to a
        real robot's control loop eventually, defensive clipping matters."""
        params_over = action_to_params(np.full(ACTION_DIM, 5.0))
        params_under = action_to_params(np.full(ACTION_DIM, -5.0))
        for i, name in enumerate(PARAM_NAMES):
            assert params_over[name] == pytest.approx(PARAM_MAX[i], abs=1e-4)
            assert params_under[name] == pytest.approx(PARAM_MIN[i], abs=1e-4)

    def test_round_trip_is_exact_for_in_range_values(self):
        original = {'Kp_theta': 1.2, 'Ki_theta': 0.25, 'Kd_theta': 0.30,
                    'fopid_lambda': 0.4, 'fopid_mu': 0.7, 'k_cross_track': 0.8}
        action = params_to_action(original)
        recovered = action_to_params(action)
        for name in PARAM_NAMES:
            assert recovered[name] == pytest.approx(original[name], abs=1e-4)

    def test_all_params_present_and_named_correctly(self):
        """Catches a real, easy mistake: PARAM_BOUNDS listing a typo'd
        name that doesn't actually match row_navigation_params.yaml --
        this test doesn't verify the yaml itself (that needs the real
        workspace), but it does pin down that this module's own naming is
        internally the set this whole pipeline expects."""
        expected = {'Kp_theta', 'Ki_theta', 'Kd_theta', 'fopid_lambda',
                    'fopid_mu', 'k_cross_track'}
        assert set(PARAM_NAMES) == expected


class TestObservationConstruction:
    def test_observation_has_expected_dimension(self):
        obs = build_observation(0.0, np.zeros(ACTION_DIM))
        assert obs.shape == (OBS_DIM,)

    def test_first_element_is_scaled_reward(self):
        obs = build_observation(prev_reward=-4.0, prev_action=np.zeros(ACTION_DIM),
                                reward_scale=2.0)
        assert obs[0] == pytest.approx(-2.0)

    def test_remaining_elements_are_clipped_action(self):
        obs = build_observation(0.0, np.full(ACTION_DIM, 3.0))
        assert np.all(obs[1:] == pytest.approx(1.0))

    def test_zero_reward_and_action_for_first_episode(self):
        """The very first episode has no prior trial -- confirm the
        documented convention (zeros) produces a valid, finite observation,
        not e.g. a NaN from an unguarded division somewhere."""
        obs = build_observation(0.0, np.zeros(ACTION_DIM))
        assert np.all(np.isfinite(obs))


class TestRewardComputation:
    def test_perfect_tracking_and_zero_vibration_gives_zero_reward(self):
        r = compute_reward(heading_rms=0.0, lateral_rms=0.0, vibration_rms=0.0)
        assert r == pytest.approx(0.0)

    def test_reward_is_always_non_positive(self):
        """Higher-is-better convention, but the BEST possible score is
        zero (perfect tracking) -- reward should never go positive for
        any non-negative error inputs, which would indicate a sign bug."""
        for h, lat, vib in [(0.1, 0.05, 0.02), (1.0, 1.0, 1.0), (0.0, 0.0, 0.5)]:
            assert compute_reward(h, lat, vib) <= 0.0

    def test_worse_heading_tracking_gives_worse_reward(self):
        r_good = compute_reward(heading_rms=0.05, lateral_rms=0.0, vibration_rms=0.0)
        r_bad = compute_reward(heading_rms=0.5, lateral_rms=0.0, vibration_rms=0.0)
        assert r_bad < r_good

    def test_worse_vibration_gives_worse_reward(self):
        r_good = compute_reward(heading_rms=0.0, lateral_rms=0.0, vibration_rms=0.05)
        r_bad = compute_reward(heading_rms=0.0, lateral_rms=0.0, vibration_rms=0.5)
        assert r_bad < r_good

    def test_weights_actually_change_relative_priority(self):
        """If w_vibration is raised well above w_heading, a run with equal
        heading and vibration error magnitudes should be penalized more
        by the vibration term than the heading term -- confirms the
        weights are actually wired to the terms they claim to control."""
        heading_only_error = compute_reward(heading_rms=0.5, lateral_rms=0.0,
                                            vibration_rms=0.0, w_heading=1.0,
                                            w_vibration=10.0)
        vibration_only_error = compute_reward(heading_rms=0.0, lateral_rms=0.0,
                                              vibration_rms=0.5, w_heading=1.0,
                                              w_vibration=10.0)
        assert vibration_only_error < heading_only_error

    def test_instability_penalty_makes_aborted_episode_strictly_worse(self):
        """An aborted (way-off-course) episode must score worse than any
        completed run, even one with fairly high tracking error -- this
        is what lets PPO reliably learn 'avoid this region' rather than
        treating a crash as just another noisy sample."""
        r_bad_but_completed = compute_reward(heading_rms=0.8, lateral_rms=0.3,
                                             vibration_rms=0.5)
        r_aborted = compute_reward(heading_rms=0.8, lateral_rms=0.3,
                                   vibration_rms=0.5, instability_penalty=100.0)
        assert r_aborted < r_bad_but_completed
        assert r_aborted == pytest.approx(r_bad_but_completed - 100.0)

    def test_default_weights_prioritize_heading_over_vibration_at_realistic_magnitudes(self):
        """Regression test for a real bug: vibration_rms was not squared
        while heading_rms/lateral_rms were, so at realistic sub-1.0
        operating magnitudes (errors are fractions of a metre/radian, not
        many multiples of one) squaring shrank the heading/lateral terms
        while vibration's raw magnitude did not -- silently inverting the
        documented default-weight priority ("staying on course is
        prioritized over ride smoothness") for exactly the magnitude range
        this reward function actually operates in during training. The
        existing test_weights_actually_change_relative_priority test above
        uses an artificially skewed w_vibration=10.0 that masks this --
        this test uses the real DEFAULT weights instead."""
        equal_magnitude = 0.1   # realistic small operating error
        heading_only = compute_reward(heading_rms=equal_magnitude, lateral_rms=0.0,
                                      vibration_rms=0.0)
        vibration_only = compute_reward(heading_rms=0.0, lateral_rms=0.0,
                                        vibration_rms=equal_magnitude)
        # w_heading (1.0) > w_vibration (0.5): equal-magnitude heading error
        # must cost strictly more reward than equal-magnitude vibration,
        # not less.
        assert heading_only < vibration_only

    def test_reward_is_deterministic(self):
        """Same inputs must always give the same reward -- a basic
        sanity check that there's no hidden randomness or unguarded
        global state in what's supposed to be a pure function."""
        args = dict(heading_rms=0.12, lateral_rms=0.04, vibration_rms=0.03)
        r1 = compute_reward(**args)
        r2 = compute_reward(**args)
        assert r1 == r2
