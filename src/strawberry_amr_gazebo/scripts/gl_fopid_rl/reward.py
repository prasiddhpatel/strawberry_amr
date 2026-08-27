"""
Pure, ROS-independent functions for GL-FOPID reinforcement-learning tuning
-- deliberately separated from gazebo_env.py's ROS/Gazebo glue code, same
reasoning as base_controller/command_safety.py elsewhere in this
workspace: this is the part that can be fully unit-tested without a live
ROS 2 + Gazebo environment, and it's also the part most worth getting
right through review, since it defines what "good" actually means to the
RL agent -- a bug here silently teaches a wrong objective, not a crash.

DESIGN: each RL EPISODE tests exactly ONE fixed set of GL-FOPID
parameters for one full simulated drive, and terminates. This is a
deliberate simplification from a "learn to adjust gains continuously
while driving" formulation -- the actual stated objective (see
docs/GL_FOPID_RL_TUNING_GUIDE.md) is to find and export ONE final static
parameter set, not a dynamically-adapting controller, so a single-step-
per-episode (contextual bandit-style) MDP matches the real objective
directly, is far easier to debug, and trains faster than a multi-step
formulation would for this specific goal.

PARAMETER BOUNDS -- a genuine judgment call, not a verified fact, stated
plainly: chosen as a generous-but-bounded range around this workspace's
existing shipped defaults (Kp_theta=1.2, Ki_theta=0.25, Kd_theta=0.30,
fopid_lambda=0.4, fopid_mu=0.7, k_cross_track=0.8 -- see
row_navigation_params.yaml), wide enough for the RL search to find a
genuine improvement, narrow enough to keep the fractional orders in the
well-behaved (0,2) range GL-FOPID literature typically explores and to
keep the integer-order gains away from obviously-unstable extremes. This
was NOT empirically validated against a real stability margin analysis --
review these bounds yourself before trusting them, same standard this
whole project applies to every other unverified default.
"""
import numpy as np

# (name, min, max) -- order matters, matches ACTION_DIM and every array
# below positionally
PARAM_BOUNDS = [
    ('Kp_theta',      0.1, 3.0),
    ('Ki_theta',      0.0, 1.0),
    ('Kd_theta',      0.0, 1.0),
    ('fopid_lambda',  0.1, 1.2),
    ('fopid_mu',      0.1, 1.5),
    ('k_cross_track', 0.1, 2.0),
]
PARAM_NAMES = [p[0] for p in PARAM_BOUNDS]
PARAM_MIN = np.array([p[1] for p in PARAM_BOUNDS], dtype=np.float32)
PARAM_MAX = np.array([p[2] for p in PARAM_BOUNDS], dtype=np.float32)
ACTION_DIM = len(PARAM_BOUNDS)

# observation = [prev_reward_normalized, *prev_action] -- see module
# docstring for why this gives the agent memory of its last trial rather
# than a constant/uninformative observation
OBS_DIM = 1 + ACTION_DIM


def action_to_params(action):
    """
    action: array-like of ACTION_DIM floats in [-1, 1] (standard SB3
    convention for a Box action space -- keeps the RL algorithm's own
    exploration noise well-scaled regardless of each parameter's real
    physical range).
    Returns: dict {param_name: real_value}, linearly mapped from
    [-1,1] to [PARAM_MIN, PARAM_MAX] and clipped defensively (SB3 should
    already respect the declared action space bounds, but clip anyway --
    never trust an external caller not to pass an out-of-range value into
    something that gets applied to a real robot's control loop).
    """
    a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
    real = PARAM_MIN + (a + 1.0) * 0.5 * (PARAM_MAX - PARAM_MIN)
    return dict(zip(PARAM_NAMES, real.tolist()))


def params_to_action(params):
    """Inverse of action_to_params -- maps a real parameter dict back to
    the [-1,1] action space. Used to seed a known-good starting point
    (e.g. the current shipped defaults) as the 'previous action' in the
    very first observation, and useful for sanity-checking exported
    results against what the action space could represent."""
    real = np.array([params[n] for n in PARAM_NAMES], dtype=np.float32)
    a = 2.0 * (real - PARAM_MIN) / (PARAM_MAX - PARAM_MIN) - 1.0
    return np.clip(a, -1.0, 1.0)


def build_observation(prev_reward, prev_action, reward_scale=1.0):
    """
    prev_reward: the raw (unnormalized) reward from the previous episode
      (0.0 for the very first episode -- no previous trial to report).
    prev_action: the ACTION_DIM action taken in the previous episode
      (zeros, i.e. the midpoint of every parameter's range, for the very
      first episode).
    reward_scale: divides prev_reward before returning, so its magnitude
      stays in a similar range to the [-1,1] action components -- keeps
      PPO's default observation normalization assumptions reasonable
      without requiring VecNormalize. Tune this to roughly the typical
      magnitude of compute_reward()'s output for your own error/vibration
      weight choices; the default assumes the weights suggested in this
      module's compute_reward() docstring.
    """
    prev_action = np.clip(np.asarray(prev_action, dtype=np.float32), -1.0, 1.0)
    return np.concatenate([[prev_reward / reward_scale], prev_action]).astype(np.float32)


def compute_reward(heading_rms, lateral_rms, vibration_rms,
                    w_heading=1.0, w_lateral=1.0, w_vibration=0.5,
                    instability_penalty=0.0):
    """
    The actual objective the RL agent optimizes -- get this right, since
    a wrong weighting here silently teaches the wrong thing no matter how
    well the RL training otherwise converges.

    heading_rms: RMS of /row_heading_error (rad) over one episode.
    lateral_rms: RMS of /row_lateral_offset (m) over one episode.
    vibration_rms: RMS of IMU angular velocity magnitude (rad/s) over one
      episode -- the "IMU-detected chassis vibration" metric named in the
      original objective. Using angular velocity rather than linear
      acceleration specifically: a real chassis vibrating/juddering over
      a threshold shows up much more distinctly as noisy angular rate
      (rocking) than as noisy linear acceleration (which is dominated by
      the intentional forward-motion signal) -- this is a reasoned choice,
      not an arbitrary one, but still worth checking against real IMU
      data once you have it (see the calibration guide).
    w_heading, w_lateral, w_vibration: relative weights -- the DEFAULT
      values here weight heading and lateral tracking equally and give
      vibration half that weight, reflecting that this workspace's
      existing headland-turn/row-following design already prioritizes
      staying on course over ride smoothness; adjust if your own priority
      differs (e.g. raise w_vibration if IMU-detected juddering turns out
      to be the more thesis-relevant failure mode for your specific
      threshold geometry).
    instability_penalty: an ADDITIONAL flat penalty (positive number,
      subtracted from reward) the environment should pass in when an
      episode had to be aborted early (e.g. the robot went far enough off
      the row centreline that continuing the simulated drive would be
      meaningless) -- keeps a clearly-bad parameter set from scoring
      merely "not great" and instead scores it as unambiguously worse
      than any completed run, which matters for PPO to reliably learn to
      avoid that region of parameter space rather than treat it as merely
      noisy.

    Returns a single float, HIGHER IS BETTER (standard RL convention --
    this is negative squared/absolute error, not raw error).
    """
    return -(w_heading * heading_rms ** 2
             + w_lateral * lateral_rms ** 2
             + w_vibration * vibration_rms ** 2) - instability_penalty
