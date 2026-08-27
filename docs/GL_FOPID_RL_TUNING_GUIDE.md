# GL-FOPID Tuning via Reinforcement Learning

**Runs on the Orin, against a real running Gazebo simulation.** Searches
for a single, static set of the six GL-FOPID parameters
(`Kp_theta`, `Ki_theta`, `Kd_theta`, `fopid_lambda`, `fopid_mu`,
`k_cross_track`) that minimizes row-tracking error and IMU-detected
chassis vibration, using Stable-Baselines3's PPO as the search algorithm.

## Read this first: what's already true, what this changes

**The GL-FOPID controller itself already exists and is already correct**
— this was verified directly against `row_nav_node.py`'s actual source
before any of this was built, not assumed. It implements genuine
Grünwald-Letnikov fractional calculus (the recursive binomial-coefficient
weight formula, `w[j] = w[j-1] * (1 - (α+1)/j)`, is the textbook-correct
GL discretization), with real fractional integral/derivative orders
already wired into the control loop. This RL pipeline does not build that
controller — it searches for better values for its six existing
parameters.

**An honest note on tool choice, stated once and then left alone**: PPO
is a substantially heavier, slower-to-converge tool than a 5-6 parameter
continuous search strictly needs — classical optimization (grid search,
Nelder-Mead, Bayesian optimization) would very likely reach a comparable
result in a fraction of the wall-clock time, with less risk of simply not
converging. This was raised and heard; this pipeline implements the RL
approach as explicitly requested, built to the same standard as
everything else in this workspace. If a training run doesn't converge
well within your available time, switching search strategy — not
abandoning the effort — is a reasonable fallback, and the reward/
observation logic in `reward.py` would work identically well as the
objective function for a classical optimizer if you ever want that
option; nothing here forecloses it.

## The RL formulation

**One episode = one fixed parameter set, one fixed-duration simulated
drive, then terminate.** This is deliberate: the actual goal is "find one
final static parameter set to export," not "learn a controller that
adapts while driving" — a single-step-per-episode design matches that
goal directly, trains faster, and is far easier to debug than a
multi-step formulation would be. See `reward.py`'s own module docstring
for the full reasoning.

- **Action** (6 floats, each in [-1, 1]): linearly mapped to each
  parameter's real range (see `PARAM_BOUNDS` in `reward.py`). Bounds are
  a genuine judgment call — a generous-but-bounded range around this
  workspace's shipped defaults, not an empirically-validated stability
  margin. Review them before trusting a result near either edge.
- **Observation** (7 floats): `[previous_reward_scaled, *previous_action]`
  — gives the agent memory of what it just tried and how well it worked,
  supporting a genuinely sequential search across episodes rather than
  blind resampling.
- **Reward**: `-(w_heading·heading_rms² + w_lateral·lateral_rms² +
  w_vibration·vibration_rms²) - instability_penalty`. Vibration is
  measured from IMU angular velocity magnitude specifically, not linear
  acceleration — a real chassis juddering over a threshold shows up far
  more distinctly as noisy angular rate than as noisy linear acceleration
  (which is dominated by the intentional forward-motion signal). Default
  weights favor tracking accuracy over ride smoothness 2:1 — see
  `compute_reward()`'s own docstring if your priority differs. (The
  vibration term was previously left unsquared while heading/lateral
  were squared, which silently inverted this 2:1 priority at the
  realistic sub-1.0 error magnitudes this function actually operates in
  during training — fixed by squaring it to match; regression test at
  default weights added specifically because the existing weighting test
  used an artificially skewed weight that masked this.)
- **Episode abort**: if lateral offset or heading error exceeds a
  threshold mid-episode, the episode ends immediately with a penalty
  rather than running out its full duration — both a training-speed
  optimization (real-time-bound Gazebo episodes are wall-clock expensive)
  and a reasonable safety property (a badly-tuned parameter set doesn't
  get left running any longer than needed to confirm it's bad).

## What was verified, and how — read before trusting a training run

This sandbox has no `rclpy` and no running Gazebo instance, so the real
ROS 2/Gazebo integration (`RealRosGazeboBackend` in `gazebo_env.py`)
could not be executed here — that genuinely needs your own Orin with
Gazebo running. What **was** verified, by actually running it, not just
reading it:

- `reward.py`'s reward/observation/action-mapping logic: 18 real pytest
  tests, all passing (`test_reward.py`) — including that deceleration-
  adjacent edge cases behave correctly, that the action space bounds are
  respected even for out-of-range inputs, and that the reward weighting
  genuinely changes which failure mode is penalized more.
- `GLFOPIDGazeboEnv`'s Gymnasium API compliance: Stable-Baselines3's own
  `check_env()` utility, run against the environment with a mock backend
  — passed.
- The full pipeline end-to-end: a real PPO training run (`MlpPolicy`, 20-
  40 timesteps) against the mock backend actually executed inside a real
  Jupyter kernel (not just syntax-checked) — completed without error and
  found progressively better synthetic rewards, confirming the
  environment, reward wiring, and SB3 integration genuinely work
  together, not merely that each piece compiles in isolation.

**What this does NOT verify**: whether the real ROS 2 service calls in
`RealRosGazeboBackend` (setting parameters via `/row_nav_node/set_parameters`,
resetting the robot's pose via `/gazebo/set_entity_state`) work correctly
against your actual running Gazebo session. Run the smoke test in the
notebook (Step 0) first — it's fast and catches pipeline bugs — but treat
the first few real episodes against live Gazebo as their own validation
step too: watch Gazebo directly for the first 3-5 episodes and confirm
the robot's pose actually resets and the parameters actually change
between them, before leaving a long unattended run going.

## Setup

```bash
cd ~/strawberry_ws/src/strawberry_amr_gazebo/scripts/gl_fopid_rl
pip install -r requirements.txt --break-system-packages
```

Confirm the base workspace (Gazebo, `strawberry_amr_gazebo`) is already
built per `GAZEBO_SIMULATION_GUIDE.md` — this pipeline assumes that
already works.

## Running it

1. Open `train_gl_fopid.ipynb` (`jupyter notebook` or your usual
   notebook environment) in this same directory.
2. **Run Step 0 (the smoke test) first, every time**, even if you've run
   it before — it costs seconds and confirms nothing has silently broken
   before you commit real time to Step 3.
3. Step 1 generates the training world (with floor thresholds — adjust
   `--thresholds`/`--threshold-height` to your best current estimate of
   Lab 3003's real threshold geometry).
4. Step 2 is a reminder, not a cell to run: launch Gazebo yourself in a
   separate terminal (the exact command is in that cell's markdown) —
   Gazebo needs to run as its own long-lived process with a GUI window,
   which doesn't work well started from inside a notebook cell.
5. Step 3 is the real training loop. **Time-box it deliberately**: at the
   default 25s/episode, 150 episodes is roughly an hour of continuous
   real time (Gazebo runs at or near real-time) — start with a number you
   can actually complete and watch, not the largest number you can
   imagine, especially for a first attempt.
6. Step 4 exports the best parameter set *actually observed* during
   training (not necessarily PPO's final policy — RL training can and
   does regress temporarily, so trusting the best-observed result is a
   deliberate, safer choice) to a timestamped YAML file.

## Applying results — do not skip the validation chain

The exported YAML has exactly six keys, meant to be copied into
`src/row_navigation/config/row_navigation_params.yaml` — **not** to
replace that file wholesale. After copying them in:

1. Rebuild (`colcon build --symlink-install`).
2. Re-run the *same* Gazebo bench test (`ALL_IN_ONE_DEPLOYMENT_GUIDE.md`
   Part 11) with the new parameters, watching for the same behaviour you
   watched for during training — clean row-following, a clean headland
   turn, sensible behaviour crossing the threshold world if you kept it.
3. Only then move to the real robot's own bench test, elevated on blocks
   first, per the same Part 11 procedure — an RL-found parameter set has
   been validated against simulated dynamics, not real chassis mass,
   real motor response, real floor friction, or real sensor noise. Treat
   it as a strong, evidence-based starting point for the real hardware
   bench test, not a finished result that skips it.

## Troubleshooting

- **Reward isn't improving after a few dozen episodes**: check the abort
  thresholds (`abort_lateral_m`, `abort_heading_rad` in the training
  cell) aren't so tight that most episodes end almost immediately —
  if every episode aborts in the first second, PPO never sees enough
  signal to distinguish a mediocre parameter set from a good one.
- **Every episode aborts near the exact same point**: likely the
  threshold geometry itself (Step 1), not the FOPID parameters — confirm
  the world's thresholds match something the current shipped defaults can
  already cross reasonably before assuming the search itself has failed.
- **`row_nav_node /set_parameters service unavailable`**: Gazebo (Step 2)
  isn't actually running, or `row_nav_node` isn't up yet — confirm with
  `ros2 node list` before re-running Step 3.
- **Training runs but the exported parameters look extreme (near a
  bound)**: worth treating as a signal the search wants to go further
  than `PARAM_BOUNDS` currently allows, not necessarily a good result to
  trust as-is — widen the specific bound deliberately and re-run, rather
  than accepting a boundary value as "the answer" without checking why it
  was pushed there.
