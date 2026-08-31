# Headland Turn Geometry — Ackermann Two-Arc "Bulb Turn"

## Why this exists
A skid-steer base can pivot in place (`v=0, ω≠0`). An Ackermann base cannot —
yaw rate requires nonzero speed (`dθ/dt = v·κ`, `κ = tan(δ)/L`), and curvature
is bounded by the mechanical steering limit, giving a minimum turning radius

```
R_min = L / tan(δ_max) = 0.25 / tan(0.6 rad) = 0.365 m
```

**UPDATED 2026-08-31 — `L` here is the PLANNING-layer wheelbase, not the true
one.** The true CAD/measured wheelbase is `0.2353 m` (bench-confirmed again
2026-08-31; giving the vehicle's actual tightest possible turn,
`R_min=0.344 m` — see `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`, "Ackermann
geometry"). `base_controller` and both URDFs still use that real value for
actuation. This node (`row_nav_node.py`, and Nav2's `min_turning_radius` in
`robot_bringup/config/nav2_params.yaml`) deliberately uses a rounder, more
conservative `L=0.25 m` instead, so the *planned* maneuver never assumes a
tighter turn than the real chassis can deliver — R_min=0.365 m here is
intentionally a little larger (more headland space assumed) than the
vehicle's true 0.344 m minimum. `δ_max=0.6 rad` was **not** split and is the
same real, CAD-sourced value everywhere.

(This whole page previously used the true `L=0.2353 m, R_min=0.344 m`
directly — itself a replacement for an earlier unverified `L=0.25 m` /
`R_min=0.537 m` that came from a *different, wrong* `δ_max=0.436 rad`. To be
clear this isn't that old error recurring: today's planning-layer `L=0.25 m`
pairs with the *correct* `δ_max=0.6 rad`, giving `R_min=0.365 m` —
nowhere near the old `0.537 m`. Whole table below recomputed accordingly.)

Tabletop row spacing (0.35–0.60 m) is well under the achievable turning
**diameter** `2·R_min ≈ 0.73 m`, so a single constant-radius arc still cannot
land the vehicle exactly on the next row for most of that range. This still
requires a two-segment maneuver.

## The maneuver
Commands are issued in the **(v, ω) Twist domain** — the same
steering-geometry-agnostic domain row_navigation always publishes in. The
physical steering angle δ is a *derived* quantity, computed downstream,
firmware-side, by Yahboom's own `Rosmaster_Lib` (via `base_controller`'s
`set_car_motion(vx, vy, vz)` call — see
`docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`'s "Rosmaster_Lib" section for the
verification trail). This distinction is the key to the whole maneuver and
to not re-introducing the sign confusion described below.

1. **Exit buffer** — straight forward creep past the last detected post
   (clears the chassis body, not just the kinematic point).
2. **Arc 1 — FORWARD** (`v = +turn_v`), commanded yaw rate `ω = +ω_const·turn_sign`,
   until heading has changed by `φ1`.
3. **Arc 2 — REVERSE** (`v = -turn_v`), commanded yaw rate held at the
   **same value and sign**, `ω = +ω_const·turn_sign` again — until heading
   has changed by a further `φ2`, where `φ1 + φ2 = π` exactly.
4. **Reacquire** — short forward creep, then hand back to RANSAC+FOPID.

`(φ1, φ2)` are solved once at node startup (2000-point local search) for
whatever `row_spacing` is configured, so the pair always lands exactly on
the next row centreline with an exact 180° heading reversal — see
`_bulb_turn_split()` in `row_navigation/row_nav_node.py`.

## Why the *commanded yaw rate* stays constant, but the *physical steering angle* flips
This is the single most confusable part of the maneuver, and an earlier
draft of this document got it backwards — worth spelling out precisely so
it doesn't happen again.

The standard bicycle-model identity is `θ̇ = (v/L)·tan(δ)`, i.e. yaw rate is
the *product* of signed speed and a term that depends on steering angle.
`set_car_motion`'s underlying firmware conversion is (by construction, for
any correct Ackermann implementation) the inverse of exactly this
relationship, which guarantees `θ̇ ≡ ω` **exactly** for *any* sign of `v`,
as long as the result isn't clipped by `max_steer_angle`. Consequently:

- Holding **ω constant** (same sign, same magnitude) across both arcs is
  *correct*, and is what makes `θ̇` — and therefore the heading sweep —
  monotonic across the whole maneuver: arc 1 and arc 2 both contribute
  heading change in the same rotational direction, summing to exactly π.
- The **physical steering angle** `δ` sent to the servo is *not* held
  constant — it flips sign automatically between arc 1 and arc 2, because
  `v` flips sign while `ω` doesn't. This is correct and intended, not a
  bug: it is exactly how a real forward/reverse 3-point turn works (lock
  the wheel one way, go forward and back while holding that lock — the
  car's *rotational direction* stays the same across both phases precisely
  *because* reversing direction of travel with a fixed wheel angle would
  normally reverse the yaw direction, and here we deliberately keep `ω`
  — not `δ` — as the invariant).

**The bug this document used to describe** (and that a first draft of the
code briefly had) was stating this the other way around — "hold the same
*steering angle* through reverse" — which is a different, physically
inconsistent claim: verify with a two-line simulation that integrating
`θ̇=(v/L)tan(δ)` with `δ` held at a fixed sign while `v` goes negative
*undoes* arc 1's heading gain instead of continuing it (the required
integration time to reach `φ2` in the forward direction comes out
negative — i.e. that combination cannot produce the desired sweep at all).
If you ever retune this maneuver, work in terms of **ω**, not δ; do not try
to reason about "the steering lock" directly when deriving the invariant.

## Verified numerically

Recomputed 2026-08-31 for the planning-layer `L=0.25 m`, `R_min=0.365 m`
(supersedes the previous table, which used the true `L=0.2353 m`,
`R_min=0.344 m` — see the note above on why the two now differ):

| row_spacing | φ1 | φ2 | forward extent | lateral bulge (away from final row) |
|---|---|---|---|---|
| 0.35 m | 118.6° | 61.4° | 0.64 m | 0.54 m |
| 0.40 m | 123.2° | 56.8° | 0.61 m | 0.57 m |
| 0.50 m | 133.2° | 46.8° | 0.53 m | 0.62 m |
| 0.60 m | 145.2° | 34.8° | 0.42 m | 0.67 m |

All four cases land within ~0.5 mm of the target lateral offset with an exact
180° heading reversal (0.447/0.208/0.141/0.222 mm respectively, including at
the tightest 0.35 m spacing — the solver still converges cleanly at the new,
larger R_min). **Verification level, stated plainly:** this table was
recomputed by calling the actual `_arc_step`/`_bulb_turn_split` primitives
from `row_nav_node.py` directly (not re-derived by hand, avoiding the exact
sign-error trap that function's own docstring warns about) and densely
sampling both arcs for the lateral-bulge peak. It has **not** been re-verified
against the closed-loop discrete state-machine simulation (yaw-threshold phase
switching via live odometry) that the original `R_min=0.344` table's
"verified numerically (closed-loop simulation)" claim rested on. Re-run that
closed-loop check before trusting this table for unattended operation. Note
the same trend as before: a *smaller* `R_min` needs *less* forward extent but
a *larger* lateral bulge — both numbers still matter, not just one.

## Real-world clearance requirement (must be checked against the actual tunnel/lab)
- **Depth** (`exit_buffer 0.40 m + forward extent + vehicle length margin
  ~0.35 m`), by `row_spacing` — updated 2026-08-31 for the new planning-layer
  R_min=0.365 m (each figure is ~5-8 cm more than the previous table, since
  the larger R_min needs more forward extent):
  - 0.35 m spacing: **≈1.39 m** clear headland depth (was ≈1.34 m)
  - 0.40 m spacing: **≈1.36 m** (was ≈1.31 m)
  - 0.50 m spacing (current codebase default): **≈1.28 m** (was ≈1.22 m)
  - 0.60 m spacing: **≈1.17 m** (was ≈1.09 m)
- **Width**: the maneuver bulges **0.54–0.67 m beyond the target row**
  (larger for wider row spacing) before arc 2 pulls it back — if there's a
  wall, a third row, or an obstacle that close beyond the target row, this
  maneuver will not clear it. Verify both numbers against the real
  polytunnel or lab layout before autonomous operation; do not assume a
  "typical" headland is sufficient. For Lab 3003 specifically, measure the
  actual clear space beyond the last row of desks before the first
  unattended headland turn — see `LAB_3003_MOCK_PLANT_AMR_DEPLOYMENT_GUIDE.md`.
- The maneuver is open-loop (odometry-tracked, not LiDAR-checked) for its
  duration — the headland must be physically verified clear before each
  autonomous run, not assumed safe from a previous one.

## Parameters (row_navigation node)
`wheelbase`, `max_steer_angle`, `row_spacing` (MUST match `coverage_planner`'s
value), `headland_exit_buffer`, `headland_turn_speed`, `headland_yaw_tol`,
`headland_reacquire_time`, `headland_reacquire_speed`, `alternate_turn_direction`.
