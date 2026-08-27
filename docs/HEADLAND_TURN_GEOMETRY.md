# Headland Turn Geometry — Ackermann Two-Arc "Bulb Turn"

## Why this exists
A skid-steer base can pivot in place (`v=0, ω≠0`). An Ackermann base cannot —
yaw rate requires nonzero speed (`dθ/dt = v·κ`, `κ = tan(δ)/L`), and curvature
is bounded by the mechanical steering limit, giving a minimum turning radius

```
R_min = L / tan(δ_max) = 0.2353 / tan(0.6 rad) = 0.344 m
```

(`L` and `δ_max` are CAD-sourced from Yahboom's own `yahboomcar_R2.urdf.xacro`
— see `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`, "Ackermann geometry", for the
extraction. This **replaces** an earlier `R_min=0.537 m` computed from an
unverified `L=0.25 m / δ_max=0.436 rad` — the whole table below was
recomputed for the real, smaller `R_min`, which needs a *tighter* turn and
therefore *less* headland space than the earlier revision of this document
stated.)

Tabletop row spacing (0.35–0.60 m) is well under the achievable turning
**diameter** `2·R_min ≈ 0.69 m`, so a single constant-radius arc still cannot
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

## Verified numerically (closed-loop simulation, not just the open-loop arc math)

Recomputed for the real, CAD-sourced `L=0.2353 m`, `R_min=0.344 m`
(supersedes an earlier table computed for the unverified `L=0.25 m` /
`R_min=0.537 m`):

| row_spacing | φ1 | φ2 | forward extent | lateral bulge (away from final row) |
|---|---|---|---|---|
| 0.35 m | 120.6° | 59.4° | 0.59 m | 0.52 m |
| 0.40 m | 125.6° | 54.4° | 0.56 m | 0.54 m |
| 0.50 m | 136.7° | 43.3° | 0.47 m | 0.59 m |
| 0.60 m | 150.8° | 29.2° | 0.34 m | 0.64 m |

All cases land within ~1 mm of the target lateral offset with an exact 180°
heading reversal, verified by forward-integrating the **exact discrete
state-machine logic** used at runtime (yaw-threshold phase switching via
odometry), not just the abstract two-arc geometry. Note the trend versus
the earlier (incorrect-geometry) table: a *smaller* `R_min` needs *less*
forward extent to complete a given lateral shift, but produces a
correspondingly *larger* lateral bulge — worth having both numbers, not
just one, when checking real headland clearance.

## Real-world clearance requirement (must be checked against the actual tunnel/lab)
- **Depth** (`exit_buffer 0.40 m + forward extent + vehicle length margin
  ~0.35 m`), by `row_spacing`:
  - 0.35 m spacing: **≈1.34 m** clear headland depth
  - 0.40 m spacing: **≈1.31 m**
  - 0.50 m spacing (current codebase default): **≈1.22 m**
  - 0.60 m spacing: **≈1.09 m**
- **Width**: the maneuver bulges **0.52–0.64 m beyond the target row**
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
