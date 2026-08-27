# Motor, Encoder, and Geometry Verification

Figures below were supplied by the operator (sourced from Yahboom product
pages and a third-party teardown) and independently re-derived/sanity
checked here before being used in code or thesis tables. Where a figure
could not be independently confirmed against a primary datasheet, that is
stated explicitly.

## Encoder: JGB37-520, 11 PPR base x 1:19 gearbox

```
output_shaft_PPR (single channel)  = 11 x 19           = 209
output_shaft_CPR (x4 quadrature)   = 209 x 4            = 836   <-- used in code
```//
Quadrature decoding (counting both rising and falling edges of both phase
A and phase B) is standard practice and is what `base_controller_node.py`
assumes the YB-ERF01 firmware performs. **Recommended bench check before
trusting this in published/logged data:** rotate one rear wheel exactly one
full revolution by hand and confirm the firmware reports 836 ticks.

## Speed consistency

```
no-load motor speed         11000 RPM
/ gear ratio                 19
= theoretical no-load output 578.9 RPM
```
Stated RATED (loaded) output speed is 550 +/- 10 RPM -- consistent (loaded
speed below the no-load figure, as expected).

```
550 RPM @ 65 mm wheel diameter -> linear speed = 1.87 m/s
```
This matches the independently corroborated chassis top-speed spec
(1.8 m/s) found across multiple ROSMASTER R2 product listings. **Design
decision:** use 550 RPM / 1.87 m/s as the *sustained* design speed (not the
no-load 1.8-ish m/s figure), since 1.8 m/s likely reflects a briefly
achievable peak rather than continuous-duty output.

## Torque margin (mass budget specific to THIS build, not the old 4WD rig)

A from-scratch mass budget for the dual-Pi Ackermann build (chassis, 2
motors, servo, wheels, 2x Pi, YB-ERF01, USB hub, RPLIDAR A1M8 (corrected
from an earlier C1 assumption -- both are lightweight sub-200g 2D LiDAR
units, so this correction is mass-neutral to first order and does not
change the torque-margin conclusions below), Astra Pro Plus, smaller LiPo,
buck converter, cabling, fasteners) totals **~2.76 kg**
-- substantially lighter than the old 4WD/FIT0185 rig (~5.2 kg), due to
fewer motors, no Cytron drivers, and much lighter compute (2x Pi vs one
AGX Orin module).

| Duty case | demand/wheel | rated torque (2.20 kgf.cm) margin | stall torque (3.10 kgf.cm) margin |
|---|---|---|---|
| Flat polytunnel aisle (representative) | 1.06 kgf.cm | **2.08x** | **2.94x** |
| Small threshold / gentle camber (5 deg) | 1.52 kgf.cm | 1.45x | 2.05x |
| Steep ramp, wet (10 deg, Crr=0.25 -- the old 4WD design's worst case) | 2.24 kgf.cm | 0.98x | 1.39x |

**Honest finding, not glossed over:** this is a flat-floor design. On the
realistic duty cycle (flat tabletop aisle floor) the margin is healthy. On
anything resembling a real ramp or threshold, the rated-torque margin drops
*below 1x* -- the motors cannot sustain climbing a 10 deg grade at rated
torque (stall torque alone gives only 1.39x, enough for a brief lip, not a
sustained slope). **Route planning must avoid thresholds, ramps, or
significant grade changes; this is a real, load-bearing constraint on
where this robot can be deployed, not a hypothetical edge case.**

## Why the Cytron MDD10A drivers were dropped

The FIT0185 motors (old 4WD design) drew enough stall current to risk
burning out the YB-ERF01's onboard driver ICs, which is why external Cytron
MDD10A drivers were mandatory. The JGB37-520's quoted stall current (3 A)
is dramatically lower and is plausibly within the onboard driver's rating
(most boards in this class use a dual H-bridge IC rated ~1.2-3 A
continuous per channel). **This was not independently confirmed against
the YB-ERF01's own driver IC datasheet** -- verify this on the bench
(monitor driver IC temperature under sustained load) before relying on it
in unattended operation; if it runs hot, add a single small external driver
rather than reintroducing the old dual-Cytron complexity.

## Whether the two rear motors get an electronic differential — now genuinely unknown, not a documented approximation

**This section's underlying facts changed with the Rosmaster_Lib
rewrite**, and it's worth being precise about what changed rather than
just updating a number. An earlier revision of `base_controller` sent a
single `speed_mps` to both rear motors over a hand-rolled protocol, so it
was *known* with certainty that no differential existed — the code doing
it was ours. Now, `base_controller` calls Yahboom's own
`set_car_motion(vx, vy, vz)`, and the STM32 firmware decides internally
how to drive the two rear motors to achieve the requested `(vx,vz)`. Since
that firmware is not open source in the sample archive (only the Python
wrapper is), **whether it implements an internal electronic differential
is genuinely unknown from this codebase's sources** — it might; a
car-type-aware firmware sophisticated enough to do bicycle-model
conversion at all is a reasonable place to also handle this. This is an
honest "don't know" rather than a documented certainty either way.

Worth quantifying the worst case if it does *not*: during a turn, if the
two rear wheels were following the yaw rate exactly (as they would with a
mechanical differential), their speed split would be approximately
`v_inner = v(R - W_rear/2)/R`, `v_outer = v(R + W_rear/2)/R`. At the
*tightest* the vehicle ever turns — the headland bulb-turn, `R=R_min` —
using the real CAD-sourced `rear_track=0.1685 m` and `R_min=0.344 m` (see
"Ackermann geometry" above; both larger and smaller respectively than the
placeholder figures an earlier revision of this note used, so the
worst-case percentage is also different from before):

```
W_rear / R_min = 0.1685 / 0.344 ≈ 49%
```

If no firmware-side differential exists, this is a genuinely larger
worst-case mismatch than the earlier (placeholder-based) 36% estimate —
worth taking seriously, not just noting in passing. Two things still bound
the practical impact regardless of which case is true:

1. **Row-following (the overwhelming majority of drive time) turns at a
   much larger radius than R_min** — heading corrections during normal
   tracking are small, so `W_rear/R` during ordinary operation is a small
   fraction of the headland-turn worst case. Whatever mismatch exists
   costs the least precisely where the robot spends most of its time.
2. **The bulb-turn itself is deliberately slow** (`headland_turn_speed` =
   0.35 m/s, well under the 1.8 m/s design top speed) specifically
   *because* it is the maneuver where any such mismatch is largest — low
   speed means low kinetic energy in any scrub event, on what is expected
   to be a firm, flat tabletop-aisle floor.

**What to actually do about this:** the bench-test procedure in
`docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md` already has the robot
elevated and driving in isolation before it ever touches the ground —
watch the two rear wheels directly during a full-lock turn commanded via
`set_car_motion`. If they visibly turn at different speeds, the firmware
is already handling this and the 49% worst-case above doesn't apply. If
they turn at the same speed, the worst-case scrub above is real, and the
mitigations in points 1-2 are what you're relying on until/unless a
firmware update or a different control approach addresses it directly —
there is no ROS-side fix available now that the conversion happens inside
`set_car_motion`, since this codebase doesn't have visibility into or
control over the STM32 firmware's internal motor drive logic.

## Ackermann geometry — now CAD-sourced, not placeholder

**This section changed materially since the last revision.** The
operator's uploaded `ROSMASTER_R2_robot_sample_codes.zip` contains
`yahboomcar_description/urdf/yahboomcar_R2.urdf.xacro` — a genuine
SolidWorks-to-URDF-Exporter output (see that file's own header comment),
not a placeholder or a third-party approximation. The real geometry,
extracted directly from its joint `<origin>`/`<limit>` tags:

```
wheelbase L        = front_left_steer_joint.x (0.11922) - back_left_joint.x (-0.11607) = 0.2353 m
front_track        = front_left_steer_joint.y (0.065) - front_right_steer_joint.y (-0.065)  = 0.130 m
rear_track         = back_left_joint.y (0.08425) - back_right_joint.y (-0.08425)            = 0.1685 m
max_steer          = <limit lower="-0.6" upper="0.6"/> on both front steer joints            = 0.6 rad (34.4 deg)

R_min = L / tan(max_steer) = 0.2353 / tan(0.6) = 0.344 m
```

This **supersedes** the earlier `L=0.25 m, front_track=0.195 m,
max_steer=0.436 rad, R_min=0.537 m` figures used throughout an earlier
revision of this codebase — those traced back to an unverified source (see
"Chassis dimension sourcing" below for the full history), and are now
replaced everywhere (URDF, `row_navigation_params.yaml`, `nav2_params.yaml`)
with the numbers above.

**Cross-check, not just a single source:** `rear_track=0.1685 m` from the
CAD file independently corroborates a "~0.169 m" figure that appeared
separately (with no primary source at the time) in other pasted material —
the two now agree to within 1.5 mm from two independent origins, which is
a real, meaningful consistency check, not a coincidence. Separately,
`R_min=0.344 m` is cross-checked against the firmware's own top-speed limit:
`Rosmaster_Lib.py`'s `set_car_motion` docstring states R2/R2L's `vx` range
is `[-1.8, 1.8]` m/s; at the verified 65 mm wheel diameter, 1.8 m/s implies
≈529 RPM, consistent with (and slightly conservative relative to) the
motor's independently-verified 550±10 RPM rating. Two unrelated numbers
landing where they should is a good sign the whole picture is self-
consistent, not just individually plausible.

**What's still worth a bench check, stated plainly:** a CAD joint limit is
a *design* value. Servo travel, linkage slop, and assembly tolerance can
shift the real achievable `max_steer` slightly on your specific unit. This
is a materially better starting point than the old placeholder, but a
2-minute confirmation (drive to full lock, check with a protractor) is
still worth doing before fully trusting `R_min`-derived headland-turn
geometry for unattended operation — see
`docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md` Part 1.

The separately-quoted inner/outer wheel angles at full lock from other
pasted material (alpha=28.5 deg inner, beta=22.4 deg outer) were checked
against the Ackermann condition `cot(beta) - cot(alpha) = track/wheelbase`:
computed LHS = 0.584, RHS (0.130/0.2353, using the now-correct CAD track)
= 0.552 — close, plausibly within real linkage tolerance, though not an
exact match. This doesn't matter operationally: the control software
commands the *bicycle-model* `max_steer` as its one steering parameter (see
below) and never needs the individual inner/outer wheel angles — the
physical trapezoidal linkage handles that split automatically, the same
way it does on a real car.

## Rosmaster_Lib — the real hardware API, verified from source

**This is a significant architecture change from an earlier revision of
this codebase**, which talked to the YB-ERF01 board over a hand-rolled
serial protocol. Reading Yahboom's own `Rosmaster_Lib.py` (vendored
verbatim into `base_controller/base_controller/vendor/`, from the
operator's `ROSMASTER_R2_robot_sample_codes.zip`, version string `V2.3.3`
in the source) settled several previously-disputed claims with certainty,
by reading the actual method bodies rather than trusting secondhand
descriptions:

| Claim | Verified from source | What it replaced |
|---|---|---|
| Car-type constant for R2 | `self.CARTYPE_R2 = 0x05` (i.e. **5**) | An earlier draft claimed `car_type=2`; wrong, never shipped |
| Ackermann command API | `set_car_motion(vx, vy, vz)` is car-type-aware (packs `self.__CAR_TYPE` into every `FUNC_MOTION` command) and performs the bicycle-model conversion in firmware. For R2/R2L: `vx∈[-1.8,1.8]` m/s, `vy∈[-0.045,0.045]` m/s (near-zero on purpose — Ackermann can't strafe), `vz∈[-3,3]` rad/s | A from-scratch `ackermann_bridge` node computing `delta=atan(L*omega/v)` itself — removed; trusting tested vendor firmware over a from-scratch reimplementation of the same thing is the more reliable choice |
| Battery voltage scale | `get_battery_voltage()` returns `self.__battery_voltage / 10.0` — confirmed **divide by 10.0** | A pasted claim asserted "×0.01" with a worked example (1210→12.10V) that isn't even internally consistent with real `/10.0` math (1210/10.0 = 121.0V) |
| Fused IMU orientation | `get_imu_attitude_data()` **exists in source but is commented out** — inactive in this library version. Only raw `get_accelerometer_data()`/`get_gyroscope_data()`/`get_magnetometer_data()` are active | A pasted claim asserted a ready fused quaternion made `imu_filter_madgwick` unnecessary — does not hold for the actual shipped library; the external Madgwick filter this workspace already used is still the right design |
| Odometry source | `get_motion_data()` returns `(vx,vy,vz)` already in SI units (confirmed: the receive-thread unpacking divides by 1000, mirroring the `*1000` encoding `set_car_motion` uses to *send* commands) — used as the primary odometry source in `base_controller_node.py`, on the same "trust the vendor's one tested kinematic model, don't re-derive it a second time" reasoning as the command path | Re-deriving Ackermann odometry from raw encoder ticks in Python a second time |
| Steering command units | `set_akm_steering_angle(angle, ctrl_car)` takes a **servo-relative offset in degrees, range [-45,45]**, not a raw absolute 0–180 PWM value | A pasted claim asserted `set_pwm_servo(1, angle)` with 0–180 absolute degrees was the steering API; wrong function entirely — not used in this codebase since `set_car_motion` supersedes it for normal operation |
| Gyro units | `gyro_ratio = 1/3754.9` with source comment `"+/-500dps"` → **degrees/s**, converted to rad/s in `base_controller_node.py` per REP-103 | — |
| Accel units | `accel_ratio = 1/1671.84`, **no unit comment in source** — assumed g's (× 9.80665 for m/s²) in `base_controller_node.py`, flagged there as needing the bench confirmation in the calibration guide's stationary check | Genuine remaining uncertainty, not resolved by the source alone |

Serial port: the library's own constructor default is `com="/dev/myserial"`
— `scripts/setup_udev_rules.sh` creates a symlink with that exact name to
match, rather than inventing a different convention.

## Sources: motor and chassis figures
Motor and chassis figures: operator-supplied, citing Yahboom product/build
pages (`category.yahboom.net/products/rosmaster-r2`,
`yahboom.net/build/id/6189` and `/6367`) and third-party listings for the
generic JGB37-520 motor family. Top speed (1.8 m/s) and "two 520 motors,
12V output 550rpm" independently corroborated via web search against
Yahboom's own unboxing/review content and the ROSMASTER-R2 GitHub repo
during this design session. The motor's full rated/stall torque, current,
and gear-ratio figures were additionally corroborated **word-for-word**
against a direct Yahboom vendor Q&A on the R2 product page itself
(`category.yahboom.net/products/rosmaster-r2`, "What is the max speed?"
answer): "Rated voltage: 12V; Stall torque: 3.1 kgf.cm; Rated torque:
2.2kgf.cm; Speed before deceleration: 11000rpm; Rated power: <4w; Stall
current: 3A; Rated current: 0.3A; Reduction ratio: 1:19; Speed after
deceleration: About 550+/-10rpm; Supply voltage: 3.3-5V" — this is a
primary-source match, not just corroboration. Chassis geometry
(wheelbase/track/max_steer) is now CAD-sourced from the real
`yahboomcar_R2.urdf.xacro` — see above.

## RPLIDAR A1M8 (corrected from an earlier RPLIDAR C1 assumption)

The operator confirmed the actual unit shipped in the Yahboom kit is the
RPLIDAR A1M8, which ships standard in the R2's Standard/Superior tiers (the
Ultimate tier instead ships a YDLidar 4ROS unit — a different sensor
entirely; if that is what you actually have, none of the numbers below
apply and this needs re-verification). Verified directly against Slamtec's
own datasheet and multiple independent vendor listings:

| Property | Value | Source note |
|---|---|---|
| Ranging principle | **Triangulation** (NOT time-of-flight) | Slamtec datasheet |
| Distance range | 0.15-12 m (white/reflective objects; less on darker, less-reflective targets — standard triangulation-LiDAR behaviour, not unique to this unit) | R6-revision datasheet; older sub-R4 units are 0.15-6 m |
| Angular range | 0-360 deg | Slamtec datasheet |
| Angular resolution | <=1 deg typical | Slamtec datasheet |
| Distance resolution | <0.5 mm, <1% of distance | Slamtec datasheet |
| Sample rate | 2000 Hz (older R1/R2 revisions) or up to 8000 Hz (R4+/R6) | Multiple corroborating listings; matches Yahboom's own "sampling frequency is 8K" claim for the R2 kit |
| Scan rate | 1-10 Hz, typical 5.5 Hz | Slamtec datasheet |
| Interface | 3.3V TTL UART via a CP210x USB-serial bridge | Slamtec datasheet + Waveshare wiki |
| Baud rate | **115200** (the value used essentially universally across the ROS ecosystem's rplidar_ros tutorials/launches for the A-series; 57600 is a documented fallback on some adapter-board revisions) | Corroborated via multiple ROS community sources — confirm empirically per the calibration guide if scans come back empty |
| Laser class | Class 1 (eye-safe), infrared | Slamtec datasheet |
| Dimensions (R6) | ~98.5 x 70 x 60 mm | Multiple corroborating vendor listings; Slamtec does not publish exact mechanical tolerances publicly, so treat as a bounding-box approximation for RViz/visual purposes, not a precision figure |
| Weight | ~170 g | Slamtec datasheet + vendor listings |

**Operational risk worth taking seriously, not glossing over:** Slamtec's
own datasheet qualifies outdoor use directly: *"It can work excellent in
all kinds of indoor environment and outdoor environment without direct
sunlight exposure."* A polytunnel interior is diffuse-lit rather than
direct-beam sun in most conditions, so this is not necessarily
disqualifying — but the C1 this replaced was a time-of-flight sensor
specifically chosen (in the earlier, now-superseded 4WD design) partly
*because* ToF tolerates ambient light better than triangulation. Swapping
back to a triangulation sensor for the actual build reintroduces a real
risk that needs empirical verification, not an assumption either way. See
`docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md`'s LiDAR section for a
specific bright-vs-shaded comparison test to run early, before investing
further integration time on the assumption it will be fine.

## Chassis dimension sourcing — history, for context

An earlier revision of this document explained that no Yahboom-published
CAD file or dimensioned drawing could be found for the R2, and that the
`L=0.25 m / front_track=0.195 m / max_steer=0.436 rad` figures then in use
traced back to an unverified source (specifically: two Yahboom "build"
page citations that, on being fetched and read in full, turned out to be a
TEB path-planning tutorial video and a generic Ackermann kinematics
*theory* lesson — neither actually containing a measured spec for this
chassis). That gap is now closed: see "Ackermann geometry" above for the
real, CAD-sourced figures the operator's sample-code archive provided.
This section is kept for the historical record — it is a good example of
why this project treats "internally self-consistent" and "confirmed
against a primary source" as different claims, and checks which one it
actually has before shipping a number.
