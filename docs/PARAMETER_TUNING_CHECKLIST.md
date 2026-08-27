# Parameter Tuning & Commissioning Checklist

Every value in this workspace that you will realistically need to change,
ordered by **when you hit it**, not by which file it lives in. Each entry
says what it is, its current value, how to determine the right one, and
what breaks if it's wrong.

**How values are classified**, so you know what to trust:

| Tag | Meaning |
|---|---|
| **[VERIFIED]** | Confirmed against a primary source (Yahboom's real `Rosmaster_Lib.py`, the real CAD URDF, a vendor datasheet). Don't change without a reason. |
| **[MEASURE]** | A placeholder or CAD-derived value that must be replaced with a real measurement from *your* robot/room. |
| **[TUNE]** | No correct value exists on paper — must be found empirically on hardware. |
| **[MUST SET]** | Currently empty or a guess; the system will not work until you set it. |

---

## STAGE 0 — Before first power-on (blocking)

### 0.1 Serial port — `base_controller_params.yaml`
`serial_port: /dev/myserial` **[VERIFIED]**

Matches `Rosmaster_Lib`'s own constructor default. Created by
`scripts/setup_udev_rules.sh`. **Verify** with `ls -l /dev/myserial` after
plugging in the board — if it's missing, the udev rule didn't apply and
`base_controller` will log a connect error and no-op every hardware call.

### 0.2 PS2 controller mapping — `ps2_mapping.yaml`
```
axis_linear: 1     axis_angular: 2    deadman_button: 6
autonomy_button: 0 estop_button: 1    reset_button: 9
```
**[MEASURE] — these are a guess for a "typical" adapter, not your unit.**

Run `ros2 run joy joy_node` then `ros2 topic echo /joy`. Press each
control one at a time and note which array index changes. **Get the
deadman right before anything else** — if it's wrong, you have no manual
override and no e-stop, which is the one thing you cannot safely discover
by driving the robot.

### 0.3 Battery thresholds — `base_controller_params.yaml`
`battery_warn_v: 9.6`, `battery_critical_v: 9.0` **[VERIFIED]**

3S LiPo, 3.2 V/cell warn and 3.0 V/cell critical. The scale factor
(`raw/10.0`) is confirmed from `Rosmaster_Lib` source. **Sanity check on
first boot**: `ros2 topic echo /battery_state` on a full pack should read
~12.6 V. If it reads ~1.26 V, the scale factor is wrong for your firmware
revision and the robot will think it's flat at full charge.

---

## STAGE 1 — Chassis geometry (measure once, everything depends on it)

All in `row_navigation_params.yaml`. These are **CAD-derived from the real
`yahboomcar_R2.urdf.xacro`** — good values, but not measured on *your*
assembled unit.

| Parameter | Current | Tag | How to get the real number |
|---|---|---|---|
| `wheelbase` | `0.2353` | **[MEASURE]** | Tape measure, front axle centre to rear axle centre. |
| `max_steer_angle` | `0.6` rad (34.4°) | **[MEASURE]** | Command full steering lock both ways, measure the actual front wheel angle with a protractor. Servos often can't reach the CAD limit once mounted. |
| `row_spacing` | `0.50` | **[MEASURE]** | See Stage 2 — must match two other files. |

`min_turn_radius` is **not a parameter and not in any YAML** — `row_nav_node`
computes it at startup as `R_min = wheelbase / tan(max_steer_angle)`. Fix the
two measurements above and it follows automatically. `row_nav_node` now
also refuses to start if `max_steer_angle` is outside `(0, pi/2)` radians —
previously a `0.0` value reached that division directly and raised a raw
`ZeroDivisionError` with no diagnostic, and a negative or ≥90° value would
have silently produced meaningless turn geometry without crashing at all.

**Why this matters more than it looks**: `wheelbase` and `max_steer_angle`
feed `_bulb_turn_split()`, which solves the headland turn geometry *once at
startup*. Wrong inputs → the robot completes its two-arc turn and lands in
the wrong place, every single time, in a way that looks like "bad control
tuning" but isn't.

**There is deliberately no `headland_turn_angle` parameter.** An earlier
revision had one; it was read by nothing. The 180° reversal is guaranteed
by construction. To change the turn, change `min_turn_radius` (via the two
measurements above) or `row_spacing`.

---

## STAGE 2 — Your actual row layout (Lab 3003, then the polytunnel)

**These three files must agree with each other.** They're separate because
different nodes own them, but a mismatch produces a robot that follows rows
correctly and then plans its coverage route as if the world were different.

| File | Parameter | Current | Tag |
|---|---|---|---|
| `coverage_params.yaml` | `row_spacing` | `0.50` | **[MEASURE]** |
| `target_manager_params.yaml` | `row_spacing` | `0.50` | **[MEASURE]** |
| `row_navigation_params.yaml` | `row_spacing` | `0.50` | **[MEASURE]** — newly surfaced; see note below |
| `row_navigation_params.yaml` | `target_half_width` | `0.20` | **[MEASURE]** |
| `coverage_params.yaml` | `num_rows` | `6` | **[MEASURE]** |
| `coverage_params.yaml` | `row_length` | `8.0` | **[MEASURE]** |

**`row_spacing` now appears in three YAMLs, not two.** It was declared in
`row_nav_node.py` with a `0.50` default but was *missing* from
`row_navigation_params.yaml` — so setting your real spacing in the other two
files would have left row-following silently on `0.50`. It has been added
explicitly. Change all three together.

**`headland_exit_buffer` had the identical gap, since fixed the same way.**
Also declared in `row_nav_node.py` but absent from
`row_navigation_params.yaml` — surfaced explicitly alongside `row_spacing`,
for the same reason: a value silently ignored in the one file most people
would think to edit is worse than a wrong value that at least changes
something.

**`target_half_width` is not the same as `row_spacing`** — it's the
half-width of the corridor RANSAC searches for post lines in. It must stay
comfortably **below** `row_spacing`, or the search window overlaps the
*next* row's posts and RANSAC fits the wrong line. (It was once set equal
to `row_spacing`; that was a real bug, already fixed. Keep the gap.) This
constraint (`2 x target_half_width < row_spacing`) is no longer just a
comment — `row_nav_node` now refuses to start if it's violated, rather than
running with silently overlapping search windows.

**For Lab 3003 with desks standing in for troughs**: measure desk-edge to
desk-edge for `row_spacing`, count the aisles for `num_rows`, and measure
the usable aisle run for `row_length`.

**Headland space needed**: at `min_turn_radius = 0.344 m` and
`row_spacing = 0.50 m`, the turn consumes roughly **1.22 m of clear depth**
beyond the row end. Measure that you actually have it before the first
autonomous run — see `docs/HEADLAND_TURN_GEOMETRY.md`.

---

## STAGE 3 — Sensor calibration

### 3.1 IMU accelerometer units — `base_controller_node.py` (NOT a YAML key)
`G_TO_MS2 = 9.80665` — **a hardcoded Python constant at the top of
`src/base_controller/base_controller/base_controller_node.py`** (~line 105),
deliberately not exposed as a parameter because it is a units correction,
not a tunable. **[MEASURE — genuinely unconfirmed]**

The code assumes `get_accelerometer_data()` returns g's. **This was never
verified against the vendor source** and is the single most likely silent
unit bug in the stack. Test: sit the robot flat and still, then
`ros2 topic echo /imu/data_raw`. Z acceleration should read **≈9.81**. If
it reads ≈1.0, the driver already returns m/s² and this constant must
become `1.0`. If it reads ≈96, it's being double-applied.

Wrong here → EKF fusion is garbage → odometry drifts badly → SLAM fights it.

### 3.2 Camera mounting angle — `strawberry_amr.urdf.xacro`
`cam_tilt: 0.61` rad (35°) **[MEASURE]**

Must match how the camera is *actually* bolted on. Formula:
`tilt = atan2(fruit_height − cam_height, forward_distance)`. Wrong →
detections project to the wrong 3D location even with perfect detection,
because the TF chain is lying about where the camera points.

### 3.3 Camera resolution / rate — `camera.launch.py`
`424×240 @ 15 fps` **[TUNE]**

A starting point, not an optimum. Watch Pi CPU (`htop`) and the WiFi link
during a real run. Raise if there's headroom, drop if the Pi saturates or
frames arrive late.

---

## STAGE 4 — Control tuning (on hardware, robot elevated on blocks first)

`row_navigation_params.yaml`. **All six are [TUNE]** — carried over from an
earlier skid-steer design and never validated on the real Ackermann chassis
at its real operating speed.

| Parameter | Current | What it does |
|---|---|---|
| `Kp_theta` | `1.2` | Proportional on heading error. Raise → snappier, more oscillation. |
| `Ki_theta` | `0.25` | Integral. Raise → kills steady-state offset, adds overshoot/wind-up. |
| `Kd_theta` | `0.30` | Derivative. Raise → damping, amplifies sensor noise. |
| `fopid_lambda` | `0.4` | Fractional **integral** order (λ). |
| `fopid_mu` | `0.7` | Fractional **derivative** order (μ). |
| `k_cross_track` | `0.8` | How hard lateral offset feeds into heading demand. |

**Two ways to tune these:**

1. **Manual** — one at a time, elevated, then at low speed on the floor.
   Start with `Kp_theta` alone (`Ki`/`Kd` to 0), get a stable response,
   then add `Kd`, then `Ki` last.
2. **Automated (RL)** — `docs/GL_FOPID_RL_TUNING_GUIDE.md` searches all six
   in Gazebo. Whatever it exports is a **starting point for the same bench
   test**, not a finished answer — it was tuned against simulated physics,
   not your real chassis mass, motor response, or floor friction.

### Also [TUNE]
| Parameter | Current | Note |
|---|---|---|
| `nominal_linear_speed` | `0.20` m/s | Cruise speed in-row. Already conservative; go lower (0.10–0.15) for first runs. |
| `min_linear_speed` | `0.05` m/s | Ackermann floor — v must stay > 0 to steer at all. |
| `max_angular_speed` | `1.2` rad/s | Clamped *before* bicycle-model conversion. |
| `headland_turn_speed` | `0.5` m/s | Turn speed. Lower if the turn overshoots. |
| `headland_yaw_tol` | `0.08` rad | Tighter → more accurate turn exit, more time spent settling. |
| `max_linear_accel_mps2` | `1.0` | Never limits braking — only acceleration. Lower if the robot lurches on start. |

---

## STAGE 5 — Perception

### 5.1 HSV detector (default — start here) — `plant_detector_params.yaml`
```
h1_lo: 0    h1_hi: 10     h2_lo: 170  h2_hi: 179    (dual red range)
s_lo: 90    v_lo: 60      min_contour_area: 600
```
**All [TUNE], under your actual Lab 3003 lighting.**

The dual hue range exists because red wraps around 0° in HSV — both bands
are needed. Use `hsv_tuner.py` (in your uploads) against a live frame with
the real plant under the real lights. **Do not tune this from a photo taken
elsewhere** — lighting dominates HSV far more than plant appearance does.

`min_contour_area: 600` px filters noise; raise if you get spurious small
detections, lower if real fruit at distance is being missed.



### 5.2 Zero-shot detector (optional, event-gated) — `plant_detector_zeroshot_params.yaml`

**Fires automatically during real missions** — `mission_control_node`
publishes its trigger the instant Nav2 confirms arrival at a plant, when
`detector:='zeroshot'` is set on the launch line. You should not need to
publish the trigger by hand except when testing the detector in
isolation (see `docs/ZERO_SHOT_PERCEPTION_GUIDE.md` Part 6).

| Parameter | Current | Tag |
|---|---|---|
| `mobile_sam_checkpoint` | `""` | **[MUST SET]** — node refuses to start without it |
| `plant_labels` | `["ripe strawberry", "strawberry plant canopy"]` | **[TUNE]** — see `docs/ZERO_SHOT_PERCEPTION_GUIDE.md` Part 5 |
| `weed_labels` | `["weed"]` | **[TUNE]** — your least reliable prompt, expect to iterate |
| `box_threshold` / `text_threshold` | `0.35` / `0.25` | **[TUNE]** — start permissive, tighten later |
| `max_plant_extent_m` / `min_plant_extent_m` | `1.0` / `0.01` | **[TUNE]** — the geometric plausibility gate; widen if real plants are being rejected as "too large" |
| `force_cuda` | `true` | Leave true. `false` only to isolate a model problem from a CUDA setup problem. |

This detector does not run continuously — it fires once per message on
`trigger_topic` (default `/zeroshot_detect_trigger`), not on a timer and
not automatically alongside HSV. See the guide's Part 0 before assuming
it behaves like HSV.

---

## STAGE 6 — Network & paths

| File | Parameter | Current | Note |
|---|---|---|---|
| `setup_network_pi_orin.sh` | SSID | `StrawberryAMR` | Change if it clashes locally. |
| `setup_network_pi_orin.sh` | band | 2.4 GHz default | **Keep 2.4 GHz for the polytunnel** — better range through plastic/metal hoops. 5 GHz is fine for short-range lab work. |
| `mission_control_params.yaml` | `map_save_path` | `~/ros2_ws/maps/tunnel_map` | **[MUST SET]** if your workspace isn't `~/ros2_ws`. |
| `semantic_mapper_params.yaml` | `csv_path` | `~/ros2_ws/maps/semantic_targets.csv` | Same — and **must match** `target_manager`'s `initial_csv_path`, or Phase 2 loads nothing. |
| `slam_toolbox_localization.yaml` | `map_file_name` | — | **[MUST SET]** before Phase 2 — point at the `.posegraph` saved at the end of Phase 1. |

---

## The order I'd actually do this in

1. **Stage 0** — udev, PS2 mapping, battery sanity. Nothing else is safe first.
2. **Stage 1** — measure the chassis. Ten minutes with a tape measure.
3. **Stage 3.1** — IMU units. One `topic echo`, catches the nastiest silent bug.
4. **Bench test elevated** (`ALL_IN_ONE_DEPLOYMENT_GUIDE.md` Part 11) — confirm wheels/steering/encoders respond correctly *before* the robot can drive away from you.
5. **Stage 2** — measure the room, make the three files agree.
6. **Stage 4** — control tuning, low speed, floor.
7. **Stage 5.1** — HSV under real lights.
8. Only then: Nav2 autonomous approach, and optionally GL-FOPID RL tuning.

**The single highest-value hour** is steps 1–4. Everything downstream
assumes the geometry is right and the sensors report what the code thinks
they report — and every one of those is a silent failure, not a crash.
