# Sensor Calibration & Bring-Up Guide

**Read this before you launch the full stack.** Every sensor gets verified
alone first, then in pairs, before you ever run everything together. This
is not caution for its own sake — when four sensors and three coordinate
frames are all live at once and something looks wrong, you have no way to
know which one is lying to you. Verified one at a time, a problem can only
be in the thing you just touched.

Rough time budget: chassis measurement ~20 min, each sensor solo ~20-30
min, fusion steps ~30-45 min each. A careful first pass through this whole
document is a half-day, not a week — it is the foundation the rest of the
week's testing stands on, so do not rush it to save an afternoon.

---

## Part 0 — Tools you need before starting

- Tape measure (mm markings) or steel ruler
- A protractor, angle finder, or a smartphone level/angle app (most phones
  have one built into the compass or a free app — "Bubble Level" /
  "Clinometer" style apps are accurate to ~0.5°, plenty good enough here)
- A large flat sheet of paper or cardboard bigger than the robot's footprint,
  and a pencil
- A Orin with RViz2 and `rqt_image_view` (`sudo apt install ros-humble-rqt-image-view`
  if missing) on the same ROS_DOMAIN_ID/network as the Pi (see
  `docs/MASTER_DEPLOYMENT_COMMANDS.md` Stage 5)

---

## Part 1 — Chassis geometry confirmation

**This changed since an earlier revision of this guide.** The chassis
geometry (wheelbase, front/rear track, max steering angle) is now
CAD-sourced from the real `yahboomcar_R2.urdf.xacro` in Yahboom's own
sample-code archive — see `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`,
"Ackermann geometry," for the extraction and the numbers already baked
into the codebase (`wheelbase=0.2353 m`, `front_track=0.130 m`,
`rear_track=0.1685 m`, `max_steer=0.6 rad`). You do **not** need to
measure these from scratch the way an earlier revision of this guide
required — but a CAD joint limit is a design value, not a guarantee about
your specific assembled unit (servo travel, linkage slop, and assembly
tolerance can all shift the real achievable range slightly), so a quick
bench **confirmation** of the one figure most likely to drift —
`max_steer` — is still worth doing before fully trusting `R_min`-derived
headland-turn geometry for unattended operation. That's what 1.1 below is
for; it's now a confirmation step, not a from-scratch measurement.

### 1.1 Confirm the maximum steering angle (δ_max)

1. With the robot powered and `base_controller` running (`ros2 launch
   robot_bringup teleop.launch.py` is the simplest way — see
   `docs/MASTER_DEPLOYMENT_COMMANDS.md` Stage 9), command a steady hard-left turn from
   the joypad (small forward speed, full left stick) and hold it. The
   front wheels will settle at the servo's mechanical or software limit.
2. Lay the protractor/angle app against the **chassis centreline** and
   against the **plane of the front wheel**, and read the angle between
   them. Repeat for hard-right.
3. Compare against the CAD value (0.6 rad / 34.4 deg). If your reading is
   meaningfully smaller on either side, **use the smaller of your two
   bench readings** as `max_steer_angle`, not the CAD figure — using a
   larger value than the chassis can actually deliver silently distorts
   the bulb-turn geometry rather than failing loudly. If your readings are
   close to 34.4 deg, the CAD value is confirmed and you don't need to
   change anything.
4. Convert degrees to radians (`rad = deg * pi / 180`) before entering any
   updated value.

### 1.2 Optional: independently confirm wheelbase and track

If you want a fully independent check rather than trusting the CAD
extraction's arithmetic (reasonable, given this project's own standard of
not treating "internally consistent" as "confirmed"): place the robot on
the sheet of paper, front wheels straight, mark the ground contact patch
of all four wheels with a pencil held vertically, remove the robot, and
measure between the dots — front-axle-midpoint to rear-axle-midpoint for
wheelbase, dot-to-dot laterally for each track. Compare against the CAD
figures (wheelbase 0.2353 m, front track 0.130 m, rear track 0.1685 m).

### 1.3 Wheel radius

Measure the tyre's outer diameter with the tape measure across the tread
(not the rim), divide by 2. The 65 mm diameter (32.5 mm radius) already in
the codebase is independently corroborated by Yahboom's own Q&A, so this
one is the most likely to already be correct — measure anyway, it takes 30
seconds and confirms it rather than assumes it.

### 1.4 Where to put updated numbers, if any of the above changed something

Record any measurement that meaningfully differs from the CAD value, then
update **both** of the following together — they must agree:

| File | Parameter(s) |
|---|---|
| `src/robot_description/urdf/strawberry_amr.urdf.xacro` | `wheelbase`, `front_track`, `rear_track`, `wheel_r`, `max_steer` (top of file) |
| `src/row_navigation/config/row_navigation_params.yaml` | `wheelbase`, `max_steer_angle`, `row_spacing` (row_spacing is tunnel-dependent, not chassis-dependent — leave unless you're also changing aisle width) |

(An earlier revision of this table also listed `base_controller_params.yaml`'s
`rear_track_width` and a separate `ackermann_bridge_params.yaml` — neither
applies anymore: `base_controller` now gets odometry directly from
Rosmaster_Lib's `get_motion_data()` rather than computing it from a
configured track width, and `ackermann_bridge` was removed entirely, its
role folded into `base_controller` via `set_car_motion()` — see
`docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`'s "Rosmaster_Lib" section.)

After editing, `colcon build --symlink-install` again and re-check
`row_nav_node`'s startup log line (`Ackermann bulb-turn solved at
startup: R_min=...`) — confirm the printed `R_min` matches
`your_L / tan(your_max_steer)` by hand on a calculator. If it doesn't,
one of the edits above didn't take.

---

## Part 2 — LiDAR (RPLIDAR A1M8) solo bring-up

### 2.1 Bring it up alone

```bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/rplidar -p serial_baudrate:=115200 -p frame_id:=laser
```

In a second terminal:
```bash
ros2 topic hz /scan          # expect ~5.5-10 Hz, steady, no long gaps
ros2 topic echo /scan --once # sanity-check ranges are plausible numbers,
                              # not all zeros or all inf
```

If `/scan` never appears or the node errors on startup, the most likely
causes in order: wrong `/dev/rplidar` symlink (re-run
`scripts/setup_udev_rules.sh`, check `ls -l /dev/rplidar`), wrong baud rate
(try `57600` as a fallback — see `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`),
or a permissions issue (`sudo usermod -aG dialout $USER`, then log out/in).

### 2.2 Visualize and check orientation

Open RViz2 on your Orin, set Fixed Frame to `laser`, add a `LaserScan`
display on `/scan`. Place a single distinct object (a box, your hand)
directly in front of the sensor's forward-facing side and confirm the
return appears at bearing ≈ 0° in the display, not off to one side or
behind. If it's rotated 90°/180° from what you expect, the physical mount
orientation doesn't match `laser_joint`'s assumed orientation in the
URDF — either physically re-orient the sensor or add a rotation to
`laser_joint`'s `rpy` to match reality (physically re-orienting is usually
simpler and less error-prone than compensating in software).

### 2.3 Distance accuracy spot-check

Tape-measure a flat object to exactly 1.000 m from the sensor's front
face, and again at 3.000 m. Compare against `ros2 topic echo /scan` at the
corresponding bearing. Agreement within a few cm is a reasonable bench
tolerance for this sensor class — this is a sanity check, not a metrology
calibration.

### 2.4 The one that actually matters for this deployment: light sensitivity

The A1M8 is a **triangulation** sensor, and Slamtec's own datasheet
qualifies outdoor use as working "without direct sunlight exposure" — see
`docs/MOTOR_AND_GEOMETRY_VERIFICATION.md` for the full sourcing. Do this
test before you invest further integration time on the assumption it'll
be fine outdoors or in a bright polytunnel:

1. Run the scan in normal indoor room lighting; note scan quality
   (`ros2 topic echo /scan` — look for `inf`/`nan` dropout rate, or watch
   the RViz display for gaps/noise on known flat surfaces).
2. Repeat under the brightest light you can access — near a sunlit window,
   under a strong work light, or (best, if you can get even a brief test)
   actually inside a polytunnel or under open sky.
3. Compare. If dropout or noise increases substantially in bright
   conditions, that's the real operational risk from the C1→A1M8 swap
   materializing, and it needs a mitigation plan (shading the sensor
   housing, testing at different times of day, accepting reduced range in
   bright conditions) before you rely on it for autonomous row-following.
   If it holds up fine, good — but verify, don't assume either way.

---

## Part 3 — IMU (MPU-9250) solo bring-up

The MPU-9250 is read by the YB-ERF01 board and forwarded over the same
serial link as the encoders (see `base_controller_node.py`'s protocol
docstring), so "solo" here means: `base_controller` running, everything
else off.

### 3.1 Bring it up alone

```bash
ros2 run base_controller base_controller_node --ros-args \
  --params-file src/base_controller/config/base_controller_params.yaml
ros2 topic echo /imu/data_raw
```

### 3.2 Stationary bias check

With the robot completely still on a flat, level surface:
- **Gyroscope** (`angular_velocity` field): all three axes should read
  close to zero — a few hundredths of a rad/s is normal sensor noise, but
  a consistent, much-larger offset on any axis is a bias worth knowing
  about (the EKF's process noise covariance in `ekf.yaml` implicitly
  assumes small, noise-like errors, not a large constant offset).
- **Accelerometer** (`linear_acceleration` field): the axis pointing
  straight up should read close to ±9.8 m/s² (sign depends on mounting
  orientation and REP-103 convention), and the other two axes should read
  close to zero. If a supposedly-horizontal axis shows a significant
  non-zero reading, the IMU is physically mounted at a tilt — remeasure
  and correct `imu_joint`'s `rpy` in the URDF, don't just accept the
  offset. **This check also resolves a specific, explicitly-flagged
  uncertainty in the code**: `base_controller_node.py` assumes
  Rosmaster_Lib's raw accelerometer output is in g's and multiplies by
  9.80665 to get m/s² (its `G_TO_MS2` constant), because the vendored
  library's source doesn't state the unit. If the vertical axis reads
  close to **9.8**, that assumption is confirmed, nothing to change. If it
  instead reads close to **1.0**, the raw value was already in m/s² and
  `G_TO_MS2` in `base_controller_node.py` should be changed from `9.80665`
  to `1.0`.

### 3.3 Yaw sanity check

Rotate the robot by hand through a known angle (e.g., align it with a
90° mark made with tape on the floor) and compare against the yaw change
reported by the Madgwick-filtered `/imu/data` topic (not the raw topic —
the filter integrates gyro into an orientation estimate). Reasonable
agreement (within a few degrees over a single 90° turn) confirms the
sensor, its mounting axis convention, and the filter are all consistent.
Large disagreement means one of: wrong axis in `imu_filter.yaml`, wrong
mounting orientation, or a scale-factor problem — work through those in
that order.

### 3.4 Magnetometer: deliberately not calibrated

The magnetometer is intentionally excluded from this whole system
(`imu_filter.yaml`: `use_mag: false`) because the drive motors corrupt its
readings at this proximity — this is an established design decision, not
an oversight, and there is nothing to calibrate here. Don't add magnetometer
calibration steps to your process; it would produce numbers that are
discarded anyway.

---

## Part 4 — Camera (Orbbec Astra Pro Plus) solo bring-up

### 4.1 Bring it up alone

```bash
ros2 launch sensor_bringup camera.launch.py
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
rqt_image_view    # select /camera/color/image_raw, then /camera/depth/image_raw
```

Confirm both streams are live at a steady rate and look sensible — a
recognizable colour image, and a depth image where nearby objects and far
objects are visually distinguishable (not a uniform flat colour, which
usually means depth data isn't actually arriving).

### 4.2 Intrinsic calibration (verify, don't necessarily redo)

RGB-D cameras like the Astra Pro Plus ship factory-calibrated; the driver
publishes this via `/camera/color/camera_info` and
`/camera/depth/camera_info`. Check these are publishing non-zero, sensible
values (`ros2 topic echo /camera/color/camera_info --once` — focal lengths
`fx`/`fy` and principal point `cx`/`cy` should be non-zero and roughly
matching the image resolution). A full checkerboard recalibration
(`ros-humble-camera-calibration`) is available if something looks visibly
wrong (obvious barrel distortion, warped straight lines), but is usually
unnecessary for a factory unit.

### 4.3 Extrinsic calibration — the one that actually matters here

**For what angle to actually target and why, see the dedicated
`docs/CAMERA_ANGLE_AND_HSV_CALIBRATION_GUIDE.md`** — this section covers
the physical measurement procedure only.

This is the camera-to-`base_link` transform (`mast_x`, `mast_z`,
`cam_tilt` in the URDF), and it directly determines whether
`plant_perception`'s 3D back-projection of a detected plant lands in the
right place in the map. It has been a placeholder ("EDIT after calibration
on the real chassis") since it was first written — now is the time.

1. With `base_link`'s origin defined as the wheelbase midpoint at ground
   level (per the URDF's own header comment), measure with a tape measure:
   - **`mast_x`**: horizontal distance from the wheelbase midpoint forward
     to the camera's optical centre (approximately the front glass of the
     lens housing).
   - **`mast_z`**: vertical height from the ground to the camera's optical
     centre.
2. **`cam_tilt`**: the camera is deliberately tilted upward toward the
   overhead fruiting zone. Rest a protractor/phone angle app flat against
   the top or front face of the camera housing (whichever is parallel to
   the lens's optical axis on your specific mount) and read the tilt angle
   from horizontal. Convert to radians; the sign convention in the URDF is
   negative pitch = tilted up (check the existing `cam_tilt` usage in
   `camera_joint`'s `rpy` before assuming sign).
3. Update `mast_x`, `mast_z`, `cam_tilt` in
   `strawberry_amr.urdf.xacro`, rebuild, and re-check in RViz2: add the
   camera's TF frame and a point-cloud or image display, and confirm the
   visualized camera frustum points toward where you'd physically expect
   it to (up and forward, roughly at the height ripe fruit would hang).

### 4.4 Depth-to-colour alignment check

`camera.launch.py` sets `depth_registration: True`, which aligns the depth
image to the colour image's viewpoint — required for `plant_perception`'s
pixel-to-3D back-projection to be correct (a detection found at a given
colour-image pixel is assumed to have its depth at the *same* pixel
coordinate in the depth image). Quick check: point the camera at a scene
with a clear near/far edge (e.g., your hand in front of a wall), and in
`rqt_image_view` compare the edge position in the colour image against the
edge position in the depth image — they should line up. A visible offset
between the two means registration isn't working and needs the driver
setting rechecked before trusting any 3D detection position.

---

## Part 5 — Fusion, in stages (only after every Part 1-4 check above passes)

Do not skip straight to the full stack. Each stage below adds exactly one
new thing to what you already trust from the previous stage, so a problem
can only be in the thing you just added.

### 5.1 Wheel odometry + IMU → EKF

```bash
ros2 launch robot_bringup bench.launch.py
```
(This brings up drivetrain + EKF + SLAM + row-following, but for this
step ignore everything except `/odometry/filtered`.)

Push the robot by hand (autonomy disabled) through a simple known path —
1 m straight, then a 90° turn, using floor tape marks as ground truth.
Compare the filtered odometry's reported displacement and heading change
against your ground-truth measurement. Reasonable agreement (a few cm and
a few degrees over this short a path) confirms the wheel+IMU fusion is
sane before SLAM is layered on top of it.

**`/wheel_odom`'s covariance fields are populated placeholders, not
measured values.** They were previously left at zero, which `robot_localization`
reads as a claim of perfect certainty and can visibly overweight wheel
odometry against the IMU during fusion. They now carry stated-as-such
estimates (see the `ODOM_*_VARIANCE` constants at the top of
`base_controller_node.py`) so the EKF has *some* uncertainty signal rather
than none, but they have not been derived from actual drift
characterization on this unit. If 5.1's agreement check is worse than
expected, treating these as a tuning knob — not just `ekf.yaml`'s own
process noise — is a reasonable next step.

### 5.2 + LiDAR → SLAM

Still within `bench.launch.py` (it already includes `slam_toolbox`), drive
or push the robot around a small, known space (a room, or a taped-out
rectangle). Check the resulting map in RViz2 (`/map`): walls should appear
straight, corners of a rectangular room should be close to 90°, and the
map should not visibly warp or double up a wall as you complete a loop.
This validates LiDAR+odometry fusion together.

### 5.3 + Camera → plant perception

Bring up `camera.launch.py` and `plant_perception` alongside the above.
Place a few proxy plants (or any consistently-coloured target the HSV
detector is tuned for) at known, tape-measured positions relative to the
map origin. Confirm `semantic_mapper`'s logged detections
(`~/ros2_ws/maps/semantic_targets.csv`) land within a reasonable tolerance
of their real positions. Disagreement here is usually the Part 4.3
extrinsic calibration, not the detector itself — re-check that first.

### 5.4 Full stack

Only now, `ros2 launch robot_bringup full_robot.launch.py` (single-host),
or the real field topology — Pi + AGX Orin
(`docs/MASTER_DEPLOYMENT_COMMANDS.md` Stage 13) or, on the legacy build,
the two-Pi deployment (`DEPLOYMENT_GUIDE.md` Sections 6-7). If something
breaks only when everything runs together that didn't break in any
earlier stage, suspect resource contention (RAM/CPU on the Pi — see the launch files' own
comments on why Phase 1/Phase 2 don't run SLAM and Nav2 simultaneously) or
a topic-name collision, not a sensor problem — you already proved each
sensor works alone and each fusion pair works, so the new failure mode is
almost certainly about running things *together*, not about any one piece.

---

## Part 6 — Teleoperation (Yahboom PS2-type controller)

The exact button/axis indices vary by which specific 2.4G receiver/adapter
Yahboom shipped with your unit, so rather than assume a mapping, determine
yours directly — it takes two minutes and is more reliable than trusting a
table that might be for a different hardware revision:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

Press each button and move each stick one at a time, watching which index
in the `buttons` or `axes` array changes. Fill in
`src/teleop_ps2/config/ps2_mapping.yaml` with what you actually observe —
the file's own comments describe what each parameter controls (deadman,
autonomy toggle, e-stop, reset, turbo). `teleop.launch.py`'s default
`max_linear: 0.30` is deliberately conservative for a first power-on test;
raise it once you've confirmed the drivetrain responds correctly and
you're comfortable with it, per that file's own docstring.
