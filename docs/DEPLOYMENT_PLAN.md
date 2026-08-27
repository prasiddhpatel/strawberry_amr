> **SUPERSEDED.** This plan was written for the earlier single-board
> (Jetson AGX Orin, 4WD skid-steer) architecture. The project has since
> moved to the Ackermann ROSMASTER R2 chassis on a dual-Raspberry-Pi rig —
> **use [`docs/MASTER_DEPLOYMENT_COMMANDS.md`](MASTER_DEPLOYMENT_COMMANDS.md) for
> the current, correct deployment procedure.** Kept here as project history
> / thesis evidence of the architecture's evolution, not as a live document
> — do not follow the hardware assumptions below on the current build.

# 4-Week Field Deployment Plan — Strawberry Tabletop AMR (ORIGINAL, AGX Orin era)

Staged, gate-driven. **Do not advance until the current gate passes.** Each week
ends with recorded evidence (rosbag + notes) for your thesis evaluation.

**Hardware reality this plan now assumes:** Jetson AGX Orin (30 W) · Yahboom
YB-ERF01 driver board → 2× Cytron MDD10A (left/right-paired) · 3S LiPo (11.1 V,
6500 mAh) · hardware E-stop (NC) that cuts the **whole** VCC rail (motors **and**
compute) · RPLIDAR C1, Astra Pro Plus, and the YB-ERF01 all on **one USB hub**.

**Golden rule:** the robot moves only on stands until odometry and the safety
chain are proven (Gate 3). Both e-stops are verified before any ground motion.

---

## Week 1 — Bring-up & plumbing (make it build, make it stop, make it talk)

**Goal:** workspace builds; the **YB-ERF01 serial contract is proven**; robot is
driveable by PS2 on stands; both e-stops verified; odometry trustworthy.

- **Day 1 — Build & devices.**
  - `install_deps.sh`, install the Astra driver, `setup_udev_rules.sh` (fix
    VID/PID from `lsusb`/`udevadm`), `colcon build --symlink-install`.
  - Because all three devices share one hub: confirm `/dev/rplidar`,
    `/dev/robot_base`, and the Astra all enumerate **stably across reboots**. If
    the LiDAR and YB-ERF01 share a USB-serial chip, pin them with `ATTRS{serial}`.
  - **Gate 0a:** `ros2 topic echo /joy` shows axis/button changes → fill in
    `ps2_mapping.yaml`.
- **Day 2 — YB-ERF01 serial contract (the critical unknown).**
  - `cat /dev/robot_base` and read the raw lines the board emits. Set
    `base_controller_params.yaml`: `odom_source` (`pose` vs `wheel_vel` vs
    `wheel_ticks`), `cmd_mode` (`twist` vs `wheel`), and the line `*_prefix`s to
    match. If the board speaks a binary/other protocol, either flash firmware
    that emits this CSV contract or write a thin adapter.
  - With wheels **off the ground**: command small `/cmd_vel` and confirm the
    correct side spins the correct direction; confirm `/wheel_odom` moves
    sensibly and `/battery_state` reads ~11.1 V.
  - **Gate 0b:** commands reach the motors and odometry/telemetry parse cleanly.
- **Day 3 — Both e-stops.**
  - On stands, `teleop.launch.py`. Confirm deadman (L1) gates motion, software
    e-stop (Circle) latches, Start clears.
  - **Gate 1 (safety):** (i) **hardware** E-stop pressed → every wheel dead and
    the Jetson powers down; on re-power the stack comes back with
    `autostart:=false` and does **not** move until re-enabled. (ii) **software**
    e-stop latched → no wheel turns. (iii) kill the teleop node → wheels stop
    within 0.5 s (watchdog). (iv) with autonomy enabled, trigger the software
    e-stop, then clear it with Start — confirm autonomous driving does **not**
    silently resume; autonomy must be re-enabled as a separate, deliberate
    step. (v) separately, `safety_supervisor`'s own LiDAR-obstacle `/e_stop`
    latches independently of the joypad and needs its own explicit
    `/e_stop_reset` publish to clear — see `docs/MASTER_DEPLOYMENT_COMMANDS.md`.
- **Day 4 — Odometry truth.**
  - Push the robot a measured 5.00 m straight + a full 360°. Tune
    `wheel_radius`, `wheel_base`, `ticks_per_rev` until `/wheel_odom` matches the
    tape within ~3–5%. (Skid-steer point-turns slip — that's expected; the IMU
    carries yaw in the EKF.)
- **Day 5 — TF + EKF.**
  - `ros2 run tf2_tools view_frames`: confirm `map→odom→base_link→{laser,imu,
    camera_*}`, single parent each, no loops. (Swap in your **real URDF** here if
    you have it.) Start EKF; drive slowly on the ground (deadman held) and
    confirm `/odometry/filtered` is smooth and drift-bounded.
  - **Gate 2:** full TF tree; EKF stable; both e-stops proven on the ground.

**Deliverable:** rosbag of a teleop drive with clean TF + odometry; note proving
both e-stops; the finalised YB-ERF01 param settings.

---

## Week 2 — Structure perception: LiDAR, SLAM, row following

**Goal:** robot follows an empty aisle autonomously with the FOPID engaged and
turns at the headland. (See the FOPID/corridor tuning notes you have — same gains.)

- **Day 1 — LiDAR + SLAM.** `bench.launch.py`; confirm dense `/scan` and a stable
  SLAM map while teleoping a loop. Check `max_laser_range` vs your tunnel.
- **Day 2 — Corridor estimator (static).** Park between rows; echo
  `/row_heading_error` + `/row_lateral_offset`; verify signs by hand. Set
  `target_half_width` = half your clear aisle; tune `side_gate`, `fwd_window_*`,
  `num_bins` so posts are caught both sides. **Gate 3:** on stands, enable
  autonomy and confirm `/cmd_vel_auto` reacts sanely to a hand-moved board.
- **Day 3 — First moving row-follow (empty aisle).** Spotter on the hardware
  E-stop. Tune `Kp_theta`, then `k_cross_track`, then `Ki/Kd` + `lambda/mu`.
  Target lateral RMS < 50 mm, heading RMS < 0.1 rad.
- **Day 4 — Safety gate in the loop.** Confirm `safety_supervisor` slows then
  stops on an aisle obstacle; tune `stop_distance`/`slow_distance`/`sector_half_deg`.
- **Day 5 — Headland turn.** Confirm `follow→row_end→turning→turn_complete→
  reacquire→follow`. Tune for **reacquisition**, not sub-degree accuracy (the
  turn is open-loop on EKF yaw). **Verify clearance** for your row pitch first;
  if too tight, fall back to manual turns and document it. **Gate 4:** one aisle
  autonomous + one headland turn.

**Deliverable:** rosbag of an autonomous empty-aisle run + lateral/heading plot +
a headland-turn clip.

---

## Week 3 — Fruit perception + integrated runs

**Goal:** ripe-fruit detections projected into the map; first integrated runs in
one bay with everything on. (Classical HSV — no training.)

- **Day 1 — Camera + tilt.** Aim the Astra at the overhead fruiting zone; set
  `cam_tilt`/`mast_*` in the xacro; verify the optical frame in RViz. Enable
  depth→colour registration in the Astra driver.
- **Day 2 — HSV tuning (no training).** Use `/plant_detector/debug_image` in
  `rqt_image_view` to set `h1/h2`, `s_lo`, `v_lo`, `min_contour_area`,
  `roi_top_fraction` live until ripe fruit lights up and foliage/unripe doesn't.
- **Day 3 — Map projection.** Drive slowly; confirm `semantic_mapper` writes
  `semantic_targets.csv` in the **map** frame and `target_manager` publishes
  `/selected_plant_goal`. Spot-check a few targets vs tape.
- **Day 4–5 — Integrated run, one bay.** `full_robot.launch.py use_camera:=true`.
  Watch **Orin thermals/CPU at 30 W** — if perception starves nav, drop camera
  FPS or detector rate. **Gate 5:** one bay autonomous with a populated map.

**Deliverable:** the semantic CSV + an annotated detection clip + a localisation-
accuracy note.

---

## Week 4 — Repeatability, metrics, robustness, buffer

**Goal:** repeatable multi-row runs and the data your evaluation needs.

- **Day 1–2 — Repeatability.** Run the same multi-row mission ≥5× (`rows_to_cover`).
  Record success/intervention per run. Target > 80%.
- **Day 3 — Metrics.** Lateral RMS, heading RMS, SLAM revisit drift, fruit-
  position error vs ground truth, and **endurance**: the 3S pack is ~72 Wh, of
  which ~58 Wh is usable above the 3.0 V/cell floor; with the Orin at 30 W plus
  drive load, **measure** real runtime/range rather than assuming. Watch
  `/battery_state`; stop at the warn voltage.
- **Day 4 — Robustness (incl. the shared USB hub).** Inject sensor dropout
  (unplug the LiDAR briefly) and a stale-command condition, and confirm the
  watchdogs halt the robot — the single hub means LiDAR+camera+motor-comms can
  drop **together**, so this matters. Condensation check on lenses/connectors for
  wet-Ireland conditions.
- **Day 5 — Buffer + write-up.** Absorb slips, re-run weak gates, freeze
  parameters, `git tag v1.0-field`, export plots.

**Deliverable:** evaluation table (RMS errors, success rate, measured endurance) +
final tagged release.

---

## Staged gate summary

| Gate | Pass criterion |
|---|---|
| 0a | Builds; joypad mapped; devices enumerate stably on the shared hub |
| 0b | **YB-ERF01 contract proven**: commands reach motors, telemetry parses |
| 1 | HW E-stop kills motors+compute & safe restart; SW e-stop stops; watchdog stops on node kill |
| 2 | Full single-parent TF tree; EKF stable; both e-stops proven on ground |
| 3 | Autonomy produces sane `/cmd_vel_auto`; safety gate reacts |
| 4 | One empty aisle autonomous + one headland turn |
| 5 | One bay autonomous with populated semantic map |
| Final | ≥5 repeats, >80% success, metrics + measured endurance recorded |

## De-risking
Mirror the nav stack in **Gazebo or Isaac Sim** (classical, **no training**)
before field days to debug the corridor estimator, FOPID, and headland logic
without risking hardware or waiting on weather.
