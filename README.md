# Strawberry Tabletop AMR — ROS 2 Humble Workspace

Autonomous **Ackermann-steered** mobile base (Yahboom ROSMASTER R2 chassis)
for **substrate tabletop** strawberry polytunnels: LiDAR-guided row following
between table support posts, a verified two-arc Ackermann headland turn,
optional ripe-fruit perception (classical HSV, or an event-gated
zero-shot open-vocabulary layer), PS2-joypad manual control, and a
two-phase (map, then navigate) autonomous mission across a **Raspberry
Pi 4B + NVIDIA Jetson AGX Orin 64GB** compute split (a legacy dual
Raspberry Pi 4B topology is also retained — see the Compute row below).
No arm/harvest logic — the 5-DOF arm is a separate, not-yet-confirmed
companion project (see Section 4).

**Start here:
[`docs/MASTER_DEPLOYMENT_COMMANDS.md`](docs/MASTER_DEPLOYMENT_COMMANDS.md)**
— every bash command, start to finish, one continuous sequence: OS
install on both machines, ROS 2 Humble, Gazebo, every dependency, the
build, network/udev setup, hardware verification, every calibration
step, and all three operating modes, in order. **Or**
[`ALL_IN_ONE_DEPLOYMENT_GUIDE.md`](ALL_IN_ONE_DEPLOYMENT_GUIDE.md) — the
same procedure with the full reasoning behind each step, for when you
want to understand *why*, not just *what to type*. The specialist guides
below remain the deeper
reference for each topic (troubleshooting depth, edge cases, the full
reasoning behind each design choice) — you shouldn't need to leave the
master commands guide on the critical path, but these are where to go if
something's not behaving as expected:
[`docs/CURRENT_STATUS_AND_NEXT_STEPS.md`](docs/CURRENT_STATUS_AND_NEXT_STEPS.md)
(honest status and a day-by-day plan; partially superseded, see its own notice),
[`docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md`](docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md),
[`docs/CAMERA_ANGLE_AND_HSV_CALIBRATION_GUIDE.md`](docs/CAMERA_ANGLE_AND_HSV_CALIBRATION_GUIDE.md),
[`GAZEBO_SIMULATION_GUIDE.md`](GAZEBO_SIMULATION_GUIDE.md),
[`LAB_3003_MOCK_PLANT_AMR_DEPLOYMENT_GUIDE.md`](LAB_3003_MOCK_PLANT_AMR_DEPLOYMENT_GUIDE.md),
and
[`docs/ORIN_PI_SPLIT_ARCHITECTURE.md`](docs/ORIN_PI_SPLIT_ARCHITECTURE.md)
for the full reasoning behind the primary Pi+Orin topology — node
placement, network, safety trade-offs, and the new continuous
explore-and-map mode for a genuinely
unknown tunnel,
[`SKILLS_REVIEW_FINDINGS.md`](SKILLS_REVIEW_FINDINGS.md), which documents
a structured review pass against a published robotics best-practices
skill set — what was found and fixed, what was already sound, and what
was deliberately left out of scope with reasoning,
[`docs/GL_FOPID_RL_TUNING_GUIDE.md`](docs/GL_FOPID_RL_TUNING_GUIDE.md),
reinforcement-learning search for `row_navigation`'s existing GL-FOPID
gains via Gazebo,
[`docs/JETPACK_SETUP_GUIDE.md`](docs/JETPACK_SETUP_GUIDE.md)
for flashing/confirming JetPack 6.x on the Orin, and
[`docs/THESIS_TIMELINE_GUIDE.md`](docs/THESIS_TIMELINE_GUIDE.md), a
realistic week-by-week plan covering the full scope through submission, and
**[`docs/PARAMETER_TUNING_CHECKLIST.md`](docs/PARAMETER_TUNING_CHECKLIST.md)
— read this one before you power the robot on**: every value you actually
need to measure, set, or tune, ordered by when you hit it, with each one
tagged as verified / measure-it-yourself / tune-on-hardware / must-set.
**For a complete written account of the system** — architecture,
hardware verification trail, every bug found and how, and the honest list
of what is *not* verified — see
[`docs/TECHNICAL_REPORT.md`](docs/TECHNICAL_REPORT.md). It also documents
**four verified divergences between this build and the thesis report** —
chiefly skid-steer (thesis) vs Ackermann (built) — that should be
reconciled before submission.

Optional perception enhancement (post-commissioning, training-free):
[`docs/ZERO_SHOT_PERCEPTION_GUIDE.md`](docs/ZERO_SHOT_PERCEPTION_GUIDE.md)
— Grounding DINO + MobileSAM, free Apache-2.0 weights, adds
plant-versus-weed discrimination that HSV cannot do.

**Set the Orin up first** via
[`docs/JETPACK_SETUP_GUIDE.md`](docs/JETPACK_SETUP_GUIDE.md) — which
JetPack/Ubuntu version you need and why, and the one upgrade path that
will break your board if you take it. **Then, for everything this
workspace's authoring environment could never verify itself** (real
CUDA/GPU inference, compiling the C++ safety node, real ROS 2
integration tests, a real Gazebo RL training run) — see
[`docs/CLAUDE_CODE_VERIFICATION_GUIDE.md`](docs/CLAUDE_CODE_VERIFICATION_GUIDE.md)
for a structured plan and concrete prompts for closing those gaps with
Claude Code running against the real hardware. This README is a
reference/overview; those guides are the procedure.

## Hardware

Two supported compute topologies — pick the section of
`docs/ORIN_PI_SPLIT_ARCHITECTURE.md` or the dual-Pi launch files that
matches what you actually have:

| Subsystem | Part |
|---|---|
| Battery | 3S LiPo, 11.1 V nominal |
| Main power chain | main toggle → **E-stop (NC) in series** → protected VCC rail |
| Compute | **Primary**: 1× Raspberry Pi 4B 4GB (on-robot I/O) + 1× **NVIDIA Jetson AGX Orin 64GB** (off-robot compute core, operator station, and visualization) — all sensors/actuation on the Pi, everything compute-heavy plus RViz2 on the Orin, linked by WiFi since the robot moves and the Orin doesn't. See `docs/ORIN_PI_SPLIT_ARCHITECTURE.md`. **Legacy**: 2× Raspberry Pi 4B (2GB + 4GB) linked by wired Ethernet, both on the robot — launch files retained, see `DEPLOYMENT_GUIDE.md`. Either way, ROS 2 Humble over CycloneDDS. |
| Chassis | Yahboom **ROSMASTER R2** — Ackermann (front-steer, rear-drive) |
| Drive motors | 2× **JGB37-520** rear motors (11 PPR base × 1:19 gearbox = 836 CPR verified, 550±10 RPM) |
| Steering | 1× **YB-P20M** digital servo (25 kg·cm), front axle |
| Low-level board | **Yahboom YB-ERF01-V2.0** (STM32F103RCT6) — driven via Yahboom's own **Rosmaster_Lib** Python library (vendored, see Section 5), not a custom protocol |
| IMU | MPU-9250 (onboard the YB-ERF01-V2.0, I2C 0x68), read via Rosmaster_Lib |
| USB | mission-brain Pi → Yahboom USB 3.0 hub → RPLIDAR A1M8, Orbbec Astra Pro Plus |
| LiDAR | RPLIDAR A1M8 (2D, triangulation — see calibration guide re: ambient light) |
| Camera | Orbbec Astra Pro Plus (depth) — tilted up at the overhead fruiting zone |

**Geometry, CAD-sourced** (see `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md` for
the extraction from Yahboom's own `yahboomcar_R2.urdf.xacro`): wheelbase
`L=0.2353 m`, front track `0.130 m`, rear track `0.1685 m`, max steer angle
`δ_max=0.6 rad` (~34.4°), minimum turning radius `R_min=0.344 m`.

> **LiDAR-only / no-camera** operation is available any time with
> `use_camera:=false` — `plant_perception`, `semantic_mapper`, `target_manager`
> simply don't start. Everything else is identical.

---

## 1. Two-layer emergency stop (important)

This robot has **two independent stop layers** — keep them straight:

- **Hardware e-stop (NC block, in the series power chain).** Pressing it breaks
  the protected VCC rail before it splits to the drivetrain and to the Pi —
  the absolute, guaranteed stop. (The Orin is off-robot with its own separate
  power source, not on this rail at all — pressing this e-stop cannot affect
  it, which is expected: it is the operator's station, not a thing that
  drives.) On power-up the stack relaunches with
  `autostart:=false`, so the robot will **not** move until autonomy is
  re-enabled from the pad.
- **Software e-stop (`/e_stop`, joypad Circle).** Latches a `twist_mux` lock
  that zeroes all velocity while the Pi stays alive — the graceful layer for
  "stop moving but keep thinking / logging / recovering". Cleared with Start.

A physical e-stop that cuts motor power is **non-negotiable**. The software
layer complements it; it does not replace it.

## 2. Packages

| Package | Type | Role | Runs on |
|---|---|---|---|
| `robot_description` | cmake | URDF/xacro (Ackermann: steering-knuckle front wheels, fixed rear), CAD-sourced geometry, `robot_state_publisher` | either |
| `base_controller` | python | Wraps Yahboom's **Rosmaster_Lib** (vendored): `/cmd_vel`→`set_car_motion()`, `/wheel_odom` (from `get_motion_data()`), `/imu/data_raw`, `/battery_state`, `/wheel_encoders_raw` (diagnostic) + watchdog. Performs the Ackermann conversion itself, firmware-side — no separate bridge node | realtime (2GB) |
| `sensor_bringup` | cmake | RPLIDAR A1M8 + IMU Madgwick filter (`lidar_imu.launch.py`); Astra camera (`camera.launch.py`) — split so either can be hosted on either Pi | split |
| `row_navigation` | python | tabletop corridor (RANSAC, unchanged) + FOPID (unchanged) + **verified two-arc Ackermann "bulb turn"** headland maneuver → `/cmd_vel_auto` | realtime (2GB) |
| `safety_supervisor` | cmake (C++) | **independent** forward obstacle gate → `/cmd_vel_safe`, `/e_stop`; scales v and ω together (preserves curvature) | realtime (2GB) |
| `teleop_ps2` | python | PS2 joypad: deadman, e-stop, autonomy toggle → `/cmd_vel_teleop`. Deliberately separate from the autonomy/perception stack — see Section 4 | realtime (2GB) |
| `plant_perception` | python | HSV ripe-fruit detector (RGB-D synced) → `/plant_targets` + debug overlay | mission brain (4GB) |
| `semantic_mapper` | python | TF-project detections to `map`, log `semantic_targets.csv` (Phase 1's output, Phase 2's input) | mission brain (4GB) |
| `target_manager` | python | **loads Phase 1's saved plant CSV at startup**, sequences it in row-order → `/selected_plant_goal` | mission brain (4GB) |
| `coverage_planner` | python | boustrophedon route over all aisles → `/coverage_plan` (JSON) + `/coverage_path` (RViz) | mission brain (4GB) |
| `mission_control` | python | mission sequencing (coverage + Nav2 approach, no arm hand-off) **plus the clean Orin command/status interface** (`/mission/command`, `/mission/status`) | mission brain (4GB) |
| `robot_bringup` | cmake | EKF / SLAM (mapping + localization configs) / twist_mux / Nav2 (**Smac Hybrid planner + car-safe Regulated Pure Pursuit**) configs + all launch files | — |
| `strawberry_amr_gazebo` | cmake | Gazebo Classic simulation: generated Irish-polytunnel world (`generate_polytunnel_world.py`), sim bringup launch. Dev Orin only — see `GAZEBO_SIMULATION_GUIDE.md` | dev Orin |

## 3. Command / safety chain

```
row_nav ──────/cmd_vel_auto→ safety_supervisor ─/cmd_vel_safe→ ┐ (prio 10)
Nav2 approach ───────────────/cmd_vel_nav───────────────────────┤ (prio 20)
PS2 joystick ────────────────/cmd_vel_teleop────────────────────┤ (prio 100)
                                                                 twist_mux ─/cmd_vel→ base_controller ─(Rosmaster_Lib set_car_motion)→ YB-ERF01 → motors+servo
                                                        /e_stop ─┘ (lock 255: overrides all)
```

Everything upstream of `base_controller` stays in the steering-geometry-
agnostic `(v, ω)` Twist domain — row-following, Nav2's RPP output, teleop,
and the safety supervisor all reason in plain `v`/`ω`. Only `base_controller`,
the very last node before the hardware, hands `(v, ω)` to Yahboom's own
`set_car_motion(vx, vy, vz)`, which is car-type-aware (`CARTYPE_R2 = 0x05`)
and performs the Ackermann bicycle-model conversion in tested vendor
firmware — see `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`'s "Rosmaster_Lib"
section for why this replaced an earlier from-scratch `ackermann_bridge`
node that duplicated logic the vendor's firmware already implements.

Autonomous motion requires **both** `/autonomy_enable` (human, from the
joypad) **and** `/mission_active` (mission_control). `autostart` defaults to
**false**.

## 4. Orin mission commands (and why teleop is separate)

`mission_control` exposes a deliberately simple, plain-topic command
interface — usable from any Orin with nothing but stock `ros2` CLI tools:

```bash
ros2 topic echo /mission/status
ros2 topic pub --once /mission/command std_msgs/String "data: finish_mapping"  # Phase 1
ros2 topic pub --once /mission/command std_msgs/String "data: start_nav"       # Phase 2
ros2 topic pub --once /mission/command std_msgs/String "data: stop"            # either
```

The PS2 joypad (`teleop_ps2`) is intentionally a **separate, independent**
node from this autonomy/perception stack: it is the manual-override/master
switch (deadman, e-stop, autonomy enable), not part of the mission-sequencing
logic. This is a deliberate separation of concerns, not an oversight — the
human override path should not depend on, or be entangled with, the
autonomous mission's own state machine.

**No arm/harvest logic**: the 5-DOF manipulator is a separate, not-yet-
confirmed companion project. This workspace's Phase 2 approaches each
mapped plant via Nav2, pauses briefly (`plant_confirm_pause_s`, default
3s) to visibly confirm arrival, then automatically advances to the next
one — it does not wait for, or expect, any arm hand-off signal.

## 5. Rosmaster_Lib — the real hardware interface

`base_controller` vendors Yahboom's own `Rosmaster_Lib.py` (from the
operator's `ROSMASTER_R2_robot_sample_codes.zip`, v2.3.3) rather than a
hand-rolled serial protocol — see `base_controller_node.py`'s module
docstring and `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`'s "Rosmaster_Lib"
section for the full verification trail (car_type=5 not 2, battery scale
÷10.0 not ×0.01, no active fused-quaternion getter, `set_car_motion`'s real
vx/vy/vz ranges — each confirmed by reading the actual library source, not
assumed). Default serial port `/dev/myserial`, matching the library's own
constructor default (see `scripts/setup_udev_rules.sh`).

## 6. Two-phase mission

Phase 1 (mapping) and Phase 2 (navigation) never run slam_toolbox's live
mapping and a live Nav2 costmap at the same time on the Pi — the RAM
budget does not support it on either topology. Each phase is its own
launch file on the compute node (Orin, or the 4GB mission-brain Pi on
the legacy dual-Pi build); the Pi's own sensor/actuation launch file is
identical across both phases. Full procedure, including the map-save
handoff between phases (automated via the `finish_mapping` command — see
Section 4), is in
[`docs/MASTER_DEPLOYMENT_COMMANDS.md`](docs/MASTER_DEPLOYMENT_COMMANDS.md)
Stage 13 (every command) or `ALL_IN_ONE_DEPLOYMENT_GUIDE.md` Part 12
(the same procedure with full reasoning).

## 7. Battery monitoring

`base_controller` publishes `/battery_state` from Rosmaster_Lib's
`get_battery_voltage()` (confirmed ÷10.0 scale from source — see Section 5).
For the 3S pack it **warns** at 9.6 V (3.2 V/cell) and flags **critical** at
9.0 V (3.0 V/cell). It does **not** auto-stop (that could strand the robot)
— the operator lands it.

## 8. Build

**Primary (Pi + AGX Orin)** — run on **both** machines:
```bash
cd ~/strawberry_ws
./scripts/install_deps.sh
./scripts/setup_udev_rules.sh       # Pi only -- where the physical devices are
./scripts/setup_network_pi_orin.sh pi      # Pi first, hosts the WiFi AP
./scripts/setup_network_pi_orin.sh orin    # then Orin, joins it
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```
Full explanation of every step, in order, with verification commands
after each one:
[`docs/MASTER_DEPLOYMENT_COMMANDS.md`](docs/MASTER_DEPLOYMENT_COMMANDS.md)
Stages 1–8.

**Legacy (dual-Pi)** — see `DEPLOYMENT_GUIDE.md` for the equivalent
`install_deps.sh` / `setup_udev_rules.sh` / `setup_network.sh` sequence
on that topology instead.

## 9. Run

**Primary (Pi + AGX Orin), Mode A — known layout, two-phase:**
```bash
# Pi:
ros2 launch robot_bringup pi_hardware_and_control.launch.py

# Orin, Phase 1 (mapping):
ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py
# ... drive every row, then:
ros2 topic pub --once /mission/command std_msgs/String "data: finish_mapping"

# Orin, Phase 2 (navigate), after updating slam_toolbox_localization.yaml's
# map_file_name and rebuilding:
ros2 launch robot_bringup pi_missionbrain_phase2_nav.launch.py
ros2 topic pub --once /mission/command std_msgs/String "data: start_nav"
```

**Primary, Mode B — continuous explore, no pre-existing map:**
```bash
# Pi (identical command to Mode A):
ros2 launch robot_bringup pi_hardware_and_control.launch.py
# Orin:
ros2 launch robot_bringup orin_explore_map_navigate.launch.py
```

**Single-host dev/bench tools** (either machine, NOT the field topology):
```bash
ros2 launch robot_bringup teleop.launch.py        # manual PS2 (start here)
ros2 launch robot_bringup bench.launch.py         # indoor row-follow + SLAM, no camera/Nav2
ros2 launch robot_bringup full_robot.launch.py    # full stack, single host
```

**Legacy (dual-Pi)** — see `DEPLOYMENT_GUIDE.md` for the full procedure:
```bash
ros2 launch robot_bringup pi_realtime.launch.py                     # 2GB Pi, both phases
ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py  # 4GB Pi, Phase 1
ros2 launch robot_bringup pi_missionbrain_phase2_nav.launch.py      # 4GB Pi, Phase 2
```

Every command, start to finish, both topologies:
[`docs/MASTER_DEPLOYMENT_COMMANDS.md`](docs/MASTER_DEPLOYMENT_COMMANDS.md).

### PS2 joypad
Hold **L1** (deadman) to drive; tap **X** (deadman released) to toggle autonomy;
tap **Circle** to latch the software e-stop, **Start** to clear; hold **R1** for
turbo. Indices vary by adapter — confirm with `ros2 topic echo /joy` and edit
`teleop_ps2/config/ps2_mapping.yaml`.

### Tuning aids
- `ros2 topic echo /row_heading_error /row_lateral_offset` for the corridor loop.
- `ros2 topic echo /headland_status` to watch the bulb-turn state machine
  (`follow`|`row_end`|`exit_buffer`|`arc1`|`arc2`|`turn_complete`|`reacquire`).
- `ros2 topic echo /mission/status` to watch mission_control's state.
- `ros2 run rqt_image_view rqt_image_view /plant_detector/debug_image` to tune
  the red HSV thresholds live.

## 10. What to tune first (field)

`target_half_width` = half your clear aisle width · camera tilt/heights in the
xacro · FOPID gains (`Kp/Ki/Kd_theta`, `fopid_lambda`, `fopid_mu`,
`k_cross_track` — **carried over from the old skid-steer/slower-speed design,
flagged for re-tuning at the new walking-pace speed**, see
`row_nav_node.py`) · red HSV thresholds · safety `stop_distance`/`slow_distance`
· `max_steer_angle` (CAD-sourced, recommend a quick bench confirmation — see
calibration guide Part 1.1).

## 11. Engineering-standards compliance & scope

Applied: single-parent tf2 tree, no loops; QoS per topic (BEST_EFFORT/
SensorData for LiDAR & images, RELIABLE for commands); **safety monitor on
an independent executable** (`safety_supervisor`) so a planner crash can't
disable it; command watchdogs; all parameters in documented YAML with units.

Known, documented limitations (see `ALL_IN_ONE_DEPLOYMENT_GUIDE.md` Part
15's checklist for the
full list): sensor timestamps are host-receive time, not hardware-acquisition
time; whether the firmware implements an electronic differential between the
two rear motors during a turn is genuinely unknown from the available
sources (quantified worst case if it doesn't, in
`docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`); accelerometer raw-to-m/s²
scaling is assumed and needs a one-time bench confirmation (calibration
guide Part 3.2); a restarted Phase 2 does not remember which plants were
already visited in an interrupted prior run.

Deliberately **out of scope**:
- **No arm/harvest logic** — see Section 4.
- **No IoT/cloud layer** — this is a local ROS 2 robot, not a connected fleet.
- **No model training** — perception is classical HSV (default) plus an
  optional zero-shot layer (Grounding DINO + MobileSAM, pre-trained
  weights only) per the project's training-free constraint.
- **No MCU firmware** — the YB-ERF01 runs its own firmware; this workspace
  talks to it only through Yahboom's own Rosmaster_Lib.

## 12. Push to GitHub

This folder is a git repo with history. Create an **empty** repo on GitHub, then:

```bash
git remote add origin git@github.com:<you>/strawberry_amr_ws.git
git branch -M main
git push -u origin main
```
