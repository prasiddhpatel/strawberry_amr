# Gazebo Simulation Guide — Irish Polytunnel Environment

**Runs on your dev Orin, not the Pi rig.** Gazebo is a full 3D physics
simulator with GPU-accelerated rendering — do not attempt to install or run
it on either Raspberry Pi. Everything in this guide happens on a separate
machine (your Orin or a desktop), used to validate the navigation,
mapping, and mission logic before spending field/lab time on the real
robot.

## Why this is worth doing before more field time

Every navigation-relevant node in this workspace runs completely unchanged
in simulation — `row_navigation`, `safety_supervisor`, `twist_mux`, the
EKF, `slam_toolbox`, `coverage_planner`, `mission_control`, and (if you
want) the full Nav2 stack. Only the hardware-specific bottom layer swaps
out: `base_controller`'s `Rosmaster_Lib` call is replaced by a Gazebo
Ackermann-drive plugin, and the real sensor drivers are replaced by Gazebo
sensor plugins — both publishing on the exact same topics the real
hardware uses. That means a clean run in simulation is real evidence the
navigation *logic* is sound, not just a nice animation — bugs in the
corridor estimator, the headland turn, the mission sequencing, or the
coverage planner will show up here exactly as they would on the real
robot. What it can't tell you: whether your specific chassis, sensors, and
firmware actually behave the way this workspace assumes — that's what the
calibration guide and the real bench/field tests are for. Treat simulation
and hardware validation as complementary, not substitutes for each other.

## 1. Install (on your dev Orin)

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros \
    ros-humble-xacro ros-humble-robot-state-publisher
```

This workspace targets **Gazebo Classic 11** (via `gazebo_ros_pkgs`), not
the newer Ignition/Gazebo Sim — Classic has the more mature
`libgazebo_ros_ackermann_drive` plugin this simulation relies on, and is
still the standard, well-documented pairing for ROS 2 Humble. Then build
this workspace on your Orin the normal way (`colcon build
--symlink-install`, same as the install procedure in
`docs/MASTER_DEPLOYMENT_COMMANDS.md` Stage 4) — you need `strawberry_amr_gazebo` plus
every package it depends on (essentially the whole workspace, since it
reuses almost every node — see Section 3).

## 2. What's simulated, what isn't — read this before trusting a result

This is genuinely important, not boilerplate. Two things do **not**
transfer faithfully from simulation to the real robot:

1. **HSV-based ripe-fruit detection.** `plant_perception`'s colour
   thresholds are tuned against real camera images under real lighting.
   Gazebo Classic's rendering is not photorealistic, so a detection in sim
   is a rough functional check ("did *something* red get detected roughly
   where a berry model is") — it validates the detection→projection→
   mapping *pipeline*, not your actual HSV threshold values. Tune those on
   the real camera (calibration guide Part 4), never in simulation.
2. **Whether the firmware differentials the two rear motors** (see
   `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`) — the Gazebo Ackermann
   plugin has its own internal wheel-speed model, unrelated to what
   Yahboom's actual STM32 firmware does. This question can only be
   answered by watching the real wheels, not simulation.

Everything else — row-following, the headland bulb-turn, SLAM map quality,
coverage sequencing, Nav2's path planning and approach, the
`/mission/command` → `/mission/status` handshake — is the *actual*
deployed code, so a failure here is a real bug worth fixing before it
costs you field time, and success here is real, if partial, evidence the
logic works.

## 3. Reused vs. replaced

| Real robot | Simulation | Same topic? |
|---|---|---|
| `base_controller` (Rosmaster_Lib `set_car_motion`) | Gazebo `libgazebo_ros_ackermann_drive` plugin | Yes — both consume `/cmd_vel`, publish `/wheel_odom` |
| `rplidar_ros` driver | Gazebo ray sensor plugin | Yes — both publish `/scan` |
| `astra_camera` driver | Gazebo depth camera plugin | Yes — both publish `/camera/color/image_raw` etc. |
| Rosmaster_Lib IMU reads | Gazebo IMU sensor plugin | Yes — both publish `/imu/data_raw` |
| **Everything else** (`row_navigation`, `safety_supervisor`, `twist_mux`, `imu_filter_madgwick`, EKF, `slam_toolbox`, `plant_perception`, `semantic_mapper`, `target_manager`, `coverage_planner`, `mission_control`, Nav2) | **Identical** — same package, same executable, same params file | N/A, unchanged |

The swap happens entirely in
`robot_description/urdf/strawberry_amr_sim.urdf.xacro`, which **includes**
the real `strawberry_amr.urdf.xacro` unmodified and adds the Gazebo
plugins on top — the file the real robot uses is never touched by
anything simulation-related. See that file's own header comment for the
full detail.

## 4. Generate the polytunnel world

```bash
cd src/strawberry_amr_gazebo/scripts
python3 generate_polytunnel_world.py --preset lab -o ../worlds/irish_polytunnel_lab.world
python3 generate_polytunnel_world.py --preset realistic -o ../worlds/irish_polytunnel_realistic.world
```

Both are already generated and included in the package — regenerate only
if you change parameters. **Two presets, pick deliberately:**

- **`lab`**: row spacing 0.50 m, half-aisle-width 0.20 m — this is an
  *exact* match to the currently shipped `row_navigation_params.yaml` /
  `coverage_params.yaml` defaults, i.e. what you're about to test in Lab
  3003. Use this first.
- **`realistic`**: row spacing 1.20 m, half-aisle-width 0.45 m, trough
  width 0.30 m — a wider, commercial-tabletop-scale estimate.

**Be clear-eyed about where the `realistic` numbers come from**: this
workspace's web-search tool wasn't available when this generator was
written, so these are a reasoned estimate from general knowledge of UK/
Irish tabletop soft-fruit systems (trough width + walking-aisle width →
row pitch), not a fetched, citable figure the way the hardware specs
elsewhere in this project were verified. If you or your supervisor have a
real, grower-verified spacing for your target polytunnel, override it
directly:

```bash
python3 generate_polytunnel_world.py --row-spacing 1.6 --half-width 0.6 \
    --trough-width 0.4 --num-rows 8 --plant-spacing 0.35 \
    -o ../worlds/custom.world
```

The generator refuses to write a world where `2*half_width >= row_spacing`
(adjacent rows' post lines would overlap) — this is the exact bug that was
just fixed in `row_navigation_params.yaml`'s `target_half_width`; the
generator won't let you reintroduce it.

**Both `lab` and `realistic` model the tabletop system as discrete
support posts, not a continuous wall** — this matches exactly what
`row_navigation`'s RANSAC corridor estimator is built to detect (see that
node's own module docstring). A Gazebo world with continuous hedge-like
walls would look plausible but wouldn't exercise the real algorithm at
all.

**A performance note**: the `lab` preset's tight plant spacing generates
just over 1000 individual models (posts, troughs, foliage blocks, berry
spheres) across 6 rows. That's a lot of small static objects for Gazebo
Classic to render smoothly on a Orin without a dedicated GPU. For your
first run, generate a smaller world to confirm everything works before
scaling up:

```bash
python3 generate_polytunnel_world.py --preset lab --num-rows 2 \
    --plant-spacing 0.6 -o ../worlds/irish_polytunnel_lab_small.world
```

## 5. Launch

```bash
ros2 launch strawberry_amr_gazebo gazebo_sim.launch.py
# or, to pick a specific world / disable pieces:
ros2 launch strawberry_amr_gazebo gazebo_sim.launch.py \
    world:=$(ros2 pkg prefix strawberry_amr_gazebo)/share/strawberry_amr_gazebo/worlds/irish_polytunnel_realistic.world \
    use_nav2:=false use_camera_stack:=false
```

This brings up Gazebo, spawns the robot at the entrance to row 0, and
starts the full stack from the table in Section 3 — SLAM mapping mode by
default (matching Phase 1 of the real deployment), with Nav2 and the
perception stack on by default too (turn either off with the arguments
above for a lighter first test).

**Explore mode (Mode B) in simulation**: this previously could not be
exercised at all here — `mission_control`'s phase was hardcoded to `'nav'`
and `row_navigation`'s exploration mode had no launch-time switch. Two new
arguments now expose it:

```bash
ros2 launch strawberry_amr_gazebo gazebo_sim.launch.py \
    mission_phase:=explore exploration_mode:=true
```

`exploration_mode:=true` selects a separate `row_nav_node` instance
(there are two duplicate `Node()` entries in the launch file, gated by
`IfCondition`/`UnlessCondition` rather than a substituted boolean
parameter, specifically because `launch_ros`'s string-to-bool coercion
for that pattern was never verified in this project's authoring
environment) — after launch, confirm only one actually started and that
it reports the right value: `ros2 param get /row_nav_node
exploration_mode` should print `true`.

## 6. What to actually test

Work through this in roughly the same order as the real deployment:

1. **Drivetrain sanity**: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist
   "{linear: {x: 0.15}, angular: {z: 0.0}}"` — confirm the robot drives
   straight down the simulated aisle in RViz/Gazebo. Try a nonzero
   `angular.z` and confirm it steers, not spins in place (spinning in
   place would mean the Ackermann plugin is misconfigured — it should be
   kinematically incapable of that, same as the real chassis).
2. **Row-following**: enable autonomy (`ros2 topic pub --once
   /autonomy_enable std_msgs/msg/Bool "data: true"` and `/mission_active`
   similarly, or drive it through `mission_control`'s normal `start_nav`
   command once you're running the full stack) and watch it track the
   simulated post line down the aisle.
3. **Headland turn**: let it reach the end of a row and watch the two-arc
   bulb turn execute — does it land in the next aisle cleanly, or clip a
   post? A clean sim turn is a good sign; a clipped one means either the
   world's headland clearance is too tight for the row spacing you chose,
   or there's a real bug worth chasing before it happens on hardware.
4. **Mapping**: drive/let it map a few rows, check the resulting
   `/map` in RViz looks like the generated world's layout.
5. **Perception (functional check only, per Section 2)**: confirm
   `/plant_targets` and `semantic_targets.csv` populate with roughly
   plausible positions as it passes the berry models.
6. **Full mission**: try the actual `/mission/command` /
   `/mission/status` workflow from `docs/MASTER_DEPLOYMENT_COMMANDS.md`
   Stage 13 end-to-end
   against the simulated world, exactly as you would in Lab 3003.

**Beyond manual bench testing**: this same simulation is also the
foundation for `docs/GL_FOPID_RL_TUNING_GUIDE.md` — automated
reinforcement-learning search for `row_navigation`'s GL-FOPID gains,
running many simulated episodes unattended rather than manually watching
each one.

## 7. Troubleshooting

- **Gazebo starts but the robot doesn't appear**: check
  `spawn_strawberry_amr`'s log output — usually a `robot_description`
  parameter or xacro syntax issue. `xacro
  src/robot_description/urdf/strawberry_amr_sim.urdf.xacro` on its own
  should produce valid XML with no errors; run that directly to isolate
  URDF problems from spawn-timing problems.
- **Robot appears but doesn't move**: confirm `/cmd_vel` is actually
  reaching the Ackermann plugin (`ros2 topic echo /cmd_vel` while
  publishing a test Twist) and that `twist_mux` isn't blocking it (check
  `/e_stop` isn't latched, `/autonomy_enable` and `/mission_active` are
  both true if you're expecting autonomous motion).
- **Very slow / low frame rate**: reduce world complexity first (Section
  4's small-world example), then check whether hardware GPU acceleration
  is actually being used by Gazebo (`gzclient` running in software
  rendering is dramatically slower).
- **LiDAR sees nothing / row-following doesn't engage**: check `/scan` in
  RViz for actual returns near the post positions — if empty, the sensor
  plugin's `frame_name` or update rate may be misconfigured; compare
  against the plugin block in `strawberry_amr_sim.urdf.xacro`.
