# Master Deployment Guide — Every Command, Start to Finish

One continuous, linear sequence of real bash commands to go from a bare
Pi + AGX Orin to an autonomous R2 robot running in Lab 3003. This
consolidates `ALL_IN_ONE_DEPLOYMENT_GUIDE.md`, `docs/JETPACK_SETUP_GUIDE.md`,
`docs/PARAMETER_TUNING_CHECKLIST.md`, `docs/ORIN_PI_SPLIT_ARCHITECTURE.md`,
and `docs/CLAUDE_CODE_VERIFICATION_GUIDE.md` (turned into commands *you*
run, not prompts for a tool) into one path — those documents remain the
place to go for the *reasoning* behind any step; this one is the
*procedure*.

**Do this in order.** Every stage after Stage 0 depends on the ones
before it being genuinely done, not skimmed.

**Machine labels below**: `[PI]` = run on the Raspberry Pi 4B (SSH'd in
or with a keyboard attached once, headless after). `[ORIN]` = run on the
AGX Orin (has its own keyboard/mouse/display — this is your operator
station). `[BOTH]` = run identically on both.

---

## Stage 0 — What you need before starting

- Raspberry Pi 4B 4GB, microSD card (32GB+), a way to write it (another PC)
- AGX Orin 64GB Developer Kit, already on JetPack 6.x (Ubuntu 22.04) — if
  not, stop and do `docs/JETPACK_SETUP_GUIDE.md` Part 1-2 first, this
  guide assumes it's done
- The R2 robot physically assembled, YB-ERF01 board connected, RPLIDAR
  A1M8 and Astra Pro Plus both on the Pi's USB
- PS2 controller + USB nano receiver
- A tape measure, a protractor or phone angle app
- This workspace's zip file, on a USB drive or downloadable to both machines

---

## Stage 1 — Operating systems

### 1.1 Pi — Ubuntu Server 22.04 (64-bit)

On any PC with an SD card reader:
```bash
# Download Raspberry Pi Imager from https://www.raspberrypi.com/software/,
# then in the Imager GUI:
#   Choose OS -> Other general-purpose OS -> Ubuntu ->
#     "Ubuntu Server 22.04.x LTS (64-bit)"  (NOT Desktop, NOT 32-bit)
#   Choose Storage -> your microSD card
#   Click the gear icon before writing: set hostname (e.g. strawberry-pi),
#   enable SSH, set username/password, optionally WiFi for first boot
#   Write, wait for verify, insert into the Pi, power on
```

Find its IP and SSH in:
```bash
ping strawberry-pi.local          # or check your router's admin page
ssh <username>@<pi-ip-address>
sudo apt update && sudo apt full-upgrade -y && sudo reboot
```

### 1.2 Orin — confirm JetPack 6.x

```bash
cat /etc/nv_tegra_release     # want R36.x
lsb_release -a                # want 22.04
```
If either doesn't match, stop here and follow `docs/JETPACK_SETUP_GUIDE.md`
in full before continuing — **do not** `do-release-upgrade` from JetPack
5.x, it breaks the driver stack. Once confirmed:
```bash
sudo nvpmodel -m 0            # max performance mode, persists across reboots
sudo jetson_clocks            # pin clocks to max -- does NOT persist, re-run after every boot
```

---

## Stage 2 — ROS 2 Humble `[BOTH]`

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

**`[PI]` only:**
```bash
sudo apt install -y ros-humble-ros-base python3-argcomplete
```

**`[ORIN]` only:**
```bash
sudo apt install -y ros-humble-desktop python3-argcomplete
```

**`[BOTH]` again:**
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  python3-vcstool build-essential git

sudo rosdep init
rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify `[BOTH]`**: `ros2 topic list` prints `/parameter_events` and
`/rosout` with no errors.

---

## Stage 3 — Gazebo `[ORIN only]`

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins ros-humble-xacro ros-humble-robot-state-publisher
gazebo --version    # expect 11.x
```

---

## Stage 4 — Workspace, dependencies, camera driver, build `[BOTH]`

```bash
mkdir -p ~/strawberry_ws
cd ~/strawberry_ws
# unpack this workspace's zip here, so you end up with
# ~/strawberry_ws/src/<packages>

sudo apt install -y \
  ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-twist-mux \
  ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-rplidar-ros \
  ros-humble-imu-filter-madgwick ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf2-geometry-msgs ros-humble-cv-bridge \
  ros-humble-message-filters ros-humble-image-transport \
  ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
  ros-humble-compressed-depth-image-transport ros-humble-rmw-cyclonedds-cpp \
  python3-opencv python3-numpy python3-serial
```

**Do not** `pip3 install Rosmaster_Lib` — no such PyPI package exists.
It's vendored in `base_controller`; a normal `colcon build` picks it up.

### Camera driver (Orbbec Astra Pro Plus) `[BOTH]`

```bash
sudo apt install -y libusb-1.0-0-dev libudev-dev pkg-config \
  libjpeg-dev libturbojpeg0-dev

cd ~/strawberry_ws/src
git clone https://github.com/orbbec/ros_astra_camera.git
cd ros_astra_camera
git branch -a
# check out whichever branch/tag explicitly names "humble" or "ros2" if
# the default branch isn't already it:
#   git checkout <humble-branch-name>

sudo bash scripts/create_udev_rules.sh
cd ~/strawberry_ws
```

### Build `[BOTH]`

```bash
cd ~/strawberry_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
echo "source ~/strawberry_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify camera `[PI]`** (physical camera connected):
```bash
ros2 launch astra_camera astra.launch.xml    # check `ros2 pkg prefix astra_camera`
                                              # /share/astra_camera/launch/ if this
                                              # exact filename doesn't exist
ros2 topic hz /camera/color/image_raw
```
No topic → replug after `sudo udevadm control --reload-rules && sudo udevadm trigger`,
or you're on the wrong branch for Humble.

**Verify build `[BOTH]`**:
```bash
ros2 pkg list | grep -E "base_controller|row_navigation|mission_control"
```
Should list every custom package, on both machines.

---

## Stage 5 — Network `[PI first, then ORIN]`

```bash
cd ~/strawberry_ws
./scripts/setup_network_pi_orin.sh pi        # [PI] -- hosts the WiFi AP
```
```bash
./scripts/setup_network_pi_orin.sh orin      # [ORIN] -- joins it
```

**Open a NEW terminal on each machine** (so `.bashrc` takes effect):
```bash
ping 192.168.30.1      # [ORIN] pinging the Pi
ping 192.168.30.100    # [PI] pinging the Orin
```
Both must succeed. If the Orin can't see `StrawberryAMR` at all:
```bash
sudo systemctl status hostapd dnsmasq    # [PI]
```

---

## Stage 6 — udev rules `[PI]`

```bash
cd ~/strawberry_ws
./scripts/setup_udev_rules.sh
```

**Verify**: unplug and replug all three devices (LiDAR, camera, YB-ERF01), then:
```bash
ls -l /dev/rplidar /dev/myserial
```
Both must show as valid symlinks.

---

## Stage 7 — Hardware verification `[PI]`

**The highest-risk integration point in the whole system.**

```bash
ros2 run base_controller base_controller_node --ros-args \
  --params-file ~/strawberry_ws/src/base_controller/config/base_controller_params.yaml
```

In a second terminal:
```bash
ros2 topic echo /wheel_encoders_raw
```
Rotate one rear wheel by hand exactly one full turn — count must change
by **836**. If it doesn't: diagnostic only, not blocking (real odometry
comes from `Rosmaster_Lib`'s `get_motion_data()`, not these raw ticks).

```bash
ros2 topic echo /imu/data_raw
```
Robot stationary and level — `linear_acceleration.z` must read **≈9.8**,
not ≈1.0. If it reads ~1.0:
```bash
nano ~/strawberry_ws/src/base_controller/base_controller/base_controller_node.py
# find G_TO_MS2 = 9.80665 near the top, change to G_TO_MS2 = 1.0
cd ~/strawberry_ws && colcon build --symlink-install --packages-select base_controller
```

---

## Stage 8 — PS2 controller mapping `[PI]`

Plug the USB nano receiver into the **Pi**, not the Orin — this is what
keeps manual override working if the Pi↔Orin WiFi link drops.

If `ros2 run joy joy_node` logs a permission error:
```bash
sudo usermod -aG input $USER
# log out and back in -- restarting the terminal alone is not enough
```

```bash
ros2 run joy joy_node
```
In a second terminal:
```bash
ros2 topic echo /joy
```
Press each button / move each stick one at a time, note which array
index changes:
```bash
nano ~/strawberry_ws/src/teleop_ps2/config/ps2_mapping.yaml
# fill in: axis_linear, axis_angular, deadman_button, autonomy_button,
#          estop_button, reset_button
cd ~/strawberry_ws && colcon build --symlink-install --packages-select teleop_ps2
```

---

## Stage 9 — Chassis geometry `[PI]`

```bash
ros2 launch robot_bringup teleop.launch.py
```
Command a steady hard-left turn (small forward speed, full left stick),
hold until the front wheels settle at their lock. Measure the angle
between the chassis centreline and the wheel plane with a protractor.

If meaningfully smaller than 34.4°:
```bash
nano ~/strawberry_ws/src/row_navigation/config/row_navigation_params.yaml
# update max_steer_angle
nano ~/strawberry_ws/src/robot_description/urdf/strawberry_amr.urdf.xacro
# update the max_steer property to the SAME value
cd ~/strawberry_ws && colcon build --symlink-install
```

Also measure wheelbase (front axle centre to rear axle centre) with a
tape measure and confirm it against `wheelbase: 0.2353` in the same yaml.

---

## Stage 10 — Sensor calibration `[PI, then ORIN for viewing]`

### LiDAR solo bring-up `[PI]`
```bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/rplidar -p serial_baudrate:=115200 -p frame_id:=laser
ros2 topic hz /scan    # expect ~5.5-10 Hz
```
View in RViz2 on the Orin (Fixed Frame `laser`) — a distinct object
directly in front should show a return near bearing 0°. Test under
normal light, then the brightest light available, and compare
dropout/noise (triangulation LiDARs are light-sensitive).

### IMU gyro `[PI]`
```bash
ros2 topic echo /imu/data
```
Should sit near zero stationary. Rotate the robot by hand through a
known 90° and confirm the filtered yaw changes by roughly that much.

### Camera angle
Full worked trigonometry in
`docs/CAMERA_ANGLE_AND_HSV_CALIBRATION_GUIDE.md` Part 1-2. Validate
against a real wall in RViz2's point cloud view — trust that over any
calculated number.

### HSV tuning `[PI running, ORIN viewing]`
```bash
sudo apt install -y ros-humble-rqt-image-view    # [ORIN] if not already installed
nano ~/strawberry_ws/src/plant_perception/config/plant_detector_params.yaml
# edit h1_lo / h1_hi / h2_lo / h2_hi / s_lo / v_lo against real fruit,
# real lighting -- watch the debug overlay:
ros2 run rqt_image_view rqt_image_view   # [ORIN] -- select /plant_detector/debug_image
```
Keep a separate params file per lighting environment.

---

## Stage 11 — Room geometry (Lab 3003 specifics)

Measure your actual desk-row aisle width, then:
```bash
nano ~/strawberry_ws/src/row_navigation/config/row_navigation_params.yaml
# target_half_width = half your measured aisle width
# row_spacing = your measured spacing
```
**This must match in THREE files** — the most common silent-corruption
trap in this whole workspace:
```bash
grep "row_spacing" ~/strawberry_ws/src/row_navigation/config/row_navigation_params.yaml
grep "row_spacing" ~/strawberry_ws/src/coverage_planner/config/coverage_params.yaml
grep "row_spacing" ~/strawberry_ws/src/target_manager/config/target_manager_params.yaml
```
All three values must agree.

Measure actual clear floor space past the last desk in each row —
`docs/HEADLAND_TURN_GEOMETRY.md` gives the required depth (~1.22 m at
0.50 m row spacing) and lateral bulge (~0.59 m) at your current
`row_spacing`. If tighter, reduce `row_spacing` or reroute before any
unattended run.

```bash
cd ~/strawberry_ws && colcon build --symlink-install
```

---

## Stage 12 — Bench test, elevated `[PI]`

Robot **elevated on blocks, wheels free**:
```bash
ros2 launch robot_bringup bench.launch.py
```
Enable autonomy on the PS2 pad, confirm row-following and the headland
bulb-turn execute correctly:
```bash
ros2 topic echo /headland_status    # [ORIN, second terminal]
```
**Watch both rear wheels during a full-lock turn** — confirm whether the
firmware differentials them (unknown until observed; see
`docs/MOTOR_AND_GEOMETRY_VERIFICATION.md` for why this matters).

**Compile and test the safety-critical C++ node**, if not already done:
```bash
cd ~/strawberry_ws
colcon build --packages-select safety_supervisor
```
Report any compiler error before attempting a fix — understand what
broke first.

**Verify compressed camera transport**:
```bash
ros2 launch robot_bringup pi_hardware_and_control.launch.py   # [PI]
ros2 topic list | grep compressed                              # [either]
```
Should show `/camera/color/image_raw/compressed` and
`/camera/depth/image_raw/compressedDepth`.

**Test the network-disconnect safety case** — walk the Orin out of WiFi
range (or disable its WiFi adapter) once, safely: confirm manual PS2
driving and the e-stop still work, and autonomous driving correctly
stops. Manual driving during a disconnect has no obstacle-based safety
gating — you are the obstacle detection in that window.

---

## Stage 13 — First real run: Lab 3003 Mode A (known layout, two-phase)

See `LAB_3003_MOCK_PLANT_AMR_DEPLOYMENT_GUIDE.md` for the room setup
(desk placement, headland clearance, mock plant positioning) alongside
this sequence.

**`[PI]`**:
```bash
ros2 launch robot_bringup pi_hardware_and_control.launch.py
```

**`[ORIN]` — Phase 1 (mapping)**:
```bash
ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py
```
Drive every row (manual PS2 first — build trust in the map before
autonomy), watching `/map` and `/plant_targets` in RViz2. When every row
is covered, **wait 5-10s after the last detection**, then:
```bash
ros2 topic pub --once /mission/command std_msgs/String "data: finish_mapping"
ros2 topic echo /mission/status    # wait for MAP_SAVED
```

**Before Phase 2**:
```bash
nano ~/strawberry_ws/src/robot_bringup/config/slam_toolbox_localization.yaml
# set map_file_name to the path just saved
cd ~/strawberry_ws && colcon build --symlink-install
```

**`[ORIN]` — Phase 2 (navigate)**:
```bash
ros2 launch robot_bringup pi_missionbrain_phase2_nav.launch.py
```
Enable autonomy on the PS2 pad, then:
```bash
ros2 topic pub --once /mission/command std_msgs/String "data: start_nav"
ros2 topic echo /mission/status
```
Watch for `NAVIGATING` → `APPROACHING_PLANT` (repeated per plant, each
with a ~3s pause) → `MISSION_COMPLETE`.

**Stop cleanly at any point** (mission-level pause, not the hardware
e-stop — keep the PS2 e-stop and a hand near main power within reach
regardless):
```bash
ros2 topic pub --once /mission/command std_msgs/String "data: stop"
```

**Success looks like**: the saved map resembles the actual desk layout;
`semantic_targets.csv` lists a plausible plant count without excessive
duplicates; Phase 2 visits every plant row-by-row without a headland turn
clipping a desk or needing the hardware e-stop. This is confidence to
proceed to real-environment testing, not a substitute for it.

---

## Stage 14 — Optional: continuous explore mode (Mode B)

**`[PI]`** (identical command to Mode A):
```bash
ros2 launch robot_bringup pi_hardware_and_control.launch.py
```
**`[ORIN]`**:
```bash
ros2 launch robot_bringup orin_explore_map_navigate.launch.py
```
Enable autonomy, `start_nav` as above. Watch `EXPLORING` →
`EXPLORATION_COMPLETE` when no further row is found. To retry: cycle
`/autonomy_enable` off then on via the PS2 pad, **then** `start_nav` again.

---

## Stage 15 — Optional: Gazebo simulation `[ORIN]`

```bash
cd ~/strawberry_ws/src/strawberry_amr_gazebo/scripts
python3 generate_polytunnel_world.py --preset lab --num-rows 2 --plant-spacing 0.6 \
  -o ../worlds/irish_polytunnel_lab_small.world
ros2 launch strawberry_amr_gazebo gazebo_sim.launch.py \
  world:=$(ros2 pkg prefix strawberry_amr_gazebo)/share/strawberry_amr_gazebo/worlds/irish_polytunnel_lab_small.world
```
HSV tuning and the rear-motor differential question do not transfer from
simulation — Gazebo Classic isn't photorealistic and its Ackermann
plugin has its own internal model.

---

## Stage 16 — Optional enhancements, only after Stage 13 works

Both of these are genuine additions on top of a system that already
works without them. Do them after, not instead of, a working Mode A run.

### 16.1 Zero-shot perception (Grounding DINO + MobileSAM) `[ORIN]`

```bash
nvcc --version
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```
Must print `True`. If not, install NVIDIA's Jetson PyTorch build (not
`pip install torch`) — see `docs/JETPACK_SETUP_GUIDE.md` Part 4.1.

```bash
cd ~/strawberry_ws/src/plant_perception
pip install -r requirements_zeroshot.txt --break-system-packages
pip install git+https://github.com/ChaoningZhang/MobileSAM.git --break-system-packages

mkdir -p ~/strawberry_ws/models
wget https://raw.githubusercontent.com/ChaoningZhang/MobileSAM/master/weights/mobile_sam.pt \
    -O ~/strawberry_ws/models/mobile_sam.pt
```
```bash
nano ~/strawberry_ws/src/plant_perception/config/plant_detector_zeroshot_params.yaml
# set mobile_sam_checkpoint to ~/strawberry_ws/models/mobile_sam.pt
cd ~/strawberry_ws && colcon build --symlink-install
```

Full standalone-verification-before-ROS sequence, prompt tuning, and
troubleshooting: `docs/ZERO_SHOT_PERCEPTION_GUIDE.md`. Once verified:
```bash
ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py detector:=zeroshot
```
`mission_control` fires the detection trigger automatically at each
plant approach — no separate command needed once `detector:=zeroshot` is
set.

### 16.2 GL-FOPID RL tuning `[ORIN]`

```bash
cd ~/strawberry_ws/src/strawberry_amr_gazebo/scripts/gl_fopid_rl
pip install -r requirements.txt --break-system-packages
jupyter notebook train_gl_fopid.ipynb
```
Run Step 0 (mock-backend smoke test) before touching Gazebo at all. Then
Step 1 (generate world), Step 2 (launch Gazebo, separate terminal), then
Step 3 with `TOTAL_EPISODES` set low (e.g. `30`) for a first attempt.
Full design and honest fallback: `docs/GL_FOPID_RL_TUNING_GUIDE.md`.

---

## Quick reference — every mission command

```bash
ros2 topic pub --once /mission/command std_msgs/String "data: finish_mapping"
ros2 topic pub --once /mission/command std_msgs/String "data: start_nav"
ros2 topic pub --once /mission/command std_msgs/String "data: stop"
ros2 topic echo /mission/status
```

**Clearing a latched `/e_stop` from the supervisor**: `safety_supervisor`
now latches its own LiDAR-obstacle stop — once tripped it stays tripped
even after the obstacle clears, and does not auto-resume. This is
separate from the joypad's own local `estop_button`/`reset_button` (Stage
8) — they reset joystick_node's own latch, not the supervisor's. Clear
the supervisor's latch explicitly once the obstacle is genuinely gone:
```bash
ros2 topic pub --once /e_stop_reset std_msgs/msg/Empty "{}"
```
A reset published while the underlying condition is still active does
not clear the latch — it re-asserts on the very next cycle.

## If something goes wrong

`docs/PARAMETER_TUNING_CHECKLIST.md` — every value tagged by confidence
(verified / needs measurement / needs tuning / must set). `ALL_IN_ONE_DEPLOYMENT_GUIDE.md`
Part 15 — the full acceptance checklist this guide's stages are drawn
from, with the reasoning behind each item.
