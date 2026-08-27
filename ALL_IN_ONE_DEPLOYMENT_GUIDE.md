# All-In-One Deployment Guide — Strawberry AMR (Yahboom ROSMASTER R2)

**Audience**: complete beginner to robotics, physical assembly already done
per Yahboom's manual. This document is the single, start-to-finish path —
OS install through autonomous field operation, every mode. Where a topic
has a deeper specialist guide elsewhere in this workspace, this document
gives you the complete, working procedure inline and points to the deeper
document only for edge cases and troubleshooting depth — you should never
have to leave this file to get unblocked on the critical path.

**Two machines, roles fixed for this whole guide**: one Raspberry Pi 4B
4GB (all sensors, all actuation, the safety-critical control loop) and
one Orin (everything computationally heavy). If you are instead running
the older two-Pi (2GB+4GB) topology, most of this guide still applies —
substitute the dual-Pi launch files named in
`docs/ORIN_PI_SPLIT_ARCHITECTURE.md` at the relevant steps.

## Judgment calls made in this document, stated up front

Per your standing instruction: here are the decisions embedded below that
are genuine judgment calls, not settled facts — validate or override them:

1. **Pi OS = Ubuntu Server 22.04 (64-bit), not stock Raspberry Pi OS.**
   Reasoning: ROS 2 Humble's official binary packages target Ubuntu 22.04
   (Jammy); Raspberry Pi OS is Debian-based and doesn't have first-class
   `ros-humble-*` apt support, meaning you'd end up building far more from
   source for no benefit. This is a strong, standard recommendation, not a
   coin flip — but it is a real choice, not a fact, and you should know
   it's being made.
2. **`ros-base` on the Pi, `desktop` on the Orin.** The Pi is headless
   (no monitor attached in normal operation, monitored remotely from the
   Orin) — it doesn't need RViz/GUI tooling, so the lighter metapackage
   is the right call there. The Orin needs the full desktop install for
   RViz and Gazebo.
3. **Orbbec camera driver: `ros_astra_camera`, not `OrbbecSDK_ROS2`.** The
   Astra Pro Plus is an older, OpenNI2-generation Orbbec camera — the
   newer `OrbbecSDK_ROS2` package targets Orbbec's Gemini/Femto line and
   may not enumerate this hardware correctly at all. This is a real
   technical judgment, not hand-waving, and it's the one part of this
   whole document I have not been able to independently re-verify against
   a live fetch this session (no web access this session) — Part 4.6
   below tells you exactly how to confirm the current branch tag yourself
   in under a minute before trusting it further.
4. **Default operating mode for your first real run = Mode A (known
   layout / Lab 3003), not Mode B (unknown-tunnel explore).** Reasoning:
   Mode A is easier to debug (you know what the map should look like) and
   exercises almost the entire stack Mode B needs anyway. Do Mode A first
   even if Mode B is your ultimate goal.
5. **If your Orin is on a newer Ubuntu than 22.04: Docker, not a
   migration to a newer ROS 2 distro or a Orin OS reinstall** (Part
   1.3). Reasoning: a newer ROS 2 distro that supports your Ubuntu version
   would need every external dependency this project uses re-verified
   against it — a real migration project, not a fix. Docker sidesteps the
   mismatch entirely by keeping Humble's actual supported OS (22.04)
   inside the container regardless of the host. Dual-boot/VM are
   legitimate alternatives if you'd rather not containerize; not silently
   ruled out, just not the default recommendation here.
6. **If you go the VM route instead of Docker: Bridged networking, not
   NAT** (Part 1.4). This isn't a style preference — NAT would silently
   prevent the VM from ever reaching the Pi directly on the physical
   network, which the whole Pi↔Orin setup in Part 5 depends on. Called
   out explicitly because it's exactly the kind of default that looks
   fine (the VM gets internet access either way) until you reach a much
   later step and can't tell why discovery isn't working.

---

## Table of contents

- Part 1 — Operating system installation (Pi + Orin)
- Part 2 — ROS 2 Humble installation (both machines)
- Part 3 — Gazebo installation (Orin)
- Part 4 — Workspace setup, dependencies, camera driver, build
- Part 5 — Network setup
- Part 6 — udev rules
- Part 7 — Hardware/firmware verification (Rosmaster_Lib)
- Part 8 — Chassis geometry confirmation
- Part 9 — Sensor calibration (LiDAR, IMU, camera angle, HSV)
- Part 10 — PS2 controller mapping
- Part 11 — Bench test
- Part 12 — Mode A: known layout (Lab 3003 / two-phase)
- Part 13 — Mode B: continuous explore (unknown tunnel)
- Part 14 — Mode C: Gazebo simulation
- Part 15 — Consolidated troubleshooting & acceptance checklist
- Appendix — Command quick-reference

---

## Part 1 — Operating system installation

### 1.1 Raspberry Pi 4B 4GB: Ubuntu Server 22.04 (64-bit)

1. Download **Raspberry Pi Imager** on any PC:
   `https://www.raspberrypi.com/software/`
2. Insert the microSD card, open Imager.
3. **Choose OS** → "Other general-purpose OS" → "Ubuntu" → **"Ubuntu
   Server 22.04.x LTS (64-bit)"** — not Ubuntu Desktop (headless is
   lighter and this Pi is never operated with a monitor attached), and
   not any 32-bit variant.
4. **Choose Storage** → your microSD card.
5. Click the gear/settings icon before writing: set a hostname (e.g.
   `strawberry-pi`), enable SSH, set a username/password, and — if you
   want WiFi available for initial setup before the wired robot link
   exists — configure WiFi credentials here too.
6. Write, wait for verification, insert into the Pi, power on.
7. Find its IP (router admin page, or `ping strawberry-pi.local` from
   another machine on the same network) and SSH in:
   `ssh <username>@<pi-ip-address>`.
8. `sudo apt update && sudo apt full-upgrade -y && sudo reboot`

### 1.2 Jetson AGX Orin 64GB: JetPack 6.x (Ubuntu 22.04)

**Full detail is in [`docs/JETPACK_SETUP_GUIDE.md`](docs/JETPACK_SETUP_GUIDE.md)** —
read that, not this summary, if the Orin isn't already set up.

The short version:

- **You need JetPack 6.x, which is Ubuntu 22.04 (L4T R36.x).** JetPack
  5.x is Ubuntu 20.04, and ROS 2 Humble has no apt packages for 20.04.
- Check what you have: `cat /etc/nv_tegra_release` (want R36.x) and
  `lsb_release -a` (want 22.04).
- **If you're on JetPack 5.x, do NOT run `do-release-upgrade`.** An
  in-place 20.04→22.04 upgrade breaks the JetPack driver stack. Flash
  JetPack 6 with NVIDIA SDK Manager from a separate x86_64 Ubuntu host PC.
- Set maximum performance before real runs: `sudo nvpmodel -m 0` (persists)
  and `sudo jetson_clocks` (does not persist — re-run after each boot).
- `nvidia-smi` does not exist on Jetson. Use `sudo tegrastats`.

Both machines end up on Ubuntu 22.04, so Part 2 onward is identical for
the Pi and the Orin — the only difference is that `dpkg --print-architecture`
resolves to `arm64` on both, which the ROS 2 apt setup handles itself.

### 1.3 Why there is no VM / Docker / dual-boot section anymore

Earlier revisions of this guide carried a lot of material about running
Ubuntu 22.04 in a VM, in Docker, or dual-booted, because the compute node
was a Windows laptop and none of those paths were clean.

**That is all obsolete.** The AGX Orin runs Ubuntu 22.04 natively as its
only OS — that is what JetPack *is*. There is no host OS to work around,
no hypervisor, no GPU passthrough problem, and no external-drive
performance caveat. This is a materially simpler and more reliable
deployment than the laptop architecture it replaces, and it is why the
GPU-accelerated perception path is viable again (see Part 5 of
`docs/JETPACK_SETUP_GUIDE.md`).

---

## Part 2 — ROS 2 Humble installation (run on BOTH machines)

```bash
# 1. Locale (UTF-8 required)
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Enable the Ubuntu Universe repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# 3. Add the ROS 2 GPG key and apt repository
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS 2 Humble
sudo apt update
```

**On the Pi**:
```bash
sudo apt install -y ros-humble-ros-base python3-argcomplete
```

**On the Orin**:
```bash
sudo apt install -y ros-humble-desktop python3-argcomplete
```

**On BOTH machines** — dev tools + rosdep:
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  python3-vcstool build-essential git

# rosdep must be initialised ONCE per machine before it's usable at all --
# this is a genuinely common first-timer stumbling block: skipping this
# makes every later `rosdep install` fail with a confusing error.
sudo rosdep init
rosdep update

# source ROS 2 in every new shell -- add to ~/.bashrc so you don't have to
# remember this every session
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify** (either machine): `ros2 topic list` should print at least
`/parameter_events` and `/rosout` with no errors.

---

## Part 3 — Gazebo installation (Orin only)

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins ros-humble-xacro ros-humble-robot-state-publisher
```

This targets **Gazebo Classic 11**, not the newer Ignition/Gazebo Sim —
Classic has the mature `libgazebo_ros_ackermann_drive` plugin this
workspace's simulation relies on. Full detail, world generation, and
usage in `GAZEBO_SIMULATION_GUIDE.md` (Part 14 below walks the essential
path).

**Verify**: `gazebo --version` should report `11.x`.

---

## Part 4 — Workspace setup, dependencies, camera driver, build

### 4.1 Get the workspace onto both machines

```bash
mkdir -p ~/strawberry_ws
cd ~/strawberry_ws
# unzip strawberry_amr_ws_orin_and_raspberry_pi_4b.zip here, so you end up
# with ~/strawberry_ws/src/<packages>
```

### 4.2 Core ROS 2 package dependencies (BOTH machines)

```bash
cd ~/strawberry_ws
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-twist-mux \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-rplidar-ros \
  ros-humble-imu-filter-madgwick \
  ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-rmw-cyclonedds-cpp \
  python3-opencv python3-numpy python3-serial
```

This list (and `scripts/install_deps.sh`, which installs the identical
set) is cross-checked directly against every package's real `package.xml`
in this workspace, not assumed — `ackermann-msgs` and `teleop-twist-joy`
were listed here in an earlier revision of this guide but are declared in
no `package.xml` and used in no code anywhere in this project (it stays
in plain `geometry_msgs/Twist` throughout, and uses its own `teleop_ps2`
package rather than the generic `teleop_twist_joy`); removed. This list
is still a convenience pre-install, not the authoritative source — Part
4.6's `rosdep install --from-paths src -y --ignore-src`, run once the
workspace's actual source exists, reads every `package.xml` directly and
will always catch the exact, current, correct list regardless of
anything hand-maintained here drifting out of date.

### 4.3 Why CycloneDDS, not the default FastDDS

`rmw_cyclonedds_cpp` above is deliberate, not incidental: this is a
two-machine deployment communicating over a single wired link, and
CycloneDDS's discovery behaviour is simpler to pin to one network
interface reliably (Part 5 does exactly this) than FastDDS's default
discovery — avoiding ROS 2 traffic accidentally trying to route over
WiFi, which would add latency and unpredictability to a safety-critical
control loop. This is set as the default RMW implementation via
`~/.bashrc` in Part 5, not here — don't set it yet if you want to sanity
check ROS 2 itself first with the default RMW.

### 4.4 The one pip-install myth to actively avoid

**Do not run `pip3 install Rosmaster_Lib`.** No package by that name
exists on PyPI — confirmed by direct query against PyPI during this
project, not assumed. `Rosmaster_Lib.py` is vendored directly into this
workspace's `base_controller` package (from Yahboom's own sample-code
archive) and needs no separate install at all; a normal `colcon build`
picks it up automatically.

### 4.5 Camera driver — Orbbec Astra Pro Plus (Orin AND Pi, real procedure)

**Judgment call #3 from the top of this document applies here.** Build
`ros_astra_camera` from source, into this same workspace:

```bash
# OpenNI2 build dependencies
sudo apt install -y libusb-1.0-0-dev libudev-dev pkg-config \
  libjpeg-dev libturbojpeg0-dev

cd ~/strawberry_ws/src
git clone https://github.com/orbbec/ros_astra_camera.git

# CONFIRM THE BRANCH before building -- this is the one detail in this
# entire document not independently re-verified this session (no web
# access when this was written). Takes ten seconds:
cd ros_astra_camera
git branch -a
# Look for a branch or tag explicitly naming "humble" or "ros2" (e.g.
# `humble` or `ros2-development`) and check it out if the default branch
# you cloned isn't already it:
#   git checkout <humble-branch-name>
cd ~/strawberry_ws

# The repo provides its own udev rule installer -- run it (adds the
# Orbbec permission rules; this is IN ADDITION TO, not a replacement for,
# this workspace's own scripts/setup_udev_rules.sh in Part 6 -- run both):
cd src/ros_astra_camera
sudo bash scripts/create_udev_rules.sh 2>/dev/null || \
  echo "If that script doesn't exist at this path, check the repo's own README for its current name/location -- driver repos do rename install scripts across releases."
cd ~/strawberry_ws
```

Then build it together with the rest of the workspace in 4.7 below —
don't build it in isolation, since this workspace's `rosdep install` step
resolves its ROS package dependencies too.

**Validate this specific driver once built** (after 4.7):
```bash
ros2 launch astra_camera astra.launch.xml   # or astra_pro_plus.launch.xml --
                                              # check `ros2 pkg prefix astra_camera`
                                              # /share/astra_camera/launch/ for the
                                              # exact launch file name shipped
ros2 topic hz /camera/color/image_raw
```
If this produces no topic, the two most common causes, in order: (1) the
udev rules from this step didn't apply — replug the camera after running
`sudo udevadm control --reload-rules && sudo udevadm trigger`; (2) you're
on the wrong branch for Humble — recheck `git branch -a` above.

### 4.6 Build

```bash
cd ~/strawberry_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
echo "source ~/strawberry_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Run this identically on **both** machines — every package needs to be
buildable on both, since `bench.launch.py`/`full_robot.launch.py` (single-
host dev tools) can run on either, even though the field deployment
splits packages across hosts by which launch file you actually run.

**Acceptance check for this whole Part**: `ros2 pkg list | grep
strawberry` (or `| grep -E "base_controller|row_navigation|mission_control"`)
should list every custom package from this workspace, on both machines.

---

## Part 5 — Network setup

**Pi first, then Orin** — the Pi has to be broadcasting its own WiFi
Access Point before the Orin can join it:

```bash
cd ~/strawberry_ws
./scripts/setup_network_pi_orin.sh pi        # on the Pi -- hosts the AP
```
```bash
./scripts/setup_network_pi_orin.sh orin    # on the Orin -- joins it
```

**Why an AP hosted by the Pi, not "both machines join the lab's WiFi"**:
a real Irish polytunnel isn't guaranteed to have any WiFi infrastructure
at all, and this needs to work there, not just somewhere with existing
coverage. The Pi hosting its own network means it works identically in
Lab 3003 and in an actual field with zero external network — nothing to
depend on but the two machines themselves. This creates a dedicated
subnet (`192.168.30.0/24`), pins CycloneDDS to the WiFi interface on each
machine, and sets `ROS_DOMAIN_ID=43` on both — all persisted via
`~/.bashrc` and (Pi) `hostapd`/`dnsmasq`, enabled to auto-start on boot,
and (Orin) a real, persisted NetworkManager profile that auto-reconnects
whenever the AP is in range. Full mechanism in the script's own header
comment and `docs/ORIN_PI_SPLIT_ARCHITECTURE.md`.

**The honest tradeoff, stated plainly**: this is WiFi, not the wired link
an earlier revision of this guide assumed — that assumption didn't
survive contact with the actual physical requirement (the robot has to
drive around; a cable to a stationary Orin doesn't work). WiFi has a
meaningfully worse latency/reliability profile than a cable, and
`row_navigation`/`safety_supervisor`/`twist_mux`/`ekf_node` all run on the
Orin (Part 12/13), so the reactive control/safety loop now depends on
this link. What still holds regardless: teleop and the e-stop button stay
fully local to the Pi (`twist_mux_pi_local`, see
`docs/ORIN_PI_SPLIT_ARCHITECTURE.md`), and `base_controller`'s own
command watchdog forces a hard stop if nothing reaches it for any reason,
including this link dropping. Signal range is a real physical
consideration too — keep the Orin within reasonable WiFi range of
wherever the robot actually drives, not an assumption to leave unchecked.

**Open a NEW terminal on each machine** (so `.bashrc` takes effect), then:
```bash
ping 192.168.30.1     # from the Orin, pinging the Pi
ping 192.168.30.100   # from the Pi, pinging the Orin
```
Both must succeed before continuing. If the Orin can't see the `StrawberryAMR`
network at all, check `sudo systemctl status hostapd dnsmasq` on the Pi first.

---

## Part 6 — udev rules (on the Pi — where the physical devices are)

```bash
cd ~/strawberry_ws
./scripts/setup_udev_rules.sh
```

Creates `/dev/rplidar` (RPLIDAR A1M8) and `/dev/myserial` (YB-ERF01-V2.0
board — this exact name matters, it's `Rosmaster_Lib`'s own constructor
default) symlinks, and a permission rule for the Astra. **If you haven't
already run `ros_astra_camera`'s own udev script from Part 4.5, do that
now too** — this workspace's script and that repo's script cover
different things (symlinks + generic Astra permission vs. the specific
rules that repo's own driver expects) and neither substitutes for the
other.

**Verify**: unplug and replug all three devices, then `ls -l /dev/rplidar
/dev/myserial` should show both as valid symlinks (not "No such file").

---

## Part 7 — Hardware/firmware verification (Rosmaster_Lib)

Two specific, bounded checks — not a vague "test everything":

```bash
# on the Pi
ros2 run base_controller base_controller_node --ros-args \
  --params-file ~/strawberry_ws/src/base_controller/config/base_controller_params.yaml
```

1. **Encoder count**: rotate one rear wheel by hand exactly one full turn,
   `ros2 topic echo /wheel_encoders_raw` — confirm the count changes by
   **836** (verified CPR: 11 PPR base × 1:19 gearbox × 4 quadrature
   edges). If it doesn't match, no code change is needed — this topic is
   diagnostic only; real odometry comes from `Rosmaster_Lib`'s
   `get_motion_data()`, not from integrating these ticks.
2. **Accelerometer scale**: robot stationary and level,
   `ros2 topic echo /imu/data_raw`, confirm `linear_acceleration.z` reads
   close to **9.8**, not **1.0**. If it reads ~1.0: open
   `src/base_controller/base_controller/base_controller_node.py`, find
   `G_TO_MS2 = 9.80665` near the top, change to `G_TO_MS2 = 1.0`. That one
   constant is the entire fix.

**This is the highest-risk integration point in the whole system** —
everything downstream (odometry, EKF, SLAM, control) is only as good as
these two checks passing.

---

## Part 8 — Chassis geometry confirmation

CAD-sourced defaults (from Yahboom's own SolidWorks-exported R2 URDF,
extracted directly from joint origins — see
`docs/MOTOR_AND_GEOMETRY_VERIFICATION.md` for the derivation): wheelbase
`0.2353 m`, front track `0.130 m`, rear track `0.1685 m`, max steer angle
`0.6 rad` (34.4°), giving `R_min = 0.344 m`. These are a materially better
starting point than a from-scratch guess, but a CAD joint limit is a
design value, not a guarantee about your specific assembled unit — confirm
`max_steer_angle` on the bench:

1. Command a steady hard-left turn (`teleop.launch.py`, Part 11), let the
   front wheels settle at their lock.
2. Measure the angle between the chassis centreline and the wheel plane
   with a protractor/phone angle app.
3. If meaningfully smaller than 34.4°, use your smaller measured value —
   update it in **both**
   `src/row_navigation/config/row_navigation_params.yaml`
   (`max_steer_angle`) **and**
   `src/robot_description/urdf/strawberry_amr.urdf.xacro` (`max_steer`
   property) together, then `colcon build --symlink-install` again.

Full procedure and the reasoning behind every number:
`docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md` Part 1.

---

## Part 9 — Sensor calibration

### 9.1 LiDAR (RPLIDAR A1M8) — solo bring-up

```bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/rplidar -p serial_baudrate:=115200 -p frame_id:=laser
ros2 topic hz /scan   # expect ~5.5-10 Hz steady
```
View in RViz2 (Fixed Frame `laser`), confirm a distinct object placed
directly in front shows a return near bearing 0°. **Do the bright-light
test now, not later**: this is a triangulation sensor (Slamtec's own
datasheet: works well indoors "and outdoor environment without direct
sunlight exposure") — scan under normal light, then under the brightest
light you can access, and compare dropout/noise. This is the one genuine
open risk from this specific sensor choice.

### 9.2 IMU — stationary checks

Already covered in Part 7 (accelerometer scale). Additionally: gyro
readings should sit near zero when stationary (`ros2 topic echo
/imu/data` after `imu_filter_madgwick` is running); rotate the robot by
hand through a known 90° and confirm the filtered yaw changes by roughly
that much.

### 9.3 Camera angle — what angle, and why

The shipped default (`cam_tilt=0.61 rad`, 35°) implies the camera is
centred on plants roughly **0.93 m ahead** of the robot — using forward
look-ahead distance, not lateral offset, as the design variable, since the
camera looks ahead as the robot drives, not sideways at whatever's beside
it. Formula: `tilt = atan2(fruiting_height - camera_height,
forward_look_distance)`. Full worked trigonometry, the structured-light
steep-angle tradeoff, and the exact URDF file/property to edit:
`docs/CAMERA_ANGLE_AND_HSV_CALIBRATION_GUIDE.md` Part 1-2. Validate with
the RViz point-cloud-against-a-real-wall check described there — trust
that over any calculated number, including the ones above.

### 9.4 HSV ripe-fruit tuning

Edit `src/plant_perception/config/plant_detector_params.yaml`
(`h1_lo`/`h1_hi`/`h2_lo`/`h2_hi`/`s_lo`/`v_lo`) against real fruit under
real lighting, watching the debug overlay via `rqt_image_view`. Full
live-tuning procedure: `docs/CAMERA_ANGLE_AND_HSV_CALIBRATION_GUIDE.md`
Part 3. **Keep a separate params file per lighting environment** — Lab
3003 values will not transfer outdoors.

---

## Part 10 — PS2 controller mapping

**Plug the USB nano receiver into the Pi, not the Orin.** This is not
interchangeable with where anything else on this project goes: `joy_node`
and `teleop_ps2` are deliberately kept local to the Pi specifically so
manual override keeps working even if the Pi↔Orin WiFi link (Part 5)
drops — see `docs/ORIN_PI_SPLIT_ARCHITECTURE.md`'s "flagged exception"
note. If the receiver were on the Orin instead, teleop would depend on
that link too, defeating the entire point of that design.

**On the terminology**: Yahboom markets this as a "wireless PS2 handle,"
and it's worth being precise about what that actually is, since it
changes what setup looks like. This is **not** Bluetooth — it's Yahboom's
own proprietary 2.4GHz RF link between the handle and that nano receiver,
auto-pairing on power-up (confirmed directly from Yahboom's own listings: "the handle and receiver can be automatically paired" on power-up,
and "no driver is required"). There's no Bluetooth pairing step, no
`bluetoothctl`, nothing to configure — plug the receiver into any USB
port on the Pi, power the handle on, and it should already be talking to
it. If you were looking for Bluetooth settings anywhere, that's why you
won't find any to configure — there's nothing there.

**If `ros2 topic echo /joy` shows nothing and `ros2 run joy joy_node`
logs a permission error**: the underlying `/dev/input/jsX` device is
often owned by the `input` group by default, not directly readable by a
normal user account. Fix once: `sudo usermod -aG input $USER`, then log
out and back in (group membership only takes effect on a fresh login
session, restarting the terminal alone isn't enough).

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```
Press each button/move each stick one at a time, note which array index
changes, enter them in `src/teleop_ps2/config/ps2_mapping.yaml`
(`axis_linear`, `axis_angular`, `deadman_button`, `autonomy_button`,
`estop_button`, `reset_button`). The shipped indices are a starting guess
for a typical adapter, not a verified mapping for your specific unit.

---

## Part 11 — Bench test (before any field/lab run)

On the **Pi alone** (single-host dev convenience — not the field
topology):
```bash
ros2 launch robot_bringup teleop.launch.py
```
Drive manually (deadman held), confirm motion matches stick input. Then:
```bash
ros2 launch robot_bringup bench.launch.py
```
With the robot **elevated on blocks**: enable autonomy, confirm
row-following and the headland bulb-turn execute correctly (`ros2 topic
echo /headland_status`). **While elevated, also watch the two rear wheels
during a full-lock turn** — whether the firmware differentials them is
genuinely unknown from available sources (quantified worst case ≈49%
mismatch if it doesn't — see `docs/MOTOR_AND_GEOMETRY_VERIFICATION.md`);
this is the only way to resolve it for your specific unit.

---

## Part 12 — Mode A: known layout (Lab 3003 / two-phase)

Use this first, even if Mode B is your ultimate goal (see judgment call
#4 at the top).

**Pi**: `ros2 launch robot_bringup pi_hardware_and_control.launch.py`
(sensors + actuation only — this Pi-side command is now identical in
both modes; see `docs/ORIN_PI_SPLIT_ARCHITECTURE.md` if you're
following an earlier revision of this guide that mentioned an
`explore_mode` argument here — that moved to the Orin side when
`row_navigation` itself was corrected to run there, not the Pi)

**Orin, Phase 1 (mapping)**:
```bash
ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py
```
Drive every row (manual or autonomous), watch `/map` and `/plant_targets`
in RViz. When done, **wait 5-10s after the last detection**, then:
```bash
ros2 topic pub --once /mission/command std_msgs/String "data: finish_mapping"
ros2 topic echo /mission/status   # wait for MAP_SAVED
```

**Required edits before Phase 2**: `slam_toolbox_localization.yaml`'s
`map_file_name` must point at the path just saved; confirm `row_spacing`
agrees across `row_navigation_params.yaml` (via `target_half_width`),
`coverage_params.yaml`, and `target_manager_params.yaml`. Rebuild.

**Orin, Phase 2 (navigate)**:
```bash
ros2 launch robot_bringup pi_missionbrain_phase2_nav.launch.py
```
Enable autonomy, then `ros2 topic pub --once /mission/command
std_msgs/String "data: start_nav"`. Watch `/mission/status` step through
`NAVIGATING` → `APPROACHING_PLANT` → `MISSION_COMPLETE`.

Full detail, including the Lab-3003-specific room setup (desk row
spacing, headland clearance, mock plant placement):
`LAB_3003_MOCK_PLANT_AMR_DEPLOYMENT_GUIDE.md`.

---

## Part 13 — Mode B: continuous explore (unknown tunnel, no pre-existing map)

**Pi**:
```bash
ros2 launch robot_bringup pi_hardware_and_control.launch.py
```
(identical command to Mode A — this Pi-side file only runs sensors and
actuation now, so nothing about it changes between modes; the actual
mode selection is entirely which ORIN launch file you run, below)

**Orin**:
```bash
ros2 launch robot_bringup orin_explore_map_navigate.launch.py
```

SLAM and Nav2 run **concurrently** here — the two-phase gate in Mode A
existed specifically because a single 4GB Pi couldn't run both at once;
that constraint doesn't apply to a 16GB Orin. `row_navigation`
reactively discovers each row's corridor via LiDAR after every headland
turn (no pre-known route needed) and reports success/failure on
`/row_found`. Enable autonomy, `start_nav` as above. Watch
`/mission/status`: `EXPLORING` → (optionally) `APPROACHING_PLANT` →
either back to `EXPLORING` or, once `row_navigation` finds no further
corridor, `EXPLORATION_COMPLETE` (the robot stops on its own here — it
does not guess at a different heading). `finish_mapping` works as a
snapshot at any point in this mode, not just once. To retry after
`EXPLORATION_COMPLETE` (e.g. you've repositioned and believe there's more
tunnel): cycle `/autonomy_enable` off then on via the PS2 pad, **then**
send `start_nav` again — both steps are required.

Full mechanism and reasoning: `docs/ORIN_PI_SPLIT_ARCHITECTURE.md`.

---

## Part 14 — Mode C: Gazebo simulation (Orin, optional but recommended before more field time)

```bash
cd ~/strawberry_ws/src/strawberry_amr_gazebo/scripts
python3 generate_polytunnel_world.py --preset lab --num-rows 2 --plant-spacing 0.6 \
  -o ../worlds/irish_polytunnel_lab_small.world   # small world for a first test
ros2 launch strawberry_amr_gazebo gazebo_sim.launch.py \
  world:=$(ros2 pkg prefix strawberry_amr_gazebo)/share/strawberry_amr_gazebo/worlds/irish_polytunnel_lab_small.world
```

Every navigation-relevant node (`row_navigation`, `safety_supervisor`,
`twist_mux`, EKF, `slam_toolbox`, `coverage_planner`, `mission_control`,
Nav2) runs **unchanged** — only `base_controller` and the sensor drivers
are replaced by Gazebo plugins, publishing on identical topics. **What
does not transfer**: HSV perception tuning (Gazebo Classic isn't
photorealistic) and the rear-motor differential question (Gazebo's
Ackermann plugin has its own internal model, unrelated to the real
firmware). Full guide, world presets (`lab` vs `realistic` spacing), and
troubleshooting: `GAZEBO_SIMULATION_GUIDE.md`.

---

## Part 15 — Consolidated troubleshooting & acceptance checklist

- [ ] `ros2 pkg list` shows every custom package on both machines (Part 4).
- [ ] `ping` succeeds both directions over the WiFi link (Part 5).
- [ ] `/dev/rplidar` and `/dev/myserial` both resolve (Part 6).
- [ ] Encoder count = 836/rev; accelerometer reads ~9.8 stationary (Part 7).
- [ ] `max_steer_angle` bench-confirmed, both files updated together if
      it changed (Part 8).
- [ ] LiDAR bright-light test done; camera angle set and validated
      against a real wall in RViz; HSV tuned against real fruit under
      real lighting (Part 9).
- [ ] PS2 mapping confirmed against your actual controller, not the
      shipped guess (Part 10).
- [ ] Elevated bench test done; rear-wheel differential behaviour
      observed and noted (Part 11).
- [ ] Real headland depth/width measured against your actual space, not
      assumed from `coverage_planner`'s schematic default (Part 12,
      `docs/HEADLAND_TURN_GEOMETRY.md`).
- [ ] A hardware e-stop that cuts motor power exists and has been tested
      — this is non-negotiable, independent of every software safety
      layer described above.
- [ ] Test a deliberate network disconnect (walk the Orin out of WiFi
      range, or disable its WiFi adapter) once, in a safe/elevated
      setting: confirm you can still drive manually via the PS2 pad the
      entire time the link is down (this works via `twist_mux_pi_local`,
      entirely on the Pi — see `docs/ORIN_PI_SPLIT_ARCHITECTURE.md`),
      and that autonomous driving correctly stops. Know before relying on
      this in the field: manual driving during a disconnect has no
      obstacle-based safety gating (`safety_supervisor` is Orin-side and
      unreachable) — you are the obstacle detection in that window, on
      top of the deadman
      requirement and the physical e-stop.
- [ ] Confirm compressed camera transport is actually working, not
      silently falling back to raw over WiFi: `ros2 topic list | grep
      compressed` after `camera.launch.py` is running should show
      `/camera/color/image_raw/compressed` and
      `/camera/depth/image_raw/compressedDepth`. This no longer depends
      on `astra_camera`'s own internals (see
      `docs/ORIN_PI_SPLIT_ARCHITECTURE.md`'s "Compressed camera
      transport" section) — if they're missing, check
      `ros-humble-image-transport-plugins` installed correctly and that
      `pi_hardware_and_control.launch.py` is what's running (only that
      file passes `compress:='true'`).
- [ ] `target_manager` persists visited-state across a restart (a JSON
      sidecar next to `semantic_targets.csv`) — confirm this on real
      hardware once: interrupt a Phase 2 run after visiting at least one
      plant, restart it, and confirm that plant is not re-offered. This
      specifically depends on `semantic_mapper` (which keeps running
      during Phase 2 for live re-verification) not rewriting
      `semantic_targets.csv` on every cycle regardless of content — it
      previously did, which changed the file's timestamp constantly and
      defeated the sidecar's staleness check on almost any restart during
      an active run; `semantic_mapper` now only writes when its target set
      has actually changed, so this test should reliably pass.
- [ ] `semantic_mapper` reloads its own previously-saved plants at startup
      rather than starting empty — confirm this on real hardware once:
      interrupt a Phase 1 mapping run after mapping at least one plant,
      restart `semantic_mapper`, and confirm its log line reports loading
      the existing plant(s) rather than an empty map.

---

## Appendix — Command quick-reference

```
# Build (either machine)
cd ~/strawberry_ws && colcon build --symlink-install && source install/setup.bash

# Mode A -- Pi
ros2 launch robot_bringup pi_hardware_and_control.launch.py
# Mode A -- Orin, Phase 1 then Phase 2
ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py
ros2 launch robot_bringup pi_missionbrain_phase2_nav.launch.py

# Mode B -- Pi (identical command to Mode A -- see Part 13)
ros2 launch robot_bringup pi_hardware_and_control.launch.py
# Mode B -- Orin
ros2 launch robot_bringup orin_explore_map_navigate.launch.py

# Mission commands (from Orin, either mode)
ros2 topic pub --once /mission/command std_msgs/String "data: finish_mapping"
ros2 topic pub --once /mission/command std_msgs/String "data: start_nav"
ros2 topic pub --once /mission/command std_msgs/String "data: stop"
ros2 topic echo /mission/status

# Mode C -- Orin only
ros2 launch strawberry_amr_gazebo gazebo_sim.launch.py
```
