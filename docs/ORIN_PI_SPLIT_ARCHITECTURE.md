# Orin + Pi Split Architecture

This replaces the original dual-Pi (2GB "realtime" + 4GB "mission brain")
topology with **one Raspberry Pi 4B 4GB + one Orin**. Both topologies'
launch files still exist in this workspace — pick the one that matches
your actual hardware. This document explains the new one: what runs
where, why, and the two mission modes it supports.

**Corrected since an earlier revision of this document**: that revision
put `row_navigation`, `safety_supervisor`, `twist_mux`, and `ekf_node` on
the Pi on latency grounds. That was a real, substantive deviation from the
operator's explicit architecture spec — **"all sensors/actuation on the
Pi, everything compute-heavy on the Orin"** — which should have been
flagged for confirmation before being silently implemented that way. It
wasn't, and the operator caught it. All four nodes now run on the Orin,
matching the spec literally. The honest tradeoff this creates, and the one
deliberate, explicitly-flagged exception to it, are both covered below —
not glossed over.

## Node placement

| Runs on | Nodes | Why here |
|---|---|---|
| **Pi (4GB)** | `rplidar_ros`, `imu_filter_madgwick`, Astra camera driver, `base_controller` (Rosmaster_Lib), `link_monitor_node`, `joy_node` + `teleop_ps2`, `twist_mux_pi_local` | True sensing and true actuation only — this is the one box with USB hardware, so every physical device connects here. `link_monitor_node` is trivial compute that only makes sense asked from here ("can I hear the Orin"). `joy_node`/`teleop_ps2` are a **flagged exception**, not a silent one — see below. `twist_mux_pi_local` is what makes that exception actually deliver on its promise of link-independence — see "The honest consequence" section below. |
| **Orin** | `ekf_node`, `row_navigation`, `safety_supervisor`, `twist_mux`, `slam_toolbox`, Nav2 (`controller_server`/`planner_server`/`behavior_server`/`bt_navigator`/`lifecycle_manager`), `plant_perception`, `semantic_mapper`, `target_manager` (known-layout mode only), `coverage_planner` (known-layout mode only), `mission_control` | Everything computational, per the operator's literal spec — including the control/estimation/safety loop, which an earlier revision kept on the Pi instead. |

### The honest consequence of this correction — and what's since been fixed

The reactive row-following and obstacle-safety loop depends on the
Pi↔Orin network link being up — `row_navigation`, `safety_supervisor`,
and the *autonomous* side of arbitration all live on the Orin now, and
none of their output can reach the Pi if the link drops. That part of the
tradeoff is real and unavoidable given the operator's literal architecture
spec, and it's not being glossed over.

**What's no longer true, since being corrected**: an earlier revision of
this document said the only things that keep working during a link drop
are the physical e-stop and a blunt local watchdog stop — with manual
teleop *also* effectively lost, since it still had to round-trip through
the Orin's `twist_mux` for arbitration. That's fixed with a **second,
Pi-local `twist_mux` instance** (`twist_mux_pi_local`, config
`twist_mux_pi_local.yaml`, launched from `pi_hardware_and_control.launch.py`)
that arbitrates purely between:

- `/cmd_vel_teleop` (priority 100) — from `joystick_node`, on this same Pi.
- `/cmd_vel_remote` (priority 10) — whatever the Orin's own `twist_mux`
  already decided between Nav2 and row-following, having crossed the
  network (that node's output was renamed from `/cmd_vel` to
  `/cmd_vel_remote` specifically so the two stages don't collide).
- `/e_stop` as a lock (priority 255), fed directly by `joystick_node`'s
  own button — zero network dependency.

Both of this stage's real inputs are meaningful without the network:
teleop is local by definition, and `/cmd_vel_remote` simply times out
(0.5s) and stops competing if the link is down — the exact same, standard
`twist_mux` timeout mechanism already used everywhere else in this
project, just applied a second time, locally. `base_controller` itself
needed **zero changes** for this — it still just subscribes to `/cmd_vel`,
now produced by this Pi-local stage instead of directly by the Orin.
(An earlier attempt at this same problem tried to duplicate this
arbitration by hand inside `base_controller` itself, with new topics and a
bespoke priority timer — reverted once this simpler, already-existing
two-stage `twist_mux` design was found; duplicating logic a standard,
tested package already provides is exactly the mistake this project
avoided when it chose `Rosmaster_Lib`'s `set_car_motion` over a hand-rolled
Ackermann converter, and the same reasoning applies here.)

**A serious, separate bug this surfaced — not specific to today's change,
predating it entirely**: `twist_mux` (and `twist_mux_pi_local`) select the
highest-priority topic that has received *any* message within its
timeout, completely independent of that message's actual value.
`joystick_node` used to publish `/cmd_vel_teleop` unconditionally on every
`/joy` callback — an explicit zero `Twist` when the deadman was released,
not silence. That means the teleop topic, at priority 100, would **never
time out**, and would therefore **permanently outrank every autonomous
behaviour** underneath it, regardless of whether the deadman was actually
held — for the entire time `joystick_node` happens to be running, which is
always, since every launch file in this workspace includes it as the
ever-present human override. Autonomous driving could never have actually
produced motion, on any hardware, at any point, until this was fixed.
Fixed by making `joystick_node` publish `/cmd_vel_teleop` **only** while
the deadman is actually held — see that node's own module docstring for
the full explanation. This is unrelated to the Pi/Orin split itself and
would have affected the original dual-Pi topology identically; it was
only found while working through today's request.

So the real, current picture during a link drop: autonomous driving stops
(nothing else is possible — `row_navigation`/`safety_supervisor` genuinely
can't be reached), but a human holding the PS2 pad's deadman can keep
driving the robot manually, continuously, with the e-stop button also
working locally the entire time. This was a deliberate design decision,
reasoned through explicitly rather than assumed: a human actively holding
a deadman on a directly-wired controller is a materially different risk
profile from unsupervised autonomous motion during a fault — it doesn't
weaken the e-stop or the "no commands anywhere means stop" guarantee, it
just stops teleop from being incorrectly caught by either of those.

**One limit that did not change and cannot, by the nature of the
problem**: `safety_supervisor`'s obstacle-triggered e-stop is Orin-side,
so it cannot reach the Pi during a link drop either way. Driving manually
during a drop means driving without that specific protection — your own
judgement, the deadman requirement, and the physical e-stop are what's
left in that window.

`link_monitor_node`'s docstring already reflects this design accurately —
no further change needed there.

### `joy_node`/`teleop_ps2` stay on the Pi — no longer just a partial exception

This was flagged as a deliberate exception to the literal "sensors/
actuation on Pi, compute on Orin" rule when the split was first
corrected, with an honest caveat at the time: since `twist_mux` was
remote, teleop's command still had to round-trip through the Orin, so
manual driving wasn't actually fully link-independent in practice, only
the deadman/e-stop release was guaranteed local. That gap is now closed —
see the section above. `joy_node`/`teleop_ps2` stay on the Pi, and manual
driving really is now fully local, matching the reasoning that justified
the exception in the first place. If you'd still rather move them to the
Orin for strict literal consistency with the "compute goes to the
Orin" rule — accepting that manual override would then also depend on
the link — that remains a one-line change in
`pi_hardware_and_control.launch.py`, not done by default.

## Two mission modes — pick per deployment, not once for good

### Known layout (Lab 3003, or any tunnel you've already mapped)

- Pi: `ros2 launch robot_bringup pi_hardware_and_control.launch.py`
  (sensors + actuation only now — no mode argument needed here anymore,
  since nothing mode-specific runs on the Pi in this corrected split)
- Orin, Phase 1: `ros2 launch robot_bringup
  pi_missionbrain_phase1_mapping.launch.py` (the file name says "pi" for
  historical reasons — it runs fine on the Orin; nothing in it is
  Pi-specific)
- Orin, Phase 2 (after `finish_mapping`): `ros2 launch robot_bringup
  pi_missionbrain_phase2_nav.launch.py`

Both Orin files now also start `ekf_node`, `row_navigation` (base
config, no exploration override), `safety_supervisor`, and `twist_mux` —
see their own updated module docstrings. See
`docs/MASTER_DEPLOYMENT_COMMANDS.md` Stage 13 for the full procedure.

### Unknown tunnel, no pre-existing map (the new capability)

- Pi: `ros2 launch robot_bringup pi_hardware_and_control.launch.py`
  (**identical command to known-layout mode** — this file no longer has
  a mode argument at all, since the only thing that ever differed between
  modes, `row_navigation`'s `exploration_mode` parameter, isn't launched
  from the Pi anymore)
- Orin: `ros2 launch robot_bringup orin_explore_map_navigate.launch.py`
  (this file always launches `row_navigation` with the exploration
  override baked in — it's unconditionally dedicated to this mode, so
  there's no toggle needed here either)

**How exploration actually works, mechanically**: `row_navigation`
(Orin, in this mode) has an opt-in `exploration_mode` parameter (see
`src/row_navigation/config/row_navigation_params_explore_override.yaml`).
When true, at the end of each headland turn it tries to reactively find
the next row's corridor purely from LiDAR returns (arriving here over the
network from the Pi) — no pre-known route needed, since the RANSAC
corridor estimator already works from raw scan data, not a map. It
reports the outcome on `/row_found`. `mission_control`
(`phase='explore'`) reflects that into `/mission/status` and declares
`EXPLORATION_COMPLETE` + stops autonomy when `row_navigation` reports it
found nothing — it does not try to guess a different heading or otherwise
improvise; per `row_navigation`'s own design, deciding "what next" after a
failed reacquire is a deliberate mission-level choice, not an automatic
one. SLAM runs continuously throughout, with no "finish_mapping" gate
required — that gate only existed because the *original 4GB Pi* could not
run live SLAM and a live Nav2 costmap at once; that RAM constraint doesn't
apply here, since both run on the Orin.

`coverage_planner` and `target_manager` are deliberately not launched in
this mode — both assume a known row count/layout, which is exactly what
an unknown-tunnel exploration doesn't have. `semantic_mapper`'s live
detections feed `mission_control` directly.

**Orin commands are identical to the known-layout workflow** —
`finish_mapping` (usable as a snapshot at any point, not just once),
`start_nav` (also restarts a completed exploration if the operator cycles
`/autonomy_enable` on the PS2 pad first), `stop`. See
`mission_control_node.py`'s own module docstring for the complete command
reference.

## Compressed camera transport

The RGB-D stream is the one genuinely bandwidth-heavy thing crossing the
Pi↔Orin WiFi link (everything else — `/scan`, odometry, commands — is
small messages at modest rates).

**Corrected from an earlier revision**: that revision assumed
`astra_camera`'s driver published via `image_transport::CameraPublisher`
internally, meaning compressed topics would appear automatically once
`ros-humble-image-transport-plugins` was installed — no launch-file
change needed on the publishing side. That assumption didn't hold up:
checked directly against `orbbec/ros2_astra_camera`'s actual GitHub
repository (not just its documentation), and the pull request that adds
`image_transport` support to that driver ("Using image_transports", #2)
is still **open, not merged**, as of this check.

The fix doesn't depend on that PR merging, or on `astra_camera`'s
internals at all: `camera.launch.py` now runs `image_transport`'s own
generic `republish raw compressed` (and `raw compressedDepth` for depth)
node directly on the Pi, reading whatever plain `sensor_msgs/Image` the
driver produces — which every camera driver produces, `image_transport`-
aware or not — and compressing it itself. This is a strictly more robust
design than relying on an unverified upstream assumption, not merely a
workaround for one. Enabled via `camera.launch.py`'s `compress` argument
(default `false`); only `pi_hardware_and_control.launch.py` passes
`compress:='true'`, since that's the one caller whose camera stream
actually crosses a network hop.

On the receiving end, all three "real deployment" Orin-side launch
files (`pi_missionbrain_phase1_mapping.launch.py`,
`pi_missionbrain_phase2_nav.launch.py`,
`orin_explore_map_navigate.launch.py`) run `image_transport`'s
`republish compressed raw`/`compressedDepth raw` to decompress locally,
then point `plant_detector_node` at the decompressed topic — deliberately
a *different* topic name than the Pi's own raw topic, not a coincidence:
subscribing to the exact same name the Pi publishes would let DDS
non-deterministically hand the node either the local decompressed copy
or the original raw stream still crossing the network, silently undoing
the entire point of compressing it. Not wired into `full_robot.launch.py`
or `gazebo_sim.launch.py` (single-host, no network hop for the camera at
all) or the original dual-Pi topology (camera and `plant_perception` are
on the same "mission-brain" Pi there too) — compression only helps where
there's an actual network hop to cross, and `camera.launch.py`'s
`compress` argument defaults to `false` precisely so those callers don't
pay for it.

**Verify, one line**: `ros2 topic list | grep compressed` after
`camera.launch.py` (with `compress:='true'`) is running should show
`/camera/color/image_raw/compressed` and
`/camera/depth/image_raw/compressedDepth` — this is now guaranteed by
`republish` running locally, not contingent on any upstream package's
internal implementation, but still worth the ten-second check before a
long unattended run.

## Network setup

**WiFi, not wired** — corrected from an earlier revision of this document
and the script it describes, which assumed a physical Ethernet cable
between the Pi and the Orin. That doesn't hold up once the Orin isn't
mounted on the robot: the robot has to actually drive around the
polytunnel, and a cable to a Orin that isn't on it makes that
impossible. The Pi hosts its own WiFi Access Point (`hostapd` + `dnsmasq`)
and the Orin joins it as a client — deliberately not "both machines
join whatever WiFi router happens to be nearby," since a real polytunnel
isn't guaranteed to have any WiFi infrastructure at all, and this needs to
work there, not just in a lab. See
`scripts/setup_network_pi_orin.sh`'s own header comment for the full
reasoning, including the honest tradeoff this introduces (WiFi's worse
latency/reliability profile than the wired link this replaces — mitigated
by, not eliminated by, teleop/e-stop staying fully local via
`twist_mux_pi_local`, and `base_controller`'s own command watchdog still
forcing a hard stop if nothing reaches it).

`scripts/setup_network_pi_orin.sh` — **not** the same script as
`scripts/setup_network.sh` (that one is for the original dual-Pi
topology, where both boxes genuinely are on the robot and a wired link
between them remains correct; keep both). Run
`./scripts/setup_network_pi_orin.sh pi` on the Pi and
`./scripts/setup_network_pi_orin.sh orin` on the Orin, Pi first
(it needs to be broadcasting before the Orin can join it). Handles the
two platforms' different network-management stacks correctly (Pi:
netplan/systemd-networkd running `hostapd`+`dnsmasq` for the AP; Orin:
a real, persisted NetworkManager profile via `nmcli` joining that AP with
a pinned static IP) — see that script's own header comment for the full
detail.

## What changed from an earlier reviewed external bundle for this same robot

An external AI-generated codebase for this same Yahboom R2 + Pi+Orin
setup was reviewed while building this. Several of its claims were
checked directly against the real, vendored `Rosmaster_Lib.py` source and
found wrong — most importantly **`car_type=2` is not a fix, it's a
regression**: the library defines `CARTYPE_X3_PLUS = 0x02` and
`CARTYPE_R2 = 0x05` as four completely distinct constants (confirmed by
reading the actual source, not a description of it) — this workspace
correctly uses `car_type=5`. Also found and not carried over: the same
`Rosmaster_Lib` PyPI-install assumption already disproven earlier this
project (no such package exists — confirmed by direct query against
PyPI), a `v=0,ω≠0` Ackermann command pattern with Nav2's Spin recovery
still registered (the exact hazard this workspace's own
`navigate_to_pose_ackermann.xml` was built specifically to avoid), two
different nodes both broadcasting `odom→base_link` TF with no
coordination between them, and a `NetworkInterfaceAddress: auto` DDS
config (this workspace pins the actual interface explicitly instead,
same principle `setup_network.sh`/`setup_network_pi_orin.sh` both
already apply: don't let DDS silently pick whichever interface it finds
first, since on a machine with more than one active interface that could
easily be the wrong one — pin it, whether that interface ends up being
wired or, as in this topology, WiFi). One genuinely good idea *was*
adapted from it: a network-link-loss monitor, rebuilt here as
`link_monitor_node` with an important, deliberate difference — honestly
framed as an operator-awareness tool, not implied to be load-bearing for
safety when it isn't.
