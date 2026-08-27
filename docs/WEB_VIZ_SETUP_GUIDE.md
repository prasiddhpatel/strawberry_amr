# R2 Web Viz — Setup and Verification Guide

Browser-based live view of R2: a 2D top-down map (occupancy grid, real
driven trajectory, assumed/planned trajectory), a persistent map-frame
RGB point-cloud reconstruction (accumulated over time via real tf2
fusion, not just the current frame), and a live camera stream — run from
a single package (`r2_web_viz`) on the AGX Orin and viewed from any
browser on the same LAN, the Orin's own monitor or an Android phone.

Written by a Claude Code session that has no ROS 2 toolchain in its
development environment (see `CLAUDE.md`'s working-environment caveat) —
everything ROS-side below is unexecuted, syntax-checked only, EXCEPT
`geometry.py`'s pure math (quaternion→rotation-matrix, point transforms,
voxel keying, RGB packing), which genuinely runs here under plain pytest
with no ROS dependency (see `test/test_geometry.py`) and includes a hand-
derived numerical case (robot at map (2,3) yawed 90°, camera 0.1 m
forward, a point 1 m ahead of the camera checked against an independently
computed expected map-frame result), not just self-consistency checks.
The frontend (the actual browser code) *was* runtime-tested: served over a
local HTTP server, every asset path confirmed reachable, the PointCloud2
binary decoder and its map-frame axis remap verified against hand-built
synthetic messages with known byte-level content, the 3D robot-marker
rotation (ROS yaw → three.js scene rotation) verified numerically to
match with no sign error, and the pose-to-yaw formula checked against
`row_nav_node.py`'s own Python formula for agreement. What genuinely has
not been checked: the actual WebGL/Canvas rendering in a real browser,
and the whole ROS-side launch/runtime chain — rosbridge, web_video_server,
depth_image_proc, and `map_accumulator_node.py`'s tf2 lookups and
PointCloud2 encoding — actually running and producing the topics the
frontend expects. All of that needs a real run on the Orin or the Ubuntu
VM — see `docs/UBUNTU_VM_VERIFICATION_HANDOFF.md` for the same "verify
what a text-only session couldn't" pattern applied there.

## Is any of this ROS1-only tooling? (No — verified, not assumed)

The package name `r2_web_viz` is this project's own name, not a reference
to Cruise's `webviz` tool (a ROS1-era browser dashboard with real, known
ROS2/rosbag2 gaps) — that tool is not used anywhere here. What this
package actually depends on was checked against primary sources (GitHub
branches/releases, index.ros.org) rather than assumed from general
familiarity, specifically because this exact category of tooling has real
ROS1/ROS2 history to get wrong:

- **`rosbridge_suite`**: has a `humble` branch; `rosbridge_server`'s
  `package.xml` on that branch depends on `rclpy`/`rosbridge_library` (no
  `rospy`) — a native ROS2 implementation. Released on index.ros.org for
  Humble at v2.0.8, status RELEASED/RECOMMENDED. `apt install
  ros-humble-rosbridge-server`.
- **`web_video_server`**: dedicated `ros2` branch, `rclcpp`-based,
  bloom-released for Humble since Oct 2024 (currently v3.1.1).
  `apt install ros-humble-web-video-server`.
- **`depth_image_proc`**: part of `image_pipeline`'s `humble` branch,
  released v3.0.9. The `point_cloud_xyzrgb.cpp` source was fetched
  directly to confirm `PointCloudXyzrgbNode`'s default topic names match
  this package's launch-file remaps exactly.
- **Wire protocol**: the rosbridge JSON-over-WebSocket protocol is
  backend-agnostic — ROS2's build adds optional QoS/action fields on top
  of the same core `subscribe`/`publish`/`call_service` ops, it doesn't
  replace them. No protocol-level ROS1/ROS2 incompatibility for a browser
  client.
- **`roslibjs`**: RobotWebTools runs CI against ROS2 Humble/Jazzy
  backends. One real caveat found and applied here: ROS2 needs
  fully-qualified message type strings (`nav_msgs/msg/OccupancyGrid`), not
  the ROS1 short form (`nav_msgs/OccupancyGrid`) — `app.js`'s topic
  subscriptions use the fully-qualified form. (rosbridge_library's ROS2
  build does have a documented fallback that normalizes the short form
  automatically, so the short form wasn't actually broken, but there's no
  reason to rely on a compatibility shim when the correct form costs
  nothing.)

Bag replay — the specific thing `webviz` had real ROS2 gaps in — is not
used anywhere in this package. This is a live topic viewer, not a
log-replay tool.

## What's new here vs. what's off-the-shelf

ROS-to-browser bridging, video streaming, and the raw per-frame RGB
point-cloud fusion are NOT reimplemented — those are existing, maintained
ROS 2 packages, wired together in `web_viz.launch.py`. What's genuinely
new: the frontend (`src/r2_web_viz/web/`), one small addition to
`row_nav_node.py` (a read-only path publisher — see below), and
`map_accumulator_node.py` — this package's own ROS node, which uses real
`tf2_ros` lookups (not a browser-side approximation) to (a) accumulate
`depth_image_proc`'s raw per-frame `/r2/points` into a persistent,
voxel-downsampled, map-frame reconstruction on `/r2/map_points`, and (b)
republish a properly tf2-composed map-frame robot pose on
`/r2/robot_pose_map`. See that file's module docstring for exactly why a
browser-side fix wasn't the right call — this project's real frame chain
(checked against `ekf.yaml` and `slam_toolbox*.yaml` before writing any
of this, not assumed) has a genuine, non-identity `map→odom` correction
published by `slam_toolbox` in both mapping and localization modes, which
a 2-hop JS composer would either have to reinvent or get wrong.

## Dependencies (install once, on the Orin)

```bash
sudo apt install ros-humble-rosbridge-suite \
                  ros-humble-web-video-server \
                  ros-humble-depth-image-proc \
                  ros-humble-rclcpp-components
```

`rclcpp_components` ships with a base ROS 2 install; the other three
usually don't and need the explicit install above. `map_accumulator_node`
itself only needs `tf2_ros`, `sensor_msgs_py`, and `numpy` — all part of
a standard ROS 2 Python install already used elsewhere in this workspace
(`row_nav_node.py` already imports `numpy`), so no additional apt install
should be needed for it specifically; `colcon build` will fail loudly on
a missing dependency if that assumption is wrong for a given install.

## Running it

```bash
colcon build --packages-select r2_web_viz --symlink-install
source install/setup.bash
ros2 launch r2_web_viz web_viz.launch.py
```

Then, from any device on the same network as the Orin:

```
http://<orin-hostname-or-ip>:8000/
```

The page defaults to connecting its rosbridge WebSocket to whatever
hostname you loaded the page from — if that's wrong (e.g. viewing through
a proxy or a different DNS name than the Orin resolves to on the LAN), the
`rosbridge` field in the top bar lets you type the correct host and
reconnect without editing anything.

## Verifying it actually works (do this on real hardware/VM, not here)

1. **Static assets load.** Open the browser's devtools console — there
   should be no 404s for `js/vendor/three.module.js`, `OrbitControls.js`,
   or `roslib.min.js`. These are vendored (committed into the repo, not
   loaded from a CDN) specifically so this works with no internet access
   in the field — confirm nothing is silently falling back to a network
   fetch.
2. **rosbridge connects.** The connection-status pill in the top bar
   should go from amber ("connecting…") to green ("connected — \<host\>")
   within a couple of seconds of the robot stack being up. If it stays
   red, check `ros2 node list | grep rosbridge` and the browser console
   for the WebSocket error.
3. **2D map and robot pose.** First, `ros2 topic echo /r2/robot_pose_map`
   — if this produces nothing, `map_accumulator_node` isn't finding a
   `map`→`base_link` transform (check `ros2 run tf2_tools view_frames`
   for a broken/missing link in the tree — most likely SLAM not running,
   or not yet published its first `map→odom` correction). Once that's
   flowing: drive or teleoperate the robot a short distance and confirm
   the green real-trajectory trail actually grows and the robot icon
   rotates with heading. Specifically check what a browser-side pose
   composition would have gotten wrong: trigger a SLAM loop closure (or
   just let mapping run long enough for a scan-matching correction) and
   confirm the drawn pose does NOT visibly jump relative to the occupancy
   grid at that moment — `/odometry/filtered` alone would have jumped;
   the whole point of routing through `map_accumulator_node`'s real tf2
   lookup is that it shouldn't. During row-following, confirm the blue
   dashed assumed-trajectory line appears ahead of the robot (needs the
   `row_nav_node.py` fix below — `ros2 topic echo /row_nav/assumed_path`
   is a faster first check than the browser).
4. **3D point cloud — raw feed first, then accumulation.** Confirm
   `/r2/points` (the raw, per-frame, camera-relative cloud) actually
   publishes (`ros2 topic hz /r2/points`) before troubleshooting anything
   downstream — `depth_image_proc`'s `point_cloud_xyzrgb` component
   publishes nothing if its three input topics (`rgb/image_rect_color`,
   `rgb/camera_info`, `depth_registered/image_rect`, remapped to this
   project's actual camera topics in `web_viz.launch.py`) aren't all
   flowing, most likely `depth_registration:=true` not actually being set
   on the camera driver for whatever launch profile you're running. Once
   `/r2/points` is confirmed flowing, check `/r2/map_points`
   (`ros2 topic hz /r2/map_points`, default ~1 Hz) — if `/r2/points` is
   fine but `/r2/map_points` isn't, `map_accumulator_node` is most likely
   failing its own tf2 lookup (check its log output for the throttled
   "No transform" warning) rather than anything frontend-side. Then, the
   thing that actually matters: point the camera at something distinctive,
   let a few seconds of accumulation happen, then physically move the
   robot (or camera) away and confirm the point cloud in the browser
   **stays** where it was rather than disappearing — that's the
   difference this pass exists to make. `points-count` in the 3D panel
   should only grow (or hold steady once the visible area is fully
   covered at the configured `voxel_size_m`), never reset to near-zero
   while the robot keeps moving through already-seen space.
5. **Camera stream.** If the `<img>` tag in the Camera tab never loads,
   check `http://<orin>:8080/stream?topic=/camera/color/image_raw&type=mjpeg`
   directly in a browser tab — that isolates whether the problem is
   `web_video_server` itself or something in `r2_web_viz`'s frontend.

## The one change to existing code

`row_nav_node.py` gained a new publisher, `/row_nav/assumed_path`
(`nav_msgs/Path`), projecting the locally-fitted corridor centerline
(already computed every cycle for the FOPID controller) into the map frame
using odometry position it previously received but discarded. It is
read-only — nothing it computes feeds back into `heading_err`, `lateral`,
`omega`, or `speed`. See the `_publish_assumed_path` method's docstring in
that file for the exact scope, and `docs/CURRENT_STATUS_AND_NEXT_STEPS.md`
/ this session's commit history for when it was added.

## Known limitations

- **No decay or re-verification of stale voxels.** A voxel, once written,
  keeps its stored color forever (most-recent-observation-wins on
  overwrite, but nothing ever removes an entry). If something in the
  scene genuinely moves — a person walks through, a chair gets
  repositioned — the accumulated map will show a ghost of its old
  position alongside (or instead of, depending on exact voxel overlap)
  its new one, indefinitely. This is the correct tradeoff for what this
  view is actually for (mapping a largely-static polytunnel/lab
  structure), but it means this is not a live occupancy view of moving
  objects — the 2D map and camera panels are what to check for that.
- **`max_voxels` (default 500,000) is a hard cap, not a smart one.** Once
  reached, new spatial regions stop being added (existing voxels still
  refine) — logged once, not silently. Tune `voxel_size_m`/`max_voxels`
  for the actual space being mapped rather than relying on the default
  for anything larger than a lab-scale run.
- **Color per voxel is single-sample, not averaged.** Whichever
  observation of a voxel arrives last wins; no running average across
  multiple views of the same point. Simpler and was judged good enough
  for a viewer (not a basis for further perception), but a lighting
  change between observations will show as a visible seam between
  regions accumulated at different times, not a blend.
- **The dict-based voxel accumulation loop is plain Python, not
  vectorized beyond the key computation.** Untested against this
  project's actual point-cloud density on real Orin hardware — if it
  turns out to be a bottleneck, a proper spatial-hash library (e.g.
  Open3D's `voxel_down_sample`) is the documented next step in
  `map_accumulator_node.py` itself, not a guess made here.
- The camera stream resolution is whatever `camera.launch.py` is
  currently configured for (424×240@15fps by default) — deliberately left
  unchanged; see that file's own docstring for why, and the mapping-app
  scoping conversation this package came out of for the tradeoff.
