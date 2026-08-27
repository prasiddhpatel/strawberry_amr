# Camera Angle Selection & HSV Ripe-Fruit Calibration Guide

Two questions, answered together because the second one only makes sense
once the first is settled: **what angle should the camera actually be
tilted to**, and **how do you tune HSV thresholds to detect ripe
strawberries** once it's mounted there.

## Part 1 — What angle should the camera be tilted to?

**Short answer: this workspace's current default is 0.61 rad (35°), and
that's a defensible starting point — but "defensible" is not the same as
"bench-verified," and the right number depends on a design choice you
should make deliberately, not accept blindly.** Here's the reasoning, so
you can decide for yourself rather than trust a number handed to you.

### The geometry

The camera is mounted near the front of the chassis
(`mast_x=0.050m`) and tilted up (`cam_tilt`), specifically so it looks
**forward and up** at plants ahead of the robot as it drives — not
straight up at whatever happens to be beside it right now. That's an
important distinction: it changes which distance matters in the
trigonometry.

```
required_tilt = atan2(fruiting_height - camera_height, forward_look_distance)
```

- `camera_height` = `mast_z` in the URDF = 0.450 m (measure yours — see
  Part 2 below for how).
- `fruiting_height` = this project's established convention, 1.0–1.2 m
  (matches the Lab 3003 mock-plant height and typical tabletop trough
  height).
- `forward_look_distance` = **the design choice** — how far ahead of the
  robot do you want the camera centred on? A short distance (e.g. 0.4 m)
  needs a steep tilt and centres only the very next plant; a longer
  distance (e.g. 1.2 m) needs a shallower tilt and centres plants further
  ahead, giving the robot more advance warning but a less tightly-centred
  view of what's immediately alongside it.

| forward look distance | required tilt |
|---|---|
| 0.40 m | 58° |
| 0.60 m | 47° |
| 0.80 m | 39° |
| **0.93 m** | **35°  ← this workspace's current default** |
| 1.20 m | 28° |

So the shipped `cam_tilt=0.61` implicitly assumes you want the camera
centred on plants roughly 0.9 m ahead — a reasonable middle ground, not an
arbitrary number, but **not bench-verified against your actual mounted
camera and lighting either.**

### A real, physical reason not to go too steep

Structured-light depth cameras (which is what the Astra Pro Plus is)
project an infrared pattern and triangulate distance from how it deforms.
At a steep angle of incidence against a relatively flat surface (like the
face of a trough), the projected pattern stretches and the triangulation
geometry degrades — this is real, well-established structured-light
physics, not something specific to this camera. It means very steep tilts
(pushing toward 50-60°+ to centre a close target) trade off depth-image
quality for tight framing. This workspace's exact vertical FOV and
minimum-range figures for the Astra Pro Plus haven't been independently
re-verified this session — check Orbbec's own spec sheet for your exact
unit before treating any specific FOV number as settled — but the
underlying physical concern is legitimate and worth respecting: **prefer a
longer look-ahead distance (shallower tilt) over a short one if you have
to choose,** rather than tilting aggressively to frame the nearest plant
as tightly as possible.

### What to actually do

1. Start with the shipped default (35°, `cam_tilt=0.61` in the URDF) —
   it's a reasonable, physically-reasoned starting point.
2. After mounting the camera at that angle (Part 2 below), run the
   validation check in Part 2.4 — if the point cloud looks distorted or
   noisy at your actual working distance, that's your real signal to
   shift toward a shallower angle (longer look-ahead), not a number from
   this or any other document.
3. If you want a different look-ahead distance deliberately (e.g. your
   aisles are longer and you'd rather see further ahead), recompute
   `cam_tilt` with the formula above and your own numbers.

## Part 2 — Physically setting and measuring the angle

This is the procedure from
`docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md` Part 4.3, repeated here
with the exact file and line you'll edit, so this document is
self-contained.

### 2.1 Measure what you've physically mounted

1. Mount the camera at your chosen angle (start with the shipped
   35°/0.61 rad if you haven't decided otherwise).
2. Place a digital inclinometer (or a phone level/clinometer app — either
   works, sub-degree accuracy isn't necessary here) flat against the
   robot's chassis deck, zero it there so chassis tilt from an uneven
   floor doesn't corrupt the reading.
3. Place the same tool flat against the camera housing's front face
   (parallel to the lens's optical axis). Read the angle.

### 2.2 Enter it in the code — exact file and line

Open **`src/robot_description/urdf/strawberry_amr.urdf.xacro`**, find:

```xml
<xacro:property name="mast_x"       value="0.050"/>
<xacro:property name="mast_z"       value="0.450"/>   <!-- camera height on mast -->
<xacro:property name="cam_tilt"     value="0.61"/>    <!-- ~35 deg UP toward overhead fruit -->
```

Convert your measured degrees to radians (`radians = degrees * pi / 180`)
and replace the `cam_tilt` value. If you also measured a different mast
height or forward offset, update `mast_z`/`mast_x` too.

**Sign check before you trust the number**: this URDF's convention is
`rpy="0 ${-cam_tilt} 0"` on the camera joint (see that same file, the
`camera_joint` definition a few lines below the properties) — a
**positive** `cam_tilt` value tilts the camera **up**. If your rebuild
shows the camera pointing down in RViz, the sign convention on your
specific mount is flipped relative to this file's assumption — flip the
sign rather than guessing.

### 2.3 Rebuild and check in RViz2

```bash
colcon build --symlink-install --packages-select robot_description
ros2 launch robot_description description.launch.py
```

In RViz2, set Fixed Frame to `base_link`, add a TF display, and confirm
the `camera_link` frame's orientation visibly points up-and-forward the
way you expect — not level, not down.

### 2.4 Validate against real depth data — the actual ground truth

Park the robot facing a real, flat, vertical surface (a wall, a large
flat board) at a known, measured distance. Launch the camera alone
(`ros2 launch sensor_bringup camera.launch.py`), open RViz2, set Fixed
Frame to `base_link`, and display `/camera/depth/points`.

- **If your tilt angle and the URDF's `cam_tilt` value genuinely match**,
  the wall will appear flat and close to vertical in the point cloud.
- **If the wall appears to lean, curve, or looks visibly distorted**, one
  of two things is true: either the physical angle and the URDF value
  don't actually match (go back to 2.1–2.2), or you've tilted steep
  enough that the structured-light degradation from Part 1 is showing up
  — in which case, per Part 1's guidance, prefer reducing the tilt (longer
  look-ahead distance) over chasing a perfect wall at a steep angle.

This check is the actual ground truth for whether your camera angle
setup is correct — trust it over any calculated number, including the
ones in Part 1.

## Part 3 — HSV ripe-fruit calibration

**Before this section existed, `plant_perception` had no external config
file at all** — every threshold was hardcoded in
`plant_detector_node.py`'s source, meaning tuning it meant editing Python
and rebuilding every time, with no way to override at launch. That's
fixed now: `src/plant_perception/config/plant_detector_params.yaml` is a
real file, wired into every launch file that starts
`plant_detector_node` (`full_robot.launch.py`, both
`pi_missionbrain_phase*` files, and the Gazebo sim launch). Edit that
YAML file, not the Python source.

### 3.1 The parameters, exactly as they appear in the file

| Parameter | Meaning | Default |
|---|---|---|
| `h1_lo`, `h1_hi` | Lower red hue band (OpenCV hue is 0-179; red wraps around 0) | 0, 10 |
| `h2_lo`, `h2_hi` | Upper red hue band (the other side of the wrap) | 170, 179 |
| `s_lo` | Saturation floor — filters washed-out pink/grey | 90 |
| `v_lo` | Brightness floor — filters near-black shadow | 60 |
| `min_contour_area` | Minimum blob size in pixels to count as a detection | 600 |
| `roi_top_fraction` | 0 = whole image; e.g. 0.6 keeps only the top 60% (useful to exclude the bare aisle floor once your camera tilt is confirmed) | 0.0 |

The node already correctly combines both hue bands (`h1_*` and `h2_*`)
with two `cv2.inRange()` calls — you don't need to write any code to
handle the red hue wraparound, just tune the numbers.

### 3.2 Live tuning procedure

1. Bring up the camera and detector:
   ```bash
   ros2 launch sensor_bringup camera.launch.py
   ros2 run plant_perception plant_detector_node --ros-args \
       --params-file src/plant_perception/config/plant_detector_params.yaml
   ```
2. View the debug overlay (`publish_debug: true` is already the default)
   to see exactly what the current thresholds are catching:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```
   Select the detector's debug/annotated image topic.
3. Place a real or mock ripe strawberry in frame under your actual
   working lighting (not a photo on a screen — real lighting matters).
4. Adjust `h1_lo`/`h1_hi`/`h2_lo`/`h2_hi`/`s_lo`/`v_lo` in
   `plant_detector_params.yaml`, then re-run the node (or use
   `ros2 param set /plant_detector_node <name> <value>` for fast
   iteration without relaunching, then copy the final values back into
   the yaml file once you're happy — `ros2 param set` changes don't
   persist across a relaunch).
5. Narrow the ranges until the debug overlay marks the ripe berry cleanly
   and ignores foliage, soil, and the mock trough's own colour.

### 3.3 Validate, don't just eyeball

Place a known number of ripe (or mock-ripe, coloured appropriately)
berries at measured positions. Drive/push the robot past them and check:
- `/plant_targets` reports a detection for each one within a few seconds
  of it entering frame.
- No detections appear where there's no berry (false positives from
  background).
- `semantic_targets.csv` (see `docs/MASTER_DEPLOYMENT_COMMANDS.md` Stage 13)
  ends up with roughly
  the right count, without excessive duplicates at slightly different
  XY for the same physical berry — if you see duplicates, that's usually
  better addressed by increasing `semantic_mapper`'s dedup radius
  (`semantic_mapper_params.yaml`) than by over-tightening HSV.

### 3.4 Lighting changes everything — retune per environment

HSV separates colour from brightness better than raw RGB, but the
thresholds still shift with the colour temperature of your light source,
shadows, and the specific red of your test berries (real vs. mock).
**Indoor Lab 3003 values will very likely not work outdoors or under
different lighting** — keep a separate copy of
`plant_detector_params.yaml` per environment (e.g.
`plant_detector_params_lab3003.yaml`) and pass the right one via
`--params-file` at launch, rather than assuming one tuned value works
everywhere.
