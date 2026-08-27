# Lab 3003 Mock-Plant AMR Deployment Guide

This is the **specific worked scenario** for testing in EE-3003 with mock
strawberry plants on elevated desks — the mechanics (install, network,
launch commands, mission commands) are all in
[`docs/MASTER_DEPLOYMENT_COMMANDS.md`](docs/MASTER_DEPLOYMENT_COMMANDS.md)
(every bash command, start to finish) and
[`ALL_IN_ONE_DEPLOYMENT_GUIDE.md`](ALL_IN_ONE_DEPLOYMENT_GUIDE.md) (the
same procedure with the full reasoning behind each step); this document
is the lab-specific context layered on top of both. Read the master
commands guide first if you haven't already.

**Corrected from an earlier revision**: this document previously pointed
at `DEPLOYMENT_GUIDE.md` throughout and referred to "both Pis" — that was
the legacy dual-Pi topology, not the current Pi + AGX Orin split.
`DEPLOYMENT_GUIDE.md` still exists for that legacy path (see its own
heading), but it is not what this document — or the current primary
architecture — uses.

## The scenario, as specified

- Mock strawberry plants sit on desks, elevated **1.0–1.2 m** — this is
  your polytunnel's overhead fruiting-zone analogue, and it's why the
  camera is tilted upward (see `docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md`
  Part 4.3 for setting the real tilt angle to actually see this height).
- Rows of desks stand in for polytunnel rows; the AMR travels the aisles
  between them.
- Mapping is done **manually first**, via the PS2 controller — not
  autonomously. This is deliberate: build a trustworthy map before ever
  letting the robot navigate it on its own.
- Once mapped, the robot signals readiness back to the Orin, the Orin
  confirms, and the robot navigates autonomously, detecting all mock
  plants along the way.
- No arm is present. The robot stops briefly at each mapped plant in
  order, then moves to the next. See `README.md` Section 4 for why this
  is a deliberate design, not a stub.

## Room-specific setup

1. **Desk row layout.** Arrange desks in at least two parallel rows with a
   consistent aisle width between them — pick a width you can hold
   consistent across the whole run (varying it mid-row will confuse the
   RANSAC corridor estimator, which assumes a roughly constant aisle).
   Measure the actual aisle width and set `target_half_width` in
   `row_navigation_params.yaml` to half of it.
2. **Headland clearance.** At the end of each desk row, you need real,
   physically clear space for the headland turn — see
   `docs/HEADLAND_TURN_GEOMETRY.md` for the exact depth/width numbers at
   your configured `row_spacing` (≈1.22 m depth, ≈0.59 m lateral bulge at
   the current 0.50 m default). Measure the actual clear floor space past
   the last desk in each row before your first unattended run; if it's
   tighter than that, reduce `row_spacing` or reroute the layout — do not
   assume the maneuver will "just fit."
3. **Mock plant placement.** Position mock plants so the upward-tilted
   camera can actually see them from the aisle — this depends on your
   confirmed `mast_x`/`mast_z`/`cam_tilt` values (calibration guide Part
   4.3). If detections aren't landing near the real plant position in the
   map, that calibration is the first thing to re-check, not the HSV
   thresholds.
4. **People in the room.** This is an occupied lab, not an empty
   polytunnel. Keep the safety supervisor's `stop_distance`/`slow_distance`
   at cautious values for this environment (people move less predictably
   than static crop-row obstacles), and brief anyone nearby that the robot
   may execute headland turns that swing wider than its own footprint —
   see the lateral bulge figures above.

## Procedure (mapping to `docs/MASTER_DEPLOYMENT_COMMANDS.md` stages)

1. **Stages 1–8**: OS, ROS 2, workspace build, network, udev, hardware
   verification, PS2 mapping — same for every environment, do this once,
   on the bench, before bringing the robot into the lab.
2. **Stage 9–10 in full**: chassis geometry confirmation, then each
   sensor solo, then HSV tuning. Do this on the bench too.
3. **Manual mapping** (Stage 13, but driven manually throughout rather
   than autonomously): launch `pi_hardware_and_control.launch.py` on the
   Pi, `pi_missionbrain_phase1_mapping.launch.py` on the Orin, then drive
   every aisle with the PS2 pad, watching RViz2 on the Orin to confirm
   the map and plant detections look sane as you go.
4. **Signal readiness / save the map**: the `finish_mapping` command
   (Stage 13) is the "robot signals back to the Orin" step —
   `mission_control` saves the map and plant list and reports
   `MAP_SAVED` on `/mission/status`. This is the handshake point: don't
   proceed until you see that status.
5. **Switch to Phase 2** (Stage 13 continued): edit
   `slam_toolbox_localization.yaml`'s `map_file_name`, rebuild, relaunch
   both machines with the Phase 2 launch files. The map loads
   automatically from that config — no manual load step.
6. **Begin autonomous navigation**: enable autonomy on the PS2 pad, then
   send `start_nav` from the Orin. Watch `/mission/status` — you should
   see `NAVIGATING`, then `APPROACHING_PLANT` at each mock plant in turn,
   with a visible pause (`plant_confirm_pause_s`, default 3s) before it
   automatically advances. `MISSION_COMPLETE` once every row is covered.
7. Send `stop` from the Orin at any point to pause the mission cleanly
   (mission-level pause, not the hardware e-stop — keep the PS2 e-stop
   button and, ideally, a hand near the main power switch, within reach
   throughout, especially on the first few runs).

## What "success" looks like for this scenario

- The saved map, viewed in RViz2, resembles the actual desk-row layout.
- `semantic_targets.csv` (see `docs/MASTER_DEPLOYMENT_COMMANDS.md`
  Stage 13) lists a plausible number of entries, roughly matching the
  real mock-plant count, without large numbers of obvious duplicates or
  false positives.
- Phase 2 visits every mapped plant in a sensible order (row by row, not
  jumping erratically) and completes without a headland turn clipping a
  desk or requiring the hardware e-stop.
- This is the honest bar for this stage of the project — see
  `docs/THESIS_TIMELINE_GUIDE.md` for how this fits into the broader
  week's plan, and note that this Lab 3003 validation is a necessary
  step before, not a substitute for, real polytunnel testing: desks are
  not raised growing gutters, and an empty aisle between tables is not a
  bare-soil aisle between tabletop beds — treat a clean run here as
  confidence to proceed to real-environment testing, not as final
  validation of the thesis's core claim.
