#!/usr/bin/env python3
"""
Generates a Gazebo Classic SDF world representing a substrate-tabletop
strawberry polytunnel: parallel elevated growing troughs on discrete
support posts, with strawberry plants at fruiting height, in equally
spaced rows.

WHY POSTS, NOT A CONTINUOUS WALL: row_navigation's RANSAC corridor
estimator specifically models the environment as discrete vertical support
posts at LiDAR height (NOT a continuous foliage wall) -- see
row_nav_node.py's own module docstring. This generator places individual
post cylinders, not a solid hedge, so the simulated LiDAR scan actually
looks like what the real algorithm expects to see. Building a Gazebo world
with continuous walls would LOOK plausible but would not exercise the real
corridor-estimation code path at all.

TWO SCALES, BOTH SUPPORTED -- pick deliberately, don't just take the default:

  --preset lab       Matches the CURRENTLY SHIPPED row_navigation_params.yaml
                      / coverage_params.yaml defaults exactly (row_spacing
                      0.50 m, target_half_width 0.20 m) -- use this to
                      validate the EXACT configuration you are about to
                      test in Lab 3003, before you ever drive the real
                      robot there.
  --preset realistic Wider, commercial-tabletop-scale spacing (see the
                      REALISTIC_* constants below) -- use this to validate
                      qualitative navigation behaviour at a scale closer to
                      a real Irish polytunnel. NOTE: if you use this
                      preset, either update row_navigation_params.yaml /
                      coverage_params.yaml to match (for a fully consistent
                      end-to-end test) or treat this as a separate,
                      wider-scale qualitative check -- don't assume the two
                      configs silently match each other.

HONESTY ABOUT THE "realistic" NUMBERS: unlike the hardware specs elsewhere
in this workspace (which were verified against Yahboom's own source/specs
or a live web search), the REALISTIC_* spacing constants below are a
REASONED ESTIMATE from general knowledge of UK/Irish tabletop soft-fruit
systems (trough width, walking-aisle width, and the resulting row pitch),
not a fetched, citable figure -- this workspace's web-search tool was not
available when this file was written. If you have a real, grower-verified
or literature-sourced spacing for your specific target polytunnel, use
`--row-spacing`, `--half-width`, and `--trough-width` to override these
defaults directly; don't trust the built-in numbers as authoritative.

Usage:
    python3 generate_polytunnel_world.py --preset lab -o ../worlds/irish_polytunnel_lab.world
    python3 generate_polytunnel_world.py --preset realistic -o ../worlds/irish_polytunnel_realistic.world
    python3 generate_polytunnel_world.py --num-rows 8 --row-spacing 1.4 --plant-spacing 0.4 \
        -o ../worlds/custom.world
"""
import argparse

# ---- lab preset: EXACTLY matches the shipped algorithm config ----
LAB_ROW_SPACING = 0.50
LAB_HALF_WIDTH = 0.20
LAB_TROUGH_WIDTH = 0.50 - 2 * 0.20   # = 0.10 m -- tight, matches the narrow lab/mock scale

# ---- realistic preset: reasoned estimate, see module docstring ----
REALISTIC_ROW_SPACING = 1.20
REALISTIC_HALF_WIDTH = 0.45
REALISTIC_TROUGH_WIDTH = 1.20 - 2 * 0.45   # = 0.30 m

DEFAULT_ROW_LENGTH = 8.0
DEFAULT_NUM_ROWS = 6
DEFAULT_POST_SPACING = 2.0        # structural support posts along a row
DEFAULT_PLANT_SPACING = 0.30      # individual plant spacing along a row
DEFAULT_TROUGH_HEIGHT = 1.10      # matches the project's established 1.0-1.2m
                                   # fruiting-height convention (Lab 3003 guide)
DEFAULT_POST_RADIUS = 0.02


def post_model(name, x, y, height, radius):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {height/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius}</radius><length>{height:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{height:.3f}</length></cylinder></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def trough_model(name, x0, x1, y, height, width):
    length = x1 - x0
    cx = (x0 + x1) / 2.0
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx:.3f} {y:.3f} {height:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length:.3f} {width:.3f} 0.15</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.3f} {width:.3f} 0.15</size></box></geometry>
          <material><ambient>0.35 0.25 0.15 1</ambient><diffuse>0.4 0.28 0.18 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def plant_model(name, x, y, trough_height):
    """A simple foliage blob (green sphere-ish box) sitting on the trough,
    with a few small red spheres as ripe berries hanging just below/around
    it -- enough visual/geometric structure for plant_perception's HSV
    detector to have SOMETHING red to find, without pretending Gazebo
    Classic's rendering is photorealistic (see the sim URDF's own honesty
    note on this -- tune HSV thresholds on the real camera, not in sim)."""
    foliage_z = trough_height + 0.08
    berry_positions = [
        (x - 0.03, y - 0.04, trough_height - 0.03),
        (x + 0.02, y + 0.03, trough_height - 0.05),
    ]
    berries = ""
    for i, (bx, by, bz) in enumerate(berry_positions):
        berries += f"""
    <model name="{name}_berry_{i}">
      <static>true</static>
      <pose>{bx:.3f} {by:.3f} {bz:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>0.012</radius></sphere></geometry>
          <material><ambient>0.75 0.05 0.05 1</ambient><diffuse>0.85 0.08 0.08 1</diffuse></material>
        </visual>
      </link>
    </model>"""
    return f"""
    <model name="{name}_foliage">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {foliage_z:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.14 0.14 0.10</size></box></geometry>
          <material><ambient>0.10 0.35 0.10 1</ambient><diffuse>0.15 0.45 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>{berries}"""


def threshold_model(name, x, y0, y1, height, length_along_x=0.03):
    """A low, thin raised strip crossing the aisle -- stands in for a real
    door threshold / floor-tile-gap seam in Lab 3003's Alice Perry
    Engineering Building corridors. Deliberately narrow along the direction
    of travel (3cm) and low (a few mm to ~1.5cm, configurable) -- a real
    threshold is a brief bump, not a ramp or a wall; making it too tall or
    too long would train a policy against an obstacle that doesn't
    resemble what it will actually meet on the real floor."""
    width = y1 - y0
    cy = (y0 + y1) / 2.0
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {cy:.3f} {height/2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length_along_x:.3f} {width:.3f} {height:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length_along_x:.3f} {width:.3f} {height:.4f}</size></box></geometry>
          <material><ambient>0.5 0.45 0.35 1</ambient><diffuse>0.55 0.5 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def build_world(num_rows, row_spacing, row_length, post_spacing, plant_spacing,
                 trough_height, trough_width, post_radius, half_width,
                 thresholds=None, threshold_height=0.012):
    models = []
    origin_x = 0.0
    first_row_y = 0.0

    for row in range(num_rows):
        row_y = first_row_y + row * row_spacing
        # posts and trough on BOTH sides of this aisle -- these are shared
        # with the ADJACENT aisle's far side in a real continuous system,
        # but modelling them per-aisle (near side only, at +/- half_width
        # from this aisle's centreline) keeps the generator simple and is
        # geometrically equivalent for a single-sided or end-of-tunnel row;
        # for interior rows the two aisles' post lines will coincide at
        # y = row_y (mod row_spacing) +/- half_width by construction, which
        # is intentional and correct for a real shared-trough layout.
        for side, sign in [("L", +1), ("R", -1)]:
            post_y = row_y + sign * half_width
            n_posts = max(2, int(row_length / post_spacing) + 1)
            post_xs = [origin_x + p * post_spacing for p in range(n_posts)]
            post_xs = [px for px in post_xs if px <= origin_x + row_length]
            if len(post_xs) < 2:
                # max(2, ...) above signals "always generate at least 2
                # posts" (RANSAC needs >=2 points for a line fit), but for a
                # short row combined with wide post_spacing the boundary
                # filter above could still drop it back down to 0 or 1
                # (e.g. row_length=1.0, post_spacing=2.0 requests 2 posts but
                # only the p=0 one survives the filter) -- not triggered by
                # this generator's own defaults, but a real trap for a short
                # custom row. Force the guarantee explicitly rather than
                # trusting request-then-filter to preserve it.
                post_xs = [origin_x, origin_x + row_length]
            for p, px in enumerate(post_xs):
                models.append(post_model(
                    f"row{row}_{side}_post{p}", px, post_y, trough_height, post_radius))
            models.append(trough_model(
                f"row{row}_{side}_trough", origin_x, origin_x + row_length,
                post_y, trough_height, trough_width))

            # Plant count derived from the SAME [0.15, row_length-0.05]
            # placement window the boundary check below actually enforces,
            # not from row_length alone -- the previous int(row_length /
            # plant_spacing) formula ignored the 0.20 m total margin,
            # under-generating by up to one plant depending on how
            # plant_spacing divides into the margin-adjusted span (e.g.
            # row_length=8.0, plant_spacing=0.9 requested 8 but 9 positions
            # actually fit within bounds).
            n_plants = max(1, int((row_length - 0.20) / plant_spacing) + 1)
            for pl in range(n_plants):
                plx = origin_x + 0.15 + pl * plant_spacing
                if plx > origin_x + row_length - 0.05:
                    continue
                models.append(plant_model(
                    f"row{row}_{side}_plant{pl}", plx, post_y, trough_height))

        # thresholds: span the FULL aisle width (post line to post line),
        # at explicit x positions along the row -- deliberately NOT
        # auto-generated/spaced, since real door/tile thresholds occur at
        # specific, known locations, not periodically
        if thresholds:
            for i, tx in enumerate(thresholds):
                models.append(threshold_model(
                    f"row{row}_threshold{i}", origin_x + tx,
                    row_y - half_width, row_y + half_width, threshold_height))

    return "\n".join(models)


WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="irish_polytunnel">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- diffuse, indoor-polytunnel-like lighting rather than harsh direct
         sun; a rough approximation only, see the sim URDF's note on
         Gazebo Classic's rendering not being photorealistic -->
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>

{models}

  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--preset', choices=['lab', 'realistic'], default=None,
                     help="Use the 'lab' (matches shipped algorithm config) or "
                          "'realistic' (wider, commercial-scale estimate) spacing "
                          "preset. Overridden by any explicit flag below.")
    ap.add_argument('--num-rows', type=int, default=DEFAULT_NUM_ROWS)
    ap.add_argument('--row-spacing', type=float, default=None,
                     help='Aisle centreline-to-centreline spacing (m).')
    ap.add_argument('--half-width', type=float, default=None,
                     help='Half-aisle-width, aisle centreline to post line (m). '
                          'MUST match row_navigation_params.yaml target_half_width '
                          'for a fully consistent test.')
    ap.add_argument('--trough-width', type=float, default=None,
                     help='Width of the elevated growing trough structure (m).')
    ap.add_argument('--row-length', type=float, default=DEFAULT_ROW_LENGTH)
    ap.add_argument('--post-spacing', type=float, default=DEFAULT_POST_SPACING)
    ap.add_argument('--plant-spacing', type=float, default=DEFAULT_PLANT_SPACING)
    ap.add_argument('--trough-height', type=float, default=DEFAULT_TROUGH_HEIGHT)
    ap.add_argument('--post-radius', type=float, default=DEFAULT_POST_RADIUS)
    ap.add_argument('--thresholds', type=str, default=None,
                     help='Comma-separated x positions (m, from row start) for '
                          'raised floor thresholds spanning the aisle, e.g. '
                          '"2.0,5.0" -- for GL-FOPID RL tuning terrain (see '
                          'docs/GL_FOPID_RL_TUNING_GUIDE.md). None by default: '
                          'existing worlds are unaffected unless you ask for this.')
    ap.add_argument('--threshold-height', type=float, default=0.012,
                     help='Threshold height in m -- default 1.2cm, a real '
                          'door-threshold/tile-seam bump, not a ramp or wall.')
    ap.add_argument('-o', '--output', required=True)
    args = ap.parse_args()

    if args.preset == 'lab':
        row_spacing = LAB_ROW_SPACING
        half_width = LAB_HALF_WIDTH
        trough_width = LAB_TROUGH_WIDTH
    elif args.preset == 'realistic':
        row_spacing = REALISTIC_ROW_SPACING
        half_width = REALISTIC_HALF_WIDTH
        trough_width = REALISTIC_TROUGH_WIDTH
    else:
        row_spacing = REALISTIC_ROW_SPACING
        half_width = REALISTIC_HALF_WIDTH
        trough_width = REALISTIC_TROUGH_WIDTH

    # explicit flags always win over the preset
    if args.row_spacing is not None:
        row_spacing = args.row_spacing
    if args.half_width is not None:
        half_width = args.half_width
    if args.trough_width is not None:
        trough_width = args.trough_width

    if 2 * half_width >= row_spacing:
        raise SystemExit(
            f"REFUSING to generate: 2*half_width ({2*half_width:.3f}) >= "
            f"row_spacing ({row_spacing:.3f}) -- adjacent rows' post lines "
            f"would overlap or collide, which is not a valid layout. "
            f"This is exactly the bug that was fixed in "
            f"row_navigation_params.yaml's target_half_width -- see that "
            f"file's comments.")

    thresholds = None
    if args.thresholds:
        thresholds = [float(x) for x in args.thresholds.split(',')]

    models = build_world(args.num_rows, row_spacing, args.row_length,
                          args.post_spacing, args.plant_spacing,
                          args.trough_height, trough_width, args.post_radius,
                          half_width, thresholds=thresholds,
                          threshold_height=args.threshold_height)

    with open(args.output, 'w') as f:
        f.write(WORLD_TEMPLATE.format(models=models))

    print(f"Wrote {args.output}")
    print(f"  {args.num_rows} rows, row_spacing={row_spacing:.3f}m, "
          f"half_width={half_width:.3f}m, trough_width={trough_width:.3f}m, "
          f"clear aisle={2*half_width:.3f}m")
    print(f"  Robot should spawn at approximately (x={-0.5:.2f}, "
          f"y={0.0:.2f}) facing +x, at the entrance to row 0.")
    if args.preset == 'lab' or (row_spacing == LAB_ROW_SPACING and half_width == LAB_HALF_WIDTH):
        print("  This matches the SHIPPED row_navigation_params.yaml / "
              "coverage_params.yaml defaults exactly.")
    else:
        print("  This does NOT match the shipped algorithm config defaults "
              "(row_spacing=0.50, target_half_width=0.20) -- update those "
              "yaml files too if you want a fully consistent end-to-end "
              "test at this scale, per this script's own module docstring.")


if __name__ == '__main__':
    main()
