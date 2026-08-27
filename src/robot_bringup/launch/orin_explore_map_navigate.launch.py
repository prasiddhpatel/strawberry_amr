"""
ORIN -- continuous explore + map + navigate, for a genuinely UNKNOWN
tunnel with no pre-existing map (Pi+Orin topology; pair with
robot_bringup/launch/pi_hardware_and_control.launch.py on the Pi -- that
file needs NO launch argument for this pairing. It is mode-agnostic by
design (see its own module docstring): it launches the same fixed sensor
and actuation node set regardless of mission phase, since all mode-
specific logic (including this Orin-side node's own `phase=explore`
parameter, set below) lives on the Orin side of the split. An earlier
revision of this docstring instructed passing `explore_mode:=true` to the
Pi-side file; that file has never declared any such launch argument
(confirmed by grep) -- it was a stale leftover from before the
mode-agnostic refactor and has been removed here.

Runs on the Orin, NOT a Pi -- see docs/ORIN_PI_SPLIT_ARCHITECTURE.md
for the full node-by-node placement reasoning. Does NOT include any sensor
driver launches (lidar_imu.launch.py / camera.launch.py) -- in this
topology every physical sensor is wired to the Pi, which publishes their
topics over the network link; this Orin only ever subscribes to them.

WHY THIS IS ONE CONTINUOUS LAUNCH, NOT A TWO-PHASE PAIR LIKE THE ORIGINAL
WORKFLOW: the original Phase1/Phase2 split existed specifically because a
single 4GB Pi could not run live SLAM and a live Nav2 costmap at the same
time (see pi_missionbrain_phase1_mapping.launch.py's own docstring, and
docs/MOTOR_AND_GEOMETRY_VERIFICATION.md for the RAM numbers behind that
constraint). That constraint is specific to a 4GB Pi -- it does not apply
here: this file runs entirely on the Orin side of the split, where SLAM
and Nav2 running concurrently is both architecturally standard (Nav2's
global costmap subscribing to a live, still-updating /map topic from
slam_toolbox is a normal, well-supported ROS 2 pattern, not a hack) and
comfortably within a 16GB machine's resources. The original two-phase
launch files still exist and are still the right choice whenever the
layout IS already known precisely (e.g. Lab 3003's desk rows) -- this file
is specifically for when it is not.

HOW EXPLORATION ACTUALLY WORKS (the mechanism, not just the launch
plumbing): row_navigation (HERE, on the Orin -- see the correction note
below) has its own opt-in `exploration_mode` parameter -- when true, at
the end of each headland turn it tries to reactively find the next row's
corridor purely from LiDAR returns (which arrive here over the network
from the Pi), with no pre-known route needed, and reports success/failure
on /row_found. This Orin-side mission_control_node (phase='explore')
just reflects that into /mission/status and declares
EXPLORATION_COMPLETE + stops when row_navigation reports it found nothing
-- see mission_control_node.py's own module docstring for the full state
machine. coverage_planner is DELIBERATELY NOT launched here: its
boustrophedon route generation assumes a known row count/layout, which is
exactly what this mode does not have.

CORRECTED from an earlier revision, which put row_navigation (along with
safety_supervisor, twist_mux, and ekf_node) on the Pi instead of here, on
latency grounds. That deviated from the operator's explicit architecture
spec ("all sensors/actuation on the Pi, everything compute-heavy on the
Orin") without being flagged for confirmation -- it should have been,
and wasn't. All four now run here. See
pi_hardware_and_control.launch.py's module docstring for the honest
consequence: the reactive corridor-discovery and obstacle-safety loop now
depends on the Pi<->Orin network link (mitigated by the physical e-stop,
which is independent of all of this, and base_controller's own local
command watchdog on the Pi, which still stops the robot on a link drop --
just via a hard stop rather than a graceful fallback).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    rownav = get_package_share_directory('row_navigation')
    semmap = get_package_share_directory('semantic_mapper')
    mission = get_package_share_directory('mission_control')
    plant = get_package_share_directory('plant_perception')

    ekf = os.path.join(bringup, 'config', 'ekf.yaml')
    mux = os.path.join(bringup, 'config', 'twist_mux.yaml')
    slam = os.path.join(bringup, 'config', 'slam_toolbox.yaml')   # mapping mode, continuous
    nav2_params = os.path.join(bringup, 'config', 'nav2_params.yaml')
    rownav_cfg = os.path.join(rownav, 'config', 'row_navigation_params.yaml')
    rownav_explore = os.path.join(rownav, 'config',
                                  'row_navigation_params_explore_override.yaml')
    semmap_cfg = os.path.join(semmap, 'config', 'semantic_mapper_params.yaml')
    mission_cfg = os.path.join(mission, 'config', 'mission_control_params.yaml')
    plant_cfg = os.path.join(plant, 'config', 'plant_detector_params.yaml')
    plant_zeroshot_cfg = os.path.join(
        plant, 'config', 'plant_detector_zeroshot_params.yaml')

    detector = LaunchConfiguration('detector')
    is_zeroshot = PythonExpression(["'", detector, "' == 'zeroshot'"])

    use_nav2 = LaunchConfiguration('use_nav2')
    nav2_lifecycle_nodes = ['controller_server', 'planner_server',
                            'behavior_server', 'bt_navigator']

    return LaunchDescription([
        DeclareLaunchArgument(
            'detector', default_value='hsv',
            description="Which plant detector to run: 'hsv' (classical, "
                       "no GPU/model needed -- plant_detector_node.py) or "
                       "'zeroshot' (plant_detector_zeroshot_node.py, "
                       "Grounding DINO + MobileSAM, needs downloaded weights "
                       "-- see docs/ZERO_SHOT_PERCEPTION_GUIDE.md). Both "
                       "publish to /plant_targets; semantic_mapper doesn't "
                       "know or care which one is running."),

        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Start Nav2 for plant approach during exploration. '
                        'false runs pure explore+map with no approach '
                        'behaviour, e.g. for a first connectivity/mapping-'
                        'only smoke test before trusting approaches too.'),

        # relocated from the Pi -- see pi_hardware_and_control.launch.py's
        # module docstring for the full reasoning. row_navigation gets
        # BOTH config files here (base + the explore override), unlike the
        # known-layout launch files which only pass the base config --
        # this file is unconditionally dedicated to explore mode, so there
        # is no need for the IfCondition/UnlessCondition branching an
        # earlier revision used when explore_mode was a toggle on this
        # same node's launch site; here it's simply always on.
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen', parameters=[ekf]),
        Node(package='safety_supervisor', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[mux],
             remappings=[('/cmd_vel_out', '/cmd_vel_remote')]),  # NOT /cmd_vel anymore -- that name is now the Pi-local mux's own output; see docs/ORIN_PI_SPLIT_ARCHITECTURE.md
        Node(package='row_navigation', executable='row_nav_node',
             name='row_nav_node', output='screen',
             parameters=[rownav_cfg, rownav_explore]),

        # SLAM: mapping mode, running CONTINUOUSLY -- no finish_mapping
        # gate required to reach this point, unlike the two-phase workflow
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[slam]),

        # Decompress the camera stream locally -- see
        # pi_missionbrain_phase1_mapping.launch.py's identical nodes for the
        # full reasoning (same topology, same network hop, same fix).
        Node(package='image_transport', executable='republish',
             name='color_decompress', output='screen',
             arguments=['compressed', 'raw'],
             remappings=[('in/compressed', '/camera/color/image_raw/compressed'),
                        ('out', '/camera/color/image_raw_decompressed')]),
        Node(package='image_transport', executable='republish',
             name='depth_decompress', output='screen',
             arguments=['compressedDepth', 'raw'],
             remappings=[('in/compressedDepth', '/camera/depth/image_raw/compressedDepth'),
                        ('out', '/camera/depth/image_raw_decompressed')]),

        # perception + semantic landmark mapping -- grows continuously as
        # new plants are found, same nodes/config as every other mode.
        # color_topic/depth_topic overridden to the locally-decompressed
        # topics above -- see the republish nodes' own comments for why.
        Node(package='plant_perception', executable='plant_detector_node',
             name='plant_detector_node', output='screen',
             condition=UnlessCondition(is_zeroshot),
             parameters=[plant_cfg, {
                 'color_topic': '/camera/color/image_raw_decompressed',
                 'depth_topic': '/camera/depth/image_raw_decompressed',
             }]),
        # detector:='zeroshot' -- Grounding DINO + MobileSAM, needs
        # downloaded weights (see docs/ZERO_SHOT_PERCEPTION_GUIDE.md).
        # color_topic/depth_topic overridden to the locally-decompressed
        # topics -- see the republish nodes' own comments for why this
        # must not be the same topic name the Pi publishes.
        Node(package='plant_perception', executable='plant_detector_zeroshot_node',
             name='plant_detector_zeroshot_node', output='screen',
             condition=IfCondition(is_zeroshot),
             parameters=[plant_zeroshot_cfg, {
                 'color_topic': '/camera/color/image_raw_decompressed',
                 'depth_topic': '/camera/depth/image_raw_decompressed',
             }]),
        Node(package='semantic_mapper', executable='semantic_mapper_node',
             name='semantic_mapper_node', output='screen',
             parameters=[semmap_cfg]),
        # NOTE: target_manager is intentionally NOT launched here.
        # target_manager's job (load a Phase-1 CSV, sequence a KNOWN plant
        # list in row-order) doesn't apply when nothing is known in
        # advance -- semantic_mapper's live detections feed mission_control
        # directly for the approach trigger. If you want the row-ordered
        # sequencing behaviour applied to what's been found SO FAR (e.g.
        # useful once exploration is well underway), target_manager can
        # still be added to this launch file; it already handles a missing
        # initial CSV gracefully (see target_manager_node.py).

        # Orin command interface + mission sequencing, phase='explore'
        Node(package='mission_control', executable='mission_control_node',
             name='mission_control_node', output='screen',
             parameters=[mission_cfg, {'phase': 'explore', 'zeroshot_enabled': is_zeroshot}]),

        # Nav2, for the per-plant approach once something is found --
        # deliberately NO coverage_planner here, see module docstring.
        # Wrapped in a GroupAction+IfCondition (same pattern as
        # full_robot.launch.py) so use_nav2:=false genuinely skips all
        # five nodes, not just the ones a stray edit happened to touch.
        GroupAction(condition=IfCondition(use_nav2), actions=[
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', output='screen',
                 parameters=[nav2_params],
                 remappings=[('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen', parameters=[nav2_params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'autostart': True, 'node_names': nav2_lifecycle_nodes}]),
        ]),
    ])
