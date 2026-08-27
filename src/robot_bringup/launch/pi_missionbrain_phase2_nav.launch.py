"""
ORIN -- Pi+Orin topology, PHASE 2: NAVIGATION (known layout, e.g. Lab 3003).

(Renamed from pi_missionbrain_phase2_harvest.launch.py -- there is no
harvest logic in this workspace; the 5-DOF arm is a separate, not-yet-
confirmed companion project, out of scope here. This phase autonomously
covers the mapped rows, approaches each mapped plant via Nav2, pauses
briefly to confirm arrival, and moves to the next one -- no arm hand-off.)

Loads the map saved at the end of Phase 1 (slam_toolbox LOCALIZATION mode --
NOT mapping; see slam_toolbox_localization.yaml), runs the full Nav2 stack
for point-to-point plant approach, the boustrophedon coverage planner for
the global row order, and mission_control for sequencing + the Orin
command interface.

CORRECTED from an earlier revision: two real bugs fixed here together.
(1) row_navigation, safety_supervisor, twist_mux, and ekf_node now run
HERE (Orin), not on the Pi -- see pi_hardware_and_control.launch.py's
module docstring for the full reasoning and the honest tradeoff (the
control/safety loop now depends on the Pi<->Orin network link). (2) This
file previously referenced `plant_cfg` in plant_detector_node's parameters
without ever defining it -- a real, undetected NameError-on-launch bug
(py_compile doesn't catch undefined-name errors; caught by a pyflakes
sweep across every launch file in the workspace after being found by
inspection in a sibling file). Also removed the sensor driver includes
(lidar_imu.launch.py / camera.launch.py) that were incorrectly present
here -- all sensors run on the Pi now, not here; their topics simply
arrive over the network.

REQUIRED before launching: edit slam_toolbox_localization.yaml's
map_file_name to point at the .posegraph saved at the end of Phase 1, and
confirm row_spacing matches across row_navigation_params.yaml (via
target_half_width), coverage_params.yaml, and target_manager_params.yaml
-- ALL_IN_ONE_DEPLOYMENT_GUIDE.md Part 12 flags all three.

ORIN COMMANDS once this is running (see mission_control_node.py):
    ros2 topic echo /mission/status
    ros2 topic pub --once /mission/command std_msgs/String "data: start_nav"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    rownav = get_package_share_directory('row_navigation')
    coverage = get_package_share_directory('coverage_planner')
    semmap = get_package_share_directory('semantic_mapper')
    targetmgr = get_package_share_directory('target_manager')
    mission = get_package_share_directory('mission_control')
    plant = get_package_share_directory('plant_perception')

    ekf = os.path.join(bringup, 'config', 'ekf.yaml')
    mux = os.path.join(bringup, 'config', 'twist_mux.yaml')
    slam = os.path.join(bringup, 'config', 'slam_toolbox_localization.yaml')
    nav2_params = os.path.join(bringup, 'config', 'nav2_params.yaml')
    rownav_cfg = os.path.join(rownav, 'config', 'row_navigation_params.yaml')
    coverage_cfg = os.path.join(coverage, 'config', 'coverage_params.yaml')
    semmap_cfg = os.path.join(semmap, 'config', 'semantic_mapper_params.yaml')
    targetmgr_cfg = os.path.join(targetmgr, 'config', 'target_manager_params.yaml')
    mission_cfg = os.path.join(mission, 'config', 'mission_control_params.yaml')
    plant_cfg = os.path.join(plant, 'config', 'plant_detector_params.yaml')
    plant_zeroshot_cfg = os.path.join(
        plant, 'config', 'plant_detector_zeroshot_params.yaml')

    detector = LaunchConfiguration('detector')
    is_zeroshot = PythonExpression(["'", detector, "' == 'zeroshot'"])


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


        # NOTE: no sensor driver includes here -- see module docstring.

        # relocated from the Pi -- see pi_hardware_and_control.launch.py
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen', parameters=[ekf]),
        Node(package='safety_supervisor', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[mux],
             remappings=[('/cmd_vel_out', '/cmd_vel_remote')]),  # NOT /cmd_vel anymore -- that name is now the Pi-local mux's own output; see docs/ORIN_PI_SPLIT_ARCHITECTURE.md
        Node(package='row_navigation', executable='row_nav_node',
             name='row_nav_node', output='screen', parameters=[rownav_cfg]),

        # localize against the frozen Phase-1 map (publishes /map + map->odom)
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

        # plant perception kept running for re-verification at each approach
        # (confirms the mapped landmark is still a ripe strawberry before
        # confirming arrival, rather than trusting a Phase-1 detection that
        # may now be stale -- e.g. already picked, or no longer ripe).
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
        # loads Phase 1's saved semantic_targets.csv at startup -- see
        # target_manager_node.py module docstring for why this load is
        # essential, not optional, to get a genuinely "pre-mapped" trajectory
        Node(package='target_manager', executable='target_manager_node',
             name='target_manager_node', output='screen',
             parameters=[targetmgr_cfg]),

        # explicit global route + mission sequencing + Orin command interface
        Node(package='coverage_planner', executable='coverage_planner_node',
             name='coverage_planner_node', output='screen',
             parameters=[coverage_cfg]),
        Node(package='mission_control', executable='mission_control_node',
             name='mission_control_node', output='screen',
             parameters=[mission_cfg, {'phase': 'nav', 'zeroshot_enabled': is_zeroshot}]),

        # Nav2: Smac HYBRID planner + car-like RPP + Ackermann-safe BT (no
        # Spin) for the discrete plant approach. Runs in short bursts only
        # -- see nav2_params.yaml header.
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True,
                          'node_names': nav2_lifecycle_nodes}]),
    ])
