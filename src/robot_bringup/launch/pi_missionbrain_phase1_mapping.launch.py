"""
ORIN -- Pi+Orin topology, PHASE 1: MAPPING (known layout, e.g. Lab 3003).

Drive every row once (manually via the Pi's joystick, or autonomously via
row_navigation -- either works, this file doesn't care) while this builds
the 2D occupancy map and the strawberry-plant landmark set. Does NOT run
Nav2 or coverage_planner -- there is nothing to navigate TO yet during
mapping.

CORRECTED from an earlier revision: row_navigation, safety_supervisor,
twist_mux, and ekf_node now run HERE (Orin), not on the Pi -- see
pi_hardware_and_control.launch.py's module docstring for the full
reasoning and the honest safety-loop-now-depends-on-the-network-link
tradeoff this creates. The Pi now runs ONLY sensors, actuation
(base_controller), and manual override (joy/teleop_ps2, a flagged
exception -- see that file).

Also fixed here: this file previously referenced `plant_cfg` in
plant_detector_node's parameters without ever defining it -- a real,
undetected bug that would have raised NameError the moment this file was
launched (py_compile doesn't catch undefined-name errors, only syntax
errors; caught by an ast-based undefined-name sweep across every launch
file after this bug was found by inspection -- see git history for the
full sweep results).
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
    semmap = get_package_share_directory('semantic_mapper')
    mission = get_package_share_directory('mission_control')
    plant = get_package_share_directory('plant_perception')

    ekf = os.path.join(bringup, 'config', 'ekf.yaml')
    mux = os.path.join(bringup, 'config', 'twist_mux.yaml')
    slam = os.path.join(bringup, 'config', 'slam_toolbox.yaml')   # mode: mapping
    rownav_cfg = os.path.join(rownav, 'config', 'row_navigation_params.yaml')
    semmap_cfg = os.path.join(semmap, 'config', 'semantic_mapper_params.yaml')
    mission_cfg = os.path.join(mission, 'config', 'mission_control_params.yaml')
    plant_cfg = os.path.join(plant, 'config', 'plant_detector_params.yaml')
    plant_zeroshot_cfg = os.path.join(
        plant, 'config', 'plant_detector_zeroshot_params.yaml')

    detector = LaunchConfiguration('detector')
    is_zeroshot = PythonExpression(["'", detector, "' == 'zeroshot'"])


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


        # NOTE: no sensor driver includes here anymore -- LiDAR/IMU/camera
        # all run on the Pi now (pi_hardware_and_control.launch.py) and
        # their topics reach this Orin over the network.

        # relocated from the Pi -- see module docstring
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen', parameters=[ekf]),
        Node(package='safety_supervisor', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[mux],
             remappings=[('/cmd_vel_out', '/cmd_vel_remote')]),  # NOT /cmd_vel anymore -- that name is now the Pi-local mux's own output; see docs/ORIN_PI_SPLIT_ARCHITECTURE.md
        Node(package='row_navigation', executable='row_nav_node',
             name='row_nav_node', output='screen', parameters=[rownav_cfg]),

        # SLAM: mapping mode, builds /map live + map->odom
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[slam]),

        # Decompress the camera stream locally -- it crossed the Pi<->Orin
        # WiFi link compressed (see camera.launch.py's own module docstring for
        # the full reasoning, including why this no longer depends on any
        # unverified assumption about astra_camera's internals). republish is
        # image_transport's own standard node; this is not custom logic.
        # Republished to a DIFFERENTLY-NAMED local topic (not the same name
        # as the Pi's raw topic) deliberately -- if plant_detector_node
        # subscribed to the exact same topic name the Pi publishes, DDS
        # discovery could non-deterministically hand it either this
        # decompressed-locally copy OR the original raw one still crossing
        # the network, silently defeating the entire point of compressing.
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

        # plant perception + semantic landmark mapping (build the plant list
        # now, ready for Phase 2 to consume). color_topic/depth_topic
        # overridden here (layered on top of plant_cfg, same pattern as
        # every other per-launch-file override in this workspace) to
        # consume the locally-decompressed topics above, not the raw ones
        # -- see the republish nodes' own comments for why this must not
        # be the same topic name the Pi publishes.
        #
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

        # Orin command interface: "finish_mapping" saves the map for you
        Node(package='mission_control', executable='mission_control_node',
             name='mission_control_node', output='screen',
             parameters=[mission_cfg, {'phase': 'mapping', 'zeroshot_enabled': is_zeroshot}]),
    ])
