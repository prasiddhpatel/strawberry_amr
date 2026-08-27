"""
SINGLE-HOST bench/development launch.

Brings up the ENTIRE stack (both the realtime control chain and the mission
brain chain) on ONE machine. This is for Orin/dev-machine bench testing
and software integration checks ONLY -- it is NOT how the robot is deployed
in the field. For field deployment, use the three per-host, per-phase launch
files instead, run on the actual 2x Raspberry Pi rig:

  ros2 launch robot_bringup pi_realtime.launch.py                    (2GB Pi)
  ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py (4GB Pi, Phase 1)
  ros2 launch robot_bringup pi_missionbrain_phase2_nav.launch.py     (4GB Pi, Phase 2)

See DEPLOYMENT_GUIDE.md for the full procedure, including why the stack is
split this way (RAM budget) and the Phase 1 -> Phase 2 map hand-off step.

No separate Ackermann-conversion node: base_controller now performs that
itself via Rosmaster_Lib's set_car_motion(), which is car-type-aware
(CARTYPE_R2) and does the bicycle-model conversion in tested vendor
firmware rather than in a from-scratch ROS node -- see
base_controller_node.py's module docstring for the full reasoning.

No harvest/arm logic: the 5-DOF arm is a separate, not-yet-confirmed
companion project (per the operator's explicit scope decision) -- this
workspace sequences mapped plants and stops at each one in order, and does
not wait for or expect any arm handoff signal.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    desc = get_package_share_directory('robot_description')
    sensors = get_package_share_directory('sensor_bringup')
    rownav = get_package_share_directory('row_navigation')
    teleop = get_package_share_directory('teleop_ps2')
    coverage = get_package_share_directory('coverage_planner')
    basec = get_package_share_directory('base_controller')
    mission = get_package_share_directory('mission_control')
    plant = get_package_share_directory('plant_perception')

    ekf = os.path.join(bringup, 'config', 'ekf.yaml')
    slam = os.path.join(bringup, 'config', 'slam_toolbox.yaml')
    mux = os.path.join(bringup, 'config', 'twist_mux.yaml')
    nav2_params = os.path.join(bringup, 'config', 'nav2_params.yaml')
    rownav_cfg = os.path.join(rownav, 'config', 'row_navigation_params.yaml')
    ps2_cfg = os.path.join(teleop, 'config', 'ps2_mapping.yaml')
    coverage_cfg = os.path.join(coverage, 'config', 'coverage_params.yaml')
    base_cfg = os.path.join(basec, 'config', 'base_controller_params.yaml')
    mission_cfg = os.path.join(mission, 'config', 'mission_control_params.yaml')
    plant_cfg = os.path.join(plant, 'config', 'plant_detector_params.yaml')

    use_camera = LaunchConfiguration('use_camera')
    use_nav2 = LaunchConfiguration('use_nav2')

    # Nav2 lifecycle nodes managed for the plant-approach phase.
    nav2_lifecycle_nodes = ['controller_server', 'planner_server',
                            'behavior_server', 'bt_navigator']

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true',
                              description='Start the Nav2 stack for plant approach.'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        # robot_state_publisher (URDF / TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc, 'launch', 'description.launch.py'))),

        # sensors (lidar + imu filter + optional camera)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensors, 'launch', 'sensors.launch.py')),
            launch_arguments={'use_camera': use_camera}.items()),

        # base hardware bridge: Rosmaster_Lib wrapper, subscribes /cmd_vel
        # directly and performs the Ackermann conversion itself
        Node(package='base_controller', executable='base_controller_node',
             name='base_controller_node', output='screen',
             parameters=[base_cfg]),

        # PS2 joypad: driver + teleop logic
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'device_name': '', 'device_id': 0,
                          'deadzone': 0.05, 'autorepeat_rate': 20.0}]),
        Node(package='teleop_ps2', executable='joystick_node',
             name='joystick_node', output='screen', parameters=[ps2_cfg]),

        # state estimation (EKF: wheel odom vx + MPU-9250 IMU yaw-rate)
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen', parameters=[ekf]),

        # SLAM (2D occupancy, mapping mode -- Phase 1 config; see
        # slam_toolbox_localization.yaml for the Phase 2 alternative used by
        # pi_missionbrain_phase2_nav.launch.py, not used here)
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[slam]),

        # safety gate (C++): /cmd_vel_auto -> /cmd_vel_safe (+ /e_stop)
        Node(package='safety_supervisor', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),

        # velocity arbitration -> /cmd_vel (teleop > nav2 > row-follow; e_stop lock)
        # -> consumed directly by base_controller (see above).
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[mux],
             remappings=[('/cmd_vel_out', '/cmd_vel')]),

        # autonomous row following (RANSAC + FOPID + Ackermann bulb-turn headland)
        Node(package='row_navigation', executable='row_nav_node',
             name='row_nav_node', output='screen', parameters=[rownav_cfg]),

        # ---- EXPLICIT PATH-PLANNING LAYER ----
        # global route: boustrophedon coverage over all aisles
        Node(package='coverage_planner', executable='coverage_planner_node',
             name='coverage_planner_node', output='screen',
             parameters=[coverage_cfg]),

        # mission sequencing: Orin mission commands in, sequences mapped
        # plants, no arm/harvest handoff (see module docstring above)
        Node(package='mission_control', executable='mission_control_node',
             name='mission_control_node', output='screen',
             parameters=[mission_cfg]),

        # Nav2 (planner_server [Smac HYBRID] + controller_server [RPP,
        # car-like] + BT [no Spin]) for the point-to-point plant approach.
        # Controller cmd_vel -> /cmd_vel_nav so it is arbitrated by
        # twist_mux (nav priority > row-follow).
        GroupAction(condition=IfCondition(use_nav2), actions=[
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
        ]),

        # perception + mapping (optional, camera)
        GroupAction(condition=IfCondition(use_camera), actions=[
            Node(package='plant_perception', executable='plant_detector_node',
                 name='plant_detector_node', output='screen',
                 parameters=[plant_cfg]),
            Node(package='semantic_mapper', executable='semantic_mapper_node',
                 name='semantic_mapper_node', output='screen'),
            Node(package='target_manager', executable='target_manager_node',
                 name='target_manager_node', output='screen'),
        ]),
    ])
