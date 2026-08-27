"""
Single-host bench launch: drivetrain + EKF + SLAM + row-following, NO
camera, NO Nav2, NO mission/harvest logic. For testing row-following and
mapping on a bench or in a corridor without the perception/mission stack
running.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    desc = get_package_share_directory('robot_description')
    sensors = get_package_share_directory('sensor_bringup')
    rownav = get_package_share_directory('row_navigation')
    teleop = get_package_share_directory('teleop_ps2')
    basec = get_package_share_directory('base_controller')

    ekf = os.path.join(bringup, 'config', 'ekf.yaml')
    slam = os.path.join(bringup, 'config', 'slam_toolbox.yaml')
    mux = os.path.join(bringup, 'config', 'twist_mux.yaml')
    rownav_cfg = os.path.join(rownav, 'config', 'row_navigation_params.yaml')
    ps2_cfg = os.path.join(teleop, 'config', 'ps2_mapping.yaml')
    base_cfg = os.path.join(basec, 'config', 'base_controller_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(desc, 'launch', 'description.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensors, 'launch', 'sensors.launch.py')),
            launch_arguments={'use_camera': 'false'}.items()),
        # base_controller performs the Ackermann conversion itself now, via
        # Rosmaster_Lib's set_car_motion() -- see base_controller_node.py.
        Node(package='base_controller', executable='base_controller_node',
             name='base_controller_node', output='screen',
             parameters=[base_cfg]),
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}]),
        Node(package='teleop_ps2', executable='joystick_node',
             name='joystick_node', output='screen', parameters=[ps2_cfg]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen', parameters=[ekf]),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[slam]),
        Node(package='safety_supervisor', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[mux],
             remappings=[('/cmd_vel_out', '/cmd_vel')]),
        Node(package='row_navigation', executable='row_nav_node',
             name='row_nav_node', output='screen', parameters=[rownav_cfg]),
    ])
