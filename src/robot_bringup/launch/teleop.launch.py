"""
Manual-drive-only launch: joystick -> twist_mux -> base_controller (which now
performs the Ackermann conversion itself via Rosmaster_Lib's set_car_motion()
-- see base_controller_node.py's module docstring). No EKF, no SLAM, no
autonomy. For a first power-on test of the drivetrain itself (does the robot
drive and steer correctly when you move the sticks) before trusting it with
any sensor-driven autonomy.
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
    teleop = get_package_share_directory('teleop_ps2')
    basec = get_package_share_directory('base_controller')
    mux = os.path.join(bringup, 'config', 'twist_mux.yaml')
    ps2_cfg = os.path.join(teleop, 'config', 'ps2_mapping.yaml')
    base_cfg = os.path.join(basec, 'config', 'base_controller_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(desc, 'launch', 'description.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensors, 'launch', 'sensors.launch.py')),
            launch_arguments={'use_camera': 'false'}.items()),
        Node(package='base_controller', executable='base_controller_node',
             name='base_controller_node', output='screen',
             parameters=[base_cfg]),
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}]),
        Node(package='teleop_ps2', executable='joystick_node',
             name='joystick_node', output='screen', parameters=[ps2_cfg]),
        Node(package='safety_supervisor', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[mux],
             remappings=[('/cmd_vel_out', '/cmd_vel')]),
    ])
