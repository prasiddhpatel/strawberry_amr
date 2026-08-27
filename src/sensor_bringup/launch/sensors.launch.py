"""
Single-host convenience wrapper: lidar_imu.launch.py + camera.launch.py
together. Used by the single-host bench launches (full_robot/bench/
teleop.launch.py in robot_bringup). For the real dual-Pi field deployment,
launch lidar_imu.launch.py and camera.launch.py SEPARATELY on whichever
Pi/Pis match your physical USB wiring -- see DEPLOYMENT_GUIDE.md.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg = get_package_share_directory('sensor_bringup')
    use_camera = LaunchConfiguration('use_camera')

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'lidar_imu.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'camera.launch.py')),
            condition=IfCondition(use_camera)),
    ])
