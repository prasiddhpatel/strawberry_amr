import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('coverage_planner'),
        'config', 'coverage_params.yaml')
    return LaunchDescription([
        Node(package='coverage_planner', executable='coverage_planner_node',
             name='coverage_planner_node', output='screen', parameters=[cfg]),
    ])
