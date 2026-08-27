"""
Brings up the R2 browser visualization stack: run this on the AGX Orin.

Most of this is off-the-shelf ROS 2 tooling (no custom node code to trust)
plus a static file server for the r2_web_viz frontend; one node,
map_accumulator_node, is genuinely new code -- see its own module
docstring for why it exists (the browser cannot correctly compose
map->odom->base_link itself; this project's frame chain has a real,
non-identity map->odom correction published by slam_toolbox):

  - rosbridge_server / rosbridge_websocket -- WebSocket<->ROS bridge
    (port 9090). The frontend uses roslibjs over this for /map,
    /row_nav/assumed_path, Nav2's /plan, /r2/map_points and
    /r2/robot_pose_map below.
  - web_video_server -- serves /camera/color/image_raw as an HTTP MJPEG
    stream (port 8080) a plain <img> tag can consume directly. No custom
    video code, no WebRTC signalling to debug.
  - depth_image_proc's point_cloud_xyzrgb component -- fuses the already-
    registered RGB-D stream into an RGB-textured sensor_msgs/PointCloud2
    on /r2/points (a per-frame, camera-relative cloud -- NOT what the
    frontend actually subscribes to; see below). Requires
    depth_registration:=true on the camera (already the default in
    camera.launch.py) since it does no registration itself.
  - map_accumulator_node (this package's own code) -- subscribes to
    /r2/points, looks up a real tf2 map<-camera_frame transform at each
    cloud's own timestamp, and accumulates a voxel-downsampled, map-frame,
    persistent reconstruction on /r2/map_points. Separately republishes a
    properly tf2-composed map-frame robot pose on /r2/robot_pose_map,
    replacing what would otherwise be the frontend treating
    /odometry/filtered's ODOM-frame pose as if it were map-frame (wrong
    whenever slam_toolbox's map->odom correction is non-identity, which is
    the normal case once SLAM has corrected any drift at all).
  - a plain `python3 -m http.server` serving web/ (this package's frontend)
    on port 8000.

None of this touches base_controller, safety_supervisor, or any existing
launch file -- it is included alongside them, not substituted for them.
Bring it up on its own, or append its actions into an existing
LaunchDescription via IncludeLaunchDescription.

Usage:
    ros2 launch r2_web_viz web_viz.launch.py
    # then, from the Orin's own browser or any device on the same LAN:
    #   http://<orin-hostname-or-ip>:8000/

Bandwidth note: the camera stream served here is whatever resolution
camera.launch.py is currently running (424x240@15fps by default, a
deliberate Pi-CPU/bandwidth tradeoff documented in that file's own
docstring) -- this launch file does not change that, by design (see the
mapping-app scoping discussion this package came out of). If a visibly
better live view is wanted later, that is a separate, deliberate change to
camera.launch.py's resolution arguments, not something to bump here.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    http_port = LaunchConfiguration('http_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    video_port = LaunchConfiguration('video_port')
    color_topic = LaunchConfiguration('color_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    web_dir = os.path.join(get_package_share_directory('r2_web_viz'), 'web')

    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='8000',
            description='Port serving the r2_web_viz frontend static files.'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
            description='rosbridge WebSocket port.'),
        DeclareLaunchArgument('video_port', default_value='8080',
            description='web_video_server HTTP port.'),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw',
            description='Camera colour topic to stream and to fuse into the point cloud.'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw',
            description='Registered depth topic (depth_registration:=true '
                        'on the camera driver -- see camera.launch.py).'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info',
            description='Camera intrinsics topic for the point-cloud fusion.'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': rosbridge_port}],
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{'port': video_port}],
        ),

        # RGB-textured point cloud from the existing registered RGB-D stream
        # -- standard depth_image_proc component, not custom fusion code.
        ComposableNodeContainer(
            name='r2_web_viz_pointcloud_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb',
                    remappings=[
                        ('rgb/image_rect_color', color_topic),
                        ('rgb/camera_info', camera_info_topic),
                        ('depth_registered/image_rect', depth_topic),
                        ('points', '/r2/points'),
                    ],
                ),
            ],
        ),

        Node(
            package='r2_web_viz',
            executable='map_accumulator_node',
            name='map_accumulator_node',
            output='screen',
        ),

        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', http_port],
            cwd=web_dir,
            name='r2_web_viz_static_server',
            output='screen',
        ),
    ])
