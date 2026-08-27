"""
Camera bringup: Orbbec Astra Pro Plus driver.

Runs on the Pi that has the USB hub -- see lidar_imu.launch.py and
DEPLOYMENT_GUIDE.md / ALL_IN_ONE_DEPLOYMENT_GUIDE.md for exactly which Pi
that is in your topology. Resolution/fps reduced from the old single-host
defaults (640x480 @30fps colour+depth, sized for an AGX Orin) -- that
combination is heavier than a Pi 4B should be asked to carry alongside
SLAM/Nav2/perception. 424x240 @15fps is a starting point, not a measured
optimum: watch actual CPU/RAM/network usage on your unit (DEPLOYMENT_GUIDE.md
has the commands) and raise it if there is headroom, or drop further if not.

COMPRESSED TRANSPORT -- `compress:=true` (default false; only
pi_hardware_and_control.launch.py passes true, see below) adds two
image_transport `republish raw compressed`/`raw compressedDepth` nodes
HERE, on this Pi, subscribing to the driver's own raw output and
re-publishing compressed versions locally, at zero network cost for this
specific hop (both ends are on the same machine).

CORRECTED from an earlier revision, which assumed astra_camera's driver
published via image_transport::CameraPublisher internally, and that
simply installing ros-humble-image-transport-plugins would make
compressed topics appear automatically alongside the raw ones -- no
launch-file change required. That assumption did not hold up: checked
directly against orbbec/ros2_astra_camera's actual GitHub repository (not
just its docs), and found the PR that adds image_transport support to
that driver ("Using image_transports", #2) is still OPEN, not merged, as
of this check. This fix does NOT depend on that PR merging, or on
astra_camera's internal implementation at all -- `republish` here reads
whatever plain sensor_msgs/Image the driver produces (which every camera
driver produces, image_transport-aware or not) and does the compression
itself, using image_transport's own standard, generic mechanism. This is
a strictly more robust design than relying on an upstream package's
unverified internals, not merely a workaround.

Why this matters specifically in the Pi+Orin topology
(pi_hardware_and_control.launch.py, which passes compress:=true):
the camera is here, on the Pi, but plant_perception's node runs on the
Orin -- meaning the RGB-D stream has to cross the Pi<->Orin WiFi
link (see docs/ORIN_PI_SPLIT_ARCHITECTURE.md) to reach it. The three
Orin-side launch files (pi_missionbrain_phase1_mapping.launch.py /
pi_missionbrain_phase2_nav.launch.py / orin_explore_map_navigate.launch.py)
subscribe to the compressed topics this produces and decompress locally
via their own `republish compressed raw`/`compressedDepth raw` nodes --
see their own comments for exactly how. NOT enabled for
sensors.launch.py (the single-host bench convenience wrapper) or the
original dual-Pi topology's launch files (camera and plant_perception
are on the same "mission-brain" Pi there too) -- compression only helps
where there's an actual network hop to cross, and adds pure overhead
(compress CPU cost, zero bandwidth benefit) anywhere there isn't one.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    compress = LaunchConfiguration('compress')

    return LaunchDescription([
        DeclareLaunchArgument('color_width', default_value='424'),
        DeclareLaunchArgument('color_height', default_value='240'),
        DeclareLaunchArgument('color_fps', default_value='15'),
        DeclareLaunchArgument('depth_width', default_value='424'),
        DeclareLaunchArgument('depth_height', default_value='240'),
        DeclareLaunchArgument('depth_fps', default_value='15'),
        DeclareLaunchArgument(
            'compress', default_value='false',
            description='Add compressed-transport republishing for this '
                        'camera stream, for callers where the camera and '
                        'its consumer are on different machines. See '
                        'module docstring -- only '
                        'pi_hardware_and_control.launch.py should need '
                        'true.'),

        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            output='screen',
            parameters=[{
                'depth_registration': True,   # align depth to colour -- required
                                               # for plant_perception's 3D back-
                                               # projection to be correct, see
                                               # that node's own ACCURACY NOTE
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'color_fps': LaunchConfiguration('color_fps'),
                'depth_width': LaunchConfiguration('depth_width'),
                'depth_height': LaunchConfiguration('depth_height'),
                'depth_fps': LaunchConfiguration('depth_fps'),
            }],
        ),

        # Compress the RAW output locally, for network-hop callers only --
        # see module docstring for the full reasoning (this is a robust
        # fix independent of astra_camera's own internals, not a reliance
        # on it using image_transport internally).
        Node(
            package='image_transport', executable='republish',
            name='color_compress', output='screen',
            arguments=['raw', 'compressed'],
            remappings=[('in', '/camera/color/image_raw'),
                       ('out/compressed', '/camera/color/image_raw/compressed')],
            condition=IfCondition(compress),
        ),
        Node(
            package='image_transport', executable='republish',
            name='depth_compress', output='screen',
            arguments=['raw', 'compressedDepth'],
            remappings=[('in', '/camera/depth/image_raw'),
                       ('out/compressedDepth', '/camera/depth/image_raw/compressedDepth')],
            condition=IfCondition(compress),
        ),
    ])
