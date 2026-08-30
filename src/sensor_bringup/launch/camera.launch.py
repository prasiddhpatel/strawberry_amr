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

CORRECTED (2026-08-29) -- this file previously launched astra_camera via
`Node(package='astra_camera', executable='astra_camera_node', ...)`. That
executable does not exist: astra_camera's CMakeLists.txt only builds
OBCameraNodeFactory as a composable-node plugin inside libastra_camera.so
(see `rclcpp_components_register_nodes` there), meant to be loaded into a
rclcpp_components component_container -- exactly what the vendored
astra_camera/launch/dabai_pro.launch.py does. This file now mirrors that
pattern instead of guessing at a plain-executable one. It also previously
passed a `depth_registration` parameter -- the driver's own C++ only
declares `depth_align` (astra_camera/src/ob_camera_node.cpp), so that
setting was silently dropped and D2C alignment was never actually turning
on, undermining plant_perception's 3D back-projection (see that node's own
ACCURACY NOTE) even before the colour bug below.

RGB BYPASS -- this unit's Astra Pro Plus colour sensor hits a libuvc
parsing bug ("unsupported descriptor subtype VS_COLORFORMAT",
orbbec/ros_astra_camera#135, open upstream, not something this vendored
copy's local patches touch). Confirmed live: astra_camera_node's own log
shows that error immediately followed by "color is not enable" -- the
driver disables its own colour stream after the failed negotiation, so
`/camera/color/image_raw` AND `/camera/color/camera_info` are both
advertised but never actually publish a single message. `enable_color`
is set False below to skip that failed attempt outright (it was going to
retry up to `uvc_camera.retry_count` times otherwise) rather than rely on
the driver's own fallback. v4l2_camera opens the exact same physical
sensor through the kernel's uvcvideo driver instead of libuvc and does not
hit this bug -- confirmed working formats via `v4l2-ctl --list-formats-ext`
on /dev/video0 (MJPG/YUYV, multiple resolutions). It publishes on the same
`/camera/color/image_raw` / `/camera/color/camera_info` topic names so
nothing downstream (plant_perception, the compressed republish nodes
below) needs to change.

TODO -- v4l2_camera_node below has no `camera_info_url` set, so it
publishes an UNCALIBRATED CameraInfo (no real fx/fy/cx/cy). That's fine
for plant_perception's HSV colour detection but NOT fine for its 3D
back-projection accuracy. Run `ros2 run camera_calibration
cameracalibrator` against this sensor once you have a checkerboard and
point `camera_info_url` at the resulting file.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    compress = LaunchConfiguration('compress')

    astra_params_file = os.path.join(
        get_package_share_directory('astra_camera'),
        'params', 'dabai_pro_params.yaml')
    with open(astra_params_file, 'r') as f:
        astra_params = yaml.safe_load(f)

    return LaunchDescription([
        # 320x240, not the old 424x240: confirmed live against this exact
        # unit that 424x240 is not a valid DEPTH resolution at all (driver
        # log: "format PIXEL_FORMAT_DEPTH_1_MM is not supported ... Stream
        # will be disabled" -- its own "Supported video modes" dump lists
        # only 160x120/320x240/640x480/1280x1024). Matching colour to the
        # same 320x240 also means plant_detector_node's width/height-RATIO
        # colour->depth pixel scaling (see its own module docstring) is an
        # exact 1:1 mapping instead of an interpolated one. Keep the v4l2
        # Node's own 'image_size' below in sync if these change.
        DeclareLaunchArgument('color_width', default_value='320'),
        DeclareLaunchArgument('color_height', default_value='240'),
        DeclareLaunchArgument('color_fps', default_value='15'),
        DeclareLaunchArgument('depth_width', default_value='320'),
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

        ComposableNodeContainer(
            name='astra_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='astra_camera',
                    plugin='astra_camera::OBCameraNodeFactory',
                    name='camera',
                    namespace='camera',
                    parameters=[astra_params, {
                        'depth_align': True,   # see CORRECTED note above --
                                                # NOT 'depth_registration'
                        'enable_color': False,  # see RGB BYPASS note above
                        'uvc_camera.enable': False,  # THE real switch: the
                                                # libuvc colour path is
                                                # started unconditionally
                                                # from use_uvc_camera_
                                                # (ob_camera_node_factory.cpp)
                                                # whenever this is true --
                                                # enable_color above does
                                                # NOT gate it. dabai_pro_
                                                # params.yaml sets this true
                                                # by default; leaving it so
                                                # while v4l2_camera_node
                                                # also holds the same device
                                                # open is what wedged the
                                                # sensor off /dev/video*
                                                # entirely during testing.
                        'color_width': LaunchConfiguration('color_width'),
                        'color_height': LaunchConfiguration('color_height'),
                        'color_fps': LaunchConfiguration('color_fps'),
                        'depth_width': LaunchConfiguration('depth_width'),
                        'depth_height': LaunchConfiguration('depth_height'),
                        'depth_fps': LaunchConfiguration('depth_fps'),
                    }],
                ),
            ],
            output='screen',
        ),

        # RGB bypass -- see module docstring. Same physical sensor as
        # astra_camera's disabled colour stream, opened via V4L2/uvcvideo
        # instead of libuvc. image_size is a literal, not tied to the
        # color_width/height launch args above (passing a LaunchConfiguration
        # through launch_ros.parameter_descriptions.ParameterValue for an
        # int-array crashes launch's parameter normalization -- tried and
        # reverted, see git history). Kept at 320x240 to match
        # depth_width/height's default above -- see the DeclareLaunchArgument
        # comment for why 320x240 specifically. Update both together if
        # either changes; this one won't follow automatically.
        Node(
            package='v4l2_camera', executable='v4l2_camera_node',
            name='v4l2_color_camera', output='screen',
            parameters=[{
                'video_device': '/dev/v4l/by-id/'
                    'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0',
                'image_size': [320, 240],
                'camera_frame_id': 'camera_color_optical_frame',
            }],
            remappings=[
                ('image_raw', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
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
