"""
Realtime-Pi sensor bringup: RPLIDAR A1M8 driver + IMU Madgwick filter.

CORRECTED (was RPLIDAR C1 -- the operator's actual Yahboom-supplied unit is
the A1M8 that ships in the ROSMASTER R2 kit). This is not just a part-number
swap -- two real differences that affect both config and field behaviour:

1. RANGING PRINCIPLE: A1M8 is TRIANGULATION-based, not time-of-flight. Per
   Slamtec's own datasheet: "It can work excellent in all kinds of indoor
   environment and outdoor environment WITHOUT DIRECT SUNLIGHT EXPOSURE."
   Triangulation ranging is materially more sensitive to strong ambient/IR
   light than the ToF principle the C1 config this replaced was tuned for.
   A polytunnel interior is diffuse-lit (not direct beam sun through the
   film in most conditions), so this is not necessarily disqualifying, but
   it IS a real risk to verify empirically and early -- see
   docs/SENSOR_CALIBRATION_AND_BRINGUP_GUIDE.md, "LiDAR solo bring-up",
   which includes a specific bright-vs-shaded scan-quality comparison step
   for exactly this reason. Do not assume it will be fine; go check.
2. INTERFACE: A1M8 talks UART at 115200 baud (the baud rate used
   essentially universally across the ROS ecosystem's rplidar_ros
   tutorials/launch files for the A-series) -- NOT the C1's 460800.
   Getting this wrong does not crash the driver, it just silently
   produces garbage/empty scans, which is a confusing failure to debug
   blind -- if /scan looks empty or corrupt, this is the first thing to
   re-check.

Deliberately split out of the old single sensors.launch.py (which also
launched the camera) for the dual-Pi deployment: this file runs on the 2GB
realtime Pi, co-located with base_controller/ekf/row_navigation for lowest
latency to the control loop. The camera runs separately on the 4GB mission-
brain Pi -- see camera.launch.py -- because that is where the USB hub
(carrying BOTH RPLIDAR and the Astra) is recommended to be plugged in (see
DEPLOYMENT_GUIDE.md for why: it keeps the much heavier camera stream off
the network entirely, at the cost of only the small /scan messages crossing
to this Pi, which is negligible on a wired link).

IF YOUR PHYSICAL WIRING DIFFERS (hub on the realtime Pi instead): that is
fine too -- launch THIS file together with camera.launch.py on the same
host in that case. Nothing in the software hard-codes one physical
topology; only the cabling and which Pi you launch which file on changes.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sensor_bringup')
    imu_cfg = os.path.join(pkg, 'config', 'imu_filter.yaml')

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,   # A1M8 (was 460800 for the C1 this replaced;
                                              # 57600 is a documented fallback on some
                                              # adapter-board revisions if 115200 gives
                                              # no data -- see calibration guide)
                'frame_id': 'laser',
                'scan_mode': 'Standard',     # safe default across A1M8 revisions;
                                              # if your unit is R4+ (8kHz sample rate),
                                              # `ros2 run rplidar_ros rplidar_node --ros-args
                                              # -p serial_port:=/dev/rplidar` then checking
                                              # available modes may offer a faster mode --
                                              # see the calibration guide's LiDAR section
                'angle_compensate': True,
            }],
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[imu_cfg],
            remappings=[('/imu/data_raw', '/imu/data_raw'),
                        ('/imu/data', '/imu/data')],
        ),
    ])
