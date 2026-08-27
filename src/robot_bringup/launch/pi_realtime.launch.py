"""
FIELD DEPLOYMENT -- 2GB Raspberry Pi ("realtime" host).

Runs everything on the safety-critical actuation path: the Rosmaster_Lib
hardware bridge to the YB-ERF01 board (which now ALSO performs the
Ackermann kinematic conversion, via Yahboom's own firmware-side
set_car_motion() -- see base_controller_node.py's module docstring for
why the previously-separate ackermann_bridge package was removed and
folded into base_controller), state estimation, reactive row-following,
the independent safety gate, velocity arbitration, and manual override.
Identical across BOTH mission phases (mapping and harvest execution) --
this file does not change between Phase 1 and Phase 2; only the OTHER
Pi's launch file does. See DEPLOYMENT_GUIDE.md.

Does NOT run: camera, perception, mapping/SLAM, Nav2, coverage planning --
all of that is the mission-brain Pi's job, to keep this host's RAM/CPU
budget small and headroom large for the control loop.

Assumes (see DEPLOYMENT_GUIDE.md "Network Setup"): this Pi has CycloneDDS
configured to bind eth0 and a working wired link to the mission-brain Pi,
so /scan (published by the OTHER Pi, see camera/lidar wiring note in
sensor_bringup/launch/lidar_imu.launch.py) and /cmd_vel_nav (from the other
Pi's Nav2, Phase 2 only) are reachable here.
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
    mux = os.path.join(bringup, 'config', 'twist_mux.yaml')
    rownav_cfg = os.path.join(rownav, 'config', 'row_navigation_params.yaml')
    ps2_cfg = os.path.join(teleop, 'config', 'ps2_mapping.yaml')
    base_cfg = os.path.join(basec, 'config', 'base_controller_params.yaml')

    return LaunchDescription([
        # URDF / TF (lightweight; fine to publish from either Pi, kept here
        # since base_link's children -- wheels, IMU -- are all on this host's
        # side of the data flow)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc, 'launch', 'description.launch.py'))),

        # IF your USB hub is wired to THIS Pi instead of the mission-brain
        # Pi, uncomment the next two lines and remove the equivalent include
        # from pi_missionbrain_*.launch.py. Default assumes the hub is on
        # the mission-brain Pi (see lidar_imu.launch.py header).
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #     os.path.join(sensors, 'launch', 'lidar_imu.launch.py'))),

        # ---- realtime actuation chain ----
        # base_controller now subscribes to /cmd_vel directly (twist_mux's
        # output, remapped below) and performs the Ackermann conversion
        # itself via Rosmaster_Lib's set_car_motion() -- no separate bridge
        # node needed anymore.
        Node(package='base_controller', executable='base_controller_node',
             name='base_controller_node', output='screen',
             parameters=[base_cfg]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen', parameters=[ekf]),
        Node(package='safety_supervisor', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[mux],
             remappings=[('/cmd_vel_out', '/cmd_vel')]),
        Node(package='row_navigation', executable='row_nav_node',
             name='row_nav_node', output='screen', parameters=[rownav_cfg]),

        # manual override (human master switch; ALWAYS available regardless
        # of mission phase)
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}]),
        Node(package='teleop_ps2', executable='joystick_node',
             name='joystick_node', output='screen', parameters=[ps2_cfg]),
    ])
