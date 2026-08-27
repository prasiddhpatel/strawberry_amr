"""
FIELD DEPLOYMENT -- single Raspberry Pi 4B 4GB, Pi+Orin topology.

CORRECTED from an earlier revision of this file, which put
row_navigation/safety_supervisor/twist_mux/ekf_node here on latency
grounds. That was a real, substantive deviation from the operator's
explicit architecture spec ("all sensors/actuation on the Pi, everything
compute-heavy on the Orin") that should have been flagged for
confirmation rather than silently decided -- it wasn't, and the operator
caught it. This file now holds ONLY true sensors and true actuation:

  - LiDAR, camera drivers (raw sensing)
  - base_controller / Rosmaster_Lib (raw actuation -- the ONLY node that
    talks to the physical motors/servo)
  - link_monitor_node (trivial, Pi-local network-link awareness)
  - joy_node + teleop_ps2 (a DELIBERATE, flagged exception to "compute
    goes on the Orin" -- see below)
  - twist_mux_pi_local (NEW -- see below; this is the piece that makes
    the exception above actually deliver on its own promise)

EVERYTHING ELSE -- row_navigation, safety_supervisor, ekf_node, and the
AUTONOMY-ARBITRATION twist_mux instance -- runs on the Orin. See
docs/ORIN_PI_SPLIT_ARCHITECTURE.md for the full node table and the
honest consequence: the reactive row-following and obstacle-safety loop
depends on the Pi<->Orin network link. Mitigations that remain
regardless: the physical hardware e-stop is independent of all software.
And base_controller's own local command watchdog stops the robot if
/cmd_vel goes stale for any reason -- including the link dropping
entirely -- within its configured timeout (default 0.5s).

UPDATED: teleop is now GENUINELY link-independent, not just "physically
local but still network-dependent in practice" as an earlier revision of
this file candidly flagged. The mechanism is a SECOND, Pi-local twist_mux
instance (`twist_mux_pi_local`, config: twist_mux_pi_local.yaml) that
arbitrates human override (/cmd_vel_teleop, priority 100) against whatever
the Orin's own twist_mux already decided between autonomous behaviours
(/cmd_vel_remote, priority 10, arriving over the network). Because BOTH
of this stage's real inputs are meaningful without the network -- teleop
is local by definition, and /cmd_vel_remote simply times out on its own
if the link is down, exactly like any twist_mux input timeout -- this
decision never depends on the Orin or the link being reachable. This
also means the PS2 pad's own e-stop button now locks out motion with
ZERO network dependency, which it did not before (previously even that
had to round-trip through the Orin's twist_mux) -- a genuine safety
improvement, not just a wash.

HONEST REMAINING LIMIT, stated plainly: safety_supervisor's own
obstacle-triggered e-stop is Orin-side, so it cannot reach the Pi during
a network drop -- no local arbitration changes that fact. If you choose to
drive manually during a drop, you are doing so without obstacle-based
software gating in that specific window; your own judgement, the deadman
requirement, and the always-independent physical e-stop are the safety
net there. See docs/ORIN_PI_SPLIT_ARCHITECTURE.md for the full
reasoning on why this tradeoff was judged worthwhile.
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
    mission = get_package_share_directory('mission_control')

    mux_local_cfg = os.path.join(bringup, 'config', 'twist_mux_pi_local.yaml')
    ps2_cfg = os.path.join(teleop, 'config', 'ps2_mapping.yaml')
    base_cfg = os.path.join(basec, 'config', 'base_controller_params.yaml')
    link_cfg = os.path.join(mission, 'config', 'link_monitor_params.yaml')

    return LaunchDescription([
        # URDF / TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc, 'launch', 'description.launch.py'))),

        # ALL sensors live on this one Pi -- LiDAR+IMU filter AND camera.
        # compress:='true' -- this topology's camera crosses the Pi<->Orin
        # WiFi link (see camera.launch.py's own module docstring).
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(sensors, 'launch', 'lidar_imu.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(sensors, 'launch', 'camera.launch.py')),
            launch_arguments={'compress': 'true'}.items()),

        # actuation -- the ONLY node that talks to the physical hardware.
        # Subscribes to /cmd_vel, now produced by twist_mux_pi_local below
        # (NOT directly by the Orin -- see module docstring). Zero
        # changes were needed in base_controller itself for this redesign.
        Node(package='base_controller', executable='base_controller_node',
             name='base_controller_node', output='screen',
             parameters=[base_cfg]),

        # NEW: Pi-local arbitration stage -- human override vs. whatever
        # the Orin's own twist_mux decided. See module docstring and
        # twist_mux_pi_local.yaml's own header comment for the full
        # reasoning. This is what makes teleop genuinely link-independent.
        Node(package='twist_mux', executable='twist_mux',
             name='twist_mux_pi_local', output='screen',
             parameters=[mux_local_cfg],
             remappings=[('/cmd_vel_out', '/cmd_vel')]),

        # network link monitor -- trivial compute, Pi-local by nature
        # (it's asking "can I hear the Orin", which only makes sense to
        # ask from here)
        Node(package='mission_control', executable='link_monitor_node',
             name='link_monitor_node', output='screen', parameters=[link_cfg]),

        # manual override -- FLAGGED EXCEPTION, see module docstring
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}]),
        Node(package='teleop_ps2', executable='joystick_node',
             name='joystick_node', output='screen', parameters=[ps2_cfg]),
    ])
