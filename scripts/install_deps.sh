#!/usr/bin/env bash
# Install ROS 2 Humble dependencies for this workspace (Ubuntu 22.04, Raspberry
# Pi 4B dual-host build). Run on BOTH Pis -- each needs the full set, since
# either could in principle run any launch file during bench testing even
# though the field deployment splits packages across hosts (see
# DEPLOYMENT_GUIDE.md).
#
# CORRECTED against the actual package.xml files across every package in this
# workspace (grep -h "depend>" src/*/package.xml), not assumed -- this list
# had drifted: ros-humble-ackermann-msgs and ros-humble-teleop-twist-joy were
# listed here but are declared in NO package.xml and used in NO code anywhere
# in this workspace (this project deliberately stays in plain geometry_msgs/
# Twist throughout, and uses its own teleop_ps2 package instead of the
# generic teleop_twist_joy -- see the architecture docs for why). Removed.
# nav2_smac_planner and nav2_regulated_pure_pursuit_controller were missing
# entirely despite being real, explicit exec_depends in robot_bringup's
# package.xml -- added.
set -e
sudo apt update
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-twist-mux \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-rplidar-ros \
  ros-humble-imu-filter-madgwick \
  ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-image-transport ros-humble-image-transport-plugins \
  ros-humble-compressed-image-transport ros-humble-compressed-depth-image-transport \
  ros-humble-rmw-cyclonedds-cpp \
  python3-opencv python3-numpy python3-serial
echo "NOTE: install the Orbbec Astra driver (ros_astra_camera, built from"
echo "      source -- see ALL_IN_ONE_DEPLOYMENT_GUIDE.md Part 4.5) separately."
echo "NOTE: zero-shot perception (detector:='zeroshot', Grounding DINO +"
echo "      MobileSAM) needs NVIDIA's own Jetson PyTorch build, transformers,"
echo "      and MobileSAM -- NONE of this is apt-installable and 'pip install"
echo "      torch' gets you the wrong (x86_64/CPU) wheel. See"
echo "      docs/ZERO_SHOT_PERCEPTION_GUIDE.md for the full sequence. Only"
echo "      needed for detector:='zeroshot'; the default detector:='hsv'"
echo "      needs none of it."
echo "NOTE: this list is a convenience pre-install, not the authoritative"
echo "      source -- once the actual workspace source exists (src/ populated),"
echo "      'rosdep install --from-paths src -y --ignore-src' from the"
echo "      workspace root reads every package.xml directly and will catch"
echo "      anything this hand-maintained list misses or has drifted from."
