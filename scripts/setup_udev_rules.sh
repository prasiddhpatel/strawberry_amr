#!/usr/bin/env bash
# Stable device names + permissions. ALL of RPLIDAR, Astra and the Yahboom
# driver board hang off ONE Yahboom USB hub (YB-CRV01) on a single Type-C link,
# so enumeration order is NOT guaranteed -- these rules are essential, not
# optional. Find each device's attributes first:
#   lsusb
#   udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'
set -e
RULES=/etc/udev/rules.d/99-strawberry-amr.rules
sudo bash -c "cat > $RULES" <<RULES
# --- RPLIDAR A1M8 (CP210x UART bridge -- same chip family as the C1 this
#     replaced, so this VID/PID rule is expected to still match; confirm
#     with lsusb per the note below) -> /dev/rplidar ---
# EDIT idProduct/serial to your unit if you have more than one CP210x device.
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0660", GROUP="dialout"

# --- Yahboom YB-ERF01-V2.0 driver board -> /dev/myserial ---
# NOTE: symlink name matches Rosmaster_Lib's own constructor default (see
# base_controller_params.yaml's serial_port) -- base_controller looks for
# this exact name, not "robot_base" (an earlier name this script used
# before the Rosmaster_Lib rewrite settled on /dev/myserial; fixed here
# to actually match, since a mismatch here means base_controller fails
# to connect at all, silently pointing at a device that does not exist).
# EDIT VID/PID to the YB-ERF01's USB-serial chip (often CH340 1a86:7523 or CP210x).
# If it ALSO enumerates as a CP210x like the LiDAR, disambiguate with ATTRS{serial}.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0660", GROUP="dialout"

# --- Orbbec Astra Pro Plus: grant non-root access so the driver can open it ---
# EDIT idVendor to your Astra's VID from lsusb (Orbbec is commonly 2bc5).
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bc5", MODE="0660", GROUP="plugdev"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger

# Rules above use 0660 (group-restricted, not world-writable) -- the
# operating user must actually be in dialout/plugdev or every device open
# fails with Permission denied. Add them here rather than leaving it as an
# undocumented prerequisite.
sudo usermod -aG dialout,plugdev "$USER"

echo "udev rules installed:"
echo "  /dev/rplidar      (RPLIDAR A1M8, group dialout)"
echo "  /dev/myserial     (Yahboom YB-ERF01 driver board -- matches"
echo "                     Rosmaster_Lib's own default port name, group dialout)"
echo "  Astra: group-restricted (plugdev) USB access for the Orbbec driver"
echo "Joypad is usually /dev/input/js0 (no rule needed)."
echo
echo "Added $USER to dialout,plugdev -- LOG OUT AND BACK IN (or reboot) for"
echo "the new group membership to take effect before running any node that"
echo "opens these devices."
echo
echo "TIP: if the LiDAR and the Yahboom board use the same USB-serial chip,"
echo "     add  ATTRS{serial}==\"<unique-serial>\"  to each rule to pin them."
