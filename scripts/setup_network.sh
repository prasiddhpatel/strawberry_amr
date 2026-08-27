#!/usr/bin/env bash
# Wired ROS 2 network setup between the two Raspberry Pis, run ONCE on each
# host with the matching role argument. See DEPLOYMENT_GUIDE.md, "Network
# Setup", for the full explanation; this script just applies it.
#
# Usage:
#   ./setup_network.sh realtime      (on the 2GB Pi)
#   ./setup_network.sh missionbrain  (on the 4GB Pi)
#
# What this does, and why:
#   1. Static IP on eth0 via netplan (PERSISTS across reboots -- a plain
#      `ip addr add` does not, and would silently revert on the next power
#      cycle in the field, which is a bad time to discover that).
#   2. CycloneDDS pinned to eth0 ONLY. Without this, ROS 2's default DDS
#      (FastDDS) will try every interface it finds, including wlan0 if
#      either Pi has WiFi active (e.g. for SSH/RViz from a Orin) -- that
#      adds latency and unpredictability to the safety-critical /cmd_vel
#      chain that has no business going anywhere near a WiFi radio.
#   3. A shared ROS_DOMAIN_ID so both hosts' DDS participants discover each
#      other (and, deliberately, so a Orin on the same domain ID can join
#      for RViz/rqt monitoring over WiFi -- only the ROBOT's own realtime
#      traffic is pinned to eth0, a monitoring Orin is not part of that
#      constraint).
#
# Subnet choice (192.168.10.0/24) is deliberately NOT 192.168.1.0/24 --
# that range is many home/office routers' default, and if either Pi also
# has WiFi active on such a network, a second static route on the same
# subnet via eth0 creates an ambiguous/conflicting route. A dedicated,
# uncommon subnet for the direct Pi<->Pi link avoids this entirely.
set -e

ROLE="$1"
if [ "$ROLE" != "realtime" ] && [ "$ROLE" != "missionbrain" ]; then
  echo "Usage: $0 [realtime|missionbrain]"
  echo "  realtime      -- run this on the 2GB Pi"
  echo "  missionbrain  -- run this on the 4GB Pi"
  exit 1
fi

if [ "$ROLE" = "realtime" ]; then
  MY_IP="192.168.10.11"
  PEER_IP="192.168.10.10"
  HOSTNAME_LABEL="amr-realtime"
else
  MY_IP="192.168.10.10"
  PEER_IP="192.168.10.11"
  HOSTNAME_LABEL="amr-missionbrain"
fi

echo ">>> Configuring role=$ROLE  this-host=$MY_IP  peer=$PEER_IP"

# Set a role-matching hostname -- HOSTNAME_LABEL was assigned above but
# never actually applied anywhere (a real, previously-undetected dead-code
# gap, found via a shellcheck sweep). Convenient for SSH/`ping
# <hostname>.local` rather than remembering which static IP is which role.
if [ "$(hostname)" != "$HOSTNAME_LABEL" ]; then
  sudo hostnamectl set-hostname "$HOSTNAME_LABEL"
  echo ">>> Hostname set to $HOSTNAME_LABEL (takes full effect after a reboot;"
  echo "    some tools may still show the old name in this session)."
fi

# --- 1. Static IP via netplan (persists across reboots) ---
NETPLAN_FILE=/etc/netplan/99-strawberry-amr-eth0.yaml
sudo bash -c "cat > $NETPLAN_FILE" <<NETPLAN
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      addresses: [${MY_IP}/24]
      # No gateway/nameservers on purpose -- this interface is ONLY the
      # direct Pi<->Pi link, not an internet route. Leave WiFi (wlan0) as
      # the default route for internet/SSH, configured separately by your
      # normal Raspberry Pi OS network setup (raspi-config / Desktop).
NETPLAN
sudo chmod 600 "$NETPLAN_FILE"
sudo netplan apply
echo ">>> eth0 set to $MY_IP/24 (persistent)"

# --- 2. CycloneDDS bound to eth0 ---
CYCLONE_FILE="$HOME/cyclonedds.xml"
cat > "$CYCLONE_FILE" <<CYCLONE
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
CYCLONE

BASHRC_MARK="# --- strawberry_amr_ws network setup (added by setup_network.sh) ---"
if ! grep -qF "$BASHRC_MARK" "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<BASHRC

$BASHRC_MARK
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$CYCLONE_FILE
BASHRC
  echo ">>> Appended ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI to ~/.bashrc"
else
  echo ">>> ~/.bashrc already has the network block -- not duplicating. If you need"
  echo "    to change domain ID or the Cyclone config, edit ~/.bashrc directly."
fi

echo ""
echo "=== Done. Open a NEW terminal (or 'source ~/.bashrc') before launching ROS. ==="
echo "=== Verify from BOTH Pis after both have been configured: ==="
echo "      ping $PEER_IP"
echo "      ros2 topic list      (should show topics from the OTHER Pi once its"
echo "                            launch file is running -- if empty, see"
echo "                            DEPLOYMENT_GUIDE.md 'Network Troubleshooting')"
