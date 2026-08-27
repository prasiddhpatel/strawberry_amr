#!/usr/bin/env bash
# WiFi-based ROS 2 network setup for the Pi(4GB)+Orin topology -- run
# ONCE on each machine with the matching role.
#
# CORRECTED from an earlier revision of this script, which assumed a
# physical Ethernet cable between the Pi and the Orin. That was a real
# design mistake, not a minor detail: the Orin is not mounted on the
# robot in this topology (unlike the original dual-Pi setup, where both
# boxes are physically on the chassis and a short cable between them costs
# nothing) -- the robot has to actually drive around the polytunnel, and
# a cable to a Orin that isn't on it makes that impossible. This should
# have been reconsidered when the Pi+Orin split was first built, not
# after the fact.
#
# THE DESIGN: the Pi hosts its own WiFi Access Point (hostapd + dnsmasq),
# and the Orin joins it as a client. This is deliberately NOT "both
# machines join whatever WiFi router happens to be nearby" -- a real
# Irish polytunnel is not guaranteed to have any WiFi coverage at all, and
# this project needs to work there, not just in a university lab with
# existing infrastructure. A Pi-hosted AP works identically in Lab 3003
# and in a field with zero external network -- it depends on nothing but
# the two machines themselves. See Part 1.5 of ALL_IN_ONE_DEPLOYMENT_GUIDE.md
# for the alternative (joining an existing WiFi network) if you know for a
# given session that reliable infrastructure is available and you'd
# rather not run the Pi as an AP.
#
# HONEST ABOUT THE REAL TRADEOFF THIS INTRODUCES: WiFi has a meaningfully
# worse latency/reliability profile than the wired link this was designed
# around, and row_navigation/safety_supervisor/twist_mux/ekf_node all
# still run on the Orin (see docs/ORIN_PI_SPLIT_ARCHITECTURE.md) --
# meaning the reactive control/safety loop now depends on a WiFi link that
# can drop or degrade more easily than a cable could. Nothing about that
# mitigation changed: base_controller's own local command watchdog still
# forces a hard stop if /cmd_vel goes stale for any reason, teleop and the
# e-stop button are still fully local to the Pi via twist_mux_pi_local
# (see that config and pi_hardware_and_control.launch.py), and the
# physical e-stop remains independent of all of this regardless. Those
# properties matter MORE now, not less, given WiFi is more likely to
# actually drop than a cable was.
#
# Usage:
#   ./setup_network_pi_orin.sh pi        (on the Raspberry Pi 4B 4GB)
#   ./setup_network_pi_orin.sh orin    (on the Orin)
#
# NOT the same script as setup_network.sh (that one is for the ORIGINAL
# dual-Pi 2GB+4GB topology, where both boxes ARE physically on the robot
# and a wired link between them remains the right choice -- keep both
# scripts, they serve genuinely different hardware topologies).
#
# Subnet 192.168.30.0/24, same as the earlier wired revision of this
# script -- deliberately its own dedicated range, distinct from
# 192.168.1.0/24 (common home/office router default), 192.168.10.0/24
# (this workspace's dual-Pi topology), and 192.168.20.0/24 (a different
# AI-generated bundle for this same robot uses that range).
#
# WHY hostapd+dnsmasq, not NetworkManager's `nmcli dev wifi hotspot`: the
# Pi already uses netplan/systemd-networkd as its one network-config
# authority (see setup_network.sh, setup_udev_rules.sh). Introducing
# NetworkManager on the Pi just for AP duty would mean two different
# tools potentially both trying to manage network interfaces on the same
# machine -- exactly the kind of "two tools fighting over the same
# interface" risk this project has deliberately avoided elsewhere (see
# this script's own Orin-role comments below, and
# docs/ORIN_PI_SPLIT_ARCHITECTURE.md's network section). hostapd+dnsmasq
# is the traditional, extremely well-documented way to do this on any
# Debian-based system regardless of which higher-level tool (netplan,
# NetworkManager, dhcpcd) is otherwise in use, and keeps netplan as the
# Pi's single authority throughout.
set -e

ROLE="$1"
if [ "$ROLE" != "pi" ] && [ "$ROLE" != "orin" ]; then
  echo "Usage: $0 [pi|orin]"
  echo "  pi      -- run this on the Raspberry Pi 4B 4GB (hosts the WiFi AP)"
  echo "  orin    -- run this on the Jetson AGX Orin (joins the Pi's AP)"
  exit 1
fi

SSID="StrawberryAMR"
PI_IP="192.168.30.1"
ORIN_IP="192.168.30.100"

if [ "$ROLE" = "pi" ]; then
  echo ">>> Configuring role=pi (WiFi AP host)  this-host=$PI_IP  peer=$ORIN_IP"
  echo ""

  sudo apt update
  sudo apt install -y hostapd dnsmasq

  echo ">>> Detected wireless interfaces:"
  iw dev 2>/dev/null | grep -A1 Interface || ip -brief link show | grep -i wlan
  read -r -p ">>> Enter the WiFi interface name (e.g. wlan0): " IFACE
  read -r -s -p ">>> Choose a WPA2 password for the '$SSID' network (8+ chars): " PSK
  echo ""
  read -r -p ">>> Use 2.4GHz for range/reliability (recommended for a field polytunnel -- plastic+metal hoop structures attenuate 5GHz more), or 5GHz for bandwidth (fine for short-range Lab 3003 testing)? [2.4/5]: " BAND

  if [ "$BAND" = "5" ]; then
    HW_MODE="a"
    CHANNEL="36"
  else
    HW_MODE="g"
    CHANNEL="6"
  fi

  # Stop both services while reconfiguring, so a partial config from a
  # previous run of this script can't interfere
  sudo systemctl stop hostapd 2>/dev/null || true
  sudo systemctl stop dnsmasq 2>/dev/null || true

  # --- static IP on the AP interface, via netplan (keeping netplan as the
  # Pi's ONE network-config authority -- see module header). This assigns
  # an address WITHOUT netplan trying to negotiate WiFi client association
  # itself (that's hostapd's job below, not netplan's `wifis:` stanza,
  # which is specifically for CLIENT connections) -- declaring a wifi
  # interface under `ethernets:` is a standard, deliberate trick for
  # exactly this "external tool manages the radio, netplan just assigns
  # the IP" case. ---
  NETPLAN_FILE=/etc/netplan/99-strawberry-amr-ap.yaml
  sudo bash -c "cat > $NETPLAN_FILE" <<NETPLAN
network:
  version: 2
  renderer: networkd
  ethernets:
    ${IFACE}:
      dhcp4: no
      addresses: [${PI_IP}/24]
NETPLAN
  sudo chmod 600 "$NETPLAN_FILE"

  # --- hostapd: the actual 802.11 AP ---
  sudo bash -c "cat > /etc/hostapd/hostapd.conf" <<HOSTAPD
interface=${IFACE}
driver=nl80211
ssid=${SSID}
hw_mode=${HW_MODE}
channel=${CHANNEL}
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=${PSK}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
HOSTAPD
  sudo chmod 600 /etc/hostapd/hostapd.conf
  if grep -q "^#DAEMON_CONF=" /etc/default/hostapd 2>/dev/null; then
    sudo sed -i 's|#DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd
  elif ! grep -q "^DAEMON_CONF=" /etc/default/hostapd 2>/dev/null; then
    echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' | sudo tee -a /etc/default/hostapd >/dev/null
  fi

  # --- dnsmasq: DHCP on the AP interface only, scoped as a small pool for
  # convenience (e.g. a phone for quick monitoring) -- the Orin itself
  # does NOT rely on this; it pins its own static IP directly (see the
  # Orin role below), same pattern this script already used for the
  # wired link, just attached to the WiFi profile now instead. ---
  sudo bash -c "cat > /etc/dnsmasq.d/strawberry-amr-ap.conf" <<DNSMASQ
interface=${IFACE}
bind-interfaces
dhcp-range=192.168.30.50,192.168.30.150,255.255.255.0,24h
DNSMASQ

  sudo netplan apply
  sudo systemctl unmask hostapd 2>/dev/null || true
  sudo systemctl enable hostapd dnsmasq
  sudo systemctl restart hostapd
  sudo systemctl restart dnsmasq

  echo ""
  echo ">>> AP '$SSID' should now be broadcasting on $IFACE ($PI_IP/24, ${BAND}GHz band)."
  echo "    Both services are enabled to auto-start on boot."
  echo ">>> Verify: sudo systemctl status hostapd dnsmasq -- both should"
  echo "    show 'active (running)'. If hostapd fails to start, the most"
  echo "    common cause is another service (wpa_supplicant, NetworkManager)"
  echo "    also trying to manage $IFACE -- check 'sudo systemctl status"
  echo "    wpa_supplicant' and disable it for this interface if so."

else
  echo ">>> Configuring role=orin (WiFi AP client)  this-host=$ORIN_IP  peer=$PI_IP"
  echo ""
  echo ">>> Make sure the Pi's AP is already up (run this script's 'pi'"
  echo "    role there first) before continuing."
  echo ""
  echo ">>> Detected WiFi networks:"
  nmcli dev wifi list | head -15
  read -r -p ">>> Confirm '$SSID' is visible above, then press Enter to join it: " _
  read -r -s -p ">>> Enter the WPA2 password you set on the Pi: " PSK
  echo ""

  CONN_NAME="strawberry-amr-ap-link"
  nmcli connection delete "$CONN_NAME" >/dev/null 2>&1 || true

  nmcli dev wifi connect "$SSID" password "$PSK" name "$CONN_NAME"

  sudo nmcli connection modify "$CONN_NAME" \
    ipv4.addresses "${ORIN_IP}/24" ipv4.method manual ipv6.method disabled \
    connection.autoconnect yes
  sudo nmcli connection up "$CONN_NAME"

  IFACE=$(nmcli -t -f DEVICE,CONNECTION dev status | awk -F: -v c="$CONN_NAME" '$2==c{print $1}')
  echo ">>> Joined '$SSID', pinned to $ORIN_IP/24 on $IFACE (persistent,"
  echo "    via NetworkManager profile '$CONN_NAME', auto-connects on boot"
  echo "    /whenever the AP is in range)."
fi

# --- CycloneDDS bound to the WiFi interface only, on BOTH roles ---
CYCLONE_FILE="$HOME/cyclonedds.xml"
cat > "$CYCLONE_FILE" <<CYCLONE
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>${IFACE}</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
CYCLONE

BASHRC_MARK="# --- strawberry_amr_ws pi-Orin network setup (added by setup_network_pi_orin.sh) ---"
if ! grep -qF "$BASHRC_MARK" "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<BASHRC

$BASHRC_MARK
export ROS_DOMAIN_ID=43
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$CYCLONE_FILE
BASHRC
  echo ">>> Appended ROS_DOMAIN_ID (43) / RMW_IMPLEMENTATION / CYCLONEDDS_URI to ~/.bashrc"
else
  echo ">>> ~/.bashrc already has the network block -- not duplicating. If you need"
  echo "    to change domain ID or the Cyclone config, edit ~/.bashrc directly."
fi

echo ""
echo "=== Done. Open a NEW terminal (or 'source ~/.bashrc') before launching ROS. ==="
echo "=== Verify from BOTH machines after both have been configured: ==="
echo "      ping $PI_IP        # from the Orin"
echo "      ping $ORIN_IP    # from the Pi"
echo "      ros2 topic list      (should show topics from the OTHER machine once its"
echo "                            launch file is running -- if empty, see"
echo "                            DEPLOYMENT_GUIDE.md 'Network Troubleshooting')"
echo ""
echo "=== A note on range: unlike the wired link this replaces, WiFi signal"
echo "=== strength genuinely degrades with distance and obstacles. If the"
echo "=== link becomes unreliable as the robot drives further from wherever"
echo "=== the Orin is positioned, that is a real, physical limitation to"
echo "=== plan around (e.g. keep the Orin within the tunnel, not outside"
echo "=== it) -- not a misconfiguration to keep chasing in software."
