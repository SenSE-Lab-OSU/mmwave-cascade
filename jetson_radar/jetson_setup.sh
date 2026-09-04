#!/bin/bash
# jetson_setup.sh - one-time setup for the radar host on a Jetson Orin Nano.
#
#   chmod +x jetson_setup.sh
#   sudo ./jetson_setup.sh            # auto-detect the wired interface
#   sudo ./jetson_setup.sh eth0       # or name it explicitly
#
# What it does (all idempotent, safe to re-run):
#   1. apt: python3-numpy python3-serial (pyserial)
#   2. sysctl: large UDP receive buffers, persisted in /etc/sysctl.d
#   3. NetworkManager: wired interface -> static 192.168.33.30/24 (DCA1000 link)
#   4. adds the invoking user to 'dialout' (serial port access, re-login needed)
#   5. max power mode + locked clocks (nvpmodel / jetson_clocks)
set -e

if [ "$(id -u)" -ne 0 ]; then echo "run with sudo"; exit 1; fi
REAL_USER=${SUDO_USER:-$USER}

echo "== 1. Python packages"
apt-get install -y python3-numpy python3-serial >/dev/null && echo "   ok"

echo "== 2. UDP receive buffers"
cat > /etc/sysctl.d/99-dca1000.conf <<EOF
net.core.rmem_max = 268435456
net.core.rmem_default = 67108864
net.core.netdev_max_backlog = 250000
EOF
sysctl -p /etc/sysctl.d/99-dca1000.conf >/dev/null && echo "   ok"

echo "== 3. Wired interface -> 192.168.33.30/24"
IFACE=$1
if [ -z "$IFACE" ]; then
  # first non-loopback, non-wireless, non-virtual ethernet interface
  for d in /sys/class/net/*; do
    n=$(basename "$d")
    case "$n" in lo|wl*|docker*|veth*|l4tbr*|usb*|dummy*) continue;; esac
    if [ -d "$d/wireless" ]; then continue; fi
    IFACE=$n; break
  done
fi
if [ -z "$IFACE" ]; then echo "   no wired interface found; pass it as argument"; exit 1; fi
echo "   using interface: $IFACE"
nmcli con delete dca1000 >/dev/null 2>&1 || true
nmcli con add type ethernet ifname "$IFACE" con-name dca1000 \
      ipv4.method manual ipv4.addresses 192.168.33.30/24 \
      ipv4.never-default yes ipv6.method disabled >/dev/null
nmcli con up dca1000 >/dev/null && echo "   ok ($IFACE = 192.168.33.30)"

echo "== 4. Serial port permission"
usermod -aG dialout "$REAL_USER" && echo "   $REAL_USER added to dialout (log out and back in once)"

echo "== 5. Power mode"
if command -v nvpmodel >/dev/null; then nvpmodel -m 0 >/dev/null 2>&1 && echo "   nvpmodel -m 0 ok"; fi
if command -v jetson_clocks >/dev/null; then jetson_clocks >/dev/null 2>&1 && echo "   jetson_clocks ok"; fi

echo
echo "Done. Log out and back in (for the serial permission), then run:"
echo "   python3 jetson_check.py"
