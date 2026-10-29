#!/usr/bin/env bash
set -e

RULE_FILE="/etc/udev/rules.d/99-rover-serial.rules"

echo "Creating udev rule file at $RULE_FILE..."

cat << 'EOF' > "$RULE_FILE"
# =============================================================================
# Rover Static Serial Rules
# Automatically binds XBee 900MHz and LC29H RTK GPS to static symlinks
# =============================================================================

# XBee 900MHz Module (FTDI FT232R) -> /dev/rover_900mhz & /dev/rover_900mh
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="B0025NTL", SYMLINK+="rover_900mhz rover_900mh", MODE="0666", GROUP="dialout"

# LC29H RTK GPS Module (Silicon Labs CP2102N) -> /dev/rover_rtk
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="ac57edb6726cef119a94a3adc169b110", SYMLINK+="rover_rtk", MODE="0666", GROUP="dialout"
EOF

echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo ""
echo "=== Static Links Created Successfully ==="
ls -l /dev/rover* 2>/dev/null || echo "Please unplug and replug the USB cables to trigger the symlinks."
