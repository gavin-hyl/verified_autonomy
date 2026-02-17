#!/bin/bash
# =============================================================================
# Unitree Go1 Network Setup Script
# =============================================================================
# This script configures your Ethernet interface to communicate with the
# Unitree Go1 robot. Run this on the HOST machine (not in Docker).
#
# Usage:
#   ./unitree_network_setup.sh [interface_name]
#
# If interface_name is not provided, the script will list available interfaces.
# =============================================================================

set -e

ROBOT_IP="192.168.123.161"
HOST_IP="192.168.123.162"
NETMASK="255.255.255.0"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=============================================="
echo "Unitree Go1 Network Setup"
echo "=============================================="
echo ""

# Function to list Ethernet interfaces
list_interfaces() {
    echo "Available Ethernet interfaces:"
    echo ""
    ip link show | grep -E "^[0-9]+:" | grep -v "lo:" | while read line; do
        iface=$(echo "$line" | awk -F': ' '{print $2}' | cut -d'@' -f1)
        state=$(echo "$line" | grep -oP 'state \K\w+' || echo "UNKNOWN")
        echo "  $iface ($state)"
    done
    echo ""
    echo "Look for an interface that appears when you plug in the Ethernet cable."
    echo "Common names: eth0, enp*, enx* (USB Ethernet adapters)"
}

# Check if interface was provided
if [ -z "$1" ]; then
    echo "No interface specified."
    echo ""
    list_interfaces
    echo ""
    echo "Usage: $0 <interface_name>"
    echo "Example: $0 enp0s25"
    exit 1
fi

INTERFACE="$1"

# Check if interface exists
if ! ip link show "$INTERFACE" &> /dev/null; then
    echo -e "${RED}Error: Interface '$INTERFACE' not found${NC}"
    echo ""
    list_interfaces
    exit 1
fi

echo "Configuring interface: $INTERFACE"
echo "  Host IP:  $HOST_IP"
echo "  Robot IP: $ROBOT_IP"
echo "  Netmask:  $NETMASK"
echo ""

# Check if we have sudo
if ! sudo -v &> /dev/null; then
    echo -e "${RED}Error: This script requires sudo privileges${NC}"
    exit 1
fi

# Configure the interface
echo "Bringing interface down..."
sudo ip link set "$INTERFACE" down

echo "Setting IP address..."
sudo ip addr flush dev "$INTERFACE"
sudo ip addr add "$HOST_IP/24" dev "$INTERFACE"

echo "Bringing interface up..."
sudo ip link set "$INTERFACE" up

echo ""
echo -e "${GREEN}Network configured successfully!${NC}"
echo ""

# Test connectivity
echo "Testing connectivity to robot at $ROBOT_IP..."
if ping -c 1 -W 2 "$ROBOT_IP" &> /dev/null; then
    echo -e "${GREEN}SUCCESS: Robot is reachable!${NC}"
else
    echo -e "${YELLOW}WARNING: Cannot ping robot at $ROBOT_IP${NC}"
    echo "  - Make sure the robot is powered on"
    echo "  - Check Ethernet cable connection"
    echo "  - The robot may take a moment to boot"
fi

echo ""
echo "=============================================="
echo "Next steps:"
echo "  1. cd docker && ./run.sh"
echo "  2. ./shell.sh"
echo "  3. Inside container: cd /unitree_ws && colcon build"
echo "  4. source install/setup.bash"
echo "  5. ros2 run unitree_legged_real ros2_udp HIGHLEVEL"
echo "=============================================="
