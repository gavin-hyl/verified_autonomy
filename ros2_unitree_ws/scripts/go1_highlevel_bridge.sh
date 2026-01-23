#!/bin/bash
# =============================================================================
# Start the Go1 High-Level UDP Bridge
# =============================================================================
# This script starts the ROS 2 node that bridges between ROS topics and the
# Go1's UDP protocol. Run this INSIDE the Docker container.
#
# Topics created:
#   - /high_cmd (subscriber) - Send commands to robot
#   - /high_state (publisher) - Receive state from robot
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_WS="/unitree_ws"

echo "=============================================="
echo "Go1 High-Level UDP Bridge"
echo "=============================================="

# Check if we're in the container
if [ ! -d "$UNITREE_WS" ]; then
    echo "Error: /unitree_ws not found. Are you inside the Docker container?"
    echo "Run: cd docker && ./shell.sh"
    exit 1
fi

# Check if workspace is built
if [ ! -f "$UNITREE_WS/install/setup.bash" ]; then
    echo "Workspace not built. Building now..."
    cd "$UNITREE_WS"
    colcon build --symlink-install
fi

# Source the workspace
source "$UNITREE_WS/install/setup.bash"

echo ""
echo "Starting high-level bridge..."
echo "  Subscribes to: /high_cmd"
echo "  Publishes to:  /high_state"
echo ""
echo "Press Ctrl+C to stop"
echo "=============================================="
echo ""

ros2 run unitree_legged_real ros2_udp HIGHLEVEL
