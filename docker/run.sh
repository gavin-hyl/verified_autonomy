#!/bin/bash
# =============================================================================
# Start the Vector Autonomy Docker container
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "Starting Vector Autonomy Container"
echo "=============================================="
echo ""

# Export user info for docker-compose
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export USERNAME=$(whoami)
export DISPLAY=${DISPLAY:-:0}

# Allow X11 connections from Docker
echo "Allowing X11 connections..."
xhost +local:docker 2>/dev/null || echo "Warning: xhost command failed (may need X11)"

# Check if container is already running
if docker compose ps --status running | grep -q "vector-autonomy-ros"; then
    echo "Container is already running."
    echo "Use './shell.sh' to open a terminal."
    exit 0
fi

# Start the container
echo "Starting container..."
docker compose up -d

echo ""
echo "=============================================="
echo "Container started!"
echo ""
echo "Commands:"
echo "  ./shell.sh  - Open a terminal in the container"
echo "  ./stop.sh   - Stop the container"
echo ""
echo "Inside the container:"
echo "  cb          - Build full workspace"
echo "  cb-sim      - Build for simulation only"
echo "  ws          - Go to workspace directory"
echo "=============================================="
