#!/bin/bash
# =============================================================================
# Clean up Docker resources for Vector Autonomy Stack
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "Cleaning Vector Autonomy Docker Resources"
echo "=============================================="
echo ""

# Export user info
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export USERNAME=$(whoami)

# Stop container if running
echo "Stopping container..."
docker compose down 2>/dev/null || true

# Ask about image removal
read -p "Remove Docker image? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Removing image..."
    docker rmi vector-autonomy:jazzy 2>/dev/null || true
fi

# Ask about volume removal
read -p "Remove persistent volumes (bash history)? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Removing volumes..."
    docker volume rm docker_vector-autonomy-bash-history 2>/dev/null || true
fi

echo ""
echo "Cleanup complete."
