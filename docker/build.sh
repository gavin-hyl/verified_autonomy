#!/bin/bash
# =============================================================================
# Build the Docker image for Vector Autonomy Stack
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "Building Vector Autonomy Docker Image"
echo "=============================================="
echo ""

# Export user info for docker-compose build args
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export USERNAME=$(whoami)

echo "Building with:"
echo "  USER_ID:  $USER_ID"
echo "  GROUP_ID: $GROUP_ID"
echo "  USERNAME: $USERNAME"
echo ""

# Build the image
docker compose build

echo ""
echo "=============================================="
echo "Build complete!"
echo ""
echo "Next steps:"
echo "  1. Run './run.sh' to start the container"
echo "  2. Run './shell.sh' to open a terminal"
echo "=============================================="
