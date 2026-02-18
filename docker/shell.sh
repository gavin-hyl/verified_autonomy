#!/bin/bash
# =============================================================================
# Open a shell in the Vector Autonomy Docker container
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Export user info
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export USERNAME=$(whoami)
export DISPLAY=${DISPLAY:-:0}

# Check if container is running
if ! docker compose ps --status running | grep -q "vector-autonomy-ros"; then
    echo "Container is not running. Starting it now..."
    ./run.sh
fi

echo "Opening shell in container..."
echo "(Type 'exit' to leave the container)"
echo ""

# Execute bash in the running container
docker compose exec ros bash
