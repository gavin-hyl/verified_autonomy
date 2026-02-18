#!/bin/bash
# =============================================================================
# Stop the Vector Autonomy Docker container
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Export user info
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export USERNAME=$(whoami)

echo "Stopping Vector Autonomy container..."
docker compose down

echo "Container stopped."
