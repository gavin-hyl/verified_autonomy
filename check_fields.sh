#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
echo "=== /lidar/scan message structure ==="
timeout 3 ros2 topic echo /lidar/scan --once --no-arr 2>&1 | head -30
echo ""
echo "=== /lidar/scan field names ==="
timeout 3 ros2 topic echo /lidar/scan --once --field fields 2>&1 | head -20
