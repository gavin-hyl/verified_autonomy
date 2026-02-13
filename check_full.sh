#!/bin/bash
source /workspace/install/setup.bash

echo "============================================"
echo "  FULL PIPELINE STATUS CHECK"
echo "  $(date)"
echo "============================================"

echo ""
echo "===== TOPIC RATES ====="
for topic in /lidar/scan /imu/data /feature_info /laser_odometry /state_estimation /terrain_map /cmd_vel; do
    echo -n "$topic: "
    timeout 4 ros2 topic hz $topic --window 5 2>&1 | grep "average rate" | tail -1
    if [ $? -ne 0 ]; then
        echo "NO DATA or timeout"
    fi
done

echo ""
echo "===== /state_estimation (SLAM pose) ====="
timeout 3 ros2 topic echo /state_estimation --once 2>&1 | head -20

echo ""
echo "===== /cmd_vel (velocity commands for robot) ====="
echo "Collecting 5 cmd_vel messages..."
timeout 10 ros2 topic echo /cmd_vel 2>&1 | head -60

echo ""
echo "===== Process check ====="
ps aux | grep -E "feature_ext|laser_map|imu_preint" | grep -v grep
