#!/bin/bash
source /workspace/install/setup.bash
echo "============================================"
echo "  Launching full exploration stack..."
echo "  (LiDAR driver + SLAM + planner)"
echo "============================================"
ros2 launch vehicle_simulator system_real_robot_with_exploration_planner.launch 2>&1
