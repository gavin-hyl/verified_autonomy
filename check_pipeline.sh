#!/bin/bash
source /workspace/install/setup.bash

echo "============================================"
echo "  FULL PIPELINE TOPIC CHECK"
echo "  $(date)"
echo "============================================"

echo ""
echo "===== 1. LiDAR INPUT ====="
echo -n "/lidar/scan: "
timeout 3 ros2 topic hz /lidar/scan --window 5 2>&1 | tail -1

echo -n "/imu/data: "
timeout 3 ros2 topic hz /imu/data --window 10 2>&1 | tail -1

echo ""
echo "===== 2. SLAM PIPELINE ====="
echo -n "/feature_info: "
timeout 3 ros2 topic hz /feature_info --window 3 2>&1 | tail -1

echo -n "/laser_odometry: "
timeout 3 ros2 topic hz /laser_odometry --window 3 2>&1 | tail -1

echo -n "/state_estimation: "
timeout 3 ros2 topic hz /state_estimation --window 3 2>&1 | tail -1

echo -n "/registered_scan: "
timeout 3 ros2 topic hz /registered_scan --window 3 2>&1 | tail -1

echo ""
echo "===== 3. TERRAIN & PLANNING ====="
echo -n "/terrain_map: "
timeout 3 ros2 topic hz /terrain_map --window 3 2>&1 | tail -1

echo -n "/terrain_map_ext: "
timeout 3 ros2 topic hz /terrain_map_ext --window 3 2>&1 | tail -1

echo -n "/way_point: "
timeout 3 ros2 topic hz /way_point --window 3 2>&1 | tail -1

echo -n "/path: "
timeout 3 ros2 topic hz /path --window 3 2>&1 | tail -1

echo -n "/cmd_vel: "
timeout 3 ros2 topic hz /cmd_vel --window 3 2>&1 | tail -1

echo ""
echo "===== 4. ALL ACTIVE TOPICS ====="
ros2 topic list 2>&1

echo ""
echo "===== 5. TF FRAMES ====="
timeout 2 ros2 run tf2_ros tf2_echo map sensor 2>&1 | head -5
