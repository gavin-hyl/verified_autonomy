#!/bin/bash
source /workspace/install/setup.bash

echo "===== Checking /lidar/scan timestamps (CustomMsg) ====="
timeout 3 ros2 topic echo /lidar/scan --field timebase --once 2>&1
echo ""

echo "===== Checking /lidar/scan header stamp ====="
timeout 3 ros2 topic echo /lidar/scan --field header.stamp --once 2>&1
echo ""

echo "===== Checking /imu/data header stamp ====="
timeout 3 ros2 topic echo /imu/data --field header.stamp --once 2>&1
echo ""

echo "===== Side-by-side timestamps (5 samples each) ====="
echo "--- LiDAR (CustomMsg timebase + header) ---"
timeout 5 ros2 topic echo /lidar/scan --field header.stamp --no-arr 2>&1 | head -12
echo ""
echo "--- IMU stamps ---"
timeout 5 ros2 topic echo /imu/data --field header.stamp --no-arr 2>&1 | head -12
