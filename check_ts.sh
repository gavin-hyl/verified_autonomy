#!/bin/bash
source /workspace/install/setup.bash

echo "===== Active topics ====="
ros2 topic list 2>&1 | grep -E "lidar|imu|feature|laser_odom|state_est|cmd_vel"

echo ""
echo "===== LiDAR timestamps (CustomMsg) ====="
# Get header stamp and timebase from a few messages
timeout 4 ros2 topic echo /lidar/scan --no-arr --once 2>&1 | grep -E "sec:|nanosec:|timebase|point_num|lidar_id"

echo ""
echo "===== IMU timestamps ====="
timeout 4 ros2 topic echo /imu/data --once 2>&1 | grep -E "sec:|nanosec:" | head -4

echo ""
echo "===== Compare raw epoch timestamps ====="
echo "Getting 3 LiDAR stamps..."
timeout 4 ros2 topic echo /lidar/scan --field header.stamp 2>&1 | head -9
echo ""
echo "Getting 3 IMU stamps..."
timeout 4 ros2 topic echo /imu/data --field header.stamp 2>&1 | head -9

echo ""
echo "===== System time ====="
date +%s.%N
echo "(Unix epoch seconds)"
