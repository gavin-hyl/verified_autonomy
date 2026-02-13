#!/bin/bash
echo "===== uniformfeatureExtraction for PointcloudXYZITR (the one called at line 1123) ====="
sed -n '26,80p' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/DepthImageKeypointExtractor.cpp

echo ""
echo "===== PointcloudXYZITR definition ====="
grep -rn 'PointcloudXYZITR\|XYZITR' /workspace/src/slam/arise_slam_mid360/include/ --include="*.h" | head -15

echo ""
echo "===== Check timestamp values: current LiDAR + IMU stamps ====="
source /workspace/install/setup.bash
echo "--- LiDAR header stamp ---"
timeout 3 ros2 topic echo /lidar/scan --field header.stamp --once 2>&1
echo ""
echo "--- LiDAR timebase ---"
timeout 3 ros2 topic echo /lidar/scan --field timebase --once 2>&1
echo ""
echo "--- IMU stamp ---"
timeout 3 ros2 topic echo /imu/data --field header.stamp --once 2>&1
