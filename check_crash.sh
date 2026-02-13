#!/bin/bash
echo "===== CRASH LOCATION: featureExtraction.cpp around line 1125 ====="
sed -n '1110,1140p' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp

echo ""
echo "===== TIMESTAMP SYNC LOGIC: getMeasurements() ====="
grep -n 'meas_start_time\|lidar_start_time\|sync\|measureBuf\|lidarBuf\|out of sync\|Large lidar' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp | head -30

echo ""
echo "===== livoxHandler: how timestamps are set ====="
grep -n 'livoxHandler\|timebase\|header.stamp\|time_base\|offset_time\|timestamp' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp | head -40
