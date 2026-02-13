#!/bin/bash
echo "===== Full removeClosestFarestPointCloud (lines 247-295) ====="
sed -n '247,300p' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp

echo ""
echo "===== undistortionAndscanregistration LIVOX branch (lines 1114-1135) ====="
sed -n '1114,1140p' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp
