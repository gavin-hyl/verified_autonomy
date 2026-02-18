#!/bin/bash
set -e

FEAT_CPP="/workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp"

echo "===== Current state around crash site (lines 1126-1142) ====="
sed -n '1126,1142p' "$FEAT_CPP"

echo ""
echo "===== Removing duplicate block (lines 1132-1138) ====="
# The previous sed run left a duplicate } else { ... publishTopic ... lidarBuf.clean block
# Lines 1132-1138 are the duplicate that broke brace structure
sed -i '1132,1138d' "$FEAT_CPP"

echo ""
echo "===== Fixed code (lines 1114-1148) ====="
sed -n '1114,1148p' "$FEAT_CPP"

echo ""
echo "===== Rebuilding arise_slam_mid360 ====="
cd /workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arise_slam_mid360 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1

echo ""
echo "===== Build complete ====="
