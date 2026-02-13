#!/bin/bash
set -e

FEAT_CPP="/workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp"

echo "===== Current code at crash site (lines 1122-1135) ====="
sed -n '1122,1135p' "$FEAT_CPP"

echo ""
echo "===== Applying fix ====="

# Fix 1: Replace assert with graceful skip
# Fix 2: Add lidarBuf.clean() after LIVOX branch (matching non-LIVOX behavior at line 783)
sed -i '1125,1126s/.*//' "$FEAT_CPP"

# Now insert the replacement at line 1125
sed -i '1125i\                if (plannerPoints->empty()) {\n                    RCLCPP_WARN(this->get_logger(), "Empty plannerPoints (input: %zu pts), skipping frame", lidar_msg->points.size());\n                } else {\n                    publishTopic(lidar_start_time, lidar_msg, edgePoints, plannerPoints, bobPoints, q_w_original_l);\n                }\n                lidarBuf.clean(lidar_start_time);' "$FEAT_CPP"

echo ""
echo "===== Fixed code (lines 1122-1140) ====="
sed -n '1122,1140p' "$FEAT_CPP"

echo ""
echo "===== Rebuilding arise_slam_mid360 ====="
cd /workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arise_slam_mid360 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1

echo ""
echo "===== Build complete ====="
