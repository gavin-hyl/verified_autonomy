#!/bin/bash
echo "===== LASER MAPPING: subscriptions ====="
grep -n 'create_subscription\|Subscriber\|subscribe(' /workspace/src/slam/arise_slam_mid360/src/LaserMapping/laserMapping.cpp | head -20

echo ""
echo "===== LASER MAPPING: what topics does it use ====="
grep -n 'LASER_TOPIC\|IMU_TOPIC\|feature_info\|laser_odom\|laser_cloud' /workspace/src/slam/arise_slam_mid360/src/LaserMapping/laserMapping.cpp | head -30

echo ""
echo "===== FEATURE EXTRACTION: what does it publish ====="
grep -n 'create_publisher\|publish(' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp | head -20

echo ""
echo "===== IMU PREINTEGRATION: subscriptions and topics ====="
grep -n 'create_subscription\|LASER_TOPIC\|IMU_TOPIC\|feature_info\|laser_odom\|state_estimation' /workspace/src/slam/arise_slam_mid360/src/ImuPreintegration/imuPreintegration.cpp | head -30

echo ""
echo "===== CROSS-CHECK: laser_mapping_node YAML params vs code ====="
echo "--- YAML says ---"
echo "  laser_topic: velodyne_points  (no leading /)"
echo "  imu_topic: imu/data  (no leading /)"
echo ""
echo "--- readGlobalparam defaults ---"
grep 'declare_parameter.*laser_topic\|declare_parameter.*imu_topic' /workspace/src/slam/arise_slam_mid360/src/parameter/parameter.cpp

echo ""
echo "===== CHECK: does laser_mapping use LASER_TOPIC global? ====="
grep -n 'LASER_TOPIC' /workspace/src/slam/arise_slam_mid360/src/LaserMapping/laserMapping.cpp

echo ""
echo "===== CHECK: feature_extraction initInterface (lines 50-100) ====="
sed -n '50,100p' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp
