#!/bin/bash
source /workspace/install/setup.bash

echo "===== FEATURE EXTRACTION: ALL declare_parameter calls ====="
FE_SRC="/workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp"
grep -n 'declare_parameter\|get_parameter' "$FE_SRC" | head -60

echo ""
echo "===== FEATURE EXTRACTION: sensor type parsing ====="
sed -n '220,240p' "$FE_SRC"

echo ""
echo "===== IMU PREINTEGRATION SOURCE FILES ====="
find /workspace/src/slam -name "*.cpp" -o -name "*.h" | xargs grep -l "imu_preintegration\|ImuPreintegration" 2>/dev/null

echo ""
echo "===== IMU PREINTEGRATION: parameter loading ====="
IMU_SRC=$(find /workspace/src/slam -name "*.cpp" | xargs grep -l "class.*ImuPreintegration\|ImuPreintegration(" 2>/dev/null | head -1)
echo "Found: $IMU_SRC"
if [ -n "$IMU_SRC" ]; then
    grep -n 'declare_parameter\|get_parameter\|param(' "$IMU_SRC" | head -60
fi

echo ""
echo "===== LASER MAPPING: parameter loading ====="
LM_SRC=$(find /workspace/src/slam -name "*.cpp" | xargs grep -l "class.*LaserMapping\|LaserMapping(" 2>/dev/null | head -1)
echo "Found: $LM_SRC"
if [ -n "$LM_SRC" ]; then
    grep -n 'declare_parameter\|get_parameter\|param(' "$LM_SRC" | head -60
fi

echo ""
echo "===== UTILITY.CPP: readParameters() detail ====="
UTIL_SRC="/workspace/src/slam/arise_slam_mid360/include/arise_slam_mid360/utils/utility.cpp"
if [ -f "$UTIL_SRC" ]; then
    cat "$UTIL_SRC"
else
    echo "Not found at expected path, searching..."
    find /workspace/src/slam -name "utility.cpp" 2>/dev/null
fi

echo ""
echo "===== CHECK: provide_imu_laser_extrinsic param ====="
grep -rn 'provide_imu_laser_extrinsic\|calibration_file' /workspace/src/slam/arise_slam_mid360/src/ --include="*.cpp" --include="*.h" | head -20

echo ""
echo "===== SLAM CONFIG YAML: all params ====="
cat /workspace/install/arise_slam_mid360/share/arise_slam_mid360/config/livox_mid360.yaml
