#!/bin/bash
source /workspace/install/setup.bash

echo "===== 1. CALIBRATION FILE ====="
CALIB="/workspace/install/arise_slam_mid360/share/arise_slam_mid360/config/livox/livox_mid360_calibration.yaml"
if [ -f "$CALIB" ]; then
    echo "[OK] Calibration file exists: $CALIB"
    echo "--- Contents ---"
    cat "$CALIB"
else
    echo "[FAIL] Calibration file NOT FOUND: $CALIB"
    echo "Looking for alternatives..."
    find /workspace -name "*calibration*" -path "*arise_slam*" 2>/dev/null
fi

echo ""
echo "===== 2. ROBOT CONFIG (local_planner) ====="
ROBOT_CONFIG_ENV=${ROBOT_CONFIG_PATH:-mechanum_drive}
echo "ROBOT_CONFIG_PATH env: '$ROBOT_CONFIG_ENV'"
LP_SHARE=$(ros2 pkg prefix local_planner 2>/dev/null)/share/local_planner
echo "local_planner share dir: $LP_SHARE"
ROBOT_YAML="$LP_SHARE/config/${ROBOT_CONFIG_ENV}.yaml"
if [ -f "$ROBOT_YAML" ]; then
    echo "[OK] Robot config exists: $ROBOT_YAML"
    echo "--- Contents ---"
    cat "$ROBOT_YAML"
else
    echo "[FAIL] Robot config NOT FOUND: $ROBOT_YAML"
    echo "Available configs in local_planner:"
    ls -la "$LP_SHARE/config/" 2>/dev/null || echo "  Config dir not found"
fi

echo ""
echo "===== 3. SLAM CONFIG (livox_mid360.yaml) ====="
SLAM_CONFIG="/workspace/install/arise_slam_mid360/share/arise_slam_mid360/config/livox_mid360.yaml"
if [ -f "$SLAM_CONFIG" ]; then
    echo "[OK] SLAM config exists"
else
    echo "[FAIL] SLAM config NOT FOUND: $SLAM_CONFIG"
fi

echo ""
echo "===== 4. FEATURE EXTRACTION - expected params ====="
echo "Checking source for declare_parameter / get_parameter calls..."
FE_SRC=$(find /workspace/src -name "featureExtraction.cpp" -path "*arise_slam*" 2>/dev/null | head -1)
if [ -n "$FE_SRC" ]; then
    echo "Source: $FE_SRC"
    grep -n 'get_parameter\|declare_parameter\|config_\.\|nh_\.' "$FE_SRC" | grep -i 'param' | head -40
else
    echo "Source not found, checking utility.cpp..."
fi

echo ""
echo "===== 5. UTILITY.CPP - readParameters() ====="
UTIL_SRC=$(find /workspace/src -name "utility.cpp" -path "*arise_slam*" 2>/dev/null | head -1)
if [ -n "$UTIL_SRC" ]; then
    echo "Source: $UTIL_SRC"
    grep -n 'get_parameter\|declare_parameter\|param(' "$UTIL_SRC" | head -60
else
    echo "utility.cpp not found"
fi

echo ""
echo "===== 6. IMU PREINTEGRATION - expected params ====="
IMU_SRC=$(find /workspace/src -name "imu_preintegration*" -path "*arise_slam*" -name "*.cpp" 2>/dev/null | head -1)
if [ -n "$IMU_SRC" ]; then
    echo "Source: $IMU_SRC"
    grep -n 'get_parameter\|declare_parameter\|param(' "$IMU_SRC" | head -40
else
    echo "imu_preintegration source not found"
fi
