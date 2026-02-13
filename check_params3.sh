#!/bin/bash
echo "===== parameter.cpp (full) ====="
cat /workspace/src/slam/arise_slam_mid360/src/parameter/parameter.cpp

echo ""
echo "===== imuPreintegration.cpp: parameter loading ====="
grep -n 'declare_parameter\|get_parameter\|param(' /workspace/src/slam/arise_slam_mid360/src/ImuPreintegration/imuPreintegration.cpp | head -80

echo ""
echo "===== laser_mapping source: parameter loading ====="
LM_SRC=$(find /workspace/src/slam -name "laserMapping.cpp" -o -name "laser_mapping.cpp" 2>/dev/null | head -1)
echo "Found: $LM_SRC"
if [ -n "$LM_SRC" ]; then
    grep -n 'declare_parameter\|get_parameter' "$LM_SRC" | head -80
fi

echo ""
echo "===== ALL cpp files with declare_parameter in arise_slam ====="
find /workspace/src/slam/arise_slam_mid360/src -name "*.cpp" | xargs grep -l 'declare_parameter' 2>/dev/null

echo ""
echo "===== parameter.h ====="
find /workspace/src/slam/arise_slam_mid360 -name "parameter.h" | head -1 | xargs cat 2>/dev/null | head -80
