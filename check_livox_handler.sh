#!/bin/bash
echo "===== livoxHandler full function (lines 1531-1640) ====="
sed -n '1531,1640p' /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp

echo ""
echo "===== uniformfeatureExtraction (what's asserting plannerPoints>0) ====="
grep -rn 'uniformfeatureExtraction' /workspace/src/slam/arise_slam_mid360/src/ --include="*.cpp" --include="*.h" | head -10
