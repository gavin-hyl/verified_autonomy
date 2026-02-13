#!/bin/bash
# Copy key SLAM source files to /tmp for inspection
CID=$(docker ps -q | head -n1)
for f in \
  /workspace/src/slam/arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp \
  /workspace/src/slam/arise_slam_mid360/src/featureExtraction_node.cpp \
  /workspace/src/slam/arise_slam_mid360/include/arise_slam_mid360/utils/utility.cpp; do
  docker cp "$CID:$f" /tmp/$(basename "$f") 2>/dev/null && echo "OK: $f" || echo "FAIL: $f"
done
echo "---"
# Also check the launch file to see how the driver is configured
docker exec $CID find /workspace -name "system_real_robot*" -type f 2>/dev/null
