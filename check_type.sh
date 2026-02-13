#!/bin/bash
source /workspace/install/setup.bash
ros2 topic info /lidar/scan
echo "---"
ros2 topic info /imu/data
echo "---"
cat /tmp/livox_launch.log | tail -20
