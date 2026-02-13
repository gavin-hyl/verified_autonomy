#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
for t in /lidar/scan /imu/data /feature_info /laser_odometry /state_estimation /registered_scan; do
  echo -n "$t: "
  timeout 5 ros2 topic hz $t --window 3 2>&1 | tail -1
done
