#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
for t in /state_estimation /registered_scan /terrain_map /way_point /path /cmd_vel; do
  echo -n "$t: "
  timeout 4 ros2 topic hz $t --window 3 2>&1 | tail -1
done
