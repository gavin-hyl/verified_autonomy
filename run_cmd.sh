#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py idle 2>&1
