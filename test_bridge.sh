#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash
export LD_LIBRARY_PATH=~/verified_autonomy/ros2_unitree_ws/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
timeout 5 ros2 run unitree_legged_real ros2_udp HIGHLEVEL 2>&1
echo "Exit code: $?"
