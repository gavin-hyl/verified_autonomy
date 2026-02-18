#!/bin/bash
source /workspace/install/setup.bash
echo "Starting livox driver with CustomMsg format..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
DRIVER_PID=$!
sleep 10
echo "--- Checking topic type ---"
ros2 topic info /lidar/scan 2>&1
echo "--- Checking imu ---"  
ros2 topic info /imu/data 2>&1
echo "--- Checking hz ---"
timeout 5 ros2 topic hz /lidar/scan 2>&1
wait $DRIVER_PID
