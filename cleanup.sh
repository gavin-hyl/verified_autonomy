#!/bin/bash
# Kill any leftover ROS/livox processes
echo "Killing leftover processes..."
pkill -9 -f livox_ros_driver2 2>/dev/null
pkill -9 -f feature_extraction 2>/dev/null
pkill -9 -f laser_mapping 2>/dev/null
pkill -9 -f imu_preintegration 2>/dev/null
pkill -9 -f local_planner 2>/dev/null
pkill -9 -f tare_planner 2>/dev/null
pkill -9 -f terrain_analysis 2>/dev/null
pkill -9 -f ros2 2>/dev/null
sleep 3
echo "Remaining ROS processes:"
ps aux | grep -E "ros2|livox|slam|planner|terrain" | grep -v grep || echo "  (none)"
echo ""
echo "Checking ports 56100-56500..."
ss -ulnp | grep -E "561|562|563|564|565" || echo "  (all free)"
echo "Done. Ready to launch."
