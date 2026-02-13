#!/bin/bash
echo "===== removeClosestFarestPointCloud function ====="
grep -rn 'removeClosestFarest\|removeClosedFarest\|removeClosest' /workspace/src/slam/arise_slam_mid360/src/ --include="*.cpp" --include="*.h" -A 15 | head -40

echo ""
echo "===== livoxHandler: point filtering tag check ====="
echo "How many points pass the filter? Check tag values from live data..."
echo ""

source /workspace/install/setup.bash
echo "--- CustomMsg point_num and point samples ---"
timeout 3 ros2 topic echo /lidar/scan --no-arr --once 2>&1 | grep -E "point_num|lidar_id"

echo ""
echo "--- Check tag/line distribution (Python) ---"
timeout 8 python3 -c "
import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
import sys

rclpy.init()
node = Node('tag_checker')
msg_received = [False]

def cb(msg):
    if msg_received[0]:
        return
    msg_received[0] = True
    total = msg.point_num
    lines = {}
    tags = {}
    valid = 0
    zero_xyz = 0
    for p in msg.points:
        lines[p.line] = lines.get(p.line, 0) + 1
        tag_upper = p.tag & 0x30
        tags[tag_upper] = tags.get(tag_upper, 0) + 1
        if (p.line < 4) and (tag_upper == 0x10 or tag_upper == 0x00):
            valid += 1
        if abs(p.x) < 1e-7 and abs(p.y) < 1e-7 and abs(p.z) < 1e-7:
            zero_xyz += 1
    print(f'Total points: {total}')
    print(f'Line distribution: {dict(sorted(lines.items()))}')
    print(f'Tag(0x30) distribution: {dict(sorted(tags.items()))}')
    print(f'Valid (line<4 && tag OK): {valid} ({100*valid/max(total,1):.1f}%)')
    print(f'Zero XYZ: {zero_xyz}')
    print(f'Non-zero: {total - zero_xyz}')
    rclpy.shutdown()

sub = node.create_subscription(CustomMsg, '/lidar/scan', cb, 1)
rclpy.spin(node)
" 2>&1
