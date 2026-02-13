#!/usr/bin/env python3
"""
Go1 Single Command Sender

Send a single high-level command to the Go1 robot.

Usage:
    python3 go1_cmd.py stand_up
    python3 go1_cmd.py stand_down
    python3 go1_cmd.py walk 0.3 0 0     # vx, vy, yaw_speed
    python3 go1_cmd.py stop
    python3 go1_cmd.py idle

The command is sent continuously for 2 seconds (or until Ctrl+C).
Make sure ros2_udp HIGHLEVEL is running first!
"""

import rclpy
from rclpy.node import Node
from ros2_unitree_legged_msgs.msg import HighCmd
import sys
import time


# Mode definitions
MODES = {
    'idle': 0,
    'force_stand': 1,
    'walk': 2,
    'stand_down': 5,
    'stand_up': 6,
    'damping': 7,
    'recovery': 8,
}


class Go1CommandSender(Node):
    def __init__(self, mode, vx=0.0, vy=0.0, yaw=0.0, duration=2.0):
        super().__init__('go1_cmd_sender')

        self.publisher = self.create_publisher(HighCmd, 'high_cmd', 10)

        self.mode = mode
        self.vx = vx
        self.vy = vy
        self.yaw = yaw
        self.duration = duration
        self.start_time = time.time()

        # Publish at 50Hz
        self.timer = self.create_timer(0.02, self.publish_cmd)

        self.get_logger().info(f'Sending command: mode={mode}, vx={vx}, vy={vy}, yaw={yaw}')

    def publish_cmd(self):
        msg = HighCmd()
        msg.head[0] = 0xFE
        msg.head[1] = 0xEF
        msg.level_flag = 0xEE

        msg.mode = self.mode
        msg.gait_type = 1  # Trot
        msg.velocity[0] = self.vx
        msg.velocity[1] = self.vy
        msg.yaw_speed = self.yaw
        msg.foot_raise_height = 0.08

        self.publisher.publish(msg)

        # Check if duration elapsed
        if time.time() - self.start_time > self.duration:
            self.get_logger().info('Command sent successfully')
            raise SystemExit


def print_usage():
    print(__doc__)
    print("Available commands:")
    for cmd in MODES.keys():
        print(f"  {cmd}")
    print("\nExamples:")
    print("  python3 go1_cmd.py stand_up")
    print("  python3 go1_cmd.py walk 0.3 0 0")
    print("  python3 go1_cmd.py walk 0 0 0.5  # Turn in place")


def main():
    if len(sys.argv) < 2:
        print_usage()
        sys.exit(1)

    command = sys.argv[1].lower()

    if command == 'help' or command == '-h' or command == '--help':
        print_usage()
        sys.exit(0)

    if command == 'stop':
        command = 'idle'

    if command not in MODES:
        print(f"Unknown command: {command}")
        print_usage()
        sys.exit(1)

    mode = MODES[command]
    vx, vy, yaw = 0.0, 0.0, 0.0

    # Parse velocity arguments for walk command
    if command == 'walk' and len(sys.argv) >= 3:
        try:
            vx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
            vy = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        except ValueError:
            print("Error: velocity values must be numbers")
            sys.exit(1)

    # Determine duration based on command
    if command in ['stand_up', 'stand_down']:
        duration = 3.0  # These take longer
    elif command == 'walk':
        duration = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
    else:
        duration = 1.0

    rclpy.init()
    node = Go1CommandSender(mode, vx, vy, yaw, duration)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
