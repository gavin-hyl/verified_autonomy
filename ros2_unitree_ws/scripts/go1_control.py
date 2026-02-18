#!/usr/bin/env python3
"""
Go1 High-Level Control Interface

This script provides a simple interface to control the Unitree Go1 robot
using ROS 2 high-level commands.

Usage:
    ros2 run unitree_legged_real ros2_udp HIGHLEVEL  # In terminal 1
    python3 go1_control.py                            # In terminal 2

Commands are sent to /high_cmd topic. The ros2_udp bridge must be running.
"""

import rclpy
from rclpy.node import Node
from ros2_unitree_legged_msgs.msg import HighCmd
import sys
import termios
import tty
import threading
import time


class Go1Controller(Node):
    """High-level controller for Go1 robot."""

    # Mode definitions (from Unitree SDK)
    MODE_IDLE = 0           # Idle, default stand
    MODE_FORCE_STAND = 1    # Force stand (can adjust body pose)
    MODE_WALK = 2           # Walking mode
    MODE_RUN = 3            # Running mode (not available on Go1)
    MODE_CLIMB = 4          # Stair climbing mode
    MODE_STAND_DOWN = 5     # Stand down (lie down)
    MODE_STAND_UP = 6       # Stand up
    MODE_DAMPING = 7        # Damping mode (motors off)
    MODE_RECOVERY = 8       # Recovery stand

    # Gait types for walking
    GAIT_IDLE = 0
    GAIT_TROT = 1
    GAIT_TROT_RUNNING = 2
    GAIT_CLIMB_STAIR = 3
    GAIT_TROT_OBSTACLE = 4

    def __init__(self):
        super().__init__('go1_controller')

        self.publisher = self.create_publisher(HighCmd, 'high_cmd', 10)
        self.timer = self.create_timer(0.02, self.publish_cmd)  # 50Hz

        # Command state
        self.mode = self.MODE_IDLE
        self.gait_type = self.GAIT_TROT
        self.velocity_x = 0.0  # Forward/backward (-1 to 1)
        self.velocity_y = 0.0  # Left/right (-1 to 1)
        self.yaw_speed = 0.0   # Rotation speed
        self.body_height = 0.0  # Body height adjustment
        self.foot_raise_height = 0.08

        # Euler angles for force stand mode
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.get_logger().info('Go1 Controller initialized')
        self.get_logger().info('Make sure ros2_udp HIGHLEVEL is running!')

    def publish_cmd(self):
        """Publish the current command at 50Hz."""
        msg = HighCmd()

        # Header
        msg.head[0] = 0xFE
        msg.head[1] = 0xEF
        msg.level_flag = 0xEE  # HIGHLEVEL

        # Mode and gait
        msg.mode = self.mode
        msg.gait_type = self.gait_type

        # Velocity (for walk mode)
        msg.velocity[0] = self.velocity_x
        msg.velocity[1] = self.velocity_y
        msg.yaw_speed = self.yaw_speed

        # Body pose (for force stand mode)
        msg.body_height = self.body_height
        msg.foot_raise_height = self.foot_raise_height
        msg.euler[0] = self.roll
        msg.euler[1] = self.pitch
        msg.euler[2] = self.yaw

        self.publisher.publish(msg)

    def stand_up(self):
        """Make the robot stand up."""
        self.mode = self.MODE_STAND_UP
        self.get_logger().info('Standing up...')

    def stand_down(self):
        """Make the robot lie down."""
        self.mode = self.MODE_STAND_DOWN
        self.get_logger().info('Standing down...')

    def idle(self):
        """Enter idle mode (default stand)."""
        self.mode = self.MODE_IDLE
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.yaw_speed = 0.0
        self.get_logger().info('Idle mode')

    def force_stand(self):
        """Enter force stand mode (can adjust body pose)."""
        self.mode = self.MODE_FORCE_STAND
        self.get_logger().info('Force stand mode - can adjust body pose')

    def walk(self, vx=0.0, vy=0.0, yaw_speed=0.0):
        """
        Walk with specified velocities.

        Args:
            vx: Forward velocity (-1 to 1), positive = forward
            vy: Lateral velocity (-1 to 1), positive = left
            yaw_speed: Rotation speed, positive = counter-clockwise
        """
        self.mode = self.MODE_WALK
        self.velocity_x = max(-1.0, min(1.0, vx))
        self.velocity_y = max(-1.0, min(1.0, vy))
        self.yaw_speed = yaw_speed
        self.get_logger().info(f'Walking: vx={vx:.2f}, vy={vy:.2f}, yaw={yaw_speed:.2f}')

    def stop(self):
        """Stop all movement."""
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.yaw_speed = 0.0
        self.mode = self.MODE_IDLE
        self.get_logger().info('Stopped')

    def damping(self):
        """Enter damping mode (motors off - robot will collapse!)."""
        self.mode = self.MODE_DAMPING
        self.get_logger().warn('DAMPING MODE - Motors off!')


def get_key():
    """Get a single keypress."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_help():
    """Print keyboard controls."""
    print("\n" + "=" * 50)
    print("Go1 Keyboard Control")
    print("=" * 50)
    print("Movement (in walk mode):")
    print("  w/s  - Forward/Backward")
    print("  a/d  - Turn left/right")
    print("  q/e  - Strafe left/right")
    print("")
    print("Modes:")
    print("  u - Stand Up")
    print("  j - Stand Down (lie down)")
    print("  i - Idle mode")
    print("  f - Force stand (body pose control)")
    print("  SPACE - Stop all movement")
    print("")
    print("Body pose (in force stand mode):")
    print("  r/t - Roll left/right")
    print("  y/h - Pitch up/down")
    print("  g/j - Yaw left/right (not j when standing)")
    print("")
    print("Other:")
    print("  x - EMERGENCY STOP (damping mode)")
    print("  ESC or Ctrl+C - Quit")
    print("=" * 50 + "\n")


def main():
    rclpy.init()
    controller = Go1Controller()

    # Start ROS spinner in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    print_help()
    print("Ready! Press keys to control the robot...")
    print("(Make sure ros2_udp HIGHLEVEL is running in another terminal)")

    speed = 0.3  # Walking speed
    turn_speed = 0.5  # Turning speed

    try:
        while True:
            key = get_key()

            if key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
                print("\nExiting...")
                break
            elif key == 'w':
                controller.walk(vx=speed)
            elif key == 's':
                controller.walk(vx=-speed)
            elif key == 'a':
                controller.walk(yaw_speed=turn_speed)
            elif key == 'd':
                controller.walk(yaw_speed=-turn_speed)
            elif key == 'q':
                controller.walk(vy=speed)
            elif key == 'e':
                controller.walk(vy=-speed)
            elif key == ' ':
                controller.stop()
            elif key == 'u':
                controller.stand_up()
            elif key == 'j':
                controller.stand_down()
            elif key == 'i':
                controller.idle()
            elif key == 'f':
                controller.force_stand()
            elif key == 'x':
                print("\n!!! EMERGENCY STOP - DAMPING MODE !!!")
                controller.damping()
            elif key == 'h':
                print_help()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
