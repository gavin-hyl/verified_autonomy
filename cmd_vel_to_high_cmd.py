#!/usr/bin/env python3
"""
cmd_vel → high_cmd Bridge
==========================
Translates geometry_msgs/TwistStamped from the autonomy stack's /cmd_vel
into ros2_unitree_legged_msgs/HighCmd on /high_cmd for the Go1 highlevel bridge.

Run in WSL (where the unitree messages are built):
    source /opt/ros/jazzy/setup.bash
    source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash
    python3 ~/verified_autonomy/cmd_vel_to_high_cmd.py

Requires:
  - ros2_udp HIGHLEVEL running (the highlevel bridge)
  - Exploration stack publishing /cmd_vel (from Docker)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from ros2_unitree_legged_msgs.msg import HighCmd
import time


class CmdVelToHighCmd(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_high_cmd')

        # Subscribe to /cmd_vel (TwistStamped from exploration stack)
        self.sub_stamped = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_stamped_cb, 10)

        # Also subscribe to unstamped /cmd_vel_unstamped just in case
        self.sub_twist = self.create_subscription(
            Twist, '/cmd_vel_unstamped', self.cmd_vel_twist_cb, 10)

        # Publish to /high_cmd (HighCmd for Go1 bridge)
        self.pub = self.create_publisher(HighCmd, '/high_cmd', 10)

        # Safety: if no cmd_vel received for 0.5s, send stop
        self.last_cmd_time = time.time()
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        # Max velocity clamps (m/s and rad/s)
        self.max_vx = 0.4
        self.max_vy = 0.3
        self.max_yaw = 0.8

        self.get_logger().info('cmd_vel → high_cmd bridge started')
        self.get_logger().info(f'  Limits: vx={self.max_vx}, vy={self.max_vy}, yaw={self.max_yaw}')

    def cmd_vel_stamped_cb(self, msg: TwistStamped):
        self.send_high_cmd(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def cmd_vel_twist_cb(self, msg: Twist):
        self.send_high_cmd(msg.linear.x, msg.linear.y, msg.angular.z)

    def send_high_cmd(self, vx: float, vy: float, yaw_speed: float):
        self.last_cmd_time = time.time()

        # Clamp velocities
        vx = max(-self.max_vx, min(self.max_vx, vx))
        vy = max(-self.max_vy, min(self.max_vy, vy))
        yaw_speed = max(-self.max_yaw, min(self.max_yaw, yaw_speed))

        msg = HighCmd()
        msg.head[0] = 0xFE
        msg.head[1] = 0xEF
        msg.level_flag = 0xEE  # HIGHLEVEL

        # If velocities are near zero, use force_stand (mode 1)
        # Otherwise use walk (mode 2)
        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(yaw_speed) < 0.01:
            msg.mode = 1  # force_stand
        else:
            msg.mode = 2  # walk
            msg.gait_type = 1  # trot

        msg.velocity[0] = float(vx)
        msg.velocity[1] = float(vy)
        msg.yaw_speed = float(yaw_speed)
        msg.foot_raise_height = 0.08

        self.pub.publish(msg)

    def safety_check(self):
        """Stop robot if no commands received for 0.5 seconds."""
        if time.time() - self.last_cmd_time > 0.5:
            msg = HighCmd()
            msg.head[0] = 0xFE
            msg.head[1] = 0xEF
            msg.level_flag = 0xEE
            msg.mode = 1  # force_stand
            msg.velocity[0] = 0.0
            msg.velocity[1] = 0.0
            msg.yaw_speed = 0.0
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelToHighCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send stop on exit
        node.send_high_cmd(0, 0, 0)
        time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
