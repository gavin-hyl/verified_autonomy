#!/usr/bin/env python3
"""
Bridge node: subscribes to /cmd_vel (TwistStamped) and publishes /high_cmd (HighCmd).

Converts the autonomy stack velocity commands into Unitree Go1 high-level commands.
The robot must already be standing (send stand_up first).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from ros2_unitree_legged_msgs.msg import HighCmd


class CmdVelToHighCmd(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_high_cmd')

        self.publisher = self.create_publisher(HighCmd, 'high_cmd', 10)
        self.subscription = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publish at 50Hz even when no cmd_vel arrives (keeps robot alive)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.latest_msg = HighCmd()
        self._init_cmd(self.latest_msg)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info('cmd_vel -> high_cmd bridge started')

    def _init_cmd(self, msg):
        msg.head[0] = 0xFE
        msg.head[1] = 0xEF
        msg.level_flag = 0xEE  # HIGHLEVEL
        msg.mode = 2           # WALK
        msg.gait_type = 1      # TROT
        msg.foot_raise_height = 0.08
        msg.body_height = 0.0
        msg.velocity = [0.0, 0.0]
        msg.yaw_speed = 0.0

    def cmd_vel_callback(self, msg: TwistStamped):
        cmd = HighCmd()
        self._init_cmd(cmd)

        # Map TwistStamped velocities to Go1 normalized velocities
        # Go1 velocity range is roughly -1 to 1 (normalized)
        # The autonomy stack outputs m/s, Go1 max ~1 m/s forward
        cmd.velocity[0] = max(-1.0, min(1.0, msg.twist.linear.x))
        cmd.velocity[1] = max(-1.0, min(1.0, msg.twist.linear.y))
        cmd.yaw_speed = max(-1.0, min(1.0, msg.twist.angular.z))

        # If velocities are near zero, switch to idle to avoid drift
        if (abs(cmd.velocity[0]) < 0.01 and
                abs(cmd.velocity[1]) < 0.01 and
                abs(cmd.yaw_speed) < 0.01):
            cmd.mode = 0  # IDLE
        else:
            cmd.mode = 2  # WALK

        self.latest_msg = cmd
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        # If no cmd_vel for >0.5s, send idle
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            idle_cmd = HighCmd()
            self._init_cmd(idle_cmd)
            idle_cmd.mode = 0  # IDLE
            self.publisher.publish(idle_cmd)
        else:
            self.publisher.publish(self.latest_msg)


def main():
    rclpy.init()
    node = CmdVelToHighCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send idle on shutdown
        idle_cmd = HighCmd()
        idle_cmd.head[0] = 0xFE
        idle_cmd.head[1] = 0xEF
        idle_cmd.level_flag = 0xEE
        idle_cmd.mode = 0
        node.publisher.publish(idle_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
