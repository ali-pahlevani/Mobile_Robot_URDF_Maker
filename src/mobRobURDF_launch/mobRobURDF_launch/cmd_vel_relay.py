#!/usr/bin/env python3
"""Relay /cmd_vel → the active controller's velocity topic.

Different ros2_control controllers expose different topic names:
  diff_drive_controller   → <name>/cmd_vel_unstamped
  tricycle_controller     → <name>/cmd_vel
  steering controllers    → <name>/reference_unstamped   (use_stamped_vel: false)
  mecanum_drive_controller→ <name>/reference_unstamped

This node subscribes to /cmd_vel and republishes each message to whichever
topic the selected controller actually listens on, so users always drive via
a single /cmd_vel topic regardless of the controller type.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

_TOPIC_SUFFIX = {
    'diffDrive_controller': 'cmd_vel_unstamped',
    'tricycle_controller':  'cmd_vel',
    'triSteer_controller':  'reference_unstamped',
    'ackerSteer_controller': 'reference_unstamped',
    'mecDrive_controller':  'reference_unstamped',
}


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('controller_name', 'diffDrive_controller')
        name = self.get_parameter('controller_name').get_parameter_value().string_value
        suffix = _TOPIC_SUFFIX.get(name, 'cmd_vel_unstamped')
        target = f'/{name}/{suffix}'
        self.get_logger().info(f'cmd_vel_relay: /cmd_vel → {target}')
        self._pub = self.create_publisher(Twist, target, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)

    def _cb(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()
