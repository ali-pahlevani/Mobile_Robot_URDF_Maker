#!/usr/bin/env python3
"""Relay /cmd_vel → the active controller's velocity topic.

Different ros2_control controllers expose different topic names *and* message
types, and the convention changed across ROS 2 distributions:

  Humble / Iron (unstamped Twist):
    diff_drive_controller   → <name>/cmd_vel_unstamped   (Twist)
    tricycle_controller     → <name>/cmd_vel             (Twist)
    steering controllers    → <name>/reference_unstamped (Twist)
    mecanum_drive_controller→ <name>/reference_unstamped (Twist)

  Jazzy and newer (TwistStamped):
    diff_drive / tricycle   → <name>/cmd_vel             (TwistStamped)
    steering / mecanum      → <name>/reference           (TwistStamped)

This node subscribes to /cmd_vel (always plain Twist, as published by teleop
tools) and republishes each message to whichever topic/type the selected
controller actually listens on, so users always drive via a single /cmd_vel
topic regardless of controller type or ROS 2 distribution.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from mobRobURDF_launch.ros_compat import uses_stamped_twist

# Topic suffix per controller for the TwistStamped (Jazzy+) convention.
_SUFFIX_STAMPED = {
    'diffDrive_controller':  'cmd_vel',
    'tricycle_controller':   'cmd_vel',
    'triSteer_controller':   'reference',
    'ackerSteer_controller': 'reference',
    'mecDrive_controller':   'reference',
}

# Topic suffix per controller for the unstamped Twist (Humble/Iron) convention.
_SUFFIX_UNSTAMPED = {
    'diffDrive_controller':  'cmd_vel_unstamped',
    'tricycle_controller':   'cmd_vel',
    'triSteer_controller':   'reference_unstamped',
    'ackerSteer_controller': 'reference_unstamped',
    'mecDrive_controller':   'reference_unstamped',
}


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('controller_name', 'diffDrive_controller')
        name = self.get_parameter('controller_name').get_parameter_value().string_value

        self._stamped = uses_stamped_twist()
        if self._stamped:
            suffix = _SUFFIX_STAMPED.get(name, 'cmd_vel')
            msg_type = TwistStamped
        else:
            suffix = _SUFFIX_UNSTAMPED.get(name, 'cmd_vel_unstamped')
            msg_type = Twist

        target = f'/{name}/{suffix}'
        self.get_logger().info(
            f'cmd_vel_relay: /cmd_vel → {target} '
            f'({"TwistStamped" if self._stamped else "Twist"})'
        )
        self._pub = self.create_publisher(msg_type, target, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)

    def _cb(self, msg: Twist) -> None:
        if self._stamped:
            stamped = TwistStamped()
            # Stamp with the current (sim) clock — controllers compare this
            # against cmd_vel_timeout, so a zero stamp would look stale.
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.twist = msg
            self._pub.publish(stamped)
        else:
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()
