#!/usr/bin/env python3
"""Bridges a single /cmd_vel (plain Twist from teleop) to the controller's actual topic.

Humble/Iron controllers use unstamped Twist on *_unstamped topics;
Jazzy+ use TwistStamped on cmd_vel/reference.  This node hides that
complexity so the rest of the stack always talks to /cmd_vel.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from mobRobURDF_launch.ros_compat import uses_stamped_twist

# Jazzy+ topic suffixes (TwistStamped)
_SUFFIX_STAMPED = {
    'diffDrive_controller':  'cmd_vel',
    'tricycle_controller':   'cmd_vel',
    'triSteer_controller':   'reference',
    'ackerSteer_controller': 'reference',
    'mecDrive_controller':   'reference',
}

# Humble/Iron topic suffixes (unstamped Twist)
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
            # zero stamp looks stale to the controller's cmd_vel_timeout check
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.twist = msg
            self._pub.publish(stamped)
        else:
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()
