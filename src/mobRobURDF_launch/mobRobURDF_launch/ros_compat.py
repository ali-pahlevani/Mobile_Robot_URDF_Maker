"""ROS 2 distro compat helpers — detects Humble vs Jazzy+ at runtime."""

import os

# Jazzy switched ros2_controllers to TwistStamped; Humble/Iron stayed on unstamped Twist
_UNSTAMPED_DISTROS = {"humble", "iron"}


def ros_distro() -> str:
    return os.environ.get("ROS_DISTRO", "").lower()


def uses_stamped_twist(distro: str = None) -> bool:
    """True on Jazzy+, False on Humble/Iron, False if $ROS_DISTRO is unset."""
    d = distro if distro is not None else ros_distro()
    if not d:
        return False
    return d not in _UNSTAMPED_DISTROS
