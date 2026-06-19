"""ROS 2 distribution compatibility helpers.

The mobile-base controllers in ``ros2_controllers`` changed their command
interface from unstamped ``geometry_msgs/msg/Twist`` (on ``*_unstamped``
topics) to ``geometry_msgs/msg/TwistStamped`` in **Jazzy**.  These helpers let
a single codebase target both conventions by inspecting the active ROS 2
distribution at runtime (``$ROS_DISTRO``), so no manual selection is needed.
"""

import os

# Distributions whose ros2_controllers still expose the unstamped Twist
# interface (``cmd_vel_unstamped`` / ``reference_unstamped``).  Everything
# newer (Jazzy, Kilted, Rolling, …) uses TwistStamped.
_UNSTAMPED_DISTROS = {"humble", "iron"}


def ros_distro() -> str:
    """Return the active ROS 2 distribution name (lower-case), or ''."""
    return os.environ.get("ROS_DISTRO", "").lower()


def uses_stamped_twist(distro: str = None) -> bool:
    """True if the active distro's controllers expect ``TwistStamped``.

    Jazzy and newer return True; Humble/Iron return False.  An unknown/empty
    ``$ROS_DISTRO`` falls back to the legacy unstamped convention.
    """
    d = distro if distro is not None else ros_distro()
    if not d:
        return False
    return d not in _UNSTAMPED_DISTROS
