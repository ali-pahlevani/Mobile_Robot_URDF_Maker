import os
import subprocess
from string import Template
import logging

logger = logging.getLogger(__name__)

# Distributions whose ros2_controllers still use the unstamped Twist interface
# (cmd_vel_unstamped / reference_unstamped). Jazzy and newer use TwistStamped.
_UNSTAMPED_DISTROS = {"humble", "iron"}


def ros_distro():
    """Return the active ROS 2 distribution name (lower-case), or ''."""
    return os.environ.get("ROS_DISTRO", "").lower()


def uses_stamped_twist(distro=None):
    """True if the active distro's controllers expect TwistStamped (Jazzy+)."""
    d = distro if distro is not None else ros_distro()
    if not d:
        return False
    return d not in _UNSTAMPED_DISTROS

def render_template(template_path, params):
    """safe_substitute so unknown $vars in templates don't blow up."""
    try:
        with open(template_path, 'r') as f:
            tmpl = Template(f.read())
        rendered = tmpl.safe_substitute(params)
        logger.debug("Rendered template: %s", template_path)
        return rendered
    except FileNotFoundError:
        logger.error("Template file not found: %s", template_path)
        return f"Error: Template file not found: {template_path}"
    except Exception as e:
        logger.error("Error rendering template %s: %s", template_path, str(e))
        return f"Error rendering template: {str(e)}"

def generate_urdf(xacro_file):
    """Run xacro on xacro_file and return the resulting URDF string."""
    try:
        result = subprocess.run(['xacro', xacro_file],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                check=True,
                                text=True)
        logger.debug("Generated URDF from: %s", xacro_file)
        return result.stdout
    except subprocess.CalledProcessError as e:
        error_msg = f"Error generating URDF from {xacro_file}:\n{e.stderr}"
        logger.error(error_msg)
        return error_msg
    except FileNotFoundError:
        error_msg = "Error: 'xacro' command not found. Ensure xacro is installed."
        logger.error(error_msg)
        return error_msg
    except Exception as e:
        error_msg = f"Unexpected error generating URDF from {xacro_file}: {str(e)}"
        logger.error(error_msg)
        return error_msg

def get_color(color_name):
    """Color name → (R, G, B) float tuple for OpenGL."""
    colors = {
        "Gray": (0.5, 0.5, 0.5),
        "Black": (0.0, 0.0, 0.0),
        "Red": (1.0, 0.0, 0.0),
        "Blue": (0.0, 0.0, 1.0),
        "Green": (0.0, 1.0, 0.0),
        "White": (1.0, 1.0, 1.0),
    }
    color = colors.get(color_name.capitalize(), (0.5, 0.5, 0.5))
    logger.debug("Color mapped: %s -> %s", color_name, color)
    return color