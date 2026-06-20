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

# All supported material names — must stay in sync with material_macros.xacro
MATERIAL_NAMES = [
    "Gray", "Silver", "Dark Gray", "Black", "White",
    "Red", "Maroon", "Orange", "Yellow",
    "Green", "Lime", "Teal", "Cyan",
    "Blue", "Navy", "Purple",
    "Pink", "Brown",
]

_COLOR_MAP = {
    "Gray":      (0.50, 0.50, 0.50),
    "Silver":    (0.75, 0.75, 0.75),
    "Dark Gray": (0.25, 0.25, 0.25),
    "Black":     (0.00, 0.00, 0.00),
    "White":     (1.00, 1.00, 1.00),
    "Red":       (1.00, 0.00, 0.00),
    "Maroon":    (0.50, 0.00, 0.00),
    "Orange":    (1.00, 0.50, 0.00),
    "Yellow":    (1.00, 1.00, 0.00),
    "Green":     (0.00, 0.80, 0.00),
    "Lime":      (0.50, 1.00, 0.00),
    "Teal":      (0.00, 0.50, 0.50),
    "Cyan":      (0.00, 1.00, 1.00),
    "Blue":      (0.00, 0.00, 1.00),
    "Navy":      (0.00, 0.00, 0.50),
    "Purple":    (0.50, 0.00, 0.50),
    "Pink":      (1.00, 0.40, 0.70),
    "Brown":     (0.60, 0.30, 0.10),
}


def get_color(color_name):
    """Color name → (R, G, B) float tuple for OpenGL."""
    color = _COLOR_MAP.get(color_name, _COLOR_MAP.get(color_name.capitalize(), (0.5, 0.5, 0.5)))
    logger.debug("Color mapped: %s -> %s", color_name, color)
    return color