import subprocess
from string import Template
import logging

logger = logging.getLogger(__name__)

def render_template(template_path, params):
    """Render a template file with given parameters."""
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
    """Generate a URDF from a Xacro file using the xacro command."""
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
    """Map a color name to an RGB tuple."""
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