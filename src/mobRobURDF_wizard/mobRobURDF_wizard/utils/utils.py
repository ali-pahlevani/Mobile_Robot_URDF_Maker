import subprocess
from string import Template
import logging

def render_template(template_path, params):
    """Render a template file with given parameters."""
    try:
        with open(template_path, 'r') as f:
            tmpl = Template(f.read())
        rendered = tmpl.safe_substitute(params)
        #logging.debug(f"Rendered template: {template_path}")
        return rendered
    except FileNotFoundError:
        logging.error(f"Template file not found: {template_path}")
        return f"Error: Template file not found: {template_path}"
    except Exception as e:
        logging.error(f"Error rendering template {template_path}: {str(e)}")
        return f"Error rendering template: {str(e)}"

def generate_urdf(xacro_file):
    """Generate a URDF from a Xacro file using the xacro command."""
    try:
        result = subprocess.run(['xacro', xacro_file],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                check=True,
                                text=True)
        #logging.debug(f"Generated URDF from: {xacro_file}")
        return result.stdout
    except subprocess.CalledProcessError as e:
        error_msg = f"Error generating URDF from {xacro_file}:\n{e.stderr}"
        logging.error(error_msg)
        return error_msg
    except FileNotFoundError:
        error_msg = "Error: 'xacro' command not found. Ensure xacro is installed."
        logging.error(error_msg)
        return error_msg
    except Exception as e:
        error_msg = f"Unexpected error generating URDF from {xacro_file}: {str(e)}"
        logging.error(error_msg)
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
    #logging.debug(f"Color mapped: {color_name} -> {color}")
    return color