import subprocess
from string import Template

def render_template(template_path, params):
    with open(template_path, 'r') as f:
        tmpl = Template(f.read())
    return tmpl.safe_substitute(params)

def generate_urdf(xacro_file):
    try:
        result = subprocess.run(['xacro', xacro_file],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                check=True,
                                text=True)
        return result.stdout
    except subprocess.CalledProcessError as e:
        return f"Error generating URDF:\n{e.stderr}"

def get_color(color_name):
    colors = {
        "Gray": (0.5, 0.5, 0.5),
        "Black": (0.0, 0.0, 0.0),
        "Red": (1.0, 0.0, 0.0),
        "Blue": (0.0, 0.0, 1.0),
        "Green": (0.0, 1.0, 0.0),
        "White": (1.0, 1.0, 1.0),
    }
    return colors.get(color_name.capitalize(), (0.5, 0.5, 0.5))
