import os
import tempfile
import random
import string
import logging
import shutil
from mobRobURDF_wizard.utils.utils import render_template, generate_urdf
from ament_index_python.packages import get_package_share_directory
from ruamel.yaml import YAML

logger = logging.getLogger(__name__)

# Name of the file (written next to the generated URDF) that stores the
# controller spawner name. Launch files read it instead of being rewritten.
SELECTED_CONTROLLER_FILE = "selected_controller.txt"
DEFAULT_CONTROLLER_NAME = "diffDrive_controller"

# (robot_type, controller_type) -> file/suffix used by the xacro templates.
CONTROLLER_SUFFIX_MAP = {
    ("2_wheeled_caster", "diff_2wc"): "2wc_diff",
    ("3_wheeled", "tricycle"): "3w_tricycle",
    ("3_wheeled", "triSteer"): "3w_triSteer",
    ("4_wheeled", "diff_4w"): "4w_diff",
    ("4_wheeled", "mecanum"): "4w_mec",
    ("4_wheeled", "ackermann"): "4w_acker",
}


class URDFManager:
    def __init__(self):
        # Package share directory (ROS convention). Under `--symlink-install`
        # this points back at the source tree, which is how generated files
        # become visible to the launch files.
        self.base_dir = os.path.join(get_package_share_directory("mobRobURDF_description"), "urdf")
        # Unique temporary directory for intermediate files.
        self.source_dir = os.path.join(
            tempfile.gettempdir(),
            f"mobRobURDF_temp_{''.join(random.choices(string.ascii_lowercase, k=8))}",
        )
        self.urdf_text = ""
        self.control_config_dir = os.path.join(get_package_share_directory("mobRobURDF_control"), "config")
        self.last_params = {}  # Store last used parameters
        self.last_robot_type = None
        self.last_controller_type = None
        os.makedirs(self.source_dir, exist_ok=True)
        logger.debug("URDFManager initialized (base_dir=%s, source_dir=%s)", self.base_dir, self.source_dir)

    def _controller_suffix(self, robot_type, controller_type):
        return CONTROLLER_SUFFIX_MAP.get((robot_type, controller_type), "4w_diff")

    def generate_urdf(self, robot_type, controller_type, params):
        try:
            self.last_params = params.copy()  # Store parameters for save_urdf
            self.last_robot_type = robot_type
            self.last_controller_type = controller_type

            submodules_dir = os.path.join(self.base_dir, "submodules", robot_type)
            if robot_type == "4_wheeled" and controller_type == "ackermann":
                submodules_dir = os.path.join(submodules_dir, "ackermann")

            controller_suffix = self._controller_suffix(robot_type, controller_type)
            mobrob_file = os.path.join(submodules_dir, f"mobRob_{controller_suffix}.xacro")

            if not os.path.exists(mobrob_file):
                self.urdf_text = (
                    f"Error: mobRob_{controller_suffix}.xacro not found for "
                    f"{robot_type} with {controller_type} in {submodules_dir}"
                )
                logger.error(self.urdf_text)
                return self.urdf_text

            # The top-level mobRob_*.xacro is the only file containing wizard
            # placeholders (e.g. ${wheel_radius}); every other template is pulled
            # in via $(find ...) includes and only contains generic xacro macros,
            # so rendering them here would be wasted work. It also already includes
            # gazebo_properties.xacro and calls gazebo_physical_properties itself,
            # so we must NOT inject that macro again (it would be duplicated).
            params_with_controller = params.copy()
            params_with_controller["controller_type"] = controller_type
            mobrob_xacro = render_template(mobrob_file, params_with_controller)

            # Process the rendered top-level file from a throwaway temp file.
            # Includes resolve through $(find ...), so no companion files are needed.
            rendered_path = os.path.join(self.source_dir, f"mobRob_{controller_suffix}.xacro")
            with open(rendered_path, "w") as f:
                f.write(mobrob_xacro)

            self.urdf_text = generate_urdf(rendered_path)
            logger.debug("URDF generated from %s", rendered_path)

            # Persist the generated artifacts so the launch files can use them.
            with open(os.path.join(self.source_dir, "mobRob.urdf"), "w") as f:
                f.write(self.urdf_text)
            with open(os.path.join(self.source_dir, "mobRob.urdf.xacro"), "w") as f:
                f.write(self._build_xacro_wrapper())

            # Refresh the install/share copies on every apply so a subsequent
            # launch reflects the latest robot (no one-shot guard).
            self._copy_to_install()

            self.generate_controller_yaml(robot_type, controller_type, params)

            return self.urdf_text
        except FileNotFoundError as e:
            self.urdf_text = f"Error: File not found during URDF generation for {robot_type} with {controller_type}: {str(e)}"
            logger.error(self.urdf_text)
            return self.urdf_text
        except Exception as e:
            self.urdf_text = f"Error generating URDF for {robot_type} with {controller_type}: {str(e)}"
            logger.error(self.urdf_text)
            return self.urdf_text

    def _build_xacro_wrapper(self):
        """Build a parametrised .urdf.xacro that wraps the chosen mobRob template."""
        controller_suffix = self._controller_suffix(self.last_robot_type, self.last_controller_type)

        lines = [
            '<?xml version="1.0" ?>',
            '<robot name="mobRob" xmlns:xacro="http://ros.org/wiki/xacro">',
            "  <!-- Xacro parameters -->",
        ]
        for param_name, param_value in self.last_params.items():
            lines.append(f'  <xacro:property name="{param_name}" value="{param_value}"/>')
        lines.append(f'  <xacro:property name="controller_type" value="{self.last_controller_type}"/>')

        submodules_dir = f"submodules/{self.last_robot_type}"
        if self.last_robot_type == "4_wheeled" and self.last_controller_type == "ackermann":
            submodules_dir += "/ackermann"
        lines.append(
            f'  <xacro:include filename="$(find mobRobURDF_description)/urdf/{submodules_dir}/mobRob_{controller_suffix}.xacro"/>'
        )
        lines.append("</robot>")
        return "\n".join(lines)

    def _copy_to_install(self):
        """Copy the generated mobRob.urdf[.xacro] from source_dir to the share dir."""
        for name in ("mobRob.urdf", "mobRob.urdf.xacro"):
            source = os.path.join(self.source_dir, name)
            dest = os.path.join(self.base_dir, name)
            try:
                if not os.path.exists(source):
                    continue
                if os.path.exists(dest) and os.path.samefile(source, dest):
                    continue
                shutil.copy2(source, dest)
                logger.debug("Copied %s to %s", name, dest)
            except Exception as e:
                logger.warning("Failed to copy %s to install directory: %s", name, str(e))

    def _write_selected_controller(self, controller_name):
        """Record the chosen controller spawner name for the launch files to read."""
        path = os.path.join(self.base_dir, SELECTED_CONTROLLER_FILE)
        try:
            with open(path, "w") as f:
                f.write(controller_name + "\n")
            logger.debug("Wrote selected controller '%s' to %s", controller_name, path)
        except Exception as e:
            logger.warning("Failed to write selected controller file %s: %s", path, str(e))

    def generate_controller_yaml(self, robot_type, controller_type, params):
        yaml_path = None
        try:
            controller_map = {
                "diff_2wc": "gazebo_controller_diffDrive_2wd_caster.yaml",
                "tricycle": "gazebo_controller_tricycle.yaml",
                "triSteer": "gazebo_controller_triSteer.yaml",
                "diff_4w": "gazebo_controller_diffDrive_4wd.yaml",
                "mecanum": "gazebo_controller_mecDrive.yaml",
                "ackermann": "gazebo_controller_ackerSteer.yaml",
            }
            controller_name_map = {
                "diff_2wc": "diffDrive_controller",
                "diff_4w": "diffDrive_controller",
                "mecanum": "mecDrive_controller",
                "tricycle": "tricycle_controller",
                "triSteer": "triSteer_controller",
                "ackermann": "ackerSteer_controller",
            }
            yaml_file = controller_map.get(controller_type)
            controller_name = controller_name_map.get(controller_type)
            if not yaml_file or not controller_name:
                logger.error("No YAML file or controller name mapped for controller_type: %s", controller_type)
                return

            # Determine subdirectory based on controller type
            if controller_type in ("diff_2wc", "diff_4w", "mecanum"):
                subdir = "drive"
            elif controller_type in ("ackermann", "triSteer"):
                subdir = "steer"
            else:
                subdir = ""
            yaml_path = os.path.join(self.control_config_dir, subdir, yaml_file)
            if not os.path.exists(yaml_path):
                logger.error("Controller YAML file not found: %s", yaml_path)
                return

            yaml = YAML()
            yaml.preserve_quotes = True
            yaml.explicit_start = False
            yaml.width = 4096
            yaml.indent(mapping=2, sequence=4, offset=2)

            with open(yaml_path, "r") as f:
                config = yaml.load(f)

            # Extract parameters from params
            chassis = params.get("chassis_size", "1.2 0.8 0.3").split()
            L = float(chassis[0])  # Length
            W = float(chassis[1])  # Width
            wheel_radius = float(params.get("wheel_radius", "0.22"))
            wheel_width = float(params.get("wheel_width", "0.12"))

            if controller_type in ("diff_2wc", "diff_4w"):
                config["diffDrive_controller"]["ros__parameters"]["wheel_separation"] = W + wheel_width
                config["diffDrive_controller"]["ros__parameters"]["wheel_radius"] = wheel_radius
            elif controller_type == "mecanum":
                config["mecDrive_controller"]["ros__parameters"]["kinematics"]["wheels_radius"] = wheel_radius
                config["mecDrive_controller"]["ros__parameters"]["kinematics"]["sum_of_robot_center_projection_on_X_Y_axis"] = L + W
            elif controller_type == "tricycle":
                config["tricycle_controller"]["ros__parameters"]["wheel_radius"] = wheel_radius
                config["tricycle_controller"]["ros__parameters"]["wheelbase"] = L
            elif controller_type == "triSteer":
                config["triSteer_controller"]["ros__parameters"]["wheelbase"] = L
                config["triSteer_controller"]["ros__parameters"]["wheel_track"] = W
                config["triSteer_controller"]["ros__parameters"]["front_wheels_radius"] = wheel_radius
                config["triSteer_controller"]["ros__parameters"]["rear_wheels_radius"] = wheel_radius
                config["triSteer_controller"]["ros__parameters"]["position_feedback"] = True
            elif controller_type == "ackermann":
                config["ackerSteer_controller"]["ros__parameters"]["wheelbase"] = L
                config["ackerSteer_controller"]["ros__parameters"]["traction_track_width"] = W
                config["ackerSteer_controller"]["ros__parameters"]["traction_wheels_radius"] = wheel_radius
                config["ackerSteer_controller"]["ros__parameters"]["front_wheel_track"] = W
                config["ackerSteer_controller"]["ros__parameters"]["rear_wheel_track"] = W
                config["ackerSteer_controller"]["ros__parameters"]["front_wheels_radius"] = wheel_radius
                config["ackerSteer_controller"]["ros__parameters"]["rear_wheels_radius"] = wheel_radius

            with open(yaml_path, "w") as f:
                yaml.dump(config, f)
            logger.debug("Updated controller YAML: %s", yaml_path)

            self._write_selected_controller(controller_name)
        except Exception as e:
            logger.error("Failed to update controller YAML %s: %s", yaml_path, str(e))

    def save_urdf(self, filename):
        if not self.last_params or not self.last_robot_type or not self.last_controller_type:
            logger.warning("No URDF parameters available to save")
            return

        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)

            # Derive a clean base name regardless of the extension the user typed.
            if filename.endswith(".urdf.xacro"):
                base = filename[: -len(".urdf.xacro")]
            elif filename.endswith(".urdf"):
                base = filename[: -len(".urdf")]
            else:
                base = filename
            static_filename = base + ".urdf"
            xacro_filename = base + ".urdf.xacro"

            # Save static URDF
            if self.urdf_text:
                with open(static_filename, "w") as f:
                    f.write(self.urdf_text)
                logger.debug("Static URDF saved to: %s", static_filename)

            # Save parametrised xacro
            with open(xacro_filename, "w") as f:
                f.write(self._build_xacro_wrapper())
            logger.debug("Xacro URDF saved to: %s", xacro_filename)

        except Exception as e:
            logger.error("Failed to save URDF files: %s", str(e))

    def get_urdf_text(self):
        return self.urdf_text
