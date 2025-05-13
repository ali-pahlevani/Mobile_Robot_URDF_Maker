import os
import tempfile
import random
import string
import logging
import shutil
import yaml
import re
from mobRobURDF_wizard.utils.utils import render_template, generate_urdf
from ament_index_python.packages import get_package_share_directory
from ruamel.yaml import YAML
from ruamel.yaml.compat import StringIO

class URDFManager:
    def __init__(self):
        # Use install directory for base_dir (ROS convention)
        self.base_dir = os.path.join(get_package_share_directory("mobRobURDF_description"), "urdf")
        # Use a unique temporary directory for source_dir
        self.source_dir = os.path.join(tempfile.gettempdir(), f"mobRobURDF_temp_{''.join(random.choices(string.ascii_lowercase, k=8))}")
        self.urdf_text = ""
        self.control_config_dir = os.path.join(get_package_share_directory("mobRobURDF_control"), "config")
        self.launch_dir = os.path.join(get_package_share_directory("mobRobURDF_launch"), "launch")
        self.last_params = {}  # Store last used parameters
        self.last_robot_type = None
        self.last_controller_type = None
        self._install_copied = False  # Track if files were copied to install directory
        os.makedirs(self.source_dir, exist_ok=True)
        #logging.debug(f"URDFManager initialized with base_dir: {self.base_dir}, source_dir: {self.source_dir}, control_config_dir: {self.control_config_dir}, launch_dir: {self.launch_dir}")

    def generate_urdf(self, robot_type, controller_type, params):
        try:
            self.last_params = params.copy()  # Store parameters for save_urdf
            self.last_robot_type = robot_type
            self.last_controller_type = controller_type

            submodules_dir = os.path.join(self.base_dir, "submodules", robot_type)
            if robot_type == "4_wheeled" and controller_type == "ackermann":
                submodules_dir = os.path.join(submodules_dir, "ackermann")
            macros_dir = os.path.join(self.base_dir, "macros")
            gazebo_dir = os.path.join(self.base_dir, "gazebo_files")
            control_dir = os.path.join(gazebo_dir, "control")
            #logging.debug(f"Generating URDF for {robot_type} with {controller_type} in {submodules_dir}")

            controller_map = {
                ("2_wheeled_caster", "diff_2wc"): "2wc_diff",
                ("3_wheeled", "tricycle"): "3w_tricycle",
                ("3_wheeled", "triSteer"): "3w_triSteer",
                ("4_wheeled", "diff_4w"): "4w_diff",
                ("4_wheeled", "mecanum"): "4w_mec",
                ("4_wheeled", "ackermann"): "4w_acker",
            }
            controller_suffix = controller_map.get((robot_type, controller_type), "4w_diff")

            base_file = os.path.join(submodules_dir, "base.xacro")
            wheels_file = os.path.join(submodules_dir, "wheels.xacro")
            sensors_file = os.path.join(submodules_dir, "sensors.xacro")
            mobrob_file = os.path.join(submodules_dir, f"mobRob_{controller_suffix}.xacro")
            inertial_file = os.path.join(macros_dir, "inertial_macros.xacro")
            material_file = os.path.join(macros_dir, "material_macros.xacro")
            gazebo_sensors_file = os.path.join(gazebo_dir, "gazebo_sensors.xacro")
            gazebo_control_file = os.path.join(control_dir, f"gazebo_ros2_control_{controller_suffix}.xacro")

            #logging.debug(f"Checking for mobrob_file: {mobrob_file}")
            if not os.path.exists(mobrob_file):
                self.urdf_text = f"Error: mobRob_{controller_suffix}.xacro not found for {robot_type} with {controller_type} in {submodules_dir}"
                #logging.error(self.urdf_text)
                return self.urdf_text

            # Add controller_type to params
            params_with_controller = params.copy()
            params_with_controller['controller_type'] = controller_type

            base_xacro = render_template(base_file, params_with_controller)
            wheels_xacro = render_template(wheels_file, params_with_controller)
            sensors_xacro = render_template(sensors_file, params_with_controller)
            inertial_xacro = render_template(inertial_file, params_with_controller)
            mobrob_xacro = render_template(mobrob_file, params_with_controller)
            gazebo_sensors_xacro = render_template(gazebo_sensors_file, params_with_controller)
            material_xacro = render_template(material_file, params_with_controller)
            gazebo_control_xacro = render_template(gazebo_control_file, params_with_controller)

            # Insert gazebo_physical_properties macro call before </robot>
            gazebo_macro = (
                '<xacro:include filename="$(find mobRobURDF_description)/urdf/gazebo_files/gazebo_properties.xacro"/>\n'
                '<xacro:gazebo_physical_properties\n'
                f'  controller_type="{controller_type}"\n'
                f'  chassis_size="{params.get("chassis_size", "1.2 0.8 0.3")}"\n'
                f'  wheel_radius="{params.get("wheel_radius", "0.22")}"\n'
                f'  wheel_width="{params.get("wheel_width", "0.12")}"\n'
                '/>\n'
            )
            # Find the last </robot> and insert before it
            mobrob_xacro_lines = mobrob_xacro.splitlines()
            for i, line in enumerate(mobrob_xacro_lines):
                if '</robot>' in line:
                    mobrob_xacro_lines.insert(i, gazebo_macro)
                    break
            mobrob_xacro = '\n'.join(mobrob_xacro_lines)

            work_dir = os.path.join(self.base_dir, "temp", robot_type, controller_type)
            os.makedirs(work_dir, exist_ok=True)
            #logging.debug(f"Created temporary directory: {work_dir}")

            with open(os.path.join(work_dir, "base.xacro"), 'w') as f:
                f.write(base_xacro)
            with open(os.path.join(work_dir, "wheels.xacro"), 'w') as f:
                f.write(wheels_xacro)
            with open(os.path.join(work_dir, "sensors.xacro"), 'w') as f:
                f.write(sensors_xacro)
            with open(os.path.join(work_dir, "inertial_macros.xacro"), 'w') as f:
                f.write(inertial_xacro)
            with open(os.path.join(work_dir, "material_macros.xacro"), 'w') as f:
                f.write(material_xacro)
            with open(os.path.join(work_dir, "gazebo_sensors.xacro"), 'w') as f:
                f.write(gazebo_sensors_xacro)
            with open(os.path.join(work_dir, f"gazebo_ros2_control_{controller_suffix}.xacro"), 'w') as f:
                f.write(gazebo_control_xacro)
            mobrob_path = os.path.join(work_dir, f"mobRob_{controller_suffix}.xacro")
            with open(mobrob_path, 'w') as f:
                f.write(mobrob_xacro)

            self.urdf_text = generate_urdf(mobrob_path)
            #logging.debug("URDF generated successfully")

            unified_urdf_path = os.path.join(self.source_dir, "mobRob.urdf")
            os.makedirs(self.source_dir, exist_ok=True)
            with open(unified_urdf_path, 'w') as f:
                f.write(self.urdf_text)
            #logging.debug(f"Unified URDF saved to: {unified_urdf_path}")

            # Copy to install directory
            if not self._install_copied:
                self._copy_to_install()
                self._install_copied = True

            self.generate_controller_yaml(robot_type, controller_type, params)

            shutil.rmtree(work_dir)
            #logging.debug(f"Cleaned up temporary directory: {work_dir}")

            return self.urdf_text
        except FileNotFoundError as e:
            self.urdf_text = f"Error: File not found during URDF generation for {robot_type} with {controller_type}: {str(e)}"
            #logging.error(self.urdf_text)
            return self.urdf_text
        except Exception as e:
            self.urdf_text = f"Error generating URDF for {robot_type} with {controller_type}: {str(e)}"
            #logging.error(self.urdf_text)
            return self.urdf_text

    def _copy_to_install(self):
        source_urdf_xacro = os.path.join(self.source_dir, "mobRob.urdf.xacro")
        dest_urdf_xacro = os.path.join(self.base_dir, "mobRob.urdf.xacro")
        source_urdf = os.path.join(self.source_dir, "mobRob.urdf")
        dest_urdf = os.path.join(self.base_dir, "mobRob.urdf")

        try:
            if os.path.exists(source_urdf_xacro):
                if os.path.exists(dest_urdf_xacro) and os.path.samefile(source_urdf_xacro, dest_urdf_xacro):
                    logging.debug(f"Skipped copying URDF.xacro: source {source_urdf_xacro} and destination {dest_urdf_xacro} are the same")
                else:
                    shutil.copy2(source_urdf_xacro, dest_urdf_xacro)
                    #logging.debug(f"Copied URDF.xacro to {dest_urdf_xacro}")
            if os.path.exists(source_urdf):
                if os.path.exists(dest_urdf) and os.path.samefile(source_urdf, dest_urdf):
                    logging.debug(f"Skipped copying URDF: source {source_urdf} and destination {dest_urdf} are the same")
                else:
                    shutil.copy2(source_urdf, dest_urdf)
                    #logging.debug(f"Copied URDF to {dest_urdf}")
        except Exception as e:
            logging.warning(f"Failed to copy files to install directory: {str(e)}")

    def generate_controller_yaml(self, robot_type, controller_type, params):
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
                #logging.error(f"No YAML file or controller name mapped for controller_type: {controller_type}")
                return

            # Determine subdirectory based on controller type
            subdir = "drive" if controller_type in ["diff_2wc", "diff_4w", "mecanum"] else "steer" if controller_type in ["ackermann", "triSteer"] else ""
            yaml_path = os.path.join(self.control_config_dir, subdir, yaml_file)
            if not os.path.exists(yaml_path):
                #logging.error(f"Controller YAML file not found: {yaml_path}")
                return

            yaml = YAML()
            yaml.preserve_quotes = True
            yaml.explicit_start = False
            yaml.width = 4096
            yaml.indent(mapping=2, sequence=4, offset=2)

            with open(yaml_path, 'r') as f:
                config = yaml.load(f)

            # Extract parameters from params
            L = float(params.get("chassis_size", "1.2 0.8 0.3").split()[0])  # Length
            W = float(params.get("chassis_size", "1.2 0.8 0.3").split()[1])  # Width
            wheel_radius = float(params.get("wheel_radius", "0.22"))
            wheel_width = float(params.get("wheel_width", "0.12"))

            if controller_type == "diff_2wc":
                config["diffDrive_controller"]["ros__parameters"]["wheel_separation"] = W + wheel_width
                config["diffDrive_controller"]["ros__parameters"]["wheel_radius"] = wheel_radius
            elif controller_type == "diff_4w":
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

            with open(yaml_path, 'w') as f:
                yaml.dump(config, f)
            #logging.debug(f"Updated controller YAML: {yaml_path}")

            self.update_launch_file(controller_name)
        except Exception as e:
            logging.error(f"Failed to update controller YAML {yaml_path}: {str(e)}")

    def update_launch_file(self, controller_name):
        try:
            # Define source and install launch file paths
            source_launch_file = os.path.join(os.path.expanduser("~"), "Mobile_Robot_URDF_Maker", "src", "mobRobURDF_launch", "launch", "gazebo_test.launch.py")
            install_launch_file = os.path.join(self.launch_dir, "gazebo_test.launch.py")
            launch_files = [source_launch_file, install_launch_file]

            for launch_file_path in launch_files:
                if not os.path.exists(launch_file_path):
                    #logging.error(f"Launch file not found: {launch_file_path}")
                    continue

                with open(launch_file_path, 'r') as f:
                    content = f.read()

                # Regex to match arguments=["<controller_name>"] in controllers node
                pattern = r'(controllers\s*=\s*Node\([^)]*arguments\s*=\s*\["[^"]*"\][^)]*\))'
                match = re.search(pattern, content, re.DOTALL)
                if not match:
                    #logging.error(f"Could not find controllers node in {launch_file_path}")
                    continue

                controllers_block = match.group(1)
                arg_match = re.search(r'arguments\s*=\s*\["([^"]*)"\]', controllers_block)
                if not arg_match:
                    #logging.error(f"Could not parse arguments in {launch_file_path}")
                    continue

                old_controller = arg_match.group(1)
                new_controllers_block = re.sub(
                    r'arguments\s*=\s*\["[^"]*"\]',
                    f'arguments=["{controller_name}"]',
                    controllers_block
                )
                new_content = content.replace(controllers_block, new_controllers_block)

                with open(launch_file_path, 'w') as f:
                    f.write(new_content)
                #logging.debug(f"Updated launch file: {launch_file_path} with controller: {controller_name}")

        except Exception as e:
            logging.error(f"Failed to update launch file {launch_file_path}: {str(e)}")

    def save_urdf(self, filename):
        if not self.last_params or not self.last_robot_type or not self.last_controller_type:
            #logging.warning("No URDF parameters available to save")
            return

        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)

            # Save static URDF
            if self.urdf_text:
                static_filename = filename if filename.endswith(".urdf") else filename + ".urdf"
                with open(static_filename, 'w') as f:
                    f.write(self.urdf_text)
                logging.debug(f"Static URDF saved to: {static_filename}")

                # Copy static URDF to install directory
                if not self._install_copied:
                    self._copy_to_install()
                    self._install_copied = True

            # Determine controller suffix
            controller_map = {
                ("2_wheeled_caster", "diff_2wc"): "2wc_diff",
                ("3_wheeled", "tricycle"): "3w_tricycle",
                ("3_wheeled", "triSteer"): "3w_triSteer",
                ("4_wheeled", "diff_4w"): "4w_diff",
                ("4_wheeled", "mecanum"): "4w_mec",
                ("4_wheeled", "ackermann"): "4w_acker",
            }
            controller_suffix = controller_map.get((self.last_robot_type, self.last_controller_type), "4w_diff")

            # Generate xacro filename
            xacro_filename = filename if filename.endswith(".urdf.xacro") else filename.rsplit(".", 1)[0] + ".urdf.xacro" if "." in filename else filename + ".urdf.xacro"

            # Generate xacro content
            xacro_content = [
                '<?xml version="1.0" ?>',
                '<robot name="mobRob" xmlns:xacro="http://ros.org/wiki/xacro">',
                '  <!-- Xacro parameters -->',
            ]

            # Add all parameters as xacro properties
            for param_name, param_value in self.last_params.items():
                xacro_content.append(f'  <xacro:property name="{param_name}" value="{param_value}"/>')
            xacro_content.append(f'  <xacro:property name="controller_type" value="{self.last_controller_type}"/>')

            # Include the appropriate mobRob xacro file
            submodules_dir = f"submodules/{self.last_robot_type}"
            if self.last_robot_type == "4_wheeled" and self.last_controller_type == "ackermann":
                submodules_dir += "/ackermann"
            xacro_content.append(f'  <xacro:include filename="$(find mobRobURDF_description)/urdf/{submodules_dir}/mobRob_{controller_suffix}.xacro"/>')
            xacro_content.append('</robot>')

            # Save xacro file in source directory
            with open(xacro_filename, 'w') as f:
                f.write('\n'.join(xacro_content))
            logging.debug(f"Xacro URDF saved to: {xacro_filename}")

            # Copy xacro to install directory
            if not self._install_copied:
                self._copy_to_install()
                self._install_copied = True

        except Exception as e:
            logging.error(f"Failed to save URDF files: {str(e)}")

    def get_urdf_text(self):
        return self.urdf_text