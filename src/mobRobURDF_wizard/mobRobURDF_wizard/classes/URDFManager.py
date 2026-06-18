import os
import tempfile
import random
import string
import logging
import shutil
from mobRobURDF_wizard.utils.utils import render_template, generate_urdf
from mobRobURDF_wizard.classes.sensor_config import (
    build_user_sensors_xacro, build_bridge_yaml, camera_image_topics,
)
from ament_index_python.packages import get_package_share_directory
from ruamel.yaml import YAML

logger = logging.getLogger(__name__)

SELECTED_CONTROLLER_FILE = "selected_controller.txt"
DEFAULT_CONTROLLER_NAME = "diffDrive_controller"
GAZEBO_SIM_PLUGIN = "gz_ros2_control/GazeboSimSystem"

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
        self.base_dir = os.path.join(get_package_share_directory("mobRobURDF_description"), "urdf")
        self.source_dir = os.path.join(
            tempfile.gettempdir(),
            f"mobRobURDF_temp_{''.join(random.choices(string.ascii_lowercase, k=8))}",
        )
        self.urdf_text = ""
        self.control_config_dir = os.path.join(get_package_share_directory("mobRobURDF_control"), "config")
        self.last_params = {}
        self.last_robot_type = None
        self.last_controller_type = None
        self.last_sensors = []
        self.last_tuner_params = {}   # persists across ConfigurationPage Apply clicks
        self.last_hardware_interface = GAZEBO_SIM_PLUGIN
        self.pending_restore = None   # set by StartSessionPage; consumed by ConfigurationPage
        os.makedirs(self.source_dir, exist_ok=True)
        logger.debug("URDFManager initialized (base_dir=%s, source_dir=%s)", self.base_dir, self.source_dir)

    def _controller_suffix(self, robot_type, controller_type):
        return CONTROLLER_SUFFIX_MAP.get((robot_type, controller_type), "4w_diff")

    def generate_urdf(self, robot_type, controller_type, params, sensors=None):
        sensors = sensors or []
        try:
            self.last_params = params.copy()
            self.last_robot_type = robot_type
            self.last_controller_type = controller_type
            self.last_sensors = sensors

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

            # 1. Write user_sensors.xacro to base_dir so $(find ...) includes work.
            sensors_xacro_path = os.path.join(self.base_dir, "user_sensors.xacro")
            with open(sensors_xacro_path, "w") as f:
                f.write(build_user_sensors_xacro(sensors))

            # 2. Write generated bridge YAML and image topics file.
            self._write_bridge_yaml(sensors)
            self._write_image_topics(sensors)

            # 3. Render body template and write to source_dir.
            params_with_controller = params.copy()
            params_with_controller["controller_type"] = controller_type
            params_with_controller["hardware_plugin"] = self.last_hardware_interface
            params_with_controller["use_gazebo_sim"] = (
                "1" if self.last_hardware_interface == GAZEBO_SIM_PLUGIN else "0"
            )
            mobrob_xacro = render_template(mobrob_file, params_with_controller)
            rendered_path = os.path.join(self.source_dir, f"mobRob_{controller_suffix}.xacro")
            with open(rendered_path, "w") as f:
                f.write(mobrob_xacro)

            # 4. Build a temporary wrapper that includes both body and sensors using
            #    absolute paths so xacro can resolve them without $(find ...).
            full_path = os.path.join(self.source_dir, "full_src.xacro")
            with open(full_path, "w") as f:
                f.write(self._build_full_xacro(rendered_path, sensors_xacro_path))

            self.urdf_text = generate_urdf(full_path)
            logger.debug("URDF generated from %s", full_path)

            # 5. Persist artifacts for launch files.
            with open(os.path.join(self.source_dir, "mobRob.urdf"), "w") as f:
                f.write(self.urdf_text)
            with open(os.path.join(self.source_dir, "mobRob.urdf.xacro"), "w") as f:
                f.write(self._build_xacro_wrapper())

            self._copy_to_install()
            self.generate_controller_yaml(robot_type, controller_type, params)
            self._write_use_sim_time_yaml()
            return self.urdf_text

        except FileNotFoundError as e:
            self.urdf_text = f"Error: File not found during URDF generation for {robot_type} with {controller_type}: {str(e)}"
            logger.error(self.urdf_text)
            return self.urdf_text
        except Exception as e:
            self.urdf_text = f"Error generating URDF for {robot_type} with {controller_type}: {str(e)}"
            logger.error(self.urdf_text)
            return self.urdf_text

    def _build_full_xacro(self, body_path: str, sensors_path: str) -> str:
        """Wrapper xacro using absolute paths (for in-process URDF generation)."""
        use_sim = "1" if self.last_hardware_interface == GAZEBO_SIM_PLUGIN else "0"
        return '\n'.join([
            '<?xml version="1.0" ?>',
            '<robot name="mobRob" xmlns:xacro="http://ros.org/wiki/xacro" xmlns:gz="http://gazebosim.org/schema">',
            f'  <xacro:property name="hardware_plugin" value="{self.last_hardware_interface}"/>',
            f'  <xacro:property name="use_gazebo_sim" value="{use_sim}"/>',
            f'  <xacro:include filename="{body_path}"/>',
            f'  <xacro:include filename="{sensors_path}"/>',
            '</robot>',
        ])

    def _build_xacro_wrapper(self):
        """Build mobRob.urdf.xacro that uses $(find ...) paths for launch files."""
        controller_suffix = self._controller_suffix(self.last_robot_type, self.last_controller_type)

        lines = [
            '<?xml version="1.0" ?>',
            '<robot name="mobRob" xmlns:xacro="http://ros.org/wiki/xacro" xmlns:gz="http://gazebosim.org/schema">',
            '  <!-- Xacro parameters -->',
        ]
        for param_name, param_value in self.last_params.items():
            lines.append(f'  <xacro:property name="{param_name}" value="{param_value}"/>')
        lines.append(f'  <xacro:property name="controller_type" value="{self.last_controller_type}"/>')
        use_sim = "1" if self.last_hardware_interface == GAZEBO_SIM_PLUGIN else "0"
        lines.append(f'  <xacro:property name="hardware_plugin" value="{self.last_hardware_interface}"/>')
        lines.append(f'  <xacro:property name="use_gazebo_sim" value="{use_sim}"/>')

        submodules_dir = f"submodules/{self.last_robot_type}"
        if self.last_robot_type == "4_wheeled" and self.last_controller_type == "ackermann":
            submodules_dir += "/ackermann"
        lines.append(
            f'  <xacro:include filename="$(find mobRobURDF_description)/urdf/{submodules_dir}/mobRob_{controller_suffix}.xacro"/>'
        )
        lines.append(
            '  <xacro:include filename="$(find mobRobURDF_description)/urdf/user_sensors.xacro"/>'
        )
        lines.append("</robot>")
        return "\n".join(lines)

    def _write_bridge_yaml(self, sensors: list):
        """Write gz_bridge_generated.yaml to the gazebo package's config dir."""
        try:
            gazebo_config = os.path.join(
                get_package_share_directory("mobRobURDF_gazebo"), "config"
            )
            path = os.path.join(gazebo_config, "gz_bridge_generated.yaml")
            with open(path, "w") as f:
                f.write(build_bridge_yaml(sensors))
            logger.debug("Wrote generated bridge YAML to %s", path)
        except Exception as e:
            logger.warning("Failed to write bridge YAML: %s", e)

    def _write_image_topics(self, sensors: list):
        """Write gz_image_topics.txt listing camera image topics for ros_gz_image bridge."""
        try:
            gazebo_config = os.path.join(
                get_package_share_directory("mobRobURDF_gazebo"), "config"
            )
            topics = camera_image_topics(sensors)
            path = os.path.join(gazebo_config, "gz_image_topics.txt")
            with open(path, "w") as f:
                f.write('\n'.join(topics) + ('\n' if topics else ''))
            logger.debug("Wrote image topics to %s: %s", path, topics)
        except Exception as e:
            logger.warning("Failed to write image topics: %s", e)

    def _write_use_sim_time_yaml(self):
        """Write use_sim_time.yaml based on the current hardware interface."""
        try:
            gazebo_config = os.path.join(
                get_package_share_directory("mobRobURDF_gazebo"), "config"
            )
            path = os.path.join(gazebo_config, "use_sim_time.yaml")
            use_sim = self.last_hardware_interface == GAZEBO_SIM_PLUGIN
            with open(path, "w") as f:
                f.write(f"gz:\n  use_sim_time: {'true' if use_sim else 'false'}\n")
            logger.debug("Updated use_sim_time.yaml: use_sim_time=%s", use_sim)
        except Exception as e:
            logger.warning("Failed to update use_sim_time.yaml: %s", e)

    def _copy_to_install(self):
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

            chassis = params.get("chassis_size", "1.2 0.8 0.3").split()
            L = float(chassis[0])
            W = float(chassis[1])
            wheel_radius = float(params.get("wheel_radius", "0.22"))
            wheel_width = float(params.get("wheel_width", "0.12"))

            if controller_type in ("diff_2wc", "diff_4w"):
                config["diffDrive_controller"]["ros__parameters"]["wheel_separation"] = W + wheel_width
                config["diffDrive_controller"]["ros__parameters"]["wheel_radius"] = wheel_radius
            elif controller_type == "mecanum":
                # sum_of_robot_center_projection_on_X_Y_axis = |wheel_x| + |wheel_y|
                # Wheels are at (L/2 - r/1.5) in X and (W/2 + w/2) in Y from robot center.
                wheel_x = L / 2 - wheel_radius / 1.5
                wheel_y = W / 2 + wheel_width / 2
                config["mecDrive_controller"]["ros__parameters"]["kinematics"]["wheels_radius"] = wheel_radius
                config["mecDrive_controller"]["ros__parameters"]["kinematics"]["sum_of_robot_center_projection_on_X_Y_axis"] = wheel_x + wheel_y
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

            # Apply any tuner params that were set from the ControllerTunerPage.
            if self.last_tuner_params:
                self._apply_tuner_params_to_config(config, controller_type, controller_name)

            with open(yaml_path, "w") as f:
                yaml.dump(config, f)
            logger.debug("Updated controller YAML: %s", yaml_path)

            self._write_selected_controller(controller_name)
        except Exception as e:
            logger.error("Failed to update controller YAML %s: %s", yaml_path, str(e))

    def _apply_tuner_params_to_config(self, config, controller_type, controller_name):
        """Write last_tuner_params into the already-loaded YAML config dict."""
        p = self.last_tuner_params
        ctrl = config[controller_name]["ros__parameters"]

        ctrl["publish_rate"]   = float(p.get("publish_rate", 50.0))
        ctrl["enable_odom_tf"] = bool(p.get("enable_odom_tf", True))
        ctrl["open_loop"]      = bool(p.get("open_loop", False))

        if "cmd_vel_timeout" in p:
            ctrl["cmd_vel_timeout"] = float(p["cmd_vel_timeout"])

        config["controller_manager"]["ros__parameters"]["update_rate"] = int(p.get("update_rate", 50))

        max_lv = float(p.get("max_linear_velocity", 0.0))
        max_av = float(p.get("max_angular_velocity", 0.0))
        max_la = float(p.get("max_linear_acceleration", 0.0))
        max_aa = float(p.get("max_angular_acceleration", 0.0))

        if controller_type in ("diff_4w", "diff_2wc"):
            # diff_drive_controller uses linear.x / angular.z nested structure.
            if "linear" not in ctrl:
                ctrl["linear"] = {}
            if "x" not in ctrl["linear"]:
                ctrl["linear"]["x"] = {}
            ctrl["linear"]["x"]["max_velocity"]    = max_lv
            ctrl["linear"]["x"]["min_velocity"]    = -max_lv
            ctrl["linear"]["x"]["max_acceleration"] = max_la
            ctrl["linear"]["x"]["max_deceleration"] = max_la

            if "angular" not in ctrl:
                ctrl["angular"] = {}
            if "z" not in ctrl["angular"]:
                ctrl["angular"]["z"] = {}
            ctrl["angular"]["z"]["max_velocity"]    = max_av
            ctrl["angular"]["z"]["min_velocity"]    = -max_av
            ctrl["angular"]["z"]["max_acceleration"] = max_aa
            ctrl["angular"]["z"]["max_deceleration"] = max_aa

        elif controller_type in ("tricycle", "triSteer", "ackermann"):
            max_sa = float(p.get("max_steering_angle", 0.785))
            max_sv = float(p.get("max_steering_velocity", 1.0))

            if "traction" not in ctrl:
                ctrl["traction"] = {}
            ctrl["traction"]["max_acceleration"] = max_la
            ctrl["traction"]["max_deceleration"] = max_la

            if "steering" not in ctrl:
                ctrl["steering"] = {}
            ctrl["steering"]["max_position"] = max_sa
            ctrl["steering"]["max_velocity"] = max_sv

        # mecanum controller does not expose velocity limits via standard YAML keys.

    def apply_tuner_params(self, robot_type, controller_type, params, tuner_params):
        """Store tuner params and regenerate the controller YAML immediately."""
        self.last_tuner_params = tuner_params.copy()
        self.generate_controller_yaml(robot_type, controller_type, params)

    def save_urdf(self, filename):
        if not self.last_params or not self.last_robot_type or not self.last_controller_type:
            logger.warning("No URDF parameters available to save")
            return

        try:
            os.makedirs(os.path.dirname(filename), exist_ok=True)

            if filename.endswith(".urdf.xacro"):
                base = filename[: -len(".urdf.xacro")]
            elif filename.endswith(".urdf"):
                base = filename[: -len(".urdf")]
            else:
                base = filename
            static_filename = base + ".urdf"
            xacro_filename = base + ".urdf.xacro"

            if self.urdf_text:
                with open(static_filename, "w") as f:
                    f.write(self.urdf_text)
                logger.debug("Static URDF saved to: %s", static_filename)

            with open(xacro_filename, "w") as f:
                f.write(self._build_xacro_wrapper())
            logger.debug("Xacro URDF saved to: %s", xacro_filename)

        except Exception as e:
            logger.error("Failed to save URDF files: %s", str(e))

    def get_urdf_text(self):
        return self.urdf_text
