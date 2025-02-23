import os
import logging
import shutil
from mobRobURDF_wizard.utils.utils import render_template, generate_urdf
from ament_index_python.packages import get_package_share_directory

# Class for URDF Generation, Previewing, and Saving
class URDFManager:
    def __init__(self):
        self.base_dir = os.path.join(get_package_share_directory("mobRobURDF_description"), "urdf")
        self.urdf_text = ""
        #logging.debug(f"URDFManager initialized with base_dir: {self.base_dir}")

    def generate_urdf(self, robot_type, params):
        try:
            # Base directory for submodules and shared files
            submodules_dir = os.path.join(self.base_dir, "submodules", robot_type)
            macros_dir = os.path.join(self.base_dir, "macros")
            gazebo_dir = os.path.join(self.base_dir, "gazebo_files")
            #logging.debug(f"Generating URDF for {robot_type} in {submodules_dir}")

            # File paths for robot-type-specific Xacro files
            base_file = os.path.join(submodules_dir, "base.xacro")
            wheels_file = os.path.join(submodules_dir, "wheels.xacro")
            sensors_file = os.path.join(submodules_dir, "sensors.xacro")
            mobrob_file = os.path.join(submodules_dir, "mobRob.xacro")

            # File paths for shared Xacro files
            inertial_file = os.path.join(macros_dir, "inertial_macros.xacro")
            material_file = os.path.join(macros_dir, "material_macros.xacro")
            gazebo_sensors_file = os.path.join(gazebo_dir, "gazebo_sensors.xacro")

            # Check if the main mobRob.xacro file exists for the robot type
            if not os.path.exists(mobrob_file):
                self.urdf_text = f"Error: mobRob.xacro not found for {robot_type} in {submodules_dir}"
                logging.error(self.urdf_text)
                return self.urdf_text

            # Render all Xacro files with the provided parameters
            base_xacro = render_template(base_file, params)
            wheels_xacro = render_template(wheels_file, params)
            sensors_xacro = render_template(sensors_file, params)
            inertial_xacro = render_template(inertial_file, params)
            mobRob_xacro = render_template(mobrob_file, params)
            gazebo_sensors_xacro = render_template(gazebo_sensors_file, params)
            material_xacro = render_template(material_file, params)

            # Create a temporary working directory in the package's share directory
            work_dir = os.path.join(self.base_dir, "temp", robot_type)
            os.makedirs(work_dir, exist_ok=True)
            #logging.debug(f"Created temporary directory: {work_dir}")

            # Write rendered Xacro files to the temporary directory
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
            mobRob_path = os.path.join(work_dir, "mobRob.xacro")
            with open(mobRob_path, 'w') as f:
                f.write(mobRob_xacro)

            # Generate the URDF from the rendered mobRob.xacro
            self.urdf_text = generate_urdf(mobRob_path)
            #logging.debug("URDF generated successfully")

            # Save the unified URDF file to the base directory
            unified_urdf_path = os.path.join(self.base_dir, "mobRob.urdf")
            with open(unified_urdf_path, 'w') as f:
                f.write(self.urdf_text)
            #logging.debug(f"Unified URDF saved to: {unified_urdf_path}")

            # Clean up temporary directory
            shutil.rmtree(work_dir)
            #logging.debug(f"Cleaned up temporary directory: {work_dir}")

            return self.urdf_text
        except FileNotFoundError as e:
            self.urdf_text = f"Error: File not found during URDF generation for {robot_type}: {str(e)}"
            logging.error(self.urdf_text)
            return self.urdf_text
        except Exception as e:
            self.urdf_text = f"Error generating URDF for {robot_type}: {str(e)}"
            logging.error(self.urdf_text)
            return self.urdf_text

    def save_urdf(self, filename):
        if self.urdf_text:
            try:
                with open(filename, 'w') as f:
                    f.write(self.urdf_text)
                #logging.debug(f"URDF saved to: {filename}")
            except Exception as e:
                logging.error(f"Failed to save URDF to {filename}: {str(e)}")
        else:
            logging.warning("No URDF text to save")

    def get_urdf_text(self):
        return self.urdf_text