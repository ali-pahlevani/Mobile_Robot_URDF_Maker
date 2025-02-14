from mobRobURDF_wizard.utils.utils import render_template, generate_urdf
import os
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QPushButton, QTextEdit, QFileDialog)
from ament_index_python.packages import get_package_share_directory

class URDFPreviewPage(QWizardPage):
    def __init__(self, parent=None):
        super(URDFPreviewPage, self).__init__(parent)
        self.setTitle("URDF Preview")
        layout = QVBoxLayout()
        self.previewTextEdit = QTextEdit()
        self.previewTextEdit.setReadOnly(True)
        layout.addWidget(self.previewTextEdit)
        self.saveButton = QPushButton("Save URDF to Folder")
        layout.addWidget(self.saveButton)
        self.setLayout(layout)
        self.saveButton.clicked.connect(self.saveURDF)
        self.urdf_text = ""

    def updatePreview(self, params):
        try:
            # Locate the xacro files directory relative to this file
            xacro_dir = os.path.join(get_package_share_directory("mobRobURDF_description"), "urdf")

            # Paths to the Xacro files based on the directory structure
            base_file    = os.path.join(xacro_dir, "submodules", "base.xacro")
            wheels_file  = os.path.join(xacro_dir, "submodules", "wheels.xacro")
            sensors_file = os.path.join(xacro_dir, "submodules", "sensors.xacro")
            inertial_file  = os.path.join(xacro_dir, "macros", "inertial_macros.xacro")
            mobrob_file  = os.path.join(xacro_dir, "mobRob.xacro")
            gazebo_sensors_file = os.path.join(xacro_dir, "gazebo_files", "gazebo_sensors.xacro")
            material_file = os.path.join(xacro_dir, "macros", "material_macros.xacro")

            # Render each xacro file with the given parameters
            base_xacro    = render_template(base_file, params)
            wheels_xacro  = render_template(wheels_file, params)
            sensors_xacro = render_template(sensors_file, params)
            inertial_xacro  = render_template(inertial_file, params)
            mobRob_xacro  = render_template(mobrob_file, params)
            gazebo_sensors_xacro = render_template(gazebo_sensors_file, params)
            material_xacro  = render_template(material_file, params)

            # Write the rendered xacro files to a temporary working directory
            work_dir = os.path.join(os.path.dirname(__file__), "temp")
            os.makedirs(work_dir, exist_ok=True)
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

            # Convert the rendered mobRob.xacro to URDF using the xacro tool
            self.urdf_text = generate_urdf(mobRob_path)
            self.previewTextEdit.setPlainText(self.urdf_text)

        except Exception as e:
            self.previewTextEdit.setPlainText(f"Error generating URDF: {str(e)}")
            print(f"Error generating URDF: {str(e)}")

    def saveURDF(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save URDF", "", "URDF Files (*.urdf)")
        if filename:
            with open(filename, 'w') as f:
                f.write(self.urdf_text)
