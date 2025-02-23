from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QLabel, QLineEdit, QPushButton, 
                             QTextEdit, QFileDialog, QWidget, QHBoxLayout)
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import pyqtSignal
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from mobRobURDF_wizard.classes.OpenGLWidget import OpenGLWidget
from mobRobURDF_wizard.utils.utils import get_color
import logging

# Configuration Page
class ConfigurationPage(QWizardPage):
    modelUpdated = pyqtSignal(float, float, float, str, str, str, str, str, tuple, tuple, tuple, tuple, str, str)

    def __init__(self, urdf_manager, parent=None):
        super().__init__(parent)
        self.urdf_manager = urdf_manager
        self.robot_type = None
        
        self.default_save_path = os.path.join(get_package_share_directory("mobRobURDF_description"), "urdf", "mobRob.urdf")

        # Main layout for the entire page
        main_layout = QHBoxLayout()

        # Left layout: Navigation bar and URDF preview
        left_widget = QWidget()
        left_layout = QVBoxLayout()
        left_widget.setLayout(left_layout)

        # Placeholder for navigation bar
        nav_spacer = QWidget()
        nav_spacer.setFixedWidth(250)  # Match nav bar width from RobotWizard

        # URDF preview text box
        self.previewTextEdit = QTextEdit()
        self.previewTextEdit.setFixedHeight(750)
        self.previewTextEdit.setFixedWidth(300)
        self.previewTextEdit.setReadOnly(True)
        self.previewTextEdit.setStyleSheet("""
            QTextEdit {
                background-color: #FFFFFF;
                border: 1px solid #CCCCCC;
                border-radius: 5px;
                padding: 5px;
                font-family: "Courier New";
                font-size: 12pt;
            }
        """)

        # Add nav spacer and preview text box to left layout
        left_layout.addWidget(nav_spacer)
        left_layout.addWidget(self.previewTextEdit)
        left_layout.addStretch()

        # Middle layout: Buttons and parameters
        middle_widget = QWidget()
        middle_layout = QVBoxLayout()
        middle_widget.setLayout(middle_layout)

        # Parameters layout (for inputs)
        self.param_layout = QVBoxLayout()

        # Buttons
        self.applyButton = QPushButton("Apply and Preview", styleSheet="font-size: 11pt; font-weight: bold; color: red;")
        self.applyButton.setFixedWidth(200)
        self.applyButton.setMinimumHeight(30)
        self.applyButton.clicked.connect(self.applyChanges)

        self.saveButton = QPushButton("Save URDF to Folder", styleSheet="font-size: 11pt; font-weight: bold; color: blue;")
        self.saveButton.setFixedWidth(200)
        self.saveButton.setMinimumHeight(30)
        self.saveButton.clicked.connect(self.saveURDF)

        # Add parameters and buttons to middle layout
        middle_layout.addLayout(self.param_layout)
        middle_layout.addWidget(self.applyButton)
        middle_layout.addWidget(self.saveButton)
        middle_layout.addStretch()

        # Right layout: OpenGL preview
        self.glWidget = OpenGLWidget()
        self.modelUpdated.connect(self.glWidget.updateRobotModel)

        # Add all sections to main layout
        main_layout.addWidget(left_widget, stretch=1)  # Left: Nav + URDF preview
        main_layout.addWidget(middle_widget, stretch=1)  # Middle: Params + buttons
        main_layout.addWidget(self.glWidget, stretch=6)  # Right: OpenGL (larger stretch)

        self.setLayout(main_layout)
        #logging.debug("ConfigurationPage initialized")

    def initializePage(self):
        self.robot_type = self.field("robot_type")
        #logging.debug(f"ConfigurationPage initialized with robot_type: {self.robot_type}")
        if self.robot_type is None:
            logging.warning("robot_type is None, defaulting to 4_wheeled")
            self.robot_type = "4_wheeled"  # Fallback to avoid crashes
        self.setTitle(f"Configure {self.robot_type.replace('_', ' ').title()} Parameters")
        self.setup_parameters()
        self.applyChanges()  # Apply default parameters to update preview

    def setup_parameters(self):
        while self.param_layout.count():
            item = self.param_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self.add_common_parameters()

        if self.robot_type == "4_wheeled":
            self.add_4w_parameters()
        elif self.robot_type == "3_wheeled":
            self.add_3w_parameters()
        elif self.robot_type == "2_wheeled_caster":
            self.add_2wc_parameters()
        else:
            logging.error(f"Unknown robot_type: {self.robot_type}, no parameters added")

    def add_common_parameters(self):
        self.chassisSizeLineEdit = QLineEdit(placeholderText="e.g., 1 1 0.5", styleSheet="font-size: 10.5pt;")
        self.chassisSizeLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Chassis Size (L W H):", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.chassisSizeLineEdit)

        self.chassisMassLineEdit = QLineEdit(placeholderText="e.g., 1.0", styleSheet="font-size: 10.5pt;")
        self.chassisMassLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Chassis Mass:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.chassisMassLineEdit)

        self.chassisMaterialLineEdit = QLineEdit(placeholderText="e.g., Gray", styleSheet="font-size: 10.5pt;")
        self.chassisMaterialLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Chassis Material:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.chassisMaterialLineEdit)

        self.lidarRadiusLineEdit = QLineEdit(placeholderText="e.g., 0.2", styleSheet="font-size: 10.5pt;")
        self.lidarRadiusLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Lidar Radius:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.lidarRadiusLineEdit)

        self.lidarHeightLineEdit = QLineEdit(placeholderText="e.g., 0.1", styleSheet="font-size: 10.5pt;")
        self.lidarHeightLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Lidar Height:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.lidarHeightLineEdit)

        self.lidarMassLineEdit = QLineEdit(placeholderText="e.g., 0.2", styleSheet="font-size: 10.5pt;")
        self.lidarMassLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Lidar Mass:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.lidarMassLineEdit)

        self.lidarMaterialLineEdit = QLineEdit(placeholderText="e.g., Red", styleSheet="font-size: 10.5pt;")
        self.lidarMaterialLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Lidar Material:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.lidarMaterialLineEdit)

        self.cameraSizeLineEdit = QLineEdit(placeholderText="e.g., 0.1 0.1 0.1", styleSheet="font-size: 10.5pt;")
        self.cameraSizeLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Camera Size (L W H):", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.cameraSizeLineEdit)

        self.cameraMassLineEdit = QLineEdit(placeholderText="e.g., 0.1", styleSheet="font-size: 10.5pt;")
        self.cameraMassLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Camera Mass:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.cameraMassLineEdit)

        self.cameraMaterialLineEdit = QLineEdit(placeholderText="e.g., Blue", styleSheet="font-size: 10.5pt;")
        self.cameraMaterialLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Camera Material:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.cameraMaterialLineEdit)

    def add_4w_parameters(self):
        self.wheelRadiusLineEdit = QLineEdit(placeholderText="e.g., 0.3", styleSheet="font-size: 10.5pt;")
        self.wheelRadiusLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Radius:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelRadiusLineEdit)

        self.wheelWidthLineEdit = QLineEdit(placeholderText="e.g., 0.1", styleSheet="font-size: 10.5pt;")
        self.wheelWidthLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Width:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelWidthLineEdit)

        self.wheelMassLineEdit = QLineEdit(placeholderText="e.g., 0.5", styleSheet="font-size: 10.5pt;")
        self.wheelMassLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Mass:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelMassLineEdit)

        self.wheelMaterialLineEdit = QLineEdit(placeholderText="e.g., Black", styleSheet="font-size: 10.5pt;")
        self.wheelMaterialLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Material:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelMaterialLineEdit)

    def add_3w_parameters(self):
        self.wheelRadiusLineEdit = QLineEdit(placeholderText="e.g., 0.3", styleSheet="font-size: 10.5pt;")
        self.wheelRadiusLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Radius:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelRadiusLineEdit)

        self.wheelWidthLineEdit = QLineEdit(placeholderText="e.g., 0.1", styleSheet="font-size: 10.5pt;")
        self.wheelWidthLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Width:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelWidthLineEdit)

        self.wheelMassLineEdit = QLineEdit(placeholderText="e.g., 0.5", styleSheet="font-size: 10.5pt;")
        self.wheelMassLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Mass:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelMassLineEdit)

        self.wheelMaterialLineEdit = QLineEdit(placeholderText="e.g., Black", styleSheet="font-size: 10.5pt;")
        self.wheelMaterialLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Material:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelMaterialLineEdit)

    def add_2wc_parameters(self):
        self.wheelRadiusLineEdit = QLineEdit(placeholderText="e.g., 0.3", styleSheet="font-size: 10.5pt;")
        self.wheelRadiusLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel and Caster Radius:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelRadiusLineEdit)

        self.wheelWidthLineEdit = QLineEdit(placeholderText="e.g., 0.1", styleSheet="font-size: 10.5pt;")
        self.wheelWidthLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Width:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelWidthLineEdit)

        self.wheelMassLineEdit = QLineEdit(placeholderText="e.g., 0.5", styleSheet="font-size: 10.5pt;")
        self.wheelMassLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Mass:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelMassLineEdit)

        self.wheelMaterialLineEdit = QLineEdit(placeholderText="e.g., Black", styleSheet="font-size: 10.5pt;")
        self.wheelMaterialLineEdit.setFixedWidth(200)
        self.param_layout.addWidget(QLabel("Wheel Material:", styleSheet="font-size: 11pt; font-weight: bold;"))
        self.param_layout.addWidget(self.wheelMaterialLineEdit)

    def applyChanges(self):
        chassis_size_str = self.chassisSizeLineEdit.text()
        try:
            L, W, H = map(float, chassis_size_str.split())
            #logging.debug(f"Chassis size parsed: L={L}, W={W}, H={H}")
        except ValueError:
            L, W, H = 1.2, 0.8, 0.3
            chassis_size_str = "1.2 0.8 0.3"
            logging.warning("Invalid chassis size input, using defaults: 1.2 0.8 0.3")

        camera_size_str = self.cameraSizeLineEdit.text()
        try:
            Lc, Wc, Hc = map(float, camera_size_str.split())
            #logging.debug(f"Camera size parsed: Lc={Lc}, Wc={Wc}, Hc={Hc}")
        except ValueError:
            Lc, Wc, Hc = 0.08, 0.2, 0.08
            camera_size_str = "0.08 0.2 0.08"
            logging.warning("Invalid camera size input, using defaults: 0.08 0.2 0.08")

        params = {
            "chassis_size": chassis_size_str,
            "chassis_mass": self.chassisMassLineEdit.text() or "1.0",
            "chassis_material": self.chassisMaterialLineEdit.text() or "Gray",
            "lidar_radius": self.lidarRadiusLineEdit.text() or "0.1",
            "lidar_height": self.lidarHeightLineEdit.text() or "0.08",
            "lidar_mass": self.lidarMassLineEdit.text() or "0.2",
            "lidar_material": self.lidarMaterialLineEdit.text() or "Red",
            "camera_size": camera_size_str,
            "camera_mass": self.cameraMassLineEdit.text() or "0.1",
            "camera_material": self.cameraMaterialLineEdit.text() or "Blue",
        }

        if self.robot_type in ["4_wheeled", "3_wheeled", "2_wheeled_caster"]:
            params["wheel_radius"] = self.wheelRadiusLineEdit.text() or "0.22"
            params["wheel_width"] = self.wheelWidthLineEdit.text() or "0.12"
            params["wheel_mass"] = self.wheelMassLineEdit.text() or "0.5"
            params["wheel_material"] = self.wheelMaterialLineEdit.text() or "Black"

        if self.robot_type == "2_wheeled_caster":
            params["caster_radius"] = params["wheel_radius"]  # Enforce same value
            caster_radius = float(params["caster_radius"])

        wheel_width = float(params["wheel_width"])
        wheel_radius = float(params["wheel_radius"])
        lidar_height = float(params["lidar_height"])
        
        if self.robot_type == "4_wheeled":
            params["fl_x"] = str(L / 2 - wheel_radius / 1.5)
            params["fl_y"] = str(W / 2 + wheel_width / 2)
            params["fl_z"] = str(-H / 2)
            params["fr_x"] = str(L / 2 - wheel_radius / 1.5)
            params["fr_y"] = str(-W / 2 - wheel_width / 2)
            params["fr_z"] = str(-H / 2)
            params["rl_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rl_y"] = str(W / 2 + wheel_width / 2)
            params["rl_z"] = str(-H / 2)
            params["rr_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rr_y"] = str(-W / 2 - wheel_width / 2)
            params["rr_z"] = str(-H / 2)
        elif self.robot_type == "3_wheeled":
            params["front_x"] = str(L / 2)
            params["front_y"] = "0"
            params["front_z"] = str(-H / 2)
            params["rl_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rl_y"] = str(W / 2 + wheel_width / 2)
            params["rl_z"] = str(-H / 2)
            params["rr_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rr_y"] = str(-W / 2 - wheel_width / 2)
            params["rr_z"] = str(-H / 2)
        elif self.robot_type == "2_wheeled_caster":
            params["l_x"] = str(-L / 4)
            params["l_y"] = str(W / 2 + wheel_width / 2)
            params["l_z"] = str(-H / 2)
            params["r_x"] = str(-L / 4)
            params["r_y"] = str(-W / 2 - wheel_width / 2)
            params["r_z"] = str(-H / 2)
            params["caster_x"] = str(L / 2 - caster_radius)
            params["caster_y"] = "0"
            params["caster_z"] = str(-H / 2)
        else:
            logging.error(f"Invalid robot_type: {self.robot_type}, using default 4-wheeled parameters")
            params["fl_x"] = str(L / 2)
            params["fl_y"] = str(W / 2 + wheel_width / 2)
            params["fl_z"] = str(-H / 2)
            params["fr_x"] = str(L / 2)
            params["fr_y"] = str(-W / 2 - wheel_width / 2)
            params["fr_z"] = str(-H / 2)
            params["rl_x"] = str(-L / 2)
            params["rl_y"] = str(W / 2 + wheel_width / 2)
            params["rl_z"] = str(-H / 2)
            params["rr_x"] = str(-L / 2)
            params["rr_y"] = str(-W / 2 - wheel_width / 2)
            params["rr_z"] = str(-H / 2)

        params["lidar_z"] = str(H / 2 + lidar_height / 2)
        params["camera_x"] = str(L / 2 + Lc / 2)
        if self.robot_type == "3_wheeled":
            params["camera_z"] = str(H / 2 - Hc / 2)

        urdf_text = self.urdf_manager.generate_urdf(self.robot_type, params)
        self.previewTextEdit.setPlainText(urdf_text)
        #logging.debug("URDF text updated in preview")

        caster_radius = params["wheel_radius"] if self.robot_type == "2_wheeled_caster" else None
        self.modelUpdated.emit(
            L, W, H,
            params["wheel_radius"],
            params["wheel_width"],
            params["lidar_radius"],
            params["lidar_height"],
            params["camera_size"],
            get_color(params["chassis_material"]),
            get_color(params["wheel_material"]),
            get_color(params["lidar_material"]),
            get_color(params["camera_material"]),
            self.robot_type,
            caster_radius
        )
        #logging.debug(f"Emitted modelUpdated signal for {self.robot_type}")
        self.glWidget.update()  # Force immediate repaint of OpenGL widget

    def saveURDF(self):
        # First, save to the default location
        try:
            self.urdf_manager.save_urdf(self.default_save_path)
            #logging.debug(f"URDF saved to default location: {self.default_save_path}")
        except Exception as e:
            logging.error(f"Failed to save URDF to default location {self.default_save_path}: {str(e)}")
            return  # Exit if default save fails

        # Then, open file explorer for optional additional save
        filename, _ = QFileDialog.getSaveFileName(self, "Save URDF", self.default_save_path, "URDF Files (*.urdf)")
        if filename:
            try:
                self.urdf_manager.save_urdf(filename)
                #logging.debug(f"URDF saved to: {filename}")
            except Exception as e:
                logging.error(f"Failed to save URDF to {filename}: {str(e)}")