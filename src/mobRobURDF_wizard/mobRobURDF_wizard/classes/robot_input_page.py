from mobRobURDF_wizard.utils.utils import get_color
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QLabel, QLineEdit, QPushButton)

class RobotInputPage(QWizardPage):
    def __init__(self, parent=None):
        super(RobotInputPage, self).__init__(parent)
        self.setTitle("Robot Configuration")
        layout = QVBoxLayout()

        # Chassis size (L W H)
        self.chassisSizeLineEdit = QLineEdit()
        self.chassisSizeLineEdit.setPlaceholderText("e.g., 1 1 0.5")
        self.chassisSizeLineEdit.setStyleSheet("font-size: 10.5pt;")
        chassisSize_label = QLabel("Chassis Size (L W H):")
        chassisSize_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(chassisSize_label)
        layout.addWidget(self.chassisSizeLineEdit)

        # Chassis mass
        self.chassisMassLineEdit = QLineEdit()
        self.chassisMassLineEdit.setPlaceholderText("e.g., 1.0")
        self.chassisMassLineEdit.setStyleSheet("font-size: 10.5pt;")
        chassisMass_label = QLabel("Chassis Mass:")
        chassisMass_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(chassisMass_label)
        layout.addWidget(self.chassisMassLineEdit)

        # Chassis material
        self.chassisMaterialLineEdit = QLineEdit()
        self.chassisMaterialLineEdit.setPlaceholderText("e.g., Gray")
        self.chassisMaterialLineEdit.setStyleSheet("font-size: 10.5pt;")
        chassisMat_label = QLabel("Chassis Material:")
        chassisMat_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(chassisMat_label)
        layout.addWidget(self.chassisMaterialLineEdit)

        # Wheel radius
        self.wheelRadiusLineEdit = QLineEdit()
        self.wheelRadiusLineEdit.setPlaceholderText("e.g., 0.3")
        self.wheelRadiusLineEdit.setStyleSheet("font-size: 10.5pt;")
        wheelRad_label = QLabel("Wheel Radius:")
        wheelRad_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(wheelRad_label)
        layout.addWidget(self.wheelRadiusLineEdit)

        # Wheel width
        self.wheelWidthLineEdit = QLineEdit()
        self.wheelWidthLineEdit.setPlaceholderText("e.g., 0.1")
        self.wheelWidthLineEdit.setStyleSheet("font-size: 10.5pt;")
        wheelWid_label = QLabel("Wheel Width:")
        wheelWid_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(wheelWid_label)
        layout.addWidget(self.wheelWidthLineEdit)

        # Wheel mass
        self.wheelMassLineEdit = QLineEdit()
        self.wheelMassLineEdit.setPlaceholderText("e.g., 0.5")
        self.wheelMassLineEdit.setStyleSheet("font-size: 10.5pt;")
        wheelMass_label = QLabel("Wheel Mass:")
        wheelMass_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(wheelMass_label)
        layout.addWidget(self.wheelMassLineEdit)

        # Wheel material
        self.wheelMaterialLineEdit = QLineEdit()
        self.wheelMaterialLineEdit.setPlaceholderText("e.g., Black")
        self.wheelMaterialLineEdit.setStyleSheet("font-size: 10.5pt;")
        wheelMat_label = QLabel("Wheel Material:")
        wheelMat_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(wheelMat_label)
        layout.addWidget(self.wheelMaterialLineEdit)

        # Lidar radius
        self.lidarRadiusLineEdit = QLineEdit()
        self.lidarRadiusLineEdit.setPlaceholderText("e.g., 0.2")
        self.lidarRadiusLineEdit.setStyleSheet("font-size: 10.5pt;")
        lidarRad_label = QLabel("Lidar Radius:")
        lidarRad_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(lidarRad_label)
        layout.addWidget(self.lidarRadiusLineEdit)

        # Lidar height
        self.lidarHeightLineEdit = QLineEdit()
        self.lidarHeightLineEdit.setPlaceholderText("e.g., 0.1")
        self.lidarHeightLineEdit.setStyleSheet("font-size: 10.5pt;")
        lidarHei_label = QLabel("Lidar Height:")
        lidarHei_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(lidarHei_label)
        layout.addWidget(self.lidarHeightLineEdit)

        # Lidar mass
        self.lidarMassLineEdit = QLineEdit()
        self.lidarMassLineEdit.setPlaceholderText("e.g., 0.2")
        self.lidarMassLineEdit.setStyleSheet("font-size: 10.5pt;")
        lidarMass_label = QLabel("Lidar Mass:")
        lidarMass_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(lidarMass_label)
        layout.addWidget(self.lidarMassLineEdit)

        # Lidar material
        self.lidarMaterialLineEdit = QLineEdit()
        self.lidarMaterialLineEdit.setPlaceholderText("e.g., Red")
        self.lidarMaterialLineEdit.setStyleSheet("font-size: 10.5pt;")
        lidarMat_label = QLabel("Lidar Material:")
        lidarMat_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(lidarMat_label)
        layout.addWidget(self.lidarMaterialLineEdit)

        # Camera size (L H W)
        self.cameraSizeLineEdit = QLineEdit()
        self.cameraSizeLineEdit.setPlaceholderText("e.g., 0.1 0.1 0.1")
        self.cameraSizeLineEdit.setStyleSheet("font-size: 10.5pt;")
        cameraSize_label = QLabel("Camera Size (L W H):")
        cameraSize_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(cameraSize_label)
        layout.addWidget(self.cameraSizeLineEdit)

        # Camera mass
        self.cameraMassLineEdit = QLineEdit()
        self.cameraMassLineEdit.setPlaceholderText("e.g., 0.1")
        self.cameraMassLineEdit.setStyleSheet("font-size: 10.5pt;")
        cameraMass_label = QLabel("Camera Mass:")
        cameraMass_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(cameraMass_label)
        layout.addWidget(self.cameraMassLineEdit)

        # Camera material
        self.cameraMaterialLineEdit = QLineEdit()
        self.cameraMaterialLineEdit.setPlaceholderText("e.g., Blue")
        self.cameraMaterialLineEdit.setStyleSheet("font-size: 10.5pt;")
        cameraMat_label = QLabel("Camera Material:")
        cameraMat_label.setStyleSheet("font-size: 11pt; font-weight: bold;")
        layout.addWidget(cameraMat_label)
        layout.addWidget(self.cameraMaterialLineEdit)

        # Apply and preview button
        self.applyButton = QPushButton("Apply and Preview")
        self.applyButton.setStyleSheet("font-size: 11pt; font-weight: bold; color: red;")
        self.applyButton.setMinimumHeight(30)
        layout.addWidget(self.applyButton)
        self.setLayout(layout)
        self.applyButton.clicked.connect(self.applyChanges)

    def applyChanges(self):
        # Get and parse chassis size (expected as "L W H")
        chassis_size_str = self.chassisSizeLineEdit.text()
        try:
            L, W, H = map(float, chassis_size_str.split())
        except Exception:
            L, W, H = 1.2, 0.8, 0.3
            chassis_size_str = "1.2 0.8 0.3"
            
        # Get and parse camera size (expected as "Lc Wc Hc")
        camera_size_str = self.cameraSizeLineEdit.text()
        try:
            Lc, Wc, Hc = map(float, camera_size_str.split())
        except Exception:
            Lc, Wc, Hc = 0.08, 0.2, 0.08
            camera_size_str = "0.08 0.2 0.08"

        # Prepare parameters for substitution (for URDF generation)
        params = {
            "chassis_size": chassis_size_str,
            "chassis_mass": self.chassisMassLineEdit.text() or "1.0",
            "chassis_material": self.chassisMaterialLineEdit.text() or "Gray",
            "wheel_radius": self.wheelRadiusLineEdit.text() or "0.22",
            "wheel_width": self.wheelWidthLineEdit.text() or "0.12",
            "wheel_mass": self.wheelMassLineEdit.text() or "0.5",
            "wheel_material": self.wheelMaterialLineEdit.text() or "Black",
            "lidar_radius": self.lidarRadiusLineEdit.text() or "0.1",
            "lidar_height": self.lidarHeightLineEdit.text() or "0.08",
            "lidar_mass": self.lidarMassLineEdit.text() or "0.2",
            "lidar_material": self.lidarMaterialLineEdit.text() or "Red",
            "camera_size": camera_size_str,
            "camera_mass": self.cameraMassLineEdit.text() or "0.1",
            "camera_material": self.cameraMaterialLineEdit.text() or "Blue",
        }

        # Convert wheel width to float
        wheel_width = float(params["wheel_width"])
        
        # Convert lidar height to float
        lidar_height = float(params["lidar_height"])

        # Compute wheel positions and lidar placement for URDF
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

        # Retrieve the top-level wizard
        wizard = self.window()
        if wizard is not None:
            if hasattr(wizard, 'previewPage'):
                wizard.previewPage.updatePreview(params)
            if hasattr(wizard, 'glWidget'):
                wizard.glWidget.updateRobotModel(
                    L, W, H,
                    params["wheel_radius"],
                    params["wheel_width"],
                    params["lidar_radius"],
                    params["lidar_height"],
                    params["camera_size"],
                    get_color(params["chassis_material"]),
                    get_color(params["wheel_material"]),
                    get_color(params["lidar_material"]),
                    get_color(params["camera_material"])
                )