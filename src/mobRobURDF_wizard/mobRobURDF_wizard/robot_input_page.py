from .utils import get_color
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QLabel, QLineEdit, QPushButton)

class RobotInputPage(QWizardPage):
    def __init__(self, parent=None):
        super(RobotInputPage, self).__init__(parent)
        self.setTitle("Robot Configuration")
        layout = QVBoxLayout()

        # Chassis size (L W H)
        self.chassisSizeLineEdit = QLineEdit()
        self.chassisSizeLineEdit.setPlaceholderText("e.g., 1 1 0.5")
        layout.addWidget(QLabel("Chassis Size (L W H):"))
        layout.addWidget(self.chassisSizeLineEdit)

        # Chassis mass
        self.chassisMassLineEdit = QLineEdit()
        self.chassisMassLineEdit.setPlaceholderText("e.g., 1.0")
        layout.addWidget(QLabel("Chassis Mass:"))
        layout.addWidget(self.chassisMassLineEdit)

        # Chassis material
        self.chassisMaterialLineEdit = QLineEdit()
        self.chassisMaterialLineEdit.setPlaceholderText("e.g., Gray")
        layout.addWidget(QLabel("Chassis Material:"))
        layout.addWidget(self.chassisMaterialLineEdit)

        # Wheel radius
        self.wheelRadiusLineEdit = QLineEdit()
        self.wheelRadiusLineEdit.setPlaceholderText("e.g., 0.3")
        layout.addWidget(QLabel("Wheel Radius:"))
        layout.addWidget(self.wheelRadiusLineEdit)

        # Wheel width
        self.wheelWidthLineEdit = QLineEdit()
        self.wheelWidthLineEdit.setPlaceholderText("e.g., 0.1")
        layout.addWidget(QLabel("Wheel Width:"))
        layout.addWidget(self.wheelWidthLineEdit)

        # Wheel mass
        self.wheelMassLineEdit = QLineEdit()
        self.wheelMassLineEdit.setPlaceholderText("e.g., 0.5")
        layout.addWidget(QLabel("Wheel Mass:"))
        layout.addWidget(self.wheelMassLineEdit)

        # Wheel material
        self.wheelMaterialLineEdit = QLineEdit()
        self.wheelMaterialLineEdit.setPlaceholderText("e.g., Black")
        layout.addWidget(QLabel("Wheel Material:"))
        layout.addWidget(self.wheelMaterialLineEdit)

        # Lidar radius
        self.lidarRadiusLineEdit = QLineEdit()
        self.lidarRadiusLineEdit.setPlaceholderText("e.g., 0.2")
        layout.addWidget(QLabel("Lidar Radius:"))
        layout.addWidget(self.lidarRadiusLineEdit)

        # Lidar height
        self.lidarHeightLineEdit = QLineEdit()
        self.lidarHeightLineEdit.setPlaceholderText("e.g., 0.1")
        layout.addWidget(QLabel("Lidar Height:"))
        layout.addWidget(self.lidarHeightLineEdit)

        # Lidar mass
        self.lidarMassLineEdit = QLineEdit()
        self.lidarMassLineEdit.setPlaceholderText("e.g., 0.2")
        layout.addWidget(QLabel("Lidar Mass:"))
        layout.addWidget(self.lidarMassLineEdit)

        # Lidar material
        self.lidarMaterialLineEdit = QLineEdit()
        self.lidarMaterialLineEdit.setPlaceholderText("e.g., Red")
        layout.addWidget(QLabel("Lidar Material:"))
        layout.addWidget(self.lidarMaterialLineEdit)

        # Camera size (L H W)
        self.cameraSizeLineEdit = QLineEdit()
        self.cameraSizeLineEdit.setPlaceholderText("e.g., 0.1 0.1 0.1")
        layout.addWidget(QLabel("Camera Size (L H W):"))
        layout.addWidget(self.cameraSizeLineEdit)

        # Camera mass
        self.cameraMassLineEdit = QLineEdit()
        self.cameraMassLineEdit.setPlaceholderText("e.g., 0.1")
        layout.addWidget(QLabel("Camera Mass:"))
        layout.addWidget(self.cameraMassLineEdit)

        # Camera material
        self.cameraMaterialLineEdit = QLineEdit()
        self.cameraMaterialLineEdit.setPlaceholderText("e.g., Blue")
        layout.addWidget(QLabel("Camera Material:"))
        layout.addWidget(self.cameraMaterialLineEdit)

        # Apply and preview button
        self.applyButton = QPushButton("Apply and Preview")
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
            "camera_size": self.cameraSizeLineEdit.text() or "0.08 0.2 0.08",
            "camera_mass": self.cameraMassLineEdit.text() or "0.1",
            "camera_material": self.cameraMaterialLineEdit.text() or "Blue",
        }

        # Compute wheel positions and lidar placement for URDF
        params["fl_x"] = str(L / 2)
        params["fl_y"] = str(W / 2)
        params["fl_z"] = str(-H / 2)
        params["fr_x"] = str(L / 2)
        params["fr_y"] = str(-W / 2)
        params["fr_z"] = str(-H / 2)
        params["rl_x"] = str(-L / 2)
        params["rl_y"] = str(W / 2)
        params["rl_z"] = str(-H / 2)
        params["rr_x"] = str(-L / 2)
        params["rr_y"] = str(-W / 2)
        params["rr_z"] = str(-H / 2)
        params["lidar_z"] = str(H / 2)

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