import os
from PyQt5.QtWidgets import QWizardPage, QPushButton, QLabel, QHBoxLayout, QVBoxLayout
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap

class RobotTypeSelectionPage(QWizardPage):
    robotTypeChanged = pyqtSignal()  # Define the custom signal

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Select Robot Type")

        # Register the field with a property and signal
        self.registerField("robot_type*", self, property="robotType", changedSignal=self.robotTypeChanged)
        self._robot_type = None  # Internal storage for the field

        self.btn_4w = QPushButton("4-Wheeled Robot")
        self.btn_3w = QPushButton("3-Wheeled Robot (Tricycle)")
        self.btn_2wc = QPushButton("2-Wheeled Robot with Caster")

        self.preview_label = QLabel()
        self.preview_label.setFixedSize(300, 300)
        
        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images")

        self.btn_4w.enterEvent = lambda event: self.show_preview(os.path.join(self.image_dir, "4w_preview.png"))
        self.btn_3w.enterEvent = lambda event: self.show_preview(os.path.join(self.image_dir, "3w_preview.png"))
        self.btn_2wc.enterEvent = lambda event: self.show_preview(os.path.join(self.image_dir, "2wc_preview.png"))

        self.btn_4w.clicked.connect(lambda: self.set_robot_type("4_wheeled"))
        self.btn_3w.clicked.connect(lambda: self.set_robot_type("3_wheeled"))
        self.btn_2wc.clicked.connect(lambda: self.set_robot_type("2_wheeled_caster"))

        robot_layout = QVBoxLayout()
        robot_layout.addWidget(self.btn_4w)
        robot_layout.addWidget(self.btn_3w)
        robot_layout.addWidget(self.btn_2wc)
        robot_layout.addStretch()

        main_layout = QHBoxLayout()
        main_layout.addLayout(robot_layout)
        main_layout.addWidget(self.preview_label)
        self.setLayout(main_layout)

    def set_robot_type(self, value):
        self.setField("robot_type", value)  # Update the wizard's field
        self._robot_type = value  # Sync local variable
        self.robotTypeChanged.emit()  # Emit custom signal
        self.completeChanged.emit()  # Update Next button state
        print(f"Set robot_type to: {self.field('robot_type')}")  # Debug output using wizard's field

    def robotType(self):
        return self._robot_type

    def setRobotType(self, value):
        self._robot_type = value
        self.robotTypeChanged.emit()

    def show_preview(self, image_path):
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            self.preview_label.setPixmap(pixmap.scaled(self.preview_label.size(), Qt.KeepAspectRatio))
        else:
            self.preview_label.setText("Preview Image Not Found")

    def isComplete(self):
        return self._robot_type is not None