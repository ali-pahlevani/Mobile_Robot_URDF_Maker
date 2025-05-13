import os
import logging
from PyQt5.QtWidgets import (QWizardPage, QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QFrame, QWidget)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from ament_index_python.packages import get_package_share_directory

class ControlConfigurationPage(QWizardPage):
    controllerTypeChanged = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Select Controller Type")
        self.registerField("controllerType*", self, property="controllerType", changedSignal=self.controllerTypeChanged)
        self._controller_type = None
        self._robot_type = None

        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images", "control_types")
        #logging.debug(f"Control image directory set to: {self.image_dir}")

        self.controllers = [
            ("Differential (2WC)", "diff_2wc", "2_wheeled_caster"),
            ("Tricycle", "tricycle", "3_wheeled"),
            ("Tricycle Steer", "triSteer", "3_wheeled"),
            ("Differential (4W)", "diff_4w", "4_wheeled"),
            ("Mecanum", "mecanum", "4_wheeled"),
            ("Ackermann", "ackermann", "4_wheeled"),
        ]

        self.buttons = {}
        self.labels = {}
        self.base_button_style = """
            QPushButton {
                background-color: #4A90E2;
                color: white;
                font-size: 14pt;
                font-weight: bold;
                font-family: "Segoe UI";
                padding: 8px;
                border-radius: 6px;
                border: 2px solid #357ABD;
            }
            QPushButton:hover {
                background-color: #66B3FF;
            }
            QPushButton:pressed {
                background-color: #2E7DB2;
            }
        """
        self.selected_button_style = """
            QPushButton {
                background-color: #28A745;
                color: white;
                font-size: 16pt;
                font-weight: bold;
                font-family: "Segoe UI";
                padding: 8px;
                border-radius: 6px;
                border: 2px solid #218838;
            }
            QPushButton:hover {
                background-color: #34C759;
            }
            QPushButton:pressed {
                background-color: #1E7E34;
            }
        """
        self.disabled_button_style = """
            QPushButton {
                background-color: #666666;
                color: #AAAAAA;
                font-size: 14pt;
                font-weight: bold;
                font-family: "Segoe UI";
                padding: 8px;
                border-radius: 6px;
                border: 2px solid #555555;
            }
        """
        self.image_style = """
            QLabel {
                border: 1px solid #CCCCCC;
                border-radius: 5px;
                background-color: #F5F5F5;
                padding: 5px;
            }
        """
        self.disabled_image_style = """
            QLabel {
                border: 1px solid #999999;
                border-radius: 5px;
                background-color: #D3D3D3;
                padding: 5px;
                opacity: 0.5;
            }
        """

        main_layout = QVBoxLayout()
        row1_layout = QHBoxLayout()
        row2_layout = QHBoxLayout()

        for i, (name, controller_id, robot_type) in enumerate(self.controllers):
            btn = QPushButton(name)
            self.buttons[controller_id] = btn
            btn.setStyleSheet(self.base_button_style)
            label = QLabel()
            label.setFixedSize(200, 250)
            self.load_image(label, os.path.join(self.image_dir, f"{controller_id}.png"))
            label.setStyleSheet(self.image_style)
            self.labels[controller_id] = label

            section_layout = QVBoxLayout()
            section_layout.addStretch(1)
            section_layout.addWidget(btn, alignment=Qt.AlignCenter)
            section_layout.addSpacing(30)
            section_layout.addWidget(label, alignment=Qt.AlignCenter)
            section_layout.addStretch(1)

            frame = QFrame()
            frame.setLayout(section_layout)
            frame.setStyleSheet("QFrame { background-color: #FFFFFF; border: 1px solid #E0E0E0; border-radius: 10px; padding: 32px; }")

            if i < 3:
                row1_layout.addWidget(frame)
            else:
                row2_layout.addWidget(frame)

            btn.clicked.connect(lambda _, cid=controller_id: self.set_controller_type(cid))

        row1_layout.setSpacing(20)
        row2_layout.setSpacing(20)
        main_layout.addLayout(row1_layout)
        main_layout.addLayout(row2_layout)
        main_layout.addStretch()

        main_widget = QWidget()
        main_widget.setLayout(main_layout)
        main_widget.setStyleSheet("background-color: #F0F4F8;")
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(main_widget)

    def initializePage(self):
        self._robot_type = self.field("robotType")
        value = self.field("controllerType")
        if value is not None and value != self._controller_type:
            self._controller_type = value
            self.set_controller_type(value)  # Update button styles
        #logging.debug(f"ControlConfigurationPage initialized with robotType: {self._robot_type}, controllerType: {self._controller_type}")
        self.update_controller_availability()

    def update_controller_availability(self):
        for controller_id, btn in self.buttons.items():
            controller_robot_type = next(c[2] for c in self.controllers if c[1] == controller_id)
            is_enabled = controller_robot_type == self._robot_type
            btn.setEnabled(is_enabled)
            btn.setStyleSheet(self.disabled_button_style if not is_enabled else self.base_button_style)
            self.labels[controller_id].setStyleSheet(self.disabled_image_style if not is_enabled else self.image_style)

    def set_controller_type(self, value):
        if not self.buttons[value].isEnabled():
            return
        self.setField("controllerType", value)
        self._controller_type = value
        self.controllerTypeChanged.emit()
        if self._controller_type != value:
            self.completeChanged.emit()  # Update Next button state only if changed
        #logging.debug(f"Set controllerType to: {self.field('controllerType')}")

        for controller_id, btn in self.buttons.items():
            if btn.isEnabled():
                btn.setStyleSheet(self.base_button_style if controller_id != value else self.selected_button_style)
            else:
                btn.setStyleSheet(self.disabled_button_style)

    def controllerType(self):
        return self._controller_type

    def setControllerType(self, value):
        #logging.debug(f"QWizard setting controllerType to: {value}")
        self._controller_type = value
        self.controllerTypeChanged.emit()

    def load_image(self, label, image_path):
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            label.setPixmap(pixmap.scaled(label.size(), Qt.KeepAspectRatio))
            #logging.debug(f"Loaded control image: {image_path}")
        else:
            label.setText("Image Not Found")
            #logging.warning(f"Failed to load control image: {image_path}")

    def isComplete(self):
        return self._controller_type is not None