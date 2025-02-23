import os
import logging
from PyQt5.QtWidgets import (QWizardPage, QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QFrame, QWidget)
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

        # Image directory
        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images")
        #logging.debug(f"Image directory set to: {self.image_dir}")

        # Define the three robot options with styled buttons
        self.btn_4w = QPushButton("4-Wheeled Robot")
        self.btn_3w = QPushButton("3-Wheeled Robot (Tricycle)")
        self.btn_2wc = QPushButton("2-Wheeled Robot with Caster")

        # Base button style (unselected state)
        self.base_button_style = """
            QPushButton {
                background-color: #4A90E2;  /* Bright blue */
                color: white;
                font-size: 16pt;
                font-weight: bold;
                font-family: "Segoe UI";
                padding: 10px;
                border-radius: 8px;
                border: 2px solid #357ABD; /* Darker blue border */
            }
            QPushButton:hover {
                background-color: #66B3FF; /* Lighter blue on hover */
            }
            QPushButton:pressed {
                background-color: #2E7DB2; /* Darker blue when pressed */
            }
        """
        # Selected button style (green)
        self.selected_button_style = """
            QPushButton {
                background-color: #28A745;  /* Green */
                color: white;
                font-size: 18pt;
                font-weight: bold;
                font-family: "Segoe UI";
                padding: 10px;
                border-radius: 8px;
                border: 2px solid #218838; /* Darker green border */
            }
            QPushButton:hover {
                background-color: #34C759; /* Lighter green on hover */
            }
            QPushButton:pressed {
                background-color: #1E7E34; /* Darker green when pressed */
            }
        """

        # Apply initial base style to all buttons
        self.btn_4w.setStyleSheet(self.base_button_style)
        self.btn_3w.setStyleSheet(self.base_button_style)
        self.btn_2wc.setStyleSheet(self.base_button_style)

        # Create framed labels for each robot type's image
        self.label_4w = QLabel()
        self.label_4w.setFixedSize(400, 350)
        self.load_image(self.label_4w, os.path.join(self.image_dir, "4w_preview.png"))

        self.label_3w = QLabel()
        self.label_3w.setFixedSize(400, 350)
        self.load_image(self.label_3w, os.path.join(self.image_dir, "3w_preview.png"))

        self.label_2wc = QLabel()
        self.label_2wc.setFixedSize(400, 350)
        self.load_image(self.label_2wc, os.path.join(self.image_dir, "2wc_preview.png"))

        # Style image labels with a subtle frame
        image_style = """
            QLabel {
                border: 1px solid #CCCCCC;  /* Light gray border */
                border-radius: 5px;         /* Slightly rounded */
                background-color: #F5F5F5;  /* Light gray background */
                padding: 5px;               /* Inner padding for frame effect */
            }
        """
        self.label_4w.setStyleSheet(image_style)
        self.label_3w.setStyleSheet(image_style)
        self.label_2wc.setStyleSheet(image_style)

        # Connect buttons to set robot type
        self.btn_4w.clicked.connect(lambda: self.set_robot_type("4_wheeled"))
        self.btn_3w.clicked.connect(lambda: self.set_robot_type("3_wheeled"))
        self.btn_2wc.clicked.connect(lambda: self.set_robot_type("2_wheeled_caster"))

        # Main layout: Three horizontal sections with frames
        main_layout = QHBoxLayout()
        main_layout.setSpacing(20)  # Space between sections

        # Section 1: 4-Wheeled Robot
        section_4w = QVBoxLayout()
        section_4w.addStretch(1)  # Stretch above to center content vertically
        section_4w.addWidget(self.btn_4w, alignment=Qt.AlignCenter)
        section_4w.addSpacing(80)  # Adjustable distance between name and image
        section_4w.addWidget(self.label_4w, alignment=Qt.AlignCenter)
        section_4w.addStretch(1)  # Stretch below to center content vertically
        
        frame_4w = QFrame()
        frame_4w.setLayout(section_4w)
        frame_4w.setStyleSheet("QFrame { background-color: #FFFFFF; border: 1px solid #E0E0E0; border-radius: 10px; padding: 10px; }")
        main_layout.addWidget(frame_4w)

        # Section 2: 3-Wheeled Robot (Tricycle)
        section_3w = QVBoxLayout()
        section_3w.addStretch(1)
        section_3w.addWidget(self.btn_3w, alignment=Qt.AlignCenter)
        section_3w.addSpacing(80)
        section_3w.addWidget(self.label_3w, alignment=Qt.AlignCenter)
        section_3w.addStretch(1)
        
        frame_3w = QFrame()
        frame_3w.setLayout(section_3w)
        frame_3w.setStyleSheet("QFrame { background-color: #FFFFFF; border: 1px solid #E0E0E0; border-radius: 10px; padding: 10px; }")
        main_layout.addWidget(frame_3w)

        # Section 3: 2-Wheeled Robot with Caster
        section_2wc = QVBoxLayout()
        section_2wc.addStretch(1)
        section_2wc.addWidget(self.btn_2wc, alignment=Qt.AlignCenter)
        section_2wc.addSpacing(80)
        section_2wc.addWidget(self.label_2wc, alignment=Qt.AlignCenter)
        section_2wc.addStretch(1)
        
        frame_2wc = QFrame()
        frame_2wc.setLayout(section_2wc)
        frame_2wc.setStyleSheet("QFrame { background-color: #FFFFFF; border: 1px solid #E0E0E0; border-radius: 10px; padding: 10px; }")
        main_layout.addWidget(frame_2wc)

        # Set the main layout with background styling
        main_widget = QWidget()
        main_widget.setLayout(main_layout)
        main_widget.setStyleSheet("background-color: #F0F4F8;")  # Light blue-gray background for the page
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(main_widget)

    def set_robot_type(self, value):
        """Set the robot type and update button styles."""
        self.setField("robot_type", value)  # Update the wizard's field
        self._robot_type = value  # Sync local variable
        self.robotTypeChanged.emit()  # Emit custom signal
        self.completeChanged.emit()  # Update Next button state
        #logging.debug(f"Set robot_type to: {self.field('robot_type')}")

        # Reset all buttons to base style
        self.btn_4w.setStyleSheet(self.base_button_style)
        self.btn_3w.setStyleSheet(self.base_button_style)
        self.btn_2wc.setStyleSheet(self.base_button_style)

        # Set the selected button to green
        if value == "4_wheeled":
            self.btn_4w.setStyleSheet(self.selected_button_style)
        elif value == "3_wheeled":
            self.btn_3w.setStyleSheet(self.selected_button_style)
        elif value == "2_wheeled_caster":
            self.btn_2wc.setStyleSheet(self.selected_button_style)

    def robotType(self):
        return self._robot_type

    def setRobotType(self, value):
        self._robot_type = value
        self.robotTypeChanged.emit()

    def load_image(self, label, image_path):
        """Load an image into a QLabel with error handling."""
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            label.setPixmap(pixmap.scaled(label.size(), Qt.KeepAspectRatio))
            #logging.debug(f"Loaded image: {image_path}")
        else:
            label.setText("Image Not Found")
            logging.warning(f"Failed to load image: {image_path}")

    def isComplete(self):
        return self._robot_type is not None