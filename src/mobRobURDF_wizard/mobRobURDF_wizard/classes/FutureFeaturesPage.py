import os
import logging
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QHBoxLayout, QLabel, QWidget)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from ament_index_python.packages import get_package_share_directory

# Future Features Page
class FutureFeaturesPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Future Features")

        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images")
        logging.debug(f"Image directory set to: {self.image_dir}")

        features = [
            ("Gazebo", os.path.join(self.image_dir, "gazebo.png")),
            ("Control", os.path.join(self.image_dir, "control.png")),
            ("SLAM", os.path.join(self.image_dir, "slam.png")),
            ("Navigation", os.path.join(self.image_dir, "navigation.png")),
            ("Object Tracking", os.path.join(self.image_dir, "object_tracking.png")),
        ]

        # Main layout: Vertical stack of feature rows
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)  # Space between rows

        for feature_name, image_path in features:
            # Feature name label (centered text)
            feature_label = QLabel(feature_name)
            feature_label.setStyleSheet("""
                font-size: 18pt;
                font-weight: bold;
                font-family: "Segoe UI";
                color: #2E2E2E;
                padding: 10px;
                background-color: #FFFFFF;
                border: 1px solid #E0E0E0;
                border-radius: 5px;
            """)
            feature_label.setFixedWidth(240)  # Fixed width for consistent spacing
            feature_label.setAlignment(Qt.AlignCenter)  # Center text horizontally and vertically

            # Image label (centered exactly in the row)
            image_label = QLabel()
            pixmap = QPixmap(image_path)
            if not pixmap.isNull():
                image_label.setPixmap(pixmap.scaled(250, 250, Qt.KeepAspectRatio))  # Height at 250
                logging.debug(f"Loaded image for {feature_name}: {image_path}")
            else:
                image_label.setText(f"{feature_name} Image Not Found")
                logging.warning(f"Failed to load image for {feature_name}: {image_path}")
            image_label.setStyleSheet("""
                border: 2px solid #4A90E2;
                border-radius: 10px;
                background-color: #F5F5F5;
                padding: 5px;
            """)

            # Row layout: Name at start, image centered exactly
            feature_row = QHBoxLayout()
            feature_row.addWidget(feature_label)  # Name at the start (left)
            feature_row.addStretch(2)  # Stretch to push image toward center
            feature_row.addWidget(image_label, alignment=Qt.AlignCenter)  # Image centered
            feature_row.addStretch(2)  # Stretch to balance centering
            feature_row.setSpacing(15)  # Space between name and image

            # Wrap row in a widget for styling
            row_widget = QWidget()
            row_widget.setLayout(feature_row)
            row_widget.setStyleSheet("""
                background-color: #FFFFFF;
                border: 1px solid #E0E0E0;
                border-radius: 8px;
                padding: 10px;
            """)
            # Hover effect
            row_widget.setProperty("class", "feature-row")
            row_widget.setStyleSheet(row_widget.styleSheet() + """
                .feature-row:hover {
                    background-color: #F0F4F8;
                    border: 1px solid #4A90E2;
                }
            """)

            main_layout.addWidget(row_widget)

        main_layout.addStretch()  # Center the rows vertically
        self.setLayout(main_layout)
        self.setStyleSheet("background-color: #F0F4F8;")  # Light blue-gray page background
        logging.debug("FutureFeaturesPage initialized")