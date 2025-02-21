from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QHBoxLayout, QLabel)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

# Future Features Page
class FutureFeaturesPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Future Features")

        features = [
            ("Gazebo", "../../images/gazebo.png"),
            ("Control", "../../images/control.png"),
            ("SLAM", "../../images/slam.png"),
            ("Navigation", "../../images/navigation.png"),
            ("Object Tracking", "../../images/object_tracking.png"),
        ]

        layout = QVBoxLayout()
        for feature_name, image_path in features:
            feature_label = QLabel(feature_name)
            feature_label.setStyleSheet("font-size: 14pt; font-weight: bold;")

            image_label = QLabel()
            pixmap = QPixmap(image_path)
            if not pixmap.isNull():
                image_label.setPixmap(pixmap.scaled(200, 200, Qt.KeepAspectRatio))
            else:
                image_label.setText(f"{feature_name} Image Not Found")

            feature_layout = QHBoxLayout()
            feature_layout.addWidget(feature_label)
            feature_layout.addWidget(image_label)
            layout.addLayout(feature_layout)

        layout.addStretch()
        self.setLayout(layout)