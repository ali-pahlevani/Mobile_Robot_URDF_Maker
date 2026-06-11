import os
import logging
from PyQt5.QtWidgets import QWizardPage, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ament_index_python.packages import get_package_share_directory

from mobRobURDF_wizard.classes.cards import FeatureCard

logger = logging.getLogger(__name__)


class FutureFeaturesPage(QWizardPage):
    # (title, image_basename, badge)
    _FEATURES = [
        ("SLAM", "slam.png", "In Progress"),
        ("Navigation", "navigation.png", "Planned"),
        ("Object Tracking", "object_tracking.png", "Planned"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Future Features")

        image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images", "future_features")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 20, 30, 20)
        layout.setSpacing(20)

        header = QLabel("What's next for the URDF Maker")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("font-size: 16pt; font-weight: bold; color: #2C3E50;")
        layout.addWidget(header)

        sub = QLabel("These capabilities are currently under development.")
        sub.setAlignment(Qt.AlignCenter)
        sub.setStyleSheet("font-size: 11pt; color: #7F8C8D;")
        layout.addWidget(sub)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(20)
        for title, image_name, badge in self._FEATURES:
            cards_row.addWidget(FeatureCard(os.path.join(image_dir, image_name), title, badge=badge))

        layout.addStretch(1)
        layout.addLayout(cards_row)
        layout.addStretch(1)
