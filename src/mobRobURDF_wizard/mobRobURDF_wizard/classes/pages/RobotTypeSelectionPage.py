import os
import logging
from PyQt5.QtWidgets import QWizardPage, QHBoxLayout, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, pyqtSignal
from ament_index_python.packages import get_package_share_directory

from mobRobURDF_wizard.classes.cards import SelectionCard

logger = logging.getLogger(__name__)


class RobotTypeSelectionPage(QWizardPage):
    robotTypeChanged = pyqtSignal()

    # (robot_id, title, image_basename)
    _ROBOTS = [
        ("4_wheeled", "4-Wheeled Robot", "4w_preview.png"),
        ("3_wheeled", "3-Wheeled Robot (Tricycle)", "3w_preview.png"),
        ("2_wheeled_caster", "2-Wheeled Robot with Caster", "2wc_preview.png"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Select Robot Type")
        self.setSubTitle("Choose the base kinematic type for your robot. "
                         "This determines which controllers are available in the next step.")
        self.registerField("robotType*", self, property="robotType", changedSignal=self.robotTypeChanged)
        self._robot_type = None

        image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images", "robot_types")

        root = QVBoxLayout(self)
        root.setContentsMargins(30, 20, 30, 20)
        root.setSpacing(16)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(20)

        self.cards = {}
        for robot_id, title, image_name in self._ROBOTS:
            card = SelectionCard(os.path.join(image_dir, image_name), title, "Select")
            card.button.clicked.connect(lambda _=False, rid=robot_id: self.set_robot_type(rid))
            self.cards[robot_id] = card
            cards_row.addWidget(card)

        root.addStretch(1)
        root.addLayout(cards_row)
        root.addStretch(1)

        self.status_label = QLabel("")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 12pt; color: #27AE60; font-weight: bold;")
        root.addWidget(self.status_label)

    def initializePage(self):
        value = self.field("robotType")
        if value is not None and value != self._robot_type:
            self.set_robot_type(value)

    def set_robot_type(self, value):
        self.setField("robotType", value)
        self._robot_type = value
        self.robotTypeChanged.emit()
        for robot_id, card in self.cards.items():
            card.set_selected(robot_id == value)
        title = next((t for rid, t, _ in self._ROBOTS if rid == value), value)
        self.status_label.setText(f"✔  {title} selected")
        self.completeChanged.emit()

    def robotType(self):
        return self._robot_type

    def setRobotType(self, value):
        self._robot_type = value
        self.robotTypeChanged.emit()

    def isComplete(self):
        return self._robot_type is not None
