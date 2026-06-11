import os
import logging
from PyQt5.QtWidgets import QWizardPage, QVBoxLayout, QGridLayout, QLabel
from PyQt5.QtCore import Qt, pyqtSignal
from ament_index_python.packages import get_package_share_directory

from mobRobURDF_wizard.classes.cards import SelectionCard

logger = logging.getLogger(__name__)


class ControlConfigurationPage(QWizardPage):
    controllerTypeChanged = pyqtSignal()

    # (title, controller_id, robot_type)
    _CONTROLLERS = [
        ("Differential (2WC)", "diff_2wc", "2_wheeled_caster"),
        ("Tricycle", "tricycle", "3_wheeled"),
        ("Tricycle Steer", "triSteer", "3_wheeled"),
        ("Differential (4W)", "diff_4w", "4_wheeled"),
        ("Mecanum", "mecanum", "4_wheeled"),
        ("Ackermann", "ackermann", "4_wheeled"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Select Controller Type")
        self.registerField("controllerType*", self, property="controllerType", changedSignal=self.controllerTypeChanged)
        self._controller_type = None
        self._robot_type = None

        image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images", "control_types")

        root = QVBoxLayout(self)
        root.setContentsMargins(30, 16, 30, 16)
        root.setSpacing(14)

        header = QLabel("Pick a controller compatible with your robot type")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("font-size: 13pt; color: #7F8C8D;")
        root.addWidget(header)

        grid = QGridLayout()
        grid.setSpacing(18)
        self.cards = {}
        self._robot_for = {}
        for i, (title, cid, robot_type) in enumerate(self._CONTROLLERS):
            card = SelectionCard(os.path.join(image_dir, f"{cid}.png"), title, "Select")
            card.button.clicked.connect(lambda _=False, c=cid: self.set_controller_type(c))
            self.cards[cid] = card
            self._robot_for[cid] = robot_type
            grid.addWidget(card, i // 3, i % 3)
        root.addLayout(grid, 1)

        self.status_label = QLabel("")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 12pt; color: #27AE60; font-weight: bold;")
        root.addWidget(self.status_label)

    def initializePage(self):
        self._robot_type = self.field("robotType")
        self.update_controller_availability()
        value = self.field("controllerType")
        if value and self._robot_for.get(value) == self._robot_type:
            self.set_controller_type(value)
        else:
            # Clear a stale selection that no longer matches the chosen robot.
            self._controller_type = None
            self.status_label.setText("")
            self.completeChanged.emit()

    def update_controller_availability(self):
        for cid, card in self.cards.items():
            enabled = self._robot_for[cid] == self._robot_type
            card.set_enabled(enabled)
            if not enabled and self._controller_type == cid:
                self._controller_type = None

    def set_controller_type(self, value):
        if not self.cards[value].button.isEnabled():
            return
        self.setField("controllerType", value)
        self._controller_type = value
        self.controllerTypeChanged.emit()
        for cid, card in self.cards.items():
            if card.button.isEnabled():
                card.set_selected(cid == value)
        title = next((t for t, c, _ in self._CONTROLLERS if c == value), value)
        self.status_label.setText(f"✔  {title} controller selected")
        self.completeChanged.emit()

    def controllerType(self):
        return self._controller_type

    def setControllerType(self, value):
        self._controller_type = value
        self.controllerTypeChanged.emit()

    def isComplete(self):
        return self._controller_type is not None
