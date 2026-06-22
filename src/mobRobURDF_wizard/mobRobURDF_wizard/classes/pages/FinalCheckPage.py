"""Final Check page — shows generated URDF in an editable text editor before launch."""

import os
import json
import logging
from PyQt5.QtWidgets import (
    QWizardPage, QVBoxLayout, QHBoxLayout, QTextEdit, QLabel,
    QPushButton, QApplication, QFileDialog, QMessageBox, QSizePolicy,
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

from mobRobURDF_wizard.classes.sensor_config import sensor_to_dict

logger = logging.getLogger(__name__)

SESSION_VERSION = 1
DEFAULT_SESSION_DIR = os.path.expanduser("~/mobRobURDF_sessions")


def save_session(path: str, urdf_manager, urdf_text: str):
    """Write the full wizard state to a JSON .mobsession file."""
    from mobRobURDF_wizard.classes.sensor_config import sensor_to_dict
    sensors = [sensor_to_dict(s) for s in (urdf_manager.last_sensors or [])]
    data = {
        "format":             "mobRobURDF_session",
        "version":            SESSION_VERSION,
        "robot_type":         urdf_manager.last_robot_type or "",
        "controller_type":    urdf_manager.last_controller_type or "",
        "parameters":         dict(urdf_manager.last_params or {}),
        "sensors":            sensors,
        "tuner_params":       dict(urdf_manager.last_tuner_params or {}),
        "hardware_interface": urdf_manager.last_hardware_interface or "",
        "urdf_text":          urdf_text,
    }
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    logger.info("Session saved to %s", path)


def load_session(path: str) -> dict:
    """Load a .mobsession file and return the raw dict; raises ValueError on bad format."""
    with open(path) as f:
        data = json.load(f)
    if not isinstance(data, dict) or data.get("format") != "mobRobURDF_session":
        raise ValueError("Not a valid mobRobURDF session file.")
    return data


class FinalCheckPage(QWizardPage):
    def __init__(self, urdf_manager, parent=None):
        super().__init__(parent)
        self.urdf_manager = urdf_manager
        self._base_text = ""   # generator output; Revert restores this

        self.setTitle("Final Check — Review & Edit Generated URDF")
        self.setSubTitle(
            "You can edit the URDF directly below. "
            "Click Revert Changes to undo manual edits and restore the generated output."
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 8, 14, 10)
        layout.setSpacing(8)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(8)

        self._status = QLabel("")
        self._status.setStyleSheet("font-size: 9pt; color: #7F8C8D;")
        toolbar.addWidget(self._status, 1)

        revert_btn = QPushButton("↺  Revert Changes")
        revert_btn.setMinimumHeight(34)
        revert_btn.setMinimumWidth(140)
        revert_btn.setToolTip("Discard manual edits and restore the generated URDF")
        revert_btn.clicked.connect(self._revert)
        toolbar.addWidget(revert_btn)

        copy_btn = QPushButton("Copy to Clipboard")
        copy_btn.setMinimumHeight(34)
        copy_btn.setMinimumWidth(150)
        copy_btn.clicked.connect(self._copy)
        toolbar.addWidget(copy_btn)

        save_urdf_btn = QPushButton("Save URDF")
        save_urdf_btn.setMinimumHeight(34)
        save_urdf_btn.setMinimumWidth(110)
        save_urdf_btn.setProperty("btnRole", "success")
        save_urdf_btn.clicked.connect(self._save_urdf)
        toolbar.addWidget(save_urdf_btn)

        save_session_btn = QPushButton("Save Session")
        save_session_btn.setMinimumHeight(34)
        save_session_btn.setMinimumWidth(120)
        save_session_btn.setProperty("btnRole", "success")
        save_session_btn.setToolTip(
            "Save the complete wizard session (robot, controller, sensors, tuner params, URDF) "
            "so it can be resumed later."
        )
        save_session_btn.clicked.connect(self._save_session)
        toolbar.addWidget(save_session_btn)

        layout.addLayout(toolbar)

        self._text = QTextEdit()
        self._text.setReadOnly(False)
        self._text.setLineWrapMode(QTextEdit.NoWrap)
        self._text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._text.setFont(QFont("Monospace", 10))
        self._text.setStyleSheet("""
            QTextEdit {
                background-color: #1E1E1E;
                color: #D4D4D4;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 6px;
            }
        """)
        layout.addWidget(self._text, 1)

    def initializePage(self):
        urdf = self.urdf_manager.get_urdf_text() or ""
        self._base_text = urdf
        self._text.setPlainText(urdf)
        self._update_status(urdf)

    def _revert(self):
        self._text.setPlainText(self._base_text)
        self._update_status(self._base_text)

    def _update_status(self, urdf: str):
        if not urdf or urdf.startswith("Error"):
            self._status.setText(
                "No URDF generated yet — go back to Configure Parameters and click Apply."
            )
            self._status.setStyleSheet("font-size: 9pt; color: #E74C3C;")
            return

        lines  = urdf.count('\n') + 1
        links  = urdf.count('<link ')
        joints = urdf.count('<joint ')
        rt = self.urdf_manager.last_robot_type or '—'
        ct = self.urdf_manager.last_controller_type or '—'
        self._status.setText(
            f"Robot: {rt}   Controller: {ct}   "
            f"{lines} lines  ·  {links} links  ·  {joints} joints"
        )
        self._status.setStyleSheet("font-size: 9pt; color: #7F8C8D;")

    def _copy(self):
        text = self._text.toPlainText()
        if text:
            QApplication.clipboard().setText(text)

    def _save_urdf(self):
        urdf = self._text.toPlainText()
        if not urdf or urdf.startswith("Error"):
            QMessageBox.warning(
                self, "No URDF",
                "Generate a URDF first by clicking Apply on the Configure Parameters page."
            )
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save URDF", os.path.expanduser("~/robot.urdf"), "URDF Files (*.urdf)"
        )
        if path:
            try:
                with open(path, "w") as f:
                    f.write(urdf)
            except OSError as e:
                QMessageBox.critical(self, "Save Failed", str(e))

    def _save_session(self):
        if not self.urdf_manager.last_robot_type:
            QMessageBox.warning(
                self, "Nothing to save",
                "Complete the configuration first (go back and click Apply)."
            )
            return
        os.makedirs(DEFAULT_SESSION_DIR, exist_ok=True)
        rt = self.urdf_manager.last_robot_type or "robot"
        ct = self.urdf_manager.last_controller_type or "ctrl"
        default_name = os.path.join(DEFAULT_SESSION_DIR, f"{rt}_{ct}.mobsession")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Session",
            default_name,
            "URDF Maker Session (*.mobsession);;All Files (*)"
        )
        if not path:
            return
        try:
            save_session(path, self.urdf_manager, self._text.toPlainText())
            QMessageBox.information(
                self, "Session Saved",
                f"Session saved to:\n{path}\n\n"
                "You can load it later from the Start page."
            )
        except Exception as e:
            QMessageBox.critical(self, "Save Failed", str(e))
