"""Start page — choose New Project or Load Saved Session."""

import os
import logging
from PyQt5.QtWidgets import (
    QWizardPage, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QMessageBox, QSizePolicy,
)
from PyQt5.QtCore import Qt

from mobRobURDF_wizard.classes.pages.FinalCheckPage import load_session, DEFAULT_SESSION_DIR

logger = logging.getLogger(__name__)

_CARD_BASE = """
    QPushButton {{
        background-color: {bg};
        border: {border_w}px solid {border};
        border-radius: 16px;
        color: {fg};
        font-size: 13pt;
        font-weight: bold;
        padding: 20px 12px;
        text-align: center;
    }}
    QPushButton:hover  {{ background-color: {hover}; border-color: {hborder}; }}
    QPushButton:pressed {{ background-color: {press}; }}
"""


def _card_qss(bg, border, fg, hover, hborder, press, selected=False):
    return _CARD_BASE.format(
        bg=bg, border=border, fg=fg,
        hover=hover, hborder=hborder, press=press,
        border_w=4 if selected else 2,
    )


class StartSessionPage(QWizardPage):
    def __init__(self, urdf_manager, parent=None):
        super().__init__(parent)
        self.urdf_manager = urdf_manager
        self._mode = "new"
        self._session_data = None

        self.setTitle("Mobile Robot URDF Maker")
        self.setSubTitle("How would you like to start?")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(48, 24, 48, 24)
        layout.setSpacing(20)
        layout.addStretch(1)

        row = QHBoxLayout()
        row.setSpacing(32)

        self._new_btn = QPushButton(
            "➕  Start New Project\n\n"
            "Begin fresh with a new\nrobot configuration"
        )
        self._new_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._new_btn.clicked.connect(self._select_new)
        row.addWidget(self._new_btn)

        self._load_btn = QPushButton(
            "📂  Load Saved Session\n\n"
            "Resume exactly where\nyou left off"
        )
        self._load_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._load_btn.clicked.connect(self._select_load)
        row.addWidget(self._load_btn)

        layout.addLayout(row, 5)

        self._info = QLabel("")
        self._info.setAlignment(Qt.AlignCenter)
        self._info.setStyleSheet("font-size: 9pt; color: #7F8C8D;")
        self._info.setWordWrap(True)
        layout.addWidget(self._info)

        layout.addStretch(1)

        self._refresh_styles()

    # ── Card selection ────────────────────────────────────────────────────────

    def _select_new(self):
        self._mode = "new"
        self._session_data = None
        self._info.setText("")
        self._refresh_styles()
        self.completeChanged.emit()

    def _select_load(self):
        start_dir = (DEFAULT_SESSION_DIR if os.path.isdir(DEFAULT_SESSION_DIR)
                     else os.path.expanduser("~"))
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Session", start_dir,
            "URDF Maker Session (*.mobsession);;All Files (*)",
        )
        if not path:
            return
        try:
            data = load_session(path)
        except Exception as exc:
            QMessageBox.warning(self, "Load Failed", f"Could not load session:\n{exc}")
            return

        self._mode = "load"
        self._session_data = data
        fname = os.path.basename(path)
        rt = data.get("robot_type", "?")
        ct = data.get("controller_type", "?")
        self._info.setText(
            f"Session loaded:  {fname}   ({rt.replace('_', ' ')} / {ct.replace('_', ' ')})"
        )
        self._refresh_styles()
        self.completeChanged.emit()

    def _refresh_styles(self):
        new_sel = self._mode == "new"
        load_sel = self._mode == "load"

        self._new_btn.setStyleSheet(_card_qss(
            bg="#EBF5FB", border="#5DADE2", fg="#1A5276",
            hover="#D6EAF8", hborder="#2980B9", press="#AED6F1",
            selected=new_sel,
        ))
        self._load_btn.setStyleSheet(_card_qss(
            bg="#EAFAF1", border="#58D68D", fg="#1E8449",
            hover="#D5F5E3", hborder="#27AE60", press="#A9DFBF",
            selected=load_sel,
        ))

    # ── Wizard page lifecycle ─────────────────────────────────────────────────

    def isComplete(self):
        return True  # always complete — either new or (load + data ready)

    def validatePage(self):
        if self._mode == "load":
            if self._session_data is None:
                QMessageBox.warning(self, "No session",
                                    "Please load a session file first.")
                return False
            # Pass session data to ConfigurationPage via the shared URDFManager slot.
            self.urdf_manager.pending_restore = self._session_data
        else:
            self.urdf_manager.pending_restore = None
        return True

    def nextId(self):
        ids = self.wizard().pageIds()
        if self._mode == "load" and self._session_data is not None:
            # Skip RobotType (idx 2) and ControlConfig (idx 3); go to ConfigurationPage (idx 4).
            return ids[4]
        # Normal flow: go to RobotType selection (idx 2).
        return ids[2]
