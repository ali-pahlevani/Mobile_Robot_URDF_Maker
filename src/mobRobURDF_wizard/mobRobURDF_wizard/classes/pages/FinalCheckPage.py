"""Final Check wizard page — displays the generated URDF for review before proceeding."""

import os
from PyQt5.QtWidgets import (
    QWizardPage, QVBoxLayout, QHBoxLayout, QTextEdit, QLabel,
    QPushButton, QApplication, QFileDialog, QMessageBox, QSizePolicy,
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt


class FinalCheckPage(QWizardPage):
    def __init__(self, urdf_manager, parent=None):
        super().__init__(parent)
        self.urdf_manager = urdf_manager

        self.setTitle("Final Check — Review Generated URDF")
        self.setSubTitle(
            "Verify the generated URDF before launching. "
            "Go back to the Configure Parameters page to make changes, then return here."
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 8, 14, 10)
        layout.setSpacing(8)

        # ── Toolbar row ──────────────────────────────────────────────────────
        toolbar = QHBoxLayout()
        toolbar.setSpacing(8)

        self._status = QLabel("")
        self._status.setStyleSheet("font-size: 9pt; color: #7F8C8D;")
        toolbar.addWidget(self._status, 1)

        refresh_btn = QPushButton("⟳  Refresh")
        refresh_btn.setMinimumHeight(34)
        refresh_btn.setMinimumWidth(100)
        refresh_btn.clicked.connect(self._refresh)
        toolbar.addWidget(refresh_btn)

        copy_btn = QPushButton("Copy to Clipboard")
        copy_btn.setMinimumHeight(34)
        copy_btn.setMinimumWidth(140)
        copy_btn.clicked.connect(self._copy)
        toolbar.addWidget(copy_btn)

        save_btn = QPushButton("Save URDF")
        save_btn.setMinimumHeight(34)
        save_btn.setMinimumWidth(100)
        save_btn.clicked.connect(self._save)
        toolbar.addWidget(save_btn)

        layout.addLayout(toolbar)

        # ── URDF viewer ──────────────────────────────────────────────────────
        self._text = QTextEdit()
        self._text.setReadOnly(True)
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

    # ── Wizard page lifecycle ────────────────────────────────────────────────

    def initializePage(self):
        self._refresh()

    # ── Actions ──────────────────────────────────────────────────────────────

    def _refresh(self):
        urdf = self.urdf_manager.get_urdf_text() or ""
        self._text.setPlainText(urdf)

        if not urdf or urdf.startswith("Error"):
            self._status.setText(
                "No URDF generated yet. Go back to Configure Parameters and click Apply."
            )
            self._status.setStyleSheet("font-size: 9pt; color: #E74C3C;")
            return

        lines = urdf.count('\n') + 1
        links = urdf.count('<link ')
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

    def _save(self):
        urdf = self._text.toPlainText()
        if not urdf or urdf.startswith("Error"):
            QMessageBox.warning(self, "No URDF", "Generate a URDF first by clicking Apply on the Configure Parameters page.")
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
