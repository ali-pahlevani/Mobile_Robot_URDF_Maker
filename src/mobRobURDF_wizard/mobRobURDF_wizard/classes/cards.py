"""Selection and feature card widgets used on the robot-type and future-features pages."""

import os
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QSizePolicy
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

from mobRobURDF_wizard.classes.responsive_widgets import ScaledPixmapLabel
from mobRobURDF_wizard.utils import style


def _image_label(image_path):
    label = ScaledPixmapLabel()
    if os.path.exists(image_path):
        label.setSourcePixmap(QPixmap(image_path))
    else:
        label.setText("Image not found")
        label.setStyleSheet("color: #95A5A6; border: none; background: transparent;")
    return label


class SelectionCard(QWidget):
    """Image + title + button card whose appearance reflects selected/enabled state."""

    def __init__(self, image_path, title_text, button_text, badge=None, parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        self.setStyleSheet(style.card_qss(selected=False))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMaximumHeight(460)

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(18, 14, 18, 14)
        vbox.setSpacing(10)

        self.image_label = _image_label(image_path)
        vbox.addWidget(self.image_label, 1)

        self.title_label = QLabel(title_text)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setWordWrap(True)
        self.title_label.setStyleSheet(
            "font-size: 14pt; font-weight: bold; color: #2C3E50; "
            "background-color: transparent; border: none;"
        )
        vbox.addWidget(self.title_label)

        if badge:
            badge_label = QLabel(badge)
            badge_label.setAlignment(Qt.AlignCenter)
            badge_label.setStyleSheet(
                "font-size: 11pt; color: #27AE60; "
                "background-color: transparent; border: none;"
            )
            vbox.addWidget(badge_label)

        self.button = QPushButton(button_text)
        self.button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        vbox.addWidget(self.button)

    def set_selected(self, selected):
        self.setStyleSheet(style.card_qss(selected=selected))

    def set_enabled(self, enabled):
        self.button.setEnabled(enabled)
        # dim the card when this controller doesn't match the chosen robot type
        self.setStyleSheet(style.card_qss(selected=False))
        self.setGraphicsEffect(None)
        self.setProperty("disabledCard", not enabled)
        self.title_label.setStyleSheet(
            "font-size: 14pt; font-weight: bold; "
            f"color: {'#9AA7B2' if not enabled else '#2C3E50'}; "
            "background-color: transparent; border: none;"
        )


class FeatureCard(QWidget):
    """Image + title + colored badge card (no button) for the Future Features page."""

    def __init__(self, image_path, title_text, badge=None, parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        self.setStyleSheet(style.card_qss(selected=False))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMaximumHeight(420)

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(18, 14, 18, 14)
        vbox.setSpacing(10)

        vbox.addWidget(_image_label(image_path), 1)

        title = QLabel(title_text)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size: 13pt; font-weight: bold; color: #2C3E50; "
            "background-color: transparent; border: none;"
        )
        vbox.addWidget(title)

        if badge:
            b = QLabel(badge)
            b.setAlignment(Qt.AlignCenter)
            b.setStyleSheet(
                "font-size: 9pt; color: #FFFFFF; background-color: #E67E22; "
                "border-radius: 4px; padding: 3px 10px; border: none;"
            )
            vbox.addWidget(b, alignment=Qt.AlignCenter)
