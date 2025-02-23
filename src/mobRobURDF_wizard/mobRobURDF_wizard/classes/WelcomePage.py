import os
import logging
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QLabel)
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QMovie, QFont

# Welcome Page
class WelcomePage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        #self.setTitle("Welcome to Robot Configuration Wizard")

        # Title label: Big, bold, centered
        title_label = QLabel("Welcome to the Robot Configuration Wizard!")
        title_label.setStyleSheet("""
            font-size: 28pt;          /* Larger font size */
            font-weight: bold;        /* Bold text */
            color: #2E2E2E;           /* Dark gray color */
        """)
        title_label.setFont(QFont("Segoe UI", 28, QFont.Bold))  # Modern font
        title_label.setAlignment(Qt.AlignCenter)  # Center horizontally

        # GIF label: No fixed size, adapts to window
        self.gif_label = QLabel(self)
        
        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images")
        gif_path = os.path.join(self.image_dir, "welcome.gif")
        logging.debug(f"Attempting to load GIF from: {gif_path}")

        self.movie = QMovie(gif_path)
        if self.movie.isValid():
            self.gif_label.setMovie(self.movie)
            self.movie.start()
            self.gif_label.setAlignment(Qt.AlignCenter)  # Center GIF horizontally
            self.gif_label.setScaledContents(True)  # Allow scaling with widget size
            logging.debug("GIF loaded and animation started")
        else:
            self.gif_label.setText("Welcome GIF not found")
            self.gif_label.setAlignment(Qt.AlignCenter)
            logging.warning(f"Failed to load welcome.gif from {gif_path}")

        # Layout: Stack title and GIF vertically, centered
        layout = QVBoxLayout()
        layout.addStretch(1)  # Push content toward the middle
        layout.addWidget(title_label, alignment=Qt.AlignHCenter)  # Center title horizontally
        layout.addWidget(self.gif_label, alignment=Qt.AlignHCenter)  # Center GIF horizontally
        layout.addStretch(1)  # Balance with stretch below to center vertically

        # Page background styling
        self.setStyleSheet("background-color: #F0F4F8;")  # Light blue-gray background

        self.setLayout(layout)

    def resizeEvent(self, event):
        """Adjust GIF size dynamically when the window is resized."""
        super().resizeEvent(event)
        if self.movie.isValid():
            # Scale GIF to fit within 80% of the window's smaller dimension (width or height)
            max_size = min(self.width(), self.height()) * 0.8
            self.movie.setScaledSize(self.movie.originalSize().scaled(max_size, max_size, Qt.KeepAspectRatio))
            self.gif_label.adjustSize()
        logging.debug(f"Window resized to {self.width()}x{self.height()}, GIF adjusted")
