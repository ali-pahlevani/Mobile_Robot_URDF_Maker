import os
import logging
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QLabel)
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtGui import QMovie

# Welcome Page
class WelcomePage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Welcome to Robot Configuration Wizard")

        #title_label = QLabel("Welcome to the Robot Configuration Wizard!")
        #title_label.setStyleSheet("font-size: 18pt; font-weight: bold;")

        gif_label = QLabel(self)
        
        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images")
        gif_path = os.path.join(self.image_dir, "welcome.gif")
        logging.debug(f"Attempting to load GIF from: {gif_path}")

        movie = QMovie(gif_path)
        if movie.isValid():
            gif_label.setMovie(movie)
            movie.start()
            logging.debug("GIF loaded and animation started")
        else:
            gif_label.setText("Welcome GIF not found")
            logging.warning(f"Failed to load welcome.gif from {gif_path}")

        layout = QVBoxLayout()
        #layout.addWidget(title_label)
        layout.addWidget(gif_label)
        layout.addStretch()
        self.setLayout(layout)