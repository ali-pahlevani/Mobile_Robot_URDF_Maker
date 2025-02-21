import os
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QLabel)
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtGui import QMovie

# Welcome Page
class WelcomePage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Welcome to Robot Configuration Wizard")

        title_label = QLabel("Welcome to the Robot Configuration Wizard!")
        title_label.setStyleSheet("font-size: 18pt; font-weight: bold;")

        gif_label = QLabel(self)
        
        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images")
        movie = QMovie(os.path.join(self.image_dir, "welcome.gif"))
        
        gif_label.setMovie(movie)
        movie.start()

        layout = QVBoxLayout()
        layout.addWidget(title_label)
        layout.addWidget(gif_label)
        layout.addStretch()
        self.setLayout(layout)