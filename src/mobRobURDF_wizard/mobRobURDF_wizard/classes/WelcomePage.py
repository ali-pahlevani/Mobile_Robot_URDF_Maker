from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QLabel)
from PyQt5.QtGui import QMovie, QPixmap

# Welcome Page
class WelcomePage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Welcome to Robot Configuration Wizard")

        title_label = QLabel("Welcome to the Robot Configuration Wizard!")
        title_label.setStyleSheet("font-size: 18pt; font-weight: bold;")

        gif_label = QLabel(self)
        movie = QMovie("../../images/welcome.gif")
        gif_label.setMovie(movie)
        movie.start()

        layout = QVBoxLayout()
        layout.addWidget(title_label)
        layout.addWidget(gif_label)
        layout.addStretch()
        self.setLayout(layout)