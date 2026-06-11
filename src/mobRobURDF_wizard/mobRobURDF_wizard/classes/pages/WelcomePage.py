import os
import logging
from PyQt5.QtWidgets import QWizardPage, QVBoxLayout, QLabel, QSizePolicy
from PyQt5.QtGui import QMovie, QImageReader
from PyQt5.QtCore import Qt, QSize
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)


class WelcomePage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("")
        self._movie = None
        self._gif_w, self._gif_h = 880, 520  # fallback native size

        layout = QVBoxLayout(self)
        layout.setContentsMargins(40, 30, 40, 20)
        layout.setSpacing(12)

        title = QLabel("Welcome to the Mobile Robot URDF Maker")
        title.setAlignment(Qt.AlignCenter)
        title.setWordWrap(True)
        title.setStyleSheet("font-size: 22pt; font-weight: bold; color: #2C3E50;")
        layout.addWidget(title)

        subtitle = QLabel("Build a ready-to-simulate URDF for your mobile robot — "
                          "visually, step by step.")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("font-size: 12pt; color: #7F8C8D;")
        layout.addWidget(subtitle)

        layout.addSpacing(8)

        self.image_dir = os.path.join(get_package_share_directory("mobRobURDF_wizard"), "images")
        gif_path = os.path.join(self.image_dir, "welcome.gif")

        self._gif_label = QLabel()
        self._gif_label.setAlignment(Qt.AlignCenter)
        self._gif_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        movie = QMovie(gif_path)
        if movie.isValid():
            self._movie = movie
            # Read the GIF's native resolution so aspect ratio stays correct.
            reader = QImageReader(gif_path)
            size = reader.size()
            if size.isValid() and size.width() > 0 and size.height() > 0:
                self._gif_w, self._gif_h = size.width(), size.height()
            self._gif_label.setMovie(movie)
            movie.start()
        else:
            self._gif_label.setText("(Welcome animation not found)")
            self._gif_label.setStyleSheet("color: #95A5A6; font-size: 12pt;")
            logger.warning("Failed to load welcome.gif from %s", gif_path)
        layout.addWidget(self._gif_label, 1)

        footer = QLabel("Press  Next  to get started →")
        footer.setAlignment(Qt.AlignCenter)
        footer.setStyleSheet("font-size: 10pt; color: #95A5A6;")
        layout.addWidget(footer)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale_gif()

    def showEvent(self, event):
        super().showEvent(event)
        self._rescale_gif()

    def _rescale_gif(self):
        if not self._movie:
            return
        w, h = self._gif_label.width(), self._gif_label.height()
        if w < 10 or h < 10:
            return
        ratio = min(w / self._gif_w, h / self._gif_h)
        self._movie.setScaledSize(QSize(max(int(self._gif_w * ratio), 100),
                                        max(int(self._gif_h * ratio), 60)))
