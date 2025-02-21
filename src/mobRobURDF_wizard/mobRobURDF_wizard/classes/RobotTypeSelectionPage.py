from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QHBoxLayout, QLabel, QPushButton)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

# Robot Type Selection Page
class RobotTypeSelectionPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Select Robot Type")
        self.registerField("robot_type*", self)

        self.btn_4w = QPushButton("4-Wheeled Robot")
        self.btn_3w = QPushButton("3-Wheeled Robot (Tricycle)")
        self.btn_2wc = QPushButton("2-Wheeled Robot with Caster")

        self.preview_label = QLabel()
        self.preview_label.setFixedSize(300, 300)

        self.btn_4w.enterEvent = lambda event: self.show_preview("../../images/4w_preview.png")
        self.btn_3w.enterEvent = lambda event: self.show_preview("../../images/3w_preview.png")
        self.btn_2wc.enterEvent = lambda event: self.show_preview("../../images/2wc_preview.png")

        self.btn_4w.clicked.connect(lambda: self.setField("robot_type", "4_wheeled"))
        self.btn_3w.clicked.connect(lambda: self.setField("robot_type", "3_wheeled"))
        self.btn_2wc.clicked.connect(lambda: self.setField("robot_type", "2_wheeled_caster"))

        robot_layout = QVBoxLayout()
        robot_layout.addWidget(self.btn_4w)
        robot_layout.addWidget(self.btn_3w)
        robot_layout.addWidget(self.btn_2wc)
        robot_layout.addStretch()

        main_layout = QHBoxLayout()
        main_layout.addLayout(robot_layout)
        main_layout.addWidget(self.preview_label)
        self.setLayout(main_layout)

    def show_preview(self, image_path):
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            self.preview_label.setPixmap(pixmap.scaled(self.preview_label.size(), Qt.KeepAspectRatio))
        else:
            self.preview_label.setText("Preview Image Not Found")

    def isComplete(self):
        return self.field("robot_type") is not None