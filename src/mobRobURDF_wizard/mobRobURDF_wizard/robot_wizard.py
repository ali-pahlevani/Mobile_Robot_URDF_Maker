#!/usr/bin/env python3

from .robot_input_page import RobotInputPage
from .urdf_preview_page import URDFPreviewPage
from .opengl_widget import OpenGLWidget

import sys
from PyQt5.QtWidgets import (QApplication, QWizard, QVBoxLayout, QSplitter, QWidget)
from PyQt5.QtCore import Qt
from OpenGL.GLUT import *

# Initialize GLUT
glutInit()

class RobotWizard(QWizard):
    def __init__(self):
        super(RobotWizard, self).__init__()
        self.setWindowTitle("Robot URDF Generator Wizard")
        self.resize(1200, 600)

        # Create a splitter to divide the window into two parts
        self.splitter = QSplitter(Qt.Horizontal)
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(self.splitter)

        # Left side: Input fields and URDF preview
        self.leftWidget = QWidget()
        self.leftLayout = QVBoxLayout()
        self.leftWidget.setLayout(self.leftLayout)
        self.splitter.addWidget(self.leftWidget)

        # Add the input page to the left side, passing 'self' as parent
        self.inputPage = RobotInputPage(self)
        self.leftLayout.addWidget(self.inputPage)

        # Add the URDF preview page to the left side, passing 'self' as parent
        self.previewPage = URDFPreviewPage(self)
        self.leftLayout.addWidget(self.previewPage)

        # Right side: 3D preview
        self.glWidget = OpenGLWidget()
        self.splitter.addWidget(self.glWidget)
    
def main(args=None):
    app = QApplication(sys.argv)
    wizard = RobotWizard()
    wizard.showMaximized()  # Open the wizard in maximized mode
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()