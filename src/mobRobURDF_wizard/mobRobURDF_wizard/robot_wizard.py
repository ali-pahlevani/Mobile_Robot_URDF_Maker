#!/usr/bin/env python3

from mobRobURDF_wizard.classes.robot_input_page import RobotInputPage
from mobRobURDF_wizard.classes.urdf_preview_page import URDFPreviewPage
from mobRobURDF_wizard.classes.opengl_widget import OpenGLWidget

import sys
from PyQt5.QtWidgets import (QApplication, QWizard, QVBoxLayout, QSplitter, QWidget)
from PyQt5.QtCore import Qt
from OpenGL.GLUT import *

# Initialize GLUT
glutInit()

class RobotWizard(QWizard):
    def __init__(self):
        super(RobotWizard, self).__init__()
        self.setWindowTitle("Mobile Robot URDF Generator Wizard (v1)")
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
        
        # Set the sizes of the left and right side (percentages based on total space)
        self.splitter.setSizes([int(0.25 * self.width()), int(0.75 * self.width())])
    
def main(args=None):
    app = QApplication(sys.argv)
    wizard = RobotWizard()
    wizard.showMaximized()  # Open the wizard in maximized mode
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()