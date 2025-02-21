#!/usr/bin/env python3

from mobRobURDF_wizard.classes.WelcomePage import WelcomePage
from mobRobURDF_wizard.classes.URDFManager import URDFManager
from mobRobURDF_wizard.classes.RobotTypeSelectionPage import RobotTypeSelectionPage
from mobRobURDF_wizard.classes.FutureFeaturesPage import FutureFeaturesPage
from mobRobURDF_wizard.classes.ConfigurationPage import ConfigurationPage

import sys
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QWidget, QApplication, QWizard, QListWidget)
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# Initialize GLUT
glutInit()

# Main Wizard Class
class RobotWizard(QWizard):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mobile Robot URDF Generator Wizard (v1)")
        self.resize(1200, 600)
        self.urdf_manager = URDFManager()

        # Navigation bar
        self.nav_list = QListWidget()
        self.nav_list.addItems(["Welcome", "Select Robot Type", "Configure Parameters", "Future Features"])
        self.nav_list.setFixedWidth(200)
        self.nav_list.setCurrentRow(0)

        # Pages
        self.addPage(WelcomePage())
        self.addPage(RobotTypeSelectionPage())
        self.addPage(ConfigurationPage(self.urdf_manager))
        self.addPage(FutureFeaturesPage())

        # Layout
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        nav_layout = QVBoxLayout()
        nav_layout.addWidget(self.nav_list)
        nav_layout.addStretch()

        content_widget = QWidget()
        content_layout = QVBoxLayout()
        content_widget.setLayout(content_layout)

        main_layout.addLayout(nav_layout)
        main_layout.addWidget(content_widget)
        main_widget.setLayout(main_layout)
        self.setLayout(main_layout)

        self.currentIdChanged.connect(self.update_navigation)

    def update_navigation(self, page_id):
        page_index = self.pageIds().index(page_id)
        self.nav_list.setCurrentRow(page_index)

def main(args=None):
    app = QApplication(sys.argv)
    wizard = RobotWizard()
    wizard.showMaximized()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()