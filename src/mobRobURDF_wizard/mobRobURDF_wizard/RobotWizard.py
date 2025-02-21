#!/usr/bin/env python3

import sys
import logging
from mobRobURDF_wizard.classes.WelcomePage import WelcomePage
from mobRobURDF_wizard.classes.URDFManager import URDFManager
from mobRobURDF_wizard.classes.RobotTypeSelectionPage import RobotTypeSelectionPage
from mobRobURDF_wizard.classes.FutureFeaturesPage import FutureFeaturesPage
from mobRobURDF_wizard.classes.ConfigurationPage import ConfigurationPage
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QWidget, QApplication, QWizard, QListWidget)
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# Initialize GLUT
glutInit()

# Set up logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s")

# Main Wizard Class
class RobotWizard(QWizard):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mobile Robot URDF Generator Wizard (v2)")
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

        # Layout: Integrate navigation bar with default wizard layout
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # Navigation bar layout
        nav_layout = QVBoxLayout()
        nav_layout.addWidget(self.nav_list)
        nav_layout.addStretch()
        
        # Wizard content area (default layout preserved)
        main_layout.addLayout(nav_layout)
        main_layout.addStretch()  # Allow wizard content to expand
        
        # Set the central widget with navigation bar on the left
        main_widget.setLayout(main_layout)
        self.setWizardStyle(QWizard.ModernStyle)  # Ensure buttons are visible
        self.setOption(QWizard.NoDefaultButton, False)  # Enable default buttons
        self.setSideWidget(main_widget)  # Add nav bar as a side widget

        self.currentIdChanged.connect(self.update_navigation)

    def update_navigation(self, page_id):
        page_index = self.pageIds().index(page_id)
        self.nav_list.setCurrentRow(page_index)
        logging.debug(f"Page ID changed to: {page_id}, robot_type field: {self.field('robot_type')}")

def main(args=None):
    app = QApplication(sys.argv)
    wizard = RobotWizard()
    wizard.showMaximized()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()