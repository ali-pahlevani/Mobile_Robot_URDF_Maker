#!/usr/bin/env python3

import sys
import logging
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QWidget, QApplication, QWizard, QListWidget)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# Custom page classes
try:
    from mobRobURDF_wizard.classes.pages.WelcomePage import WelcomePage
    from mobRobURDF_wizard.classes.pages.RobotTypeSelectionPage import RobotTypeSelectionPage
    from mobRobURDF_wizard.classes.pages.FutureFeaturesPage import FutureFeaturesPage
    from mobRobURDF_wizard.classes.pages.ConfigurationPage import ConfigurationPage
    from mobRobURDF_wizard.classes.URDFManager import URDFManager
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit(1)

class RobotWizard(QWizard):
    def __init__(self):
        super().__init__()
        # Add window flags to include minimize, maximize, and close buttons
        self.setWindowFlags(Qt.Window | Qt.WindowMinimizeButtonHint | Qt.WindowMaximizeButtonHint | Qt.WindowCloseButtonHint)
        self.setWindowTitle("Mobile Robot URDF Generator Wizard (v2)")
        self.setFixedSize(1800, 900)

        # Initialize URDFManager
        try:
            self.urdf_manager = URDFManager()
        except Exception as e:
            logging.error(f"Failed to initialize URDFManager: {e}")
            raise

        # Navigation bar
        self.nav_list = QListWidget()
        self.nav_list.addItems(["Welcome", "Select Robot Type", "Configure Parameters", "Future Features"])
        self.nav_list.setFixedWidth(270)
        self.nav_list.setFixedHeight(500)
        self.nav_list.setStyleSheet("""
            QListWidget {
                background-color: #2E2E2E;
                color: #FFFFFF;
                border: 1px solid #555555;
                padding: 5px;
            }
            QListWidget::item {
                padding: 10px;
                font-size: 16pt;
                font-weight: bold;
            }
            QListWidget::item:selected {
                background-color: #4A90E2;
                color: #FFFFFF;
            }
            QListWidget::item:hover {
                background-color: #666666;
            }
        """)
        font = QFont("Arial", 16, QFont.Bold)
        self.nav_list.setFont(font)
        self.nav_list.setCurrentRow(0)

        # Connect navigation
        self.nav_list.itemClicked.connect(self.navigate_to_page)

        # Add pages
        try:
            self.addPage(WelcomePage())
            self.addPage(RobotTypeSelectionPage())
            self.addPage(ConfigurationPage(self.urdf_manager))
            self.addPage(FutureFeaturesPage())
        except Exception as e:
            logging.error(f"Failed to add pages: {e}")
            raise

        # Layout
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        nav_layout = QVBoxLayout()
        nav_layout.addWidget(self.nav_list)
        nav_layout.addStretch()
        main_layout.addLayout(nav_layout)
        main_layout.addStretch()
        main_widget.setLayout(main_layout)
        self.setSideWidget(main_widget)

        self.currentIdChanged.connect(self.update_navigation)
        #logging.debug("RobotWizard initialized")

    def update_navigation(self, page_id):
        page_index = self.pageIds().index(page_id)
        if self.nav_list.currentRow() != page_index:
            self.nav_list.setCurrentRow(page_index)
        #logging.debug(f"Page ID changed to: {page_id}")

    def navigate_to_page(self, item):
        """Navigate to the target page by simulating Next/Back button presses."""
        page_names = ["Welcome", "Select Robot Type", "Configure Parameters", "Future Features"]
        target_index = page_names.index(item.text())
        current_index = self.pageIds().index(self.currentId())

        # Navigate forward or backward
        while current_index < target_index:
            self.next()
            current_index += 1
        while current_index > target_index:
            self.back()
            current_index -= 1

        #logging.debug(f"Navigated to page: {item.text()} (Index: {target_index})")

def main():
    glutInit(sys.argv)
    #logging.basicConfig(level=logging.debug, format="%(asctime)s - %(levelname)s - %(message)s")
    app = QApplication(sys.argv)
    wizard = RobotWizard()
    wizard.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()