#!/usr/bin/env python3

import sys
import logging
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QWidget, QApplication, QWizard, QListWidget)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt  # Added for window flags
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

# Classes from your package
try:
    from mobRobURDF_wizard.classes.WelcomePage import WelcomePage
    from mobRobURDF_wizard.classes.URDFManager import URDFManager
    from mobRobURDF_wizard.classes.RobotTypeSelectionPage import RobotTypeSelectionPage
    from mobRobURDF_wizard.classes.FutureFeaturesPage import FutureFeaturesPage
    from mobRobURDF_wizard.classes.ConfigurationPage import ConfigurationPage
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit(1)

# Main Wizard Class
class RobotWizard(QWizard):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mobile Robot URDF Generator Wizard (v2)")
        self.resize(1200, 600)

        # Ensure window flags include minimize, maximize, and close buttons
        self.setWindowFlags(Qt.Window | Qt.WindowMinimizeButtonHint | Qt.WindowMaximizeButtonHint | Qt.WindowCloseButtonHint)

        try:
            self.urdf_manager = URDFManager()
        except Exception as e:
            logging.error(f"Failed to initialize URDFManager: {e}")
            raise

        # Navigation bar
        self.nav_list = QListWidget()
        self.nav_list.addItems(["Welcome", "Select Robot Type", "Configure Parameters", "Future Features"])
        
        # Customize navigation bar appearance
        self.nav_list.setFixedWidth(250)  # Adjusted width
        self.nav_list.setFixedHeight(400)  # Optional: Set a fixed height
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

        # Pages
        try:
            self.addPage(WelcomePage())
            self.addPage(RobotTypeSelectionPage())
            self.addPage(ConfigurationPage(self.urdf_manager))
            self.addPage(FutureFeaturesPage())
        except Exception as e:
            logging.error(f"Failed to add pages: {e}")
            raise

        # Layout: Integrate navigation bar with default wizard layout
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        nav_layout = QVBoxLayout()
        nav_layout.addWidget(self.nav_list)
        nav_layout.addStretch()
        
        main_layout.addLayout(nav_layout)
        main_layout.addStretch()  # Allow wizard content to expand
        
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
    # Initialize GLUT
    try:
        glutInit()
    except Exception as e:
        print(f"Failed to initialize GLUT: {e}")
        sys.exit(1)

    # Set up logging
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s")

    app = QApplication(sys.argv)
    wizard = RobotWizard()
    wizard.showMaximized()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()