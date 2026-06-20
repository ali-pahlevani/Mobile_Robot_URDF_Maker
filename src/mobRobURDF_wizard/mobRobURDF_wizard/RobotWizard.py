#!/usr/bin/env python3

import sys
import logging
from PyQt5.QtWidgets import (QVBoxLayout, QWidget, QApplication, QWizard, QListWidget, QLabel)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
from OpenGL.GLUT import glutInit

try:
    from mobRobURDF_wizard.classes.pages.WelcomePage import WelcomePage
    from mobRobURDF_wizard.classes.pages.StartSessionPage import StartSessionPage
    from mobRobURDF_wizard.classes.pages.RobotTypeSelectionPage import RobotTypeSelectionPage
    from mobRobURDF_wizard.classes.pages.ControlConfigurationPage import ControlConfigurationPage
    from mobRobURDF_wizard.classes.pages.ConfigurationPage import ConfigurationPage
    from mobRobURDF_wizard.classes.pages.ControllerTunerPage import ControllerTunerPage
    from mobRobURDF_wizard.classes.pages.FinalCheckPage import FinalCheckPage
    from mobRobURDF_wizard.classes.pages.TeleoperationPage import TeleoperationPage
    from mobRobURDF_wizard.classes.pages.FutureFeaturesPage import FutureFeaturesPage
    from mobRobURDF_wizard.classes.URDFManager import URDFManager
    from mobRobURDF_wizard.utils.style import (
        APP_STYLESHEET, SIDEBAR_BG, SIDEBAR_QSS, SIDEBAR_HEADER_QSS, SIDEBAR_FOOTER_QSS,
    )
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit(1)

_NAV_LABELS = ["Welcome", "Start Project", "Select Robot Type", "Select Controller",
               "Configure Parameters", "Tune Controller", "Final Check",
               "Simulate & Teleop", "Future Features"]


class RobotWizard(QWizard):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(
            Qt.Window | Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint | Qt.WindowCloseButtonHint
        )
        self.setWindowTitle("Mobile Robot URDF Maker (V4)")
        self.setWizardStyle(QWizard.ModernStyle)
        self.resize(1600, 860)
        self.setMinimumSize(1000, 600)

        try:
            self.urdf_manager = URDFManager()
        except Exception as e:
            logging.error(f"Failed to initialize URDFManager: {e}")
            raise

        sidebar = QWidget()
        sidebar.setObjectName("urdfSidebar")
        sidebar.setStyleSheet(f"QWidget#urdfSidebar {{ background-color: {SIDEBAR_BG}; }}")

        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_layout.setSpacing(0)

        header = QLabel("URDF Maker")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet(SIDEBAR_HEADER_QSS)
        sidebar_layout.addWidget(header)

        self.nav_list = QListWidget()
        self.nav_list.addItems(_NAV_LABELS)
        for i in range(self.nav_list.count()):
            self.nav_list.item(i).setTextAlignment(Qt.AlignCenter)
        self.nav_list.setFont(QFont("Arial", 11))
        self.nav_list.setStyleSheet(SIDEBAR_QSS)
        self.nav_list.setWordWrap(True)
        self.nav_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.nav_list.setCurrentRow(0)
        self.nav_list.itemClicked.connect(self.navigate_to_page)
        sidebar_layout.addWidget(self.nav_list, 1)

        version_label = QLabel("V4")
        version_label.setAlignment(Qt.AlignCenter)
        version_label.setStyleSheet(SIDEBAR_FOOTER_QSS)
        sidebar_layout.addWidget(version_label)
        self.setSideWidget(sidebar)

        try:
            self.addPage(WelcomePage())
            self.start_session_page = StartSessionPage(self.urdf_manager)
            self.addPage(self.start_session_page)
            self.addPage(RobotTypeSelectionPage())
            self.addPage(ControlConfigurationPage())
            self.config_page = ConfigurationPage(self.urdf_manager)
            self.addPage(self.config_page)
            self.tuner_page = ControllerTunerPage(self.urdf_manager)
            self.addPage(self.tuner_page)
            self.final_check_page = FinalCheckPage(self.urdf_manager)
            self.addPage(self.final_check_page)
            self.teleop_page = TeleoperationPage(self.urdf_manager)
            self.addPage(self.teleop_page)
            self.addPage(FutureFeaturesPage())
        except Exception as e:
            logging.error(f"Failed to add pages: {e}")
            raise

        self.currentIdChanged.connect(self.update_navigation)
        self.showMaximized()

    def adjustSize(self):
        # suppress Qt's auto-resize so page transitions don't jump the window
        pass

    def showEvent(self, event):
        super().showEvent(event)
        if not getattr(self, "_buttons_styled", False):
            self._buttons_styled = True
            self._style_wizard_buttons()
            self._apply_responsive_layout()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._apply_responsive_layout()

    def closeEvent(self, event):
        teleop = getattr(self, "teleop_page", None)
        if teleop is not None:
            teleop.shutdown()
        event.accept()

    def _style_wizard_buttons(self):
        buttons = [
            (self.button(QWizard.BackButton), "secondary"),
            (self.button(QWizard.NextButton), "success"),
            (self.button(QWizard.FinishButton), "success"),
            (self.button(QWizard.CancelButton), "cancel"),
        ]
        for btn, role in buttons:
            if btn:
                btn.setProperty("btnRole", role)
                btn.style().unpolish(btn)
                btn.style().polish(btn)
                btn.setMinimumWidth(100)
                btn.setMinimumHeight(36)

    def _apply_responsive_layout(self):
        w = max(self.width(), 1000)
        # sidebar: ~17 % of width, clamped [200, 260]
        nav_w = max(min(int(w * 0.17), 260), 200)
        self.nav_list.setFixedWidth(nav_w)

        page = getattr(self, "config_page", None)
        if page is None:
            return

        # left panel: 31 % of content area, clamped [320, 460]
        content_w = w - nav_w
        controls_w = max(min(int(content_w * 0.31), 460), 320)
        page.left_widget.setFixedWidth(controls_w)

        tuner = getattr(self, "tuner_page", None)
        if tuner is not None:
            tuner.left_widget.setFixedWidth(controls_w)

    def update_navigation(self, page_id):
        if page_id == -1:
            return
        idx = self.pageIds().index(page_id)
        if self.nav_list.currentRow() != idx:
            self.nav_list.setCurrentRow(idx)

    def navigate_to_page(self, item):
        target = _NAV_LABELS.index(item.text())
        current = self.pageIds().index(self.currentId())
        # going forward respects page validation — stop at the first incomplete page
        while current < target:
            if self.currentPage().isComplete():
                self.next()
                reached = self.pageIds().index(self.currentId())
                if reached == current:
                    break
                current = reached
            else:
                break
        while current > target:
            self.back()
            current = self.pageIds().index(self.currentId())
        self.update_navigation(self.currentId())


def main():
    glutInit(sys.argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    app = QApplication(sys.argv)
    app.setFont(QFont("Arial", 11))
    app.setStyleSheet(APP_STYLESHEET)
    wizard = RobotWizard()
    wizard.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
