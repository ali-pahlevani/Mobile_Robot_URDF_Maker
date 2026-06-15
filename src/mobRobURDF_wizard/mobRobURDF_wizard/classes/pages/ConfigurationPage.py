import os
import time
import logging
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
                             QLabel, QLineEdit, QTextEdit, QFileDialog, QWidget, QScrollArea,
                             QMessageBox, QSizePolicy)
from PyQt5.QtCore import pyqtSignal, Qt
from ament_index_python.packages import get_package_share_directory
from mobRobURDF_wizard.classes.OpenGLWidget import OpenGLWidget
from mobRobURDF_wizard.classes.responsive_widgets import WrapButton, ButtonRow
from mobRobURDF_wizard.classes.launch_manager import LaunchManager
from mobRobURDF_wizard.utils.utils import get_color
from mobRobURDF_wizard.utils import presets

logger = logging.getLogger(__name__)


class ConfigurationPage(QWizardPage):
    # The last argument (caster_radius) is `object` because it is None for
    # every non-2WC robot type.
    modelUpdated = pyqtSignal(float, float, float, str, str, str, str, str, tuple, tuple, tuple, tuple, str, object)

    # Preset key -> line-edit attribute (attributes exist after setup_parameters()).
    _PRESET_FIELDS = [
        ("chassis_size", "chassisSizeLineEdit"),
        ("chassis_mass", "chassisMassLineEdit"),
        ("chassis_material", "chassisMaterialLineEdit"),
        ("lidar_radius", "lidarRadiusLineEdit"),
        ("lidar_height", "lidarHeightLineEdit"),
        ("lidar_mass", "lidarMassLineEdit"),
        ("lidar_material", "lidarMaterialLineEdit"),
        ("camera_size", "cameraSizeLineEdit"),
        ("camera_mass", "cameraMassLineEdit"),
        ("camera_material", "cameraMaterialLineEdit"),
        ("wheel_radius", "wheelRadiusLineEdit"),
        ("wheel_width", "wheelWidthLineEdit"),
        ("wheel_mass", "wheelMassLineEdit"),
        ("wheel_material", "wheelMaterialLineEdit"),
    ]

    def __init__(self, urdf_manager, parent=None):
        super().__init__(parent)
        self.urdf_manager = urdf_manager
        self.robot_type = None
        self.controller_type = None

        # Default save location: the description package's urdf directory.
        self.default_save_path = os.path.join(
            get_package_share_directory("mobRobURDF_description"), "urdf", "mobRob"
        )

        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ── Left control panel (scrollable) ───────────────────────────────
        self.left_widget = QWidget()
        self.left_widget.setFixedWidth(300)
        left_layout = QVBoxLayout(self.left_widget)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.setSpacing(8)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QScrollArea.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._params_container = QWidget()
        self.params_layout = QVBoxLayout(self._params_container)
        self.params_layout.setContentsMargins(0, 0, 0, 0)
        self.params_layout.setSpacing(10)
        self.scroll.setWidget(self._params_container)
        left_layout.addWidget(self.scroll, 1)

        # Preset row (responsive: stacks vertically when narrow)
        self.loadPresetButton = WrapButton("Load Preset", "secondary")
        self.loadPresetButton.setMinimumHeight(34)
        self.loadPresetButton.clicked.connect(self.loadPreset)
        self.savePresetButton = WrapButton("Save Preset", "secondary")
        self.savePresetButton.setMinimumHeight(34)
        self.savePresetButton.clicked.connect(self.savePreset)
        left_layout.addWidget(ButtonRow(self.loadPresetButton, self.savePresetButton))

        self.applyButton = WrapButton("Apply and Preview", "success")
        self.applyButton.setMinimumHeight(38)
        self.applyButton.clicked.connect(self.applyChanges)
        left_layout.addWidget(self.applyButton)

        self.saveButton = WrapButton("Save URDF to Folder")
        self.saveButton.setMinimumHeight(38)
        self.saveButton.clicked.connect(self.saveURDF)
        left_layout.addWidget(self.saveButton)

        # One-click simulation launch
        self.launchButton = WrapButton("Launch Simulation", "success")
        self.launchButton.setMinimumHeight(38)
        self.launchButton.clicked.connect(self.launchSimulation)
        left_layout.addWidget(self.launchButton)

        self.launchStatus = QLabel("")
        self.launchStatus.setAlignment(Qt.AlignCenter)
        self.launchStatus.setWordWrap(True)
        self.launchStatus.setStyleSheet("font-size: 9pt; color: #7F8C8D;")
        left_layout.addWidget(self.launchStatus)

        self.launch_manager = LaunchManager(self)
        self.launch_manager.started.connect(self._on_launch_started)
        self.launch_manager.stopped.connect(self._on_launch_stopped)
        self.launch_manager.output.connect(self._on_launch_output)
        self._launch_start_time = 0.0

        # ── URDF text preview ─────────────────────────────────────────────
        self.previewTextEdit = QTextEdit()
        self.previewTextEdit.setReadOnly(True)
        self.previewTextEdit.setFixedWidth(340)
        self.previewTextEdit.setLineWrapMode(QTextEdit.NoWrap)

        # ── 3D preview ─────────────────────────────────────────────────────
        self.glWidget = OpenGLWidget()
        self.glWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.modelUpdated.connect(self.glWidget.updateRobotModel)

        main_layout.addWidget(self.left_widget)
        main_layout.addWidget(self.previewTextEdit)
        main_layout.addWidget(self.glWidget, 1)

    # ── Form helpers ────────────────────────────────────────────────────────

    def _group(self, title):
        box = QGroupBox(title)
        form = QFormLayout(box)
        form.setSpacing(5)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        form.setLabelAlignment(Qt.AlignLeft)
        return box, form

    def _field(self, form, label, placeholder):
        edit = QLineEdit(placeholderText=placeholder)
        form.addRow(label, edit)
        return edit

    # ── Page lifecycle ────────────────────────────────────────────────────

    def initializePage(self):
        self.robot_type = self.field("robotType")
        self.controller_type = self.field("controllerType")
        if self.robot_type is None:
            logger.warning("robotType is None, defaulting to 4_wheeled")
            self.robot_type = "4_wheeled"
        if self.controller_type is None:
            logger.warning("controllerType is None, defaulting based on robotType")
            self.controller_type = {"2_wheeled_caster": "diff_2wc", "3_wheeled": "tricycle",
                                    "4_wheeled": "diff_4w"}.get(self.robot_type, "diff_4w")
        self.setTitle(f"Configure {self.robot_type.replace('_', ' ').title()} Parameters "
                      f"with {self.controller_type.replace('_', ' ').title()} Controller")
        self.setup_parameters()
        self.applyChanges()

    def setup_parameters(self):
        while self.params_layout.count():
            item = self.params_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self.add_common_parameters()

        if self.robot_type == "4_wheeled":
            self.add_wheel_parameters("Wheel Radius")
        elif self.robot_type == "3_wheeled":
            self.add_wheel_parameters("Wheel Radius")
        elif self.robot_type == "2_wheeled_caster":
            self.add_wheel_parameters("Wheel and Caster Radius")
        else:
            logger.error("Unknown robot_type: %s, no parameters added", self.robot_type)

        self.params_layout.addStretch(1)

    def add_common_parameters(self):
        chassis_box, chassis_form = self._group("Chassis")
        self.chassisSizeLineEdit = self._field(chassis_form, "Size (L W H)", "e.g., 1 1 0.5")
        self.chassisMassLineEdit = self._field(chassis_form, "Mass", "e.g., 1.0")
        self.chassisMaterialLineEdit = self._field(chassis_form, "Material", "e.g., Gray")
        self.params_layout.addWidget(chassis_box)

        lidar_box, lidar_form = self._group("Lidar")
        self.lidarRadiusLineEdit = self._field(lidar_form, "Radius", "e.g., 0.2")
        self.lidarHeightLineEdit = self._field(lidar_form, "Height", "e.g., 0.1")
        self.lidarMassLineEdit = self._field(lidar_form, "Mass", "e.g., 0.2")
        self.lidarMaterialLineEdit = self._field(lidar_form, "Material", "e.g., Red")
        self.params_layout.addWidget(lidar_box)

        camera_box, camera_form = self._group("Camera")
        self.cameraSizeLineEdit = self._field(camera_form, "Size (L W H)", "e.g., 0.1 0.1 0.1")
        self.cameraMassLineEdit = self._field(camera_form, "Mass", "e.g., 0.1")
        self.cameraMaterialLineEdit = self._field(camera_form, "Material", "e.g., Blue")
        self.params_layout.addWidget(camera_box)

    def add_wheel_parameters(self, radius_label):
        wheel_box, wheel_form = self._group("Wheels")
        self.wheelRadiusLineEdit = self._field(wheel_form, radius_label, "e.g., 0.3")
        self.wheelWidthLineEdit = self._field(wheel_form, "Wheel Width", "e.g., 0.1")
        self.wheelMassLineEdit = self._field(wheel_form, "Wheel Mass", "e.g., 0.5")
        self.wheelMaterialLineEdit = self._field(wheel_form, "Wheel Material", "e.g., Black")
        self.params_layout.addWidget(wheel_box)

    # ── Validation ──────────────────────────────────────────────────────────

    def validate_float(self, value, field_name, default, min_val=0.0):
        """Validate a string input as a float, returning default if invalid or below min_val."""
        try:
            if value.strip():
                val = float(value)
                if val >= min_val:
                    return val
            return default
        except ValueError:
            return default

    # ── Apply ─────────────────────────────────────────────────────────────

    def applyChanges(self):
        chassis_size_str = self.chassisSizeLineEdit.text()
        try:
            L, W, H = map(float, chassis_size_str.split())
        except ValueError:
            L, W, H = 1.2, 0.8, 0.3
            chassis_size_str = "1.2 0.8 0.3"

        camera_size_str = self.cameraSizeLineEdit.text()
        try:
            Lc, Wc, Hc = map(float, camera_size_str.split())
        except ValueError:
            Lc, Wc, Hc = 0.08, 0.2, 0.08
            camera_size_str = "0.08 0.2 0.08"

        # Validate numeric inputs to prevent float conversion errors
        wheel_radius = self.validate_float(self.wheelRadiusLineEdit.text(), "wheel_radius", 0.22)
        wheel_width = self.validate_float(self.wheelWidthLineEdit.text(), "wheel_width", 0.12)
        lidar_radius = self.validate_float(self.lidarRadiusLineEdit.text(), "lidar_radius", 0.1)
        lidar_height = self.validate_float(self.lidarHeightLineEdit.text(), "lidar_height", 0.08)

        params = {
            "chassis_size": chassis_size_str,
            "chassis_mass": self.chassisMassLineEdit.text() or "1.0",
            "chassis_material": self.chassisMaterialLineEdit.text() or "Gray",
            "lidar_radius": str(lidar_radius),
            "lidar_height": str(lidar_height),
            "lidar_mass": self.lidarMassLineEdit.text() or "0.2",
            "lidar_material": self.lidarMaterialLineEdit.text() or "Red",
            "camera_size": camera_size_str,
            "camera_mass": self.cameraMassLineEdit.text() or "0.1",
            "camera_material": self.cameraMaterialLineEdit.text() or "Blue",
        }

        if self.robot_type in ["4_wheeled", "3_wheeled", "2_wheeled_caster"]:
            params["wheel_radius"] = str(wheel_radius)
            params["wheel_width"] = str(wheel_width)
            params["wheel_mass"] = self.wheelMassLineEdit.text() or "0.5"
            params["wheel_material"] = self.wheelMaterialLineEdit.text() or "Black"

        if self.robot_type == "2_wheeled_caster":
            params["caster_radius"] = params["wheel_radius"]
            caster_radius = str(wheel_radius)
        else:
            caster_radius = None

        if self.robot_type == "4_wheeled":
            params["fl_x"] = str(L / 2 - wheel_radius / 1.5)
            params["fl_y"] = str(W / 2 + wheel_width / 2)
            params["fl_z"] = str(-H / 2)
            params["fr_x"] = str(L / 2 - wheel_radius / 1.5)
            params["fr_y"] = str(-W / 2 - wheel_width / 2)
            params["fr_z"] = str(-H / 2)
            params["rl_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rl_y"] = str(W / 2 + wheel_width / 2)
            params["rl_z"] = str(-H / 2)
            params["rr_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rr_y"] = str(-W / 2 - wheel_width / 2)
            params["rr_z"] = str(-H / 2)
        elif self.robot_type == "3_wheeled":
            params["front_x"] = str(L / 2)
            params["front_y"] = "0"
            params["front_z"] = str(-H / 2)
            params["rl_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rl_y"] = str(W / 2 + wheel_width / 2)
            params["rl_z"] = str(-H / 2)
            params["rr_x"] = str(-L / 2 + wheel_radius / 1.5)
            params["rr_y"] = str(-W / 2 - wheel_width / 2)
            params["rr_z"] = str(-H / 2)
        elif self.robot_type == "2_wheeled_caster":
            params["l_x"] = str(-L / 4)
            params["l_y"] = str(W / 2 + wheel_width / 2)
            params["l_z"] = str(-H / 2)
            params["r_x"] = str(-L / 4)
            params["r_y"] = str(-W / 2 - wheel_width / 2)
            params["r_z"] = str(-H / 2)
            params["caster_x"] = str(L / 2 - float(caster_radius))
            params["caster_y"] = "0"
            params["caster_z"] = str(-H / 2)

        params["lidar_z"] = str(H / 2 + lidar_height / 2)
        params["camera_x"] = str(L / 2 + Lc / 2)
        if self.robot_type == "3_wheeled":
            params["camera_z"] = str(H / 2 - Hc / 2)

        urdf_text = self.urdf_manager.generate_urdf(self.robot_type, self.controller_type, params)
        self.previewTextEdit.setPlainText(urdf_text)

        self.modelUpdated.emit(
            L, W, H,
            str(wheel_radius),
            str(wheel_width),
            str(lidar_radius),
            str(lidar_height),
            camera_size_str,
            get_color(params["chassis_material"]),
            get_color(params["wheel_material"]),
            get_color(params["lidar_material"]),
            get_color(params["camera_material"]),
            self.robot_type,
            caster_radius
        )
        self.glWidget.update()

    # ── Save ──────────────────────────────────────────────────────────────

    def saveURDF(self):
        try:
            self.urdf_manager.save_urdf(self.default_save_path)
        except Exception as e:
            logger.error("Failed to save URDF to default location %s: %s", self.default_save_path, str(e))
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "Save URDF and URDF.xacro", self.default_save_path,
            "URDF Files (*.urdf *.urdf.xacro)")
        if filename:
            try:
                self.urdf_manager.save_urdf(filename)
                logger.debug("URDF and URDF.xacro saved to: %s", filename)
                QMessageBox.information(self, "Saved",
                                        f"URDF and URDF.xacro saved to:\n{filename}")
            except Exception as e:
                logger.error("Failed to save URDF to %s: %s", filename, str(e))
                QMessageBox.warning(self, "Save failed", str(e))

    # ── Presets ─────────────────────────────────────────────────────────────

    def _gather_params(self):
        params = {}
        for key, attr in self._PRESET_FIELDS:
            edit = getattr(self, attr, None)
            if edit is not None:
                params[key] = edit.text()
        return params

    def savePreset(self):
        if not self.robot_type or not self.controller_type:
            return
        default = os.path.join(presets.default_preset_dir(),
                               f"{self.robot_type}_{self.controller_type}.yaml")
        path, _ = QFileDialog.getSaveFileName(self, "Save Preset", default,
                                              "Preset Files (*.yaml *.yml)")
        if not path:
            return
        if not path.endswith((".yaml", ".yml")):
            path += ".yaml"
        try:
            presets.save_preset(path, self.robot_type, self.controller_type, self._gather_params())
            QMessageBox.information(self, "Preset saved", f"Preset saved to:\n{path}")
        except Exception as e:
            logger.error("Failed to save preset to %s: %s", path, str(e))
            QMessageBox.warning(self, "Save failed", str(e))

    def loadPreset(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Preset", presets.default_preset_dir(),
                                              "Preset Files (*.yaml *.yml)")
        if not path:
            return
        try:
            robot_type, controller_type, params = presets.load_preset(path)
        except Exception as e:
            logger.error("Failed to load preset %s: %s", path, str(e))
            QMessageBox.warning(self, "Load failed", f"Could not load preset:\n{e}")
            return

        # Update wizard fields so the earlier selection pages reflect the load
        # (they re-sync their highlighted cards in initializePage).
        self.setField("robotType", robot_type)
        self.setField("controllerType", controller_type)
        self.robot_type = robot_type
        self.controller_type = controller_type
        self.setTitle(f"Configure {robot_type.replace('_', ' ').title()} Parameters "
                      f"with {controller_type.replace('_', ' ').title()} Controller")

        self.setup_parameters()
        for key, attr in self._PRESET_FIELDS:
            edit = getattr(self, attr, None)
            if edit is not None and key in params:
                edit.setText(params[key])
        self.applyChanges()
        QMessageBox.information(self, "Preset loaded", f"Loaded configuration from:\n{path}")

    # ── One-click launch ──────────────────────────────────────────────────

    def launchSimulation(self):
        if self.launch_manager.is_running():
            self.launch_manager.stop()
            self.launchButton.setEnabled(False)
            self.launchStatus.setText("Stopping simulation…")
            return

        # Apply the current configuration so the launch uses the latest URDF
        # and selected controller.
        self.applyChanges()
        urdf = self.urdf_manager.get_urdf_text()
        if not urdf or urdf.startswith("Error"):
            QMessageBox.warning(
                self, "Cannot launch",
                "The current configuration did not produce a valid URDF:\n\n"
                + (urdf[:400] if urdf else "(empty output)"))
            return

        self._launch_start_time = time.monotonic()
        self.launch_manager.start()

    def _set_launch_button_role(self, role):
        self.launchButton.setProperty("btnRole", role)
        self.launchButton.style().unpolish(self.launchButton)
        self.launchButton.style().polish(self.launchButton)

    def _on_launch_started(self):
        self.launchButton.setText("Stop Simulation")
        self.launchButton.setEnabled(True)
        self._set_launch_button_role("danger")
        self.launchStatus.setText("Simulation running — Gazebo, controllers and RViz are starting…")

    def _on_launch_stopped(self, rc):
        self.launchButton.setText("Launch Simulation")
        self.launchButton.setEnabled(True)
        self._set_launch_button_role("success")
        elapsed = time.monotonic() - self._launch_start_time
        if rc not in (0, -2) and elapsed < 4:
            # Exited almost immediately with an error — most likely the
            # workspace wasn't built/sourced.
            self.launchStatus.setText("Simulation failed to start.")
            QMessageBox.warning(
                self, "Launch failed",
                "The simulation exited immediately (code "
                f"{rc}).\n\nMake sure the workspace is built and sourced:\n"
                "  colcon build --symlink-install\n  source install/setup.bash")
        else:
            self.launchStatus.setText("Simulation stopped.")

    def _on_launch_output(self, line):
        logger.info("[sim] %s", line)

