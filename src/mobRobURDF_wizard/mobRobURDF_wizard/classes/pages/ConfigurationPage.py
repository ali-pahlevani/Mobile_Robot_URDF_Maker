import os
import logging
from PyQt5.QtWidgets import (QWizardPage, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
                             QLabel, QLineEdit, QFileDialog, QWidget, QScrollArea,
                             QMessageBox, QSizePolicy, QPushButton)
from PyQt5.QtCore import pyqtSignal, Qt
from ament_index_python.packages import get_package_share_directory
from mobRobURDF_wizard.classes.OpenGLWidget import OpenGLWidget
from mobRobURDF_wizard.classes.responsive_widgets import WrapButton, ButtonRow
from mobRobURDF_wizard.classes.SensorEditor import SensorCard
from mobRobURDF_wizard.classes.sensor_config import (
    SensorConfig, default_sensors, sensor_to_dict, sensor_from_dict,
)
from mobRobURDF_wizard.utils.utils import get_color
from mobRobURDF_wizard.utils import presets
from mobRobURDF_wizard.classes.URDFManager import GAZEBO_SIM_PLUGIN

logger = logging.getLogger(__name__)


class ConfigurationPage(QWizardPage):
    modelUpdated = pyqtSignal(float, float, float, str, str, tuple, tuple, str, object)

    # Preset key -> line-edit attribute (chassis + wheels only; sensors handled separately).
    _PRESET_FIELDS = [
        ("chassis_size", "chassisSizeLineEdit"),
        ("chassis_mass", "chassisMassLineEdit"),
        ("chassis_material", "chassisMaterialLineEdit"),
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
        self._sensor_cards = []   # list[SensorCard], in insertion order

        self.default_save_path = os.path.join(
            get_package_share_directory("mobRobURDF_description"), "urdf", "mobRob"
        )

        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ── Left control panel (scrollable) ───────────────────────────────
        self.left_widget = QWidget()
        self.left_widget.setFixedWidth(360)
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

        # Preset row
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

        # ── 3D preview ─────────────────────────────────────────────────────
        self.glWidget = OpenGLWidget()
        self.glWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.modelUpdated.connect(self.glWidget.updateRobotModel)

        main_layout.addWidget(self.left_widget)
        main_layout.addWidget(self.glWidget, 1)

    # ── Form helpers ────────────────────────────────────────────────────────

    def _group(self, title):
        box = QGroupBox(title)
        form = QFormLayout(box)
        form.setSpacing(5)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        form.setLabelAlignment(Qt.AlignLeft)
        return box, form

    def _field(self, form, label, placeholder, unit=''):
        edit = QLineEdit(placeholderText=placeholder)
        if unit:
            row = QHBoxLayout()
            row.setSpacing(4)
            row.addWidget(edit)
            u = QLabel(unit)
            u.setStyleSheet("color: #95A5A6; font-size: 9pt;")
            row.addWidget(u)
            form.addRow(label, row)
        else:
            form.addRow(label, edit)
        return edit

    # ── Page lifecycle ────────────────────────────────────────────────────

    def initializePage(self):
        pending = getattr(self.urdf_manager, 'pending_restore', None)
        if pending:
            self._restore_from_session(pending)
            self.urdf_manager.pending_restore = None
            return

        new_robot_type = self.field("robotType") or "4_wheeled"
        new_controller_type = self.field("controllerType")
        if not new_controller_type:
            new_controller_type = {"2_wheeled_caster": "diff_2wc",
                                   "3_wheeled": "tricycle",
                                   "4_wheeled": "diff_4w"}.get(new_robot_type, "diff_4w")

        robot_changed = (new_robot_type != self.robot_type or
                         new_controller_type != self.controller_type)

        # Save current sensor configs before setup_parameters() wipes the card list.
        saved_sensors = [card.to_sensor_config() for card in self._sensor_cards]

        self.robot_type = new_robot_type
        self.controller_type = new_controller_type

        self.setTitle(f"Configure {self.robot_type.replace('_', ' ').title()} Parameters "
                      f"with {self.controller_type.replace('_', ' ').title()} Controller")
        self.setup_parameters()

        chassis_str = self.chassisSizeLineEdit.text() or "1.2 0.8 0.3"
        if robot_changed or not saved_sensors:
            # New robot/controller → start fresh with one default lidar + camera.
            self._reset_sensors(default_sensors(self.robot_type, chassis_str))
        else:
            # Navigated back to same config → restore what the user had.
            self._reset_sensors(saved_sensors)

        self.applyChanges()

    def _restore_from_session(self, data: dict):
        """Fully restore wizard state from a session dict (loaded from .mobsession)."""
        robot_type = data.get("robot_type", "4_wheeled")
        controller_type = data.get("controller_type", "diff_4w")
        params = data.get("parameters", {})
        sensor_dicts = data.get("sensors", [])
        tuner_params = data.get("tuner_params", {})
        saved_urdf = data.get("urdf_text", "")

        # Sync wizard-level fields so other pages stay consistent.
        try:
            self.setField("robotType", robot_type)
            self.setField("controllerType", controller_type)
        except Exception:
            pass

        self.robot_type = robot_type
        self.controller_type = controller_type
        self.setTitle(
            f"Configure {robot_type.replace('_', ' ').title()} Parameters "
            f"with {controller_type.replace('_', ' ').title()} Controller"
        )

        self.setup_parameters()

        for key, attr in self._PRESET_FIELDS:
            edit = getattr(self, attr, None)
            if edit is not None and key in params:
                edit.setText(params[key])

        sensor_configs = [sensor_from_dict(d) for d in sensor_dicts]
        if not sensor_configs:
            sensor_configs = default_sensors(
                robot_type, params.get("chassis_size", "1.2 0.8 0.3")
            )
        self._reset_sensors(sensor_configs)

        # Restore tuner params so ControllerTunerPage picks them up.
        if tuner_params:
            self.urdf_manager.last_tuner_params = tuner_params

        # Restore hardware interface selection.
        hw_interface = data.get("hardware_interface", GAZEBO_SIM_PLUGIN)
        self.urdf_manager.last_hardware_interface = hw_interface or GAZEBO_SIM_PLUGIN

        # Generate fresh URDF from restored params (updates 3D preview).
        self.applyChanges()

        # Override with the saved URDF text (may include manual edits from FinalCheckPage).
        if saved_urdf:
            self.urdf_manager.urdf_text = saved_urdf

    def setup_parameters(self):
        # Remove all existing form widgets but preserve sensor cards externally.
        while self.params_layout.count():
            item = self.params_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._sensor_cards.clear()

        self.add_common_parameters()

        if self.robot_type in ("4_wheeled", "3_wheeled", "2_wheeled_caster"):
            self.add_wheel_parameters(
                "Wheel and Caster Radius" if self.robot_type == "2_wheeled_caster" else "Wheel Radius"
            )

        self._add_sensor_panel()
        self.params_layout.addStretch(1)

    def add_common_parameters(self):
        chassis_box, chassis_form = self._group("Chassis")
        self.chassisSizeLineEdit = self._field(chassis_form, "Size (L W H)", "e.g., 1.2 0.8 0.3", "m")
        self.chassisMassLineEdit = self._field(chassis_form, "Mass", "e.g., 1.0", "kg")
        self.chassisMaterialLineEdit = self._field(chassis_form, "Material", "e.g., Gray")
        self.params_layout.addWidget(chassis_box)

    def add_wheel_parameters(self, radius_label):
        wheel_box, wheel_form = self._group("Wheels")
        self.wheelRadiusLineEdit = self._field(wheel_form, radius_label, "e.g., 0.22", "m")
        self.wheelWidthLineEdit = self._field(wheel_form, "Wheel Width", "e.g., 0.12", "m")
        self.wheelMassLineEdit = self._field(wheel_form, "Wheel Mass", "e.g., 0.5", "kg")
        self.wheelMaterialLineEdit = self._field(wheel_form, "Wheel Material", "e.g., Black")
        self.params_layout.addWidget(wheel_box)

    # ── Sensor panel ──────────────────────────────────────────────────────

    def _add_sensor_panel(self):
        """Add the "Sensors" group with Add-Lidar / Add-Camera buttons."""
        sensor_group = QGroupBox("Sensors")
        vbox = QVBoxLayout(sensor_group)
        vbox.setSpacing(6)
        vbox.setContentsMargins(6, 8, 6, 8)

        btn_row = QHBoxLayout()
        add_lidar_btn = QPushButton("+ Lidar")
        add_lidar_btn.setStyleSheet("QPushButton { color: white; font-weight: bold; }")
        add_lidar_btn.clicked.connect(lambda: self._add_sensor('lidar'))
        add_cam_btn = QPushButton("+ Camera")
        add_cam_btn.setStyleSheet("QPushButton { color: white; font-weight: bold; }")
        add_cam_btn.clicked.connect(lambda: self._add_sensor('camera'))
        btn_row.addWidget(add_lidar_btn)
        btn_row.addWidget(add_cam_btn)
        vbox.addLayout(btn_row)

        self._sensor_cards_layout = vbox
        self.params_layout.addWidget(sensor_group)

    def _add_sensor(self, sensor_type: str, config: SensorConfig = None):
        card = SensorCard(sensor_type)
        if config is not None:
            card.load_sensor_config(config)
        else:
            # Auto-name using next available index.
            existing = [c for c in self._sensor_cards if c._type == sensor_type]
            idx = len(existing) + 1
            card.nameEdit.setText(f'{sensor_type}_{idx}')

        card.deleted.connect(self._remove_sensor_card)
        self._sensor_cards.append(card)
        self._sensor_cards_layout.addWidget(card)

    def _remove_sensor_card(self, card: SensorCard):
        if card in self._sensor_cards:
            self._sensor_cards.remove(card)
        self._sensor_cards_layout.removeWidget(card)
        card.deleteLater()

    def _reset_sensors(self, sensor_configs: list):
        """Replace all sensor cards with those from sensor_configs."""
        for card in list(self._sensor_cards):
            self._remove_sensor_card(card)
        for sc in sensor_configs:
            self._add_sensor(sc.sensor_type, sc)

    def _gather_sensors(self) -> list:
        """Read all SensorCard widgets and return a list of SensorConfig."""
        return [card.to_sensor_config() for card in self._sensor_cards]

    # ── Validation ──────────────────────────────────────────────────────────

    def validate_float(self, value, field_name, default, min_val=0.0):
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

        wheel_radius = self.validate_float(self.wheelRadiusLineEdit.text(), "wheel_radius", 0.22)
        wheel_width = self.validate_float(self.wheelWidthLineEdit.text(), "wheel_width", 0.12)

        params = {
            "chassis_size": chassis_size_str,
            "chassis_mass": self.chassisMassLineEdit.text() or "1.0",
            "chassis_material": self.chassisMaterialLineEdit.text() or "Gray",
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

        sensors = self._gather_sensors()
        urdf_text = self.urdf_manager.generate_urdf(
            self.robot_type, self.controller_type, params, sensors
        )
        # Update chassis + wheels in the 3D preview via signal.
        self.modelUpdated.emit(
            L, W, H,
            str(wheel_radius),
            str(wheel_width),
            get_color(params["chassis_material"]),
            get_color(params.get("wheel_material", "Black")),
            self.robot_type,
            caster_radius
        )

        # Update sensors directly — all sensors with their exact positions and colors.
        self.glWidget.updateSensors(sensors)

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
            sensor_dicts = [sensor_to_dict(s) for s in self._gather_sensors()]
            presets.save_preset(path, self.robot_type, self.controller_type,
                                self._gather_params(), sensor_dicts)
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
            robot_type, controller_type, params, sensor_dicts = presets.load_preset(path)
        except Exception as e:
            logger.error("Failed to load preset %s: %s", path, str(e))
            QMessageBox.warning(self, "Load failed", f"Could not load preset:\n{e}")
            return

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

        # Restore sensor cards from preset.
        sensor_configs = [sensor_from_dict(d) for d in sensor_dicts]
        if not sensor_configs:
            sensor_configs = default_sensors(robot_type, params.get('chassis_size', '1.2 0.8 0.3'))
        self._reset_sensors(sensor_configs)

        self.applyChanges()
        QMessageBox.information(self, "Preset loaded", f"Loaded configuration from:\n{path}")

