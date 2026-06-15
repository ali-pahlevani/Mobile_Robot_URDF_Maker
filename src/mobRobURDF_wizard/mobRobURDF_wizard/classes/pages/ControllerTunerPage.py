"""Controller Parameter Tuner page.

Exposes every tunable ros2_control parameter for the active controller in a
clean form. Auto-computed geometry values (wheel_separation, wheelbase, …) are
displayed read-only so the user can see what was derived from the URDF.
Velocity / acceleration / steering limits can be set manually or filled in with
physics-based suggestions via the "Auto-suggest" button.

The page writes its parameters into URDFManager.last_tuner_params so that
subsequent Apply clicks on the ConfigurationPage always re-apply them.
"""

import os
import logging
import math
from PyQt5.QtWidgets import (
    QWizardPage, QHBoxLayout, QVBoxLayout, QFormLayout, QGroupBox,
    QLabel, QLineEdit, QCheckBox, QScrollArea, QWidget, QSizePolicy,
    QMessageBox,
)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap
from ament_index_python.packages import get_package_share_directory

from mobRobURDF_wizard.classes.responsive_widgets import WrapButton, ButtonRow, ScaledPixmapLabel

logger = logging.getLogger(__name__)

# Human-readable controller names
_CTRL_DISPLAY = {
    'diff_4w':   'Differential Drive (4-wheel)',
    'diff_2wc':  'Differential Drive (2-wheel + caster)',
    'mecanum':   'Mecanum Drive',
    'tricycle':  'Tricycle',
    'triSteer':  'Tricycle Steering',
    'ackermann': 'Ackermann Steering',
}

# Which controller types have steering limits
_HAS_STEERING = {'tricycle', 'triSteer', 'ackermann'}

# controller_type → image filename in images/control_types/
_CTRL_IMAGE = {
    'diff_4w':   'diff_4w.png',
    'diff_2wc':  'diff_2wc.png',
    'mecanum':   'mecanum.png',
    'tricycle':  'tricycle.png',
    'triSteer':  'triSteer.png',
    'ackermann': 'ackermann.png',
}

# Typical max motor angular velocity (rad/s) used for auto-suggest
_MOTOR_MAX_RAD_S = 12.0


class ControllerTunerPage(QWizardPage):
    def __init__(self, urdf_manager, parent=None):
        super().__init__(parent)
        self.urdf_manager = urdf_manager
        self.setTitle("Tune Controller Parameters")

        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # ── Left: scrollable form ─────────────────────────────────────────
        left_wrap = QWidget()
        left_wrap.setFixedWidth(360)
        lv = QVBoxLayout(left_wrap)
        lv.setContentsMargins(10, 10, 10, 10)
        lv.setSpacing(8)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._form_container = QWidget()
        self._form_layout = QVBoxLayout(self._form_container)
        self._form_layout.setContentsMargins(0, 0, 0, 0)
        self._form_layout.setSpacing(10)
        scroll.setWidget(self._form_container)
        lv.addWidget(scroll, 1)

        self._suggest_btn = WrapButton("Auto-suggest Limits", "secondary")
        self._suggest_btn.setMinimumHeight(36)
        self._suggest_btn.clicked.connect(self._auto_suggest)
        self._apply_btn = WrapButton("Apply to Controller", "success")
        self._apply_btn.setMinimumHeight(36)
        self._apply_btn.clicked.connect(self._apply)
        lv.addWidget(ButtonRow(self._suggest_btn, self._apply_btn))

        self._status = QLabel("")
        self._status.setAlignment(Qt.AlignCenter)
        self._status.setWordWrap(True)
        self._status.setStyleSheet("font-size: 9pt; color: #27AE60;")
        lv.addWidget(self._status)

        outer.addWidget(left_wrap)

        # ── Right: explanatory info panel ────────────────────────────────
        info_widget = QWidget()
        info_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        iv = QVBoxLayout(info_widget)
        iv.setContentsMargins(30, 30, 30, 30)
        iv.setSpacing(16)

        title_lbl = QLabel("How parameters are chosen")
        title_lbl.setStyleSheet("font-size: 14pt; font-weight: bold; color: #2C3E50;")
        iv.addWidget(title_lbl)

        info_text = (
            "<b>Auto-computed geometry</b> values (wheel separation, radius, wheelbase) "
            "are derived directly from the dimensions you set on the previous page — "
            "no manual entry needed.<br><br>"
            "<b>General settings</b> control how often the controller publishes odometry "
            "(<i>publish rate</i>), how quickly it stops if no command arrives "
            "(<i>cmd_vel timeout</i>), and whether it integrates wheel encoders "
            "(<i>open loop = false</i>) or ignores them (<i>open loop = true</i>).<br><br>"
            "<b>Motion limits</b> cap the velocity and acceleration commands the "
            "controller accepts. Start with the <b>Auto-suggest</b> values — they are "
            "computed from wheel radius and typical motor speed — then lower them if "
            "the robot feels too aggressive in simulation.<br><br>"
            "<b>Steering limits</b> (available for tricycle / Ackermann) constrain how "
            "far and how fast the steering joint can move. Exceeding the mechanical "
            "joint range will cause the robot to lose control, so keep "
            "<i>max steer angle</i> within the URDF joint limits."
        )
        info_lbl = QLabel(info_text)
        info_lbl.setWordWrap(True)
        info_lbl.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        info_lbl.setStyleSheet("font-size: 10.5pt; color: #2C3E50; line-height: 160%;")
        iv.addWidget(info_lbl)

        iv.addSpacing(12)

        # ── Controller kinematics image ───────────────────────────────────
        self._ctrl_img_dir = os.path.join(
            get_package_share_directory("mobRobURDF_wizard"),
            "images", "control_types",
        )
        self._ctrl_img = ScaledPixmapLabel(hint=QSize(340, 220))
        self._ctrl_img.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        iv.addWidget(self._ctrl_img, 1)

        outer.addWidget(info_widget, 1)

    # ── Form helpers ──────────────────────────────────────────────────────

    def _group(self, title):
        box = QGroupBox(title)
        form = QFormLayout(box)
        form.setSpacing(6)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        form.setLabelAlignment(Qt.AlignLeft)
        return box, form

    def _field(self, form, label, value, unit='', readonly=False):
        row = QHBoxLayout()
        row.setSpacing(4)
        edit = QLineEdit(str(value))
        edit.setReadOnly(readonly)
        if readonly:
            edit.setStyleSheet(
                "QLineEdit { background: #ECF0F1; color: #7F8C8D; border: 1px solid #BDC3C7; }"
            )
        row.addWidget(edit)
        if unit:
            u = QLabel(unit)
            u.setStyleSheet("color: #95A5A6; font-size: 9pt;")
            row.addWidget(u)
        form.addRow(label, row)
        return edit

    def _check(self, form, label, checked):
        cb = QCheckBox()
        cb.setChecked(checked)
        form.addRow(label, cb)
        return cb

    def _clear_form(self):
        while self._form_layout.count():
            item = self._form_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    # ── Page lifecycle ────────────────────────────────────────────────────

    def initializePage(self):
        ct = self.urdf_manager.last_controller_type or 'diff_4w'
        display = _CTRL_DISPLAY.get(ct, ct)
        self.setTitle(f"Tune Controller Parameters — {display}")
        self._build_form(ct)
        self._load_existing_tuner_params()
        self._update_ctrl_image(ct)

    def _update_ctrl_image(self, controller_type: str):
        img_file = _CTRL_IMAGE.get(controller_type, '')
        if img_file:
            path = os.path.join(self._ctrl_img_dir, img_file)
            if os.path.exists(path):
                self._ctrl_img.setSourcePixmap(QPixmap(path))
                return
        self._ctrl_img.setText("No image available")
        self._ctrl_img.setStyleSheet("color: #95A5A6; border: none; background: transparent;")

    def _build_form(self, controller_type):
        self._clear_form()
        self._controller_type = controller_type

        params = self.urdf_manager.last_params

        # ── Auto-computed geometry (read-only) ────────────────────────────
        geo_box, geo_form = self._group("Auto-computed Geometry (from URDF)")
        self._geo_fields = {}

        if controller_type in ('diff_4w', 'diff_2wc'):
            W = float(params.get('chassis_size', '1.2 0.8 0.3').split()[1])
            ww = float(params.get('wheel_width', '0.12'))
            sep = round(W + ww, 4)
            wr  = float(params.get('wheel_radius', '0.22'))
            self._geo_fields['wheel_separation'] = self._field(geo_form, 'Wheel separation:', sep, 'm', readonly=True)
            self._geo_fields['wheel_radius']     = self._field(geo_form, 'Wheel radius:', wr,  'm', readonly=True)

        elif controller_type == 'mecanum':
            L = float(params.get('chassis_size', '1.2 0.8 0.3').split()[0])
            W = float(params.get('chassis_size', '1.2 0.8 0.3').split()[1])
            wr = float(params.get('wheel_radius', '0.22'))
            self._geo_fields['wheels_radius'] = self._field(geo_form, 'Wheel radius:', wr, 'm', readonly=True)
            self._geo_fields['sum_xy']        = self._field(geo_form, 'Sum of X+Y axes:', round(L+W, 4), 'm', readonly=True)

        elif controller_type in ('tricycle', 'triSteer', 'ackermann'):
            L  = float(params.get('chassis_size', '1.2 0.8 0.3').split()[0])
            W  = float(params.get('chassis_size', '1.2 0.8 0.3').split()[1])
            wr = float(params.get('wheel_radius', '0.22'))
            self._geo_fields['wheelbase']    = self._field(geo_form, 'Wheelbase:', L,  'm', readonly=True)
            self._geo_fields['wheel_track']  = self._field(geo_form, 'Wheel track:', W, 'm', readonly=True)
            self._geo_fields['wheel_radius'] = self._field(geo_form, 'Wheel radius:', wr, 'm', readonly=True)

        hint = QLabel("These values are derived from your robot's dimensions and updated automatically.")
        hint.setWordWrap(True)
        hint.setStyleSheet("font-size: 8.5pt; color: #95A5A6; font-style: italic;")
        geo_form.addRow(hint)
        self._form_layout.addWidget(geo_box)

        # ── General settings ──────────────────────────────────────────────
        gen_box, gen_form = self._group("General Settings")
        self._pub_rate   = self._field(gen_form, 'Publish rate:', 50.0, 'Hz')
        self._upd_rate   = self._field(gen_form, 'Update rate:', 50, 'Hz')
        self._timeout    = self._field(gen_form, 'Cmd vel timeout:', 0.5, 's')
        self._open_loop  = self._check(gen_form, 'Open loop:', False)
        self._odom_tf    = self._check(gen_form, 'Enable odom TF:', True)
        self._form_layout.addWidget(gen_box)

        # ── Motion limits ─────────────────────────────────────────────────
        lim_box, lim_form = self._group("Motion Limits")
        self._max_lin_vel  = self._field(lim_form, 'Max linear vel:', 1.5, 'm/s')
        self._max_ang_vel  = self._field(lim_form, 'Max angular vel:', 2.0, 'rad/s')
        self._max_lin_acc  = self._field(lim_form, 'Max linear acc:', 0.5, 'm/s²')
        self._max_ang_acc  = self._field(lim_form, 'Max angular acc:', 1.0, 'rad/s²')
        self._form_layout.addWidget(lim_box)

        # ── Steering limits (steering controllers only) ───────────────────
        self._steer_box = None
        if controller_type in _HAS_STEERING:
            steer_box, steer_form = self._group("Steering Limits")
            self._max_steer_angle = self._field(steer_form, 'Max steer angle:', 0.785, 'rad (≈ 45°)')
            self._max_steer_vel   = self._field(steer_form, 'Max steer vel:', 1.0, 'rad/s')
            self._form_layout.addWidget(steer_box)
            self._steer_box = steer_box

        self._form_layout.addStretch(1)

    def _load_existing_tuner_params(self):
        """Re-populate fields if the user already applied tuner params earlier."""
        p = self.urdf_manager.last_tuner_params
        if not p:
            return
        self._safe_set(self._pub_rate,  p.get('publish_rate'))
        self._safe_set(self._upd_rate,  p.get('update_rate'))
        self._safe_set(self._timeout,   p.get('cmd_vel_timeout'))
        if 'open_loop' in p:
            self._open_loop.setChecked(p['open_loop'])
        if 'enable_odom_tf' in p:
            self._odom_tf.setChecked(p['enable_odom_tf'])
        self._safe_set(self._max_lin_vel, p.get('max_linear_velocity'))
        self._safe_set(self._max_ang_vel, p.get('max_angular_velocity'))
        self._safe_set(self._max_lin_acc, p.get('max_linear_acceleration'))
        self._safe_set(self._max_ang_acc, p.get('max_angular_acceleration'))
        if self._steer_box:
            self._safe_set(self._max_steer_angle, p.get('max_steering_angle'))
            self._safe_set(self._max_steer_vel,   p.get('max_steering_velocity'))

    @staticmethod
    def _safe_set(edit, value):
        if value is not None and edit is not None:
            edit.setText(str(value))

    # ── Auto-suggest ──────────────────────────────────────────────────────

    def _auto_suggest(self):
        ct = self._controller_type
        params = self.urdf_manager.last_params

        wr  = float(params.get('wheel_radius', '0.22'))
        L   = float(params.get('chassis_size', '1.2 0.8 0.3').split()[0])
        W   = float(params.get('chassis_size', '1.2 0.8 0.3').split()[1])
        ww  = float(params.get('wheel_width', '0.12'))

        max_lin_vel = round(wr * _MOTOR_MAX_RAD_S, 2)

        if ct in ('diff_4w', 'diff_2wc'):
            sep = W + ww
            max_ang_vel = round(max_lin_vel / (sep / 2), 2)
        elif ct == 'mecanum':
            max_ang_vel = round(max_lin_vel / (max(L, W) / 2), 2)
        elif ct in ('tricycle', 'triSteer', 'ackermann'):
            # Minimum turning radius ≈ wheelbase / tan(max_steer_angle ≈ 45°)
            min_turn_r = L  # rough approximation
            max_ang_vel = round(max_lin_vel / min_turn_r, 2)
        else:
            max_ang_vel = 2.0

        max_lin_acc = round(max_lin_vel * 0.4, 2)
        max_ang_acc = round(max_ang_vel * 0.5, 2)

        self._max_lin_vel.setText(str(max_lin_vel))
        self._max_ang_vel.setText(str(max_ang_vel))
        self._max_lin_acc.setText(str(max_lin_acc))
        self._max_ang_acc.setText(str(max_ang_acc))

        if self._steer_box:
            self._max_steer_angle.setText('0.785')
            self._max_steer_vel.setText('1.0')

        self._status.setText("Limits suggested from wheel radius and motor speed. Adjust as needed.")
        self._status.setStyleSheet("font-size: 9pt; color: #2980B9;")

    # ── Collect & apply ───────────────────────────────────────────────────

    def _gather_tuner_params(self):
        def fv(edit, default):
            try:
                return float(edit.text())
            except ValueError:
                return default

        def iv(edit, default):
            try:
                return int(edit.text())
            except ValueError:
                return default

        p = {
            'publish_rate':          fv(self._pub_rate, 50.0),
            'update_rate':           iv(self._upd_rate, 50),
            'cmd_vel_timeout':       fv(self._timeout, 0.5),
            'open_loop':             self._open_loop.isChecked(),
            'enable_odom_tf':        self._odom_tf.isChecked(),
            'max_linear_velocity':   fv(self._max_lin_vel, 1.5),
            'max_angular_velocity':  fv(self._max_ang_vel, 2.0),
            'max_linear_acceleration':  fv(self._max_lin_acc, 0.5),
            'max_angular_acceleration': fv(self._max_ang_acc, 1.0),
        }
        if self._steer_box:
            p['max_steering_angle']    = fv(self._max_steer_angle, 0.785)
            p['max_steering_velocity'] = fv(self._max_steer_vel, 1.0)
        return p

    def _apply(self):
        if not self.urdf_manager.last_controller_type:
            QMessageBox.warning(self, "Not ready",
                                "Please configure the robot on the previous page first.")
            return
        try:
            tuner_params = self._gather_tuner_params()
            self.urdf_manager.apply_tuner_params(
                self.urdf_manager.last_robot_type,
                self.urdf_manager.last_controller_type,
                self.urdf_manager.last_params,
                tuner_params,
            )
            self._status.setText("Controller YAML updated successfully.")
            self._status.setStyleSheet("font-size: 9pt; color: #27AE60;")
        except Exception as e:
            logger.error("Failed to apply tuner params: %s", e)
            self._status.setText(f"Error: {e}")
            self._status.setStyleSheet("font-size: 9pt; color: #E74C3C;")
