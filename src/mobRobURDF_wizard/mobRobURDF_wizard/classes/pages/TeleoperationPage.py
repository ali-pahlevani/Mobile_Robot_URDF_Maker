"""Built-in teleoperation page.

Lets the user drive the robot directly from the wizard using on-screen
D-pad buttons or keyboard shortcuts (WASD / arrow keys).  Publishes
geometry_msgs/Twist to /cmd_vel at 50 Hz via an rclpy node running in a
background daemon thread.  An immediate publish is also triggered on every
key-press/release and button press/release so latency is minimised.
"""

import time
import threading
import logging
from PyQt5.QtWidgets import (
    QWizardPage, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QGroupBox, QWidget, QSizePolicy,
    QLineEdit, QMessageBox,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from mobRobURDF_wizard.classes.launch_manager import LaunchManager

logger = logging.getLogger(__name__)

try:
    import rclpy
    from geometry_msgs.msg import Twist
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False
    logger.warning("rclpy not available — teleop publishing disabled")

# Cardinal key groups
_FWD_KEYS   = {Qt.Key_W, Qt.Key_Up}
_BWD_KEYS   = {Qt.Key_S, Qt.Key_Down}
_LEFT_KEYS  = {Qt.Key_A, Qt.Key_Left}
_RIGHT_KEYS = {Qt.Key_D, Qt.Key_Right}
_STOP_KEYS  = {Qt.Key_Space}

# Diagonal key groups
_FWD_LEFT_KEYS  = {Qt.Key_Q}
_FWD_RIGHT_KEYS = {Qt.Key_E}
_BWD_LEFT_KEYS  = {Qt.Key_Z}
_BWD_RIGHT_KEYS = {Qt.Key_C}

_STRAFE_BTN_QSS = """
    QPushButton {
        background-color: #F0F3F4;
        border: 2px solid #BDC3C7;
        border-radius: 8px;
        color: #5D6D7E;
        font-size: 10pt;
        padding: 4px 12px;
    }
    QPushButton:checked {
        background-color: #2980B9;
        border-color: #2471A3;
        color: white;
        font-weight: bold;
    }
    QPushButton:disabled {
        background-color: #F8F9FA;
        border-color: #DEE2E6;
        color: #ADB5BD;
    }
    QPushButton:hover:!checked:!disabled {
        background-color: #EBF5FB;
        border-color: #AED6F1;
    }
"""

_DPAD_DIAG_QSS = """
    QPushButton {
        background-color: #F0F3F4;
        border: 2px solid #BDC3C7;
        border-radius: 10px;
        color: #5D6D7E;
        font-size: 12pt;
        font-weight: bold;
    }
    QPushButton:hover  { background-color: #EBF5FB; border-color: #AED6F1; }
    QPushButton:pressed { background-color: #2980B9; color: white; border-color: #2980B9; }
"""

_DPAD_BTN_QSS = """
    QPushButton {{
        background-color: {bg};
        border: 2px solid {border};
        border-radius: 10px;
        color: {fg};
        font-size: 22pt;
    }}
    QPushButton:hover  {{ background-color: {hover}; }}
    QPushButton:pressed {{ background-color: {press}; color: white; border-color: {press}; }}
"""


def _dpad_style(stop=False):
    if stop:
        return _DPAD_BTN_QSS.format(
            bg="#FADBD8", border="#E74C3C", fg="#C0392B",
            hover="#F1948A", press="#E74C3C",
        )
    return _DPAD_BTN_QSS.format(
        bg="#EBF5FB", border="#AED6F1", fg="#2C3E50",
        hover="#D6EAF8", press="#2980B9",
    )


class TeleoperationPage(QWizardPage):

    def __init__(self, urdf_manager, parent=None):
        super().__init__(parent)
        self.urdf_manager = urdf_manager

        self._node        = None
        self._pub         = None
        self._spin_thread = None
        self._spinning    = False
        self._held_keys   = set()
        self._btn_dirs    = set()   # active directions from D-pad buttons
        self._strafe_mode = False   # True → left/right → linear.y (mecanum only)
        self._launch_start_time = 0.0

        self.setTitle("Simulate & Teleop")
        self.setSubTitle(
            "Launch the simulation, then connect and drive the robot with the D-pad "
            "or keyboard (WASD / arrow keys)."
        )
        self.setFocusPolicy(Qt.StrongFocus)

        # 50 Hz publish timer — matches controller update_rate so every
        # controller cycle receives a fresh setpoint (reduces closed-loop hunting)
        self._timer = QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._tick)

        # Simulation launch manager
        self._launch_manager = LaunchManager(self)
        self._launch_manager.started.connect(self._on_sim_started)
        self._launch_manager.stopped.connect(self._on_sim_stopped)
        self._launch_manager.output.connect(lambda line: logger.info("[sim] %s", line))

        self._build_ui()

    # ── UI ─────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 12, 24, 12)
        root.setSpacing(10)

        # Full-width launch section — twice as wide as when it was in the left panel
        root.addWidget(self._build_launch_section())

        panels = QHBoxLayout()
        panels.setSpacing(24)
        panels.addWidget(self._build_left_panel(), 1)
        panels.addWidget(self._build_right_panel(), 1)
        root.addLayout(panels, 1)

    # ── Full-width launch section ───────────────────────────────────────────

    def _build_launch_section(self):
        box = QGroupBox()
        box.setFlat(True)
        vbox = QVBoxLayout(box)
        vbox.setContentsMargins(0, 0, 0, 4)
        vbox.setSpacing(4)

        self._launch_btn = QPushButton("Launch Simulation")
        self._launch_btn.setMinimumHeight(48)
        self._launch_btn.setProperty("btnRole", "success")
        self._launch_btn.clicked.connect(self._toggle_simulation)
        vbox.addWidget(self._launch_btn)

        self._sim_status = QLabel("")
        self._sim_status.setAlignment(Qt.AlignCenter)
        self._sim_status.setWordWrap(True)
        self._sim_status.setStyleSheet("font-size: 9pt; color: #7F8C8D;")
        vbox.addWidget(self._sim_status)

        return box

    # ── Left panel: D-pad + velocity readout ──────────────────────────────

    def _build_left_panel(self):
        box = QGroupBox("Controls")
        vbox = QVBoxLayout(box)
        vbox.setSpacing(14)

        # ── Teleop connection status row ───────────────────────────────────
        status_row = QHBoxLayout()
        self._status_dot = QLabel("●")
        self._status_dot.setStyleSheet("font-size: 14pt; color: #E74C3C;")
        self._status_lbl = QLabel("Not connected")
        self._status_lbl.setStyleSheet("font-size: 10pt; color: #7F8C8D;")
        status_row.addWidget(self._status_dot)
        status_row.addWidget(self._status_lbl)
        status_row.addStretch()
        vbox.addLayout(status_row)

        # D-pad grid (centered)
        dpad = self._build_dpad()
        center = QHBoxLayout()
        center.addStretch()
        center.addWidget(dpad)
        center.addStretch()
        vbox.addLayout(center)

        # Strafe mode toggle — enabled only for mecanum controller
        self._strafe_btn = QPushButton("⇔  Strafe Mode")
        self._strafe_btn.setMinimumHeight(36)
        self._strafe_btn.setCheckable(True)
        self._strafe_btn.setChecked(False)
        self._strafe_btn.setEnabled(False)
        self._strafe_btn.setToolTip(
            "Mecanum only — toggle lateral strafing.\n"
            "When active, ◄ / ► (and A/D) slide the robot sideways\n"
            "instead of rotating (sets linear.y instead of angular.z)."
        )
        self._strafe_btn.setStyleSheet(_STRAFE_BTN_QSS)
        self._strafe_btn.toggled.connect(self._on_strafe_toggled)
        vbox.addWidget(self._strafe_btn)

        # Velocity readout
        self._vel_lbl = QLabel("linear:  0.00 m/s    angular:  0.00 rad/s")
        self._vel_lbl.setAlignment(Qt.AlignCenter)
        self._vel_lbl.setStyleSheet(
            "font-size: 10pt; color: #5D6D7E; font-family: monospace;"
        )
        vbox.addWidget(self._vel_lbl)

        # Connect / Disconnect button
        self._conn_btn = QPushButton("Connect to /cmd_vel")
        self._conn_btn.setMinimumHeight(42)
        self._conn_btn.setProperty("btnRole", "success")
        self._conn_btn.clicked.connect(self._toggle_connection)
        vbox.addWidget(self._conn_btn)

        vbox.addStretch()
        return box

    def _build_dpad(self):
        w = QWidget()
        g = QGridLayout(w)
        g.setSpacing(6)
        g.setContentsMargins(0, 0, 0, 0)

        def cardinal(symbol, stop=False):
            b = QPushButton(symbol)
            b.setFixedSize(80, 80)
            b.setStyleSheet(_dpad_style(stop))
            return b

        def diagonal(symbol):
            b = QPushButton(symbol)
            b.setFixedSize(70, 70)
            b.setStyleSheet(_DPAD_DIAG_QSS)
            return b

        self._btn_fwd  = cardinal("▲")
        self._btn_bwd  = cardinal("▼")
        self._btn_left = cardinal("◄")
        self._btn_rgt  = cardinal("►")
        self._btn_stop = cardinal("■", stop=True)

        # Diagonal buttons: Q ↖  E ↗  Z ↙  C ↘
        self._btn_fl = diagonal("Q\n↖")
        self._btn_fr = diagonal("E\n↗")
        self._btn_bl = diagonal("Z\n↙")
        self._btn_br = diagonal("C\n↘")

        # All movement buttons: add/remove their direction tag while held,
        # then immediately publish so there's no wait for the next timer tick.
        for b, d in (
            (self._btn_fwd,  "fwd"),
            (self._btn_bwd,  "bwd"),
            (self._btn_left, "left"),
            (self._btn_rgt,  "right"),
            (self._btn_fl,   "diag_fl"),
            (self._btn_fr,   "diag_fr"),
            (self._btn_bl,   "diag_bl"),
            (self._btn_br,   "diag_br"),
        ):
            b.pressed.connect(lambda d=d: (self._btn_dirs.add(d), self._tick()))
            b.released.connect(lambda d=d: (self._btn_dirs.discard(d), self._tick()))

        self._btn_stop.clicked.connect(self._emergency_stop)

        # 3 × 3 grid:
        #  [Q↖] [W▲] [E↗]
        #  [A◄] [■ ] [D►]
        #  [Z↙] [S▼] [C↘]
        g.addWidget(self._btn_fl,   0, 0)
        g.addWidget(self._btn_fwd,  0, 1)
        g.addWidget(self._btn_fr,   0, 2)
        g.addWidget(self._btn_left, 1, 0)
        g.addWidget(self._btn_stop, 1, 1)
        g.addWidget(self._btn_rgt,  1, 2)
        g.addWidget(self._btn_bl,   2, 0)
        g.addWidget(self._btn_bwd,  2, 1)
        g.addWidget(self._btn_br,   2, 2)

        return w

    # ── Right panel: speed sliders + keyboard reference ────────────────────

    def _build_right_panel(self):
        box = QGroupBox("Settings")
        vbox = QVBoxLayout(box)
        vbox.setSpacing(16)

        # Topic field
        topic_row = QHBoxLayout()
        topic_row.addWidget(QLabel("Topic:"))
        self._topic_edit = QLineEdit("/cmd_vel")
        self._topic_edit.setPlaceholderText("/cmd_vel")
        topic_row.addWidget(self._topic_edit, 1)
        vbox.addLayout(topic_row)

        # Linear speed slider
        vbox.addWidget(self._speed_section(
            "Linear Speed", 1, 40, 5, "m/s",
            attr="_lin_slider", lbl_attr="_lin_lbl",
        ))

        # Angular speed slider
        vbox.addWidget(self._speed_section(
            "Angular Speed", 1, 40, 10, "rad/s",
            attr="_ang_slider", lbl_attr="_ang_lbl",
        ))

        # Keyboard reference card
        vbox.addWidget(self._build_keyboard_ref())
        vbox.addStretch()
        return box

    def _speed_section(self, title, lo, hi, default, unit, attr, lbl_attr):
        grp = QGroupBox(title)
        h = QHBoxLayout(grp)
        sl = QSlider(Qt.Horizontal)
        sl.setRange(lo, hi)
        sl.setValue(default)
        sl.setTickInterval(5)
        sl.setTickPosition(QSlider.TicksBelow)
        lbl = QLabel(f"{default/10:.1f} {unit}")
        lbl.setMinimumWidth(68)
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        sl.valueChanged.connect(lambda v, l=lbl, u=unit: l.setText(f"{v/10:.1f} {u}"))
        h.addWidget(sl, 1)
        h.addWidget(lbl)
        setattr(self, attr, sl)
        setattr(self, lbl_attr, lbl)
        return grp

    def _build_keyboard_ref(self):
        grp = QGroupBox("Keyboard Shortcuts")
        g = QGridLayout(grp)
        g.setSpacing(5)
        entries = [
            ("W  /  ↑",  "Forward"),
            ("S  /  ↓",  "Backward"),
            ("A  /  ←",  "Turn left  /  Strafe left ⇔"),
            ("D  /  →",  "Turn right  /  Strafe right ⇔"),
            ("Q",        "Forward-left  ↖"),
            ("E",        "Forward-right  ↗"),
            ("Z",        "Backward-left  ↙"),
            ("C",        "Backward-right  ↘"),
            ("Space",    "Emergency stop"),
        ]
        for i, (key, action) in enumerate(entries):
            key_lbl = QLabel(key)
            key_lbl.setAlignment(Qt.AlignCenter)
            key_lbl.setStyleSheet(
                "background:#EBF5FB; border:1px solid #AED6F1; "
                "border-radius:4px; padding:2px 8px; font-weight:bold; font-size:9pt;"
            )
            act_lbl = QLabel(action)
            act_lbl.setStyleSheet("font-size:9pt;")
            g.addWidget(key_lbl, i, 0)
            g.addWidget(act_lbl, i, 1)
        return grp

    # ── Simulation launch ───────────────────────────────────────────────────

    def _toggle_simulation(self):
        if self._launch_manager.is_running():
            self._launch_manager.stop()
            self._launch_btn.setEnabled(False)
            self._sim_status.setText("Stopping simulation…")
        else:
            urdf = self.urdf_manager.get_urdf_text()
            if not urdf or urdf.startswith("Error"):
                QMessageBox.warning(
                    self, "Cannot launch",
                    "No valid URDF found. Go back to Configure Parameters and click Apply first."
                )
                return
            self._launch_start_time = time.monotonic()
            self._launch_manager.start()

    def _on_sim_started(self):
        self._launch_btn.setText("Stop Simulation")
        self._launch_btn.setEnabled(True)
        self._launch_btn.setProperty("btnRole", "danger")
        self._launch_btn.style().unpolish(self._launch_btn)
        self._launch_btn.style().polish(self._launch_btn)
        self._sim_status.setText("Simulation running — Gazebo, controllers and RViz are starting…")

    def _on_sim_stopped(self, rc):
        self._launch_btn.setText("Launch Simulation")
        self._launch_btn.setEnabled(True)
        self._launch_btn.setProperty("btnRole", "success")
        self._launch_btn.style().unpolish(self._launch_btn)
        self._launch_btn.style().polish(self._launch_btn)
        elapsed = time.monotonic() - self._launch_start_time
        if rc not in (0, -2) and elapsed < 4:
            self._sim_status.setText("Simulation failed to start.")
            QMessageBox.warning(
                self, "Launch failed",
                f"The simulation exited immediately (code {rc}).\n\n"
                "Make sure the workspace is built and sourced:\n"
                "  colcon build --symlink-install\n  source install/setup.bash"
            )
        else:
            self._sim_status.setText("Simulation stopped.")

    # ── Connection lifecycle ────────────────────────────────────────────────

    def _toggle_connection(self):
        if self._pub is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if not _HAS_RCLPY:
            self._set_status(False, "rclpy unavailable")
            return
        topic = self._topic_edit.text().strip() or "/cmd_vel"
        try:
            if not rclpy.ok():
                rclpy.init()
            if self._node is None:
                self._node = rclpy.create_node("mobRobURDF_teleop")
            self._pub = self._node.create_publisher(Twist, topic, 10)
            self._spinning = True
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
            self._timer.start()
            self._conn_btn.setText("Disconnect")
            self._set_status(True, f"Connected  →  {topic}")
        except Exception as exc:
            self._set_status(False, str(exc))

    def _disconnect(self):
        self._timer.stop()
        self._spinning = False
        self._held_keys.clear()
        self._btn_dirs.clear()
        if self._pub:
            try:
                self._pub.publish(Twist())
            except Exception:
                pass
            self._pub = None
        self._conn_btn.setText("Connect to /cmd_vel")
        self._set_status(False, "Disconnected")
        self._vel_lbl.setText(
            "linear.x:  0.00 m/s    linear.y:  0.00 m/s"
            if self._strafe_mode else
            "linear:  0.00 m/s    angular:  0.00 rad/s"
        )

    def _spin_loop(self):
        while self._spinning and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.02)

    def _set_status(self, connected: bool, text: str):
        color = "#27AE60" if connected else "#E74C3C"
        self._status_dot.setStyleSheet(f"font-size: 14pt; color: {color};")
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(f"font-size: 10pt; color: {color};")

    # ── Strafe mode ─────────────────────────────────────────────────────────

    def _on_strafe_toggled(self, checked: bool):
        self._strafe_mode = checked
        self._tick()  # immediately reflect the mode change in the readout

    # ── Velocity publishing ─────────────────────────────────────────────────

    @property
    def _lin(self):
        return self._lin_slider.value() / 10.0

    @property
    def _ang(self):
        return self._ang_slider.value() / 10.0

    def _tick(self):
        linear = angular = lateral = 0.0

        # Map button directions to virtual keys (including diagonal combinations).
        extra: set = set()
        for tag, keys in (
            ("fwd",     {Qt.Key_W}),
            ("bwd",     {Qt.Key_S}),
            ("left",    {Qt.Key_A}),
            ("right",   {Qt.Key_D}),
            ("diag_fl", {Qt.Key_Q}),
            ("diag_fr", {Qt.Key_E}),
            ("diag_bl", {Qt.Key_Z}),
            ("diag_br", {Qt.Key_C}),
        ):
            if tag in self._btn_dirs:
                extra |= keys

        active = self._held_keys | extra

        # Linear.x: any forward key (including diagonals) → positive; backward → negative.
        if active & (_FWD_KEYS | _FWD_LEFT_KEYS | _FWD_RIGHT_KEYS):
            linear = self._lin
        elif active & (_BWD_KEYS | _BWD_LEFT_KEYS | _BWD_RIGHT_KEYS):
            linear = -self._lin

        if self._strafe_mode:
            # Strafe mode (mecanum): left/right → linear.y (sideways); angular stays 0.
            # Diagonal buttons produce true diagonal translation (linear.x + linear.y).
            if active & (_LEFT_KEYS | _FWD_LEFT_KEYS | _BWD_LEFT_KEYS):
                lateral = self._lin   # positive Y = left in ROS convention
            elif active & (_RIGHT_KEYS | _FWD_RIGHT_KEYS | _BWD_RIGHT_KEYS):
                lateral = -self._lin  # negative Y = right
        else:
            # Normal mode: left/right → angular.z (rotation).
            if active & (_LEFT_KEYS | _FWD_LEFT_KEYS | _BWD_LEFT_KEYS):
                angular = self._ang
            elif active & (_RIGHT_KEYS | _FWD_RIGHT_KEYS | _BWD_RIGHT_KEYS):
                angular = -self._ang

        self._publish(linear, angular, lateral)

    def _publish(self, linear: float, angular: float, lateral: float = 0.0):
        if self._pub is None:
            return
        msg = Twist()
        msg.linear.x  = linear
        msg.linear.y  = lateral
        msg.angular.z = angular
        try:
            self._pub.publish(msg)
        except Exception as e:
            logger.debug(f"Teleop publish error: {e}")
        if self._strafe_mode:
            self._vel_lbl.setText(
                f"linear.x: {linear:+.2f} m/s    linear.y: {lateral:+.2f} m/s"
            )
        else:
            self._vel_lbl.setText(
                f"linear:  {linear:+.2f} m/s    angular:  {angular:+.2f} rad/s"
            )

    def _emergency_stop(self):
        self._held_keys.clear()
        self._btn_dirs.clear()
        self._publish(0.0, 0.0, 0.0)

    # ── Keyboard events ─────────────────────────────────────────────────────

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in _STOP_KEYS:
            self._emergency_stop()
        else:
            self._held_keys.add(key)
            self._tick()   # publish immediately instead of waiting for next timer fire

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self._held_keys.discard(event.key())
            self._tick()   # send the stop (or reduced) command right away

    # ── Wizard page lifecycle ───────────────────────────────────────────────

    def initializePage(self):
        self.setFocus()
        # Enable strafe toggle only for mecanum controller.
        controller = ""
        try:
            controller = self.wizard().field("controllerType") or ""
        except Exception:
            pass
        is_mecanum = (controller == "mecanum")
        self._strafe_btn.setEnabled(is_mecanum)
        if not is_mecanum:
            # Uncheck without calling _tick (no connection yet on page entry)
            self._strafe_btn.blockSignals(True)
            self._strafe_btn.setChecked(False)
            self._strafe_btn.blockSignals(False)
            self._strafe_mode = False

    def cleanupPage(self):
        self._disconnect()

    def shutdown(self):
        """Called on window close to cleanly stop simulation and teleop."""
        self._disconnect()
        if self._launch_manager.is_running():
            self._launch_manager.stop()
