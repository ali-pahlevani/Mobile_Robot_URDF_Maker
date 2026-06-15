"""Built-in teleoperation page.

Lets the user drive the robot directly from the wizard using on-screen
D-pad buttons or keyboard shortcuts (WASD / arrow keys).  Publishes
geometry_msgs/Twist to /cmd_vel at 10 Hz via an rclpy node running in a
background daemon thread.
"""

import threading
import logging
from PyQt5.QtWidgets import (
    QWizardPage, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QGroupBox, QWidget, QSizePolicy,
    QLineEdit,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

logger = logging.getLogger(__name__)

try:
    import rclpy
    from geometry_msgs.msg import Twist
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False
    logger.warning("rclpy not available — teleop publishing disabled")

# Key groups
_FWD_KEYS   = {Qt.Key_W, Qt.Key_Up}
_BWD_KEYS   = {Qt.Key_S, Qt.Key_Down}
_LEFT_KEYS  = {Qt.Key_A, Qt.Key_Left}
_RIGHT_KEYS = {Qt.Key_D, Qt.Key_Right}
_STOP_KEYS  = {Qt.Key_Space}

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

        self.setTitle("Teleoperation")
        self.setSubTitle(
            "Drive the robot in real time using the D-pad or keyboard (WASD / arrow keys). "
            "Launch the simulation first, then click Connect."
        )
        self.setFocusPolicy(Qt.StrongFocus)

        # 10 Hz publish timer
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._tick)

        self._build_ui()

    # ── UI ─────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(24, 12, 24, 12)
        root.setSpacing(24)
        root.addWidget(self._build_left_panel(), 1)
        root.addWidget(self._build_right_panel(), 1)

    # ── Left panel: status + D-pad + velocity readout ──────────────────────

    def _build_left_panel(self):
        box = QGroupBox("Controls")
        vbox = QVBoxLayout(box)
        vbox.setSpacing(14)

        # Status row
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
        g.setSpacing(8)
        g.setContentsMargins(0, 0, 0, 0)

        def btn(symbol, stop=False):
            b = QPushButton(symbol)
            b.setFixedSize(80, 80)
            b.setStyleSheet(_dpad_style(stop))
            return b

        self._btn_fwd  = btn("▲")
        self._btn_bwd  = btn("▼")
        self._btn_left = btn("◄")
        self._btn_rgt  = btn("►")
        self._btn_stop = btn("■", stop=True)

        # Forward / backward / left / right: held while pressed
        for b, d in (
            (self._btn_fwd,  "fwd"),
            (self._btn_bwd,  "bwd"),
            (self._btn_left, "left"),
            (self._btn_rgt,  "right"),
        ):
            b.pressed.connect(lambda d=d: self._btn_dirs.add(d))
            b.released.connect(lambda d=d: self._btn_dirs.discard(d))

        self._btn_stop.clicked.connect(self._emergency_stop)

        g.addWidget(self._btn_fwd,  0, 1)
        g.addWidget(self._btn_left, 1, 0)
        g.addWidget(self._btn_stop, 1, 1)
        g.addWidget(self._btn_rgt,  1, 2)
        g.addWidget(self._btn_bwd,  2, 1)

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
        g.setSpacing(6)
        entries = [
            ("W  /  ↑",    "Forward"),
            ("S  /  ↓",    "Backward"),
            ("A  /  ←",    "Turn left"),
            ("D  /  →",    "Turn right"),
            ("Space",      "Emergency stop"),
        ]
        for i, (key, action) in enumerate(entries):
            key_lbl = QLabel(key)
            key_lbl.setAlignment(Qt.AlignCenter)
            key_lbl.setStyleSheet(
                "background:#EBF5FB; border:1px solid #AED6F1; "
                "border-radius:4px; padding:2px 10px; font-weight:bold; font-size:9pt;"
            )
            act_lbl = QLabel(action)
            act_lbl.setStyleSheet("font-size:9.5pt;")
            g.addWidget(key_lbl, i, 0)
            g.addWidget(act_lbl, i, 1)
        return grp

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
        self._vel_lbl.setText("linear:  0.00 m/s    angular:  0.00 rad/s")

    def _spin_loop(self):
        while self._spinning and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.02)

    def _set_status(self, connected: bool, text: str):
        color = "#27AE60" if connected else "#E74C3C"
        self._status_dot.setStyleSheet(f"font-size: 14pt; color: {color};")
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(f"font-size: 10pt; color: {color};")

    # ── Velocity publishing ─────────────────────────────────────────────────

    @property
    def _lin(self):
        return self._lin_slider.value() / 10.0

    @property
    def _ang(self):
        return self._ang_slider.value() / 10.0

    def _tick(self):
        linear = angular = 0.0

        active = self._held_keys | {
            Qt.Key_W     if "fwd"   in self._btn_dirs else None,
            Qt.Key_S     if "bwd"   in self._btn_dirs else None,
            Qt.Key_A     if "left"  in self._btn_dirs else None,
            Qt.Key_D     if "right" in self._btn_dirs else None,
        } - {None}

        if active & _FWD_KEYS:
            linear  =  self._lin
        elif active & _BWD_KEYS:
            linear  = -self._lin
        if active & _LEFT_KEYS:
            angular =  self._ang
        elif active & _RIGHT_KEYS:
            angular = -self._ang

        self._publish(linear, angular)

    def _publish(self, linear: float, angular: float):
        if self._pub is None:
            return
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        try:
            self._pub.publish(msg)
        except Exception as e:
            logger.debug(f"Teleop publish error: {e}")
        self._vel_lbl.setText(
            f"linear:  {linear:+.2f} m/s    angular:  {angular:+.2f} rad/s"
        )

    def _emergency_stop(self):
        self._held_keys.clear()
        self._btn_dirs.clear()
        self._publish(0.0, 0.0)

    # ── Keyboard events ─────────────────────────────────────────────────────

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in _STOP_KEYS:
            self._emergency_stop()
        else:
            self._held_keys.add(key)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self._held_keys.discard(event.key())

    # ── Wizard page lifecycle ───────────────────────────────────────────────

    def initializePage(self):
        self.setFocus()

    def cleanupPage(self):
        self._disconnect()
