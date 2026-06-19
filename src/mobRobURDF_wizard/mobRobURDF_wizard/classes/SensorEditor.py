"""SensorCard — compact QGroupBox with QLineEdit fields for every SensorConfig parameter."""

from PyQt5.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QFrame, QWidget,
)
from PyQt5.QtCore import pyqtSignal, Qt

from mobRobURDF_wizard.classes.sensor_config import SensorConfig


class SensorCard(QGroupBox):
    """Editable card for one sensor. Emits ``deleted`` when the × button is clicked."""

    deleted = pyqtSignal(object)   # payload: self

    def __init__(self, sensor_type: str = 'lidar', parent=None):
        super().__init__(parent)
        self._type = sensor_type
        self._build_ui()

    def _edit(self, value='', width=None):
        e = QLineEdit(str(value))
        if width:
            e.setFixedWidth(width)
        return e

    def _triplet(self, v1=0.0, v2=0.0, v3=0.0, l1='x', l2='y', l3='z'):
        """Three small labeled fields in one row. Returns (widget, e1, e2, e3)."""
        w = QWidget()
        h = QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(3)
        edits = []
        for lbl_txt, val in ((l1, v1), (l2, v2), (l3, v3)):
            col = QVBoxLayout()
            col.setSpacing(1)
            lbl = QLabel(lbl_txt)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet('font-size: 7pt; color: #7F8C8D;')
            e = QLineEdit(str(val))
            e.setFixedWidth(68)
            col.addWidget(lbl)
            col.addWidget(e)
            h.addLayout(col)
            edits.append(e)
        return w, edits[0], edits[1], edits[2]

    def _unit_lbl(self, text):
        u = QLabel(text)
        u.setStyleSheet('color: #95A5A6; font-size: 8pt;')
        return u

    def _pair_grid_row(self, grid, row, lbl1, val1, lbl2, val2, unit1='', unit2=''):
        """Add two label+edit(+unit) pairs to a 6-column grid row."""
        grid.addWidget(QLabel(lbl1), row, 0)
        e1 = self._edit(val1)
        grid.addWidget(e1, row, 1)
        if unit1:
            grid.addWidget(self._unit_lbl(unit1), row, 2)
        grid.addWidget(QLabel(lbl2), row, 3)
        e2 = self._edit(val2)
        grid.addWidget(e2, row, 4)
        if unit2:
            grid.addWidget(self._unit_lbl(unit2), row, 5)
        return e1, e2

    def _separator(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def _build_ui(self):
        vbox = QVBoxLayout(self)
        vbox.setSpacing(5)
        vbox.setContentsMargins(7, 6, 7, 8)

        hdr = QHBoxLayout()
        icon = 'Lidar' if self._type == 'lidar' else 'Camera'
        hdr_lbl = QLabel(f'<b>{icon}</b>')
        hdr_lbl.setStyleSheet('font-size: 9pt;')
        del_btn = QPushButton('×')
        del_btn.setFixedSize(22, 22)
        del_btn.setStyleSheet(
            'QPushButton { border: none; color: white; font-weight: bold; font-size: 13pt; }'
            'QPushButton:hover { color: #DDDDDD; }'
        )
        del_btn.clicked.connect(lambda: self.deleted.emit(self))
        hdr.addWidget(hdr_lbl)
        hdr.addStretch()
        hdr.addWidget(del_btn)
        vbox.addLayout(hdr)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel('Name:'))
        default_name = 'lidar_1' if self._type == 'lidar' else 'camera_1'
        self.nameEdit = self._edit(default_name)
        name_row.addWidget(self.nameEdit)
        vbox.addLayout(name_row)

        pos_w, self.xEdit, self.yEdit, self.zEdit = self._triplet(
            0.0, 0.0, 0.3 if self._type == 'lidar' else 0.0, 'x', 'y', 'z')
        pos_row = QHBoxLayout()
        pos_row.addWidget(QLabel('Pos (m):'))
        pos_row.addWidget(pos_w)
        vbox.addLayout(pos_row)

        rot_w, self.rollEdit, self.pitchEdit, self.yawEdit = self._triplet(
            0.0, 0.0, 0.0, 'R', 'P', 'Y')
        rot_row = QHBoxLayout()
        rot_row.addWidget(QLabel('RPY (rad):'))
        rot_row.addWidget(rot_w)
        vbox.addLayout(rot_row)

        cm = QGridLayout()
        cm.setSpacing(4)
        cm.addWidget(QLabel('Color:'), 0, 0)
        self.colorEdit = self._edit('Black' if self._type == 'lidar' else 'Blue')
        cm.addWidget(self.colorEdit, 0, 1)
        cm.addWidget(QLabel('Mass:'), 0, 2)
        self.massEdit = self._edit('0.1')
        cm.addWidget(self.massEdit, 0, 3)
        cm.addWidget(self._unit_lbl('kg'), 0, 4)
        vbox.addLayout(cm)

        vbox.addWidget(self._separator())

        if self._type == 'lidar':
            self._build_lidar_fields(vbox)
        else:
            self._build_camera_fields(vbox)

        self.setStyleSheet('QGroupBox { border: 1px solid #BDC3C7; border-radius: 4px; margin-top: 2px; }')

    def _build_lidar_fields(self, vbox):
        g = QGridLayout()
        g.setSpacing(4)
        g.setColumnStretch(1, 1)
        g.setColumnStretch(4, 1)
        self.radiusEdit, self.heightEdit   = self._pair_grid_row(g, 0, 'Radius:',  '0.1',      'Height:',  '0.08',     'm',   'm')
        self.samplesEdit, self.rateEdit    = self._pair_grid_row(g, 1, 'Samples:', '360',       'Rate:',    '10.0',     '',    'Hz')
        self.minAngEdit, self.maxAngEdit   = self._pair_grid_row(g, 2, 'MinAng:',  '-3.14159',  'MaxAng:',  '3.14159',  'rad', 'rad')
        self.minRngEdit, self.maxRngEdit   = self._pair_grid_row(g, 3, 'MinRng:',  '0.3',       'MaxRng:',  '12.0',     'm',   'm')
        vbox.addLayout(g)

    def _build_camera_fields(self, vbox):
        g = QGridLayout()
        g.setSpacing(4)
        g.setColumnStretch(1, 1)
        g.setColumnStretch(4, 1)
        self.camDepthEdit, self.camWidthEdit   = self._pair_grid_row(g, 0, 'Depth:',   '0.08',    'Width:',   '0.08',    'm',   'm')
        self.camHeightEdit, self.camRateEdit   = self._pair_grid_row(g, 1, 'Height:',  '0.06',    'Rate:',    '10.0',    'm',   'Hz')
        self.hFovEdit, self.vFovEdit           = self._pair_grid_row(g, 2, 'H-FOV:',   '1.089',   'V-FOV:',   '0.785',   'rad', 'rad')
        self.imgWEdit, self.imgHEdit           = self._pair_grid_row(g, 3, 'ImgW:',    '640',     'ImgH:',    '480',     'px',  'px')
        self.nearEdit, self.farEdit            = self._pair_grid_row(g, 4, 'Near:',    '0.05',    'Far:',     '8.0',     'm',   'm')
        vbox.addLayout(g)

    def _f(self, edit: QLineEdit, default: float) -> float:
        try:
            return float(edit.text())
        except ValueError:
            return default

    def _i(self, edit: QLineEdit, default: int) -> int:
        try:
            return int(edit.text())
        except ValueError:
            return default

    def to_sensor_config(self) -> SensorConfig:
        s = SensorConfig(sensor_type=self._type, name=self.nameEdit.text().strip() or self._type + '_1')
        s.x = self._f(self.xEdit, 0.0)
        s.y = self._f(self.yEdit, 0.0)
        s.z = self._f(self.zEdit, 0.3 if self._type == 'lidar' else 0.0)
        s.roll = self._f(self.rollEdit, 0.0)
        s.pitch = self._f(self.pitchEdit, 0.0)
        s.yaw = self._f(self.yawEdit, 0.0)
        s.color = self.colorEdit.text().strip() or ('Black' if self._type == 'lidar' else 'Blue')
        s.mass = self._f(self.massEdit, 0.1)

        if self._type == 'lidar':
            s.radius = self._f(self.radiusEdit, 0.1)
            s.length = self._f(self.heightEdit, 0.08)
            s.h_samples = self._i(self.samplesEdit, 360)
            s.update_rate = self._f(self.rateEdit, 10.0)
            s.h_min_angle = self._f(self.minAngEdit, -3.14159)
            s.h_max_angle = self._f(self.maxAngEdit, 3.14159)
            s.min_range = self._f(self.minRngEdit, 0.3)
            s.max_range = self._f(self.maxRngEdit, 12.0)
        else:
            s.cam_depth = self._f(self.camDepthEdit, 0.08)
            s.cam_width = self._f(self.camWidthEdit, 0.08)
            s.cam_height = self._f(self.camHeightEdit, 0.06)
            s.cam_update_rate = self._f(self.camRateEdit, 10.0)
            s.h_fov = self._f(self.hFovEdit, 1.089)
            s.v_fov = self._f(self.vFovEdit, 0.785)
            s.img_width = self._i(self.imgWEdit, 640)
            s.img_height = self._i(self.imgHEdit, 480)
            s.near_clip = self._f(self.nearEdit, 0.05)
            s.far_clip = self._f(self.farEdit, 8.0)
        return s

    def load_sensor_config(self, s: SensorConfig):
        """Populate all fields from a SensorConfig (e.g., when loading a preset)."""
        self.nameEdit.setText(s.name)
        self.xEdit.setText(str(s.x))
        self.yEdit.setText(str(s.y))
        self.zEdit.setText(str(s.z))
        self.rollEdit.setText(str(s.roll))
        self.pitchEdit.setText(str(s.pitch))
        self.yawEdit.setText(str(s.yaw))
        self.colorEdit.setText(s.color)
        self.massEdit.setText(str(s.mass))

        if self._type == 'lidar':
            self.radiusEdit.setText(str(s.radius))
            self.heightEdit.setText(str(s.length))
            self.samplesEdit.setText(str(s.h_samples))
            self.rateEdit.setText(str(s.update_rate))
            self.minAngEdit.setText(str(s.h_min_angle))
            self.maxAngEdit.setText(str(s.h_max_angle))
            self.minRngEdit.setText(str(s.min_range))
            self.maxRngEdit.setText(str(s.max_range))
        else:
            self.camDepthEdit.setText(str(s.cam_depth))
            self.camWidthEdit.setText(str(s.cam_width))
            self.camHeightEdit.setText(str(s.cam_height))
            self.camRateEdit.setText(str(s.cam_update_rate))
            self.hFovEdit.setText(str(s.h_fov))
            self.vFovEdit.setText(str(s.v_fov))
            self.imgWEdit.setText(str(s.img_width))
            self.imgHEdit.setText(str(s.img_height))
            self.nearEdit.setText(str(s.near_clip))
            self.farEdit.setText(str(s.far_clip))
