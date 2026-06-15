import os
import logging
from PyQt5.QtWidgets import (
    QWizardPage, QVBoxLayout, QHBoxLayout, QLabel, QWidget, QScrollArea, QSizePolicy,
)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap
from ament_index_python.packages import get_package_share_directory

from mobRobURDF_wizard.classes.responsive_widgets import ScaledPixmapLabel

logger = logging.getLogger(__name__)

_CARD_H = 160   # card height

# (title, image_basename, badge_text, badge_color)
_FEATURES = [
    ("More Robot Models and Kinematics", "gazebo.png",    "In Progress", "#E67E22"),
    ("SLAM",                             "slam.png",       "In Progress", "#E67E22"),
    ("Navigation",                       "navigation.png", "Planned",     "#2980B9"),
    ("Fleet Management",                 "control.png",    "Planned",     "#2980B9"),
]


def _make_item(image_path: str, title_text: str, badge_text: str, badge_color: str) -> QWidget:
    """Feature row: image on left (fixed width), title + badge stacked on right (expanding)."""
    item = QWidget()
    item.setObjectName("featureItem")
    item.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    item.setFixedHeight(_CARD_H)
    item.setStyleSheet("""
        QWidget#featureItem {
            background-color: #FFFFFF;
            border: 1px solid #DDE1E4;
            border-radius: 10px;
        }
    """)

    row = QHBoxLayout(item)
    row.setContentsMargins(0, 0, 28, 0)
    row.setSpacing(0)

    # ── Left: image — expands to ~65 % of card width via stretch ──────────
    img = ScaledPixmapLabel(hint=QSize(600, _CARD_H))
    img.setFixedHeight(_CARD_H)
    img.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    img.setStyleSheet("border: none; border-radius: 10px;")
    if os.path.exists(image_path):
        img.setSourcePixmap(QPixmap(image_path))
    else:
        img.setText("Image not found")
        img.setStyleSheet(
            "color: #95A5A6; border: none; background: #F2F3F4; border-radius: 10px;"
        )
    row.addWidget(img, 13)   # image gets 13 parts

    row.addSpacing(24)

    # ── Right: title on top, badge below — ~35 % of card width ────────────
    text_col = QVBoxLayout()
    text_col.setContentsMargins(0, 0, 0, 0)
    text_col.setSpacing(10)
    text_col.addStretch()

    title_lbl = QLabel(title_text)
    title_lbl.setWordWrap(True)
    title_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    title_lbl.setStyleSheet(
        "font-size: 14pt; font-weight: bold; color: #2C3E50; "
        "background: transparent; border: none;"
    )
    text_col.addWidget(title_lbl)

    badge_lbl = QLabel(badge_text)
    badge_lbl.setFixedHeight(28)
    badge_lbl.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
    badge_lbl.setAlignment(Qt.AlignCenter)
    badge_lbl.setStyleSheet(
        f"font-size: 9pt; font-weight: bold; color: #FFFFFF; "
        f"background-color: {badge_color}; "
        f"border-radius: 5px; padding: 0px 16px; border: none;"
    )
    text_col.addWidget(badge_lbl, alignment=Qt.AlignLeft)

    text_col.addStretch()
    row.addLayout(text_col, 7)   # text gets 7 parts → image ≈ 65 %, text ≈ 35 %

    return item


class FutureFeaturesPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Future Features")

        image_dir = os.path.join(
            get_package_share_directory("mobRobURDF_wizard"),
            "images", "future_features",
        )

        outer = QVBoxLayout(self)
        outer.setContentsMargins(20, 14, 20, 16)
        outer.setSpacing(12)

        # ── Page header ────────────────────────────────────────────────────
        header = QLabel("What's next for the URDF Maker")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("font-size: 16pt; font-weight: bold; color: #2C3E50;")
        outer.addWidget(header)

        sub = QLabel(
            "These capabilities are currently under development or planned for future releases."
        )
        sub.setAlignment(Qt.AlignCenter)
        sub.setStyleSheet("font-size: 10pt; color: #7F8C8D;")
        outer.addWidget(sub)

        # ── Scrollable expanding card list ─────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        inner = QWidget()
        cards = QVBoxLayout(inner)
        cards.setContentsMargins(16, 10, 16, 10)
        cards.setSpacing(16)

        for title, image_name, badge, color in _FEATURES:
            path = os.path.join(image_dir, image_name)
            cards.addWidget(_make_item(path, title, badge, color))

        cards.addStretch()

        scroll.setWidget(inner)
        outer.addWidget(scroll, 1)
