"""Reusable widgets that adapt to the available space.

These give the wizard its responsive feel across window sizes:
  * WrapButton       – wraps its label to two lines when it gets narrow
  * ButtonRow        – switches between a row and a column when narrow
  * ScaledPixmapLabel– aspect-fit rescales its pixmap on every resize
"""

from PyQt5.QtWidgets import QWidget, QPushButton, QBoxLayout, QSizePolicy, QLabel
from PyQt5.QtCore import Qt, QSize


class WrapButton(QPushButton):
    """QPushButton that breaks multi-word text onto two lines when narrow.

    Below ``_WRAP_WIDTH`` px the text splits so the button stays readable
    without clipping; above the threshold the single-line text is restored.
    """
    _WRAP_WIDTH = 180  # button width (px) below which wrapping activates

    def __init__(self, text, role=None, parent=None):
        super().__init__(text, parent)
        self._full_text = text
        self._words = text.split()
        if role:
            self.setProperty("btnRole", role)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.setMinimumHeight(40)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if len(self._words) < 2:
            return
        w = event.size().width()
        wants_wrap = (w < self._WRAP_WIDTH)
        mid = len(self._words) // 2
        wrapped = '\n'.join([
            ' '.join(self._words[:mid]),
            ' '.join(self._words[mid:]),
        ])
        cur = self.text()
        if wants_wrap and cur == self._full_text:
            self.setText(wrapped)
            self.setMinimumHeight(52)  # two wrapped lines need more height
        elif not wants_wrap and cur != self._full_text:
            self.setText(self._full_text)
            self.setMinimumHeight(40)


class ButtonRow(QWidget):
    """Two or more buttons laid out horizontally that stack vertically when
    the widget's own width drops below ``_STACK_WIDTH``."""
    _STACK_WIDTH = 220

    def __init__(self, *buttons, parent=None):
        super().__init__(parent)
        self._buttons = buttons
        self._layout = QBoxLayout(QBoxLayout.LeftToRight, self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(6)
        for b in buttons:
            self._layout.addWidget(b)
        self._horiz = True

    def resizeEvent(self, event):
        super().resizeEvent(event)
        horiz = event.size().width() >= self._STACK_WIDTH
        if horiz != self._horiz:
            self._horiz = horiz
            self._layout.setDirection(
                QBoxLayout.LeftToRight if horiz else QBoxLayout.TopToBottom
            )


class ScaledPixmapLabel(QLabel):
    """QLabel that rescales its pixmap to fill the available space on resize.

    Size hints are pinned to constants so a large loaded pixmap can't skew
    QWizard's ModernStyle layout (which divides vertical space using the
    page's size hint).
    """

    def __init__(self, hint=QSize(280, 260)):
        super().__init__()
        self._source = None
        self._hint = hint   # pinned hint prevents large pixmaps from skewing QWizard layout
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(80)

    def setSourcePixmap(self, pixmap):
        self._source = pixmap
        self._refresh()

    def sizeHint(self):
        return self._hint

    def minimumSizeHint(self):
        return QSize(80, 100)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh()

    def _refresh(self):
        if self._source and self.width() > 0 and self.height() > 0:
            super().setPixmap(self._source.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            ))
