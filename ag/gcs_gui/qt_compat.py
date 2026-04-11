"""Qt compatibility layer for PySide6 / PyQt5."""
from __future__ import annotations

API_NAME = ""

try:
    from PySide6.QtCore import Qt, QObject, Signal, Slot, QSize, QPointF, QRectF, QTimer
    from PySide6.QtGui import QAction, QCloseEvent, QColor, QFont, QImage, QKeySequence, QPainter, QPainterPath, QPen, QBrush, QPolygonF
    from PySide6.QtWidgets import (
        QApplication,
        QAbstractItemView,
        QButtonGroup,
        QCheckBox,
        QComboBox,
        QFileDialog,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QSizePolicy,
        QDoubleSpinBox,
        QSpinBox,
        QSplitter,
        QStatusBar,
        QTabWidget,
        QToolBar,
        QVBoxLayout,
        QWidget,
    )
    API_NAME = "PySide6"
except ImportError:  # pragma: no cover
    from PyQt5.QtCore import Qt, QObject, pyqtSignal as Signal, pyqtSlot as Slot, QSize, QPointF, QRectF, QTimer
    from PyQt5.QtGui import QAction, QCloseEvent, QColor, QFont, QImage, QKeySequence, QPainter, QPainterPath, QPen, QBrush, QPolygonF
    from PyQt5.QtWidgets import (
        QApplication,
        QAbstractItemView,
        QButtonGroup,
        QCheckBox,
        QComboBox,
        QFileDialog,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QSizePolicy,
        QDoubleSpinBox,
        QSpinBox,
        QSplitter,
        QStatusBar,
        QTabWidget,
        QToolBar,
        QVBoxLayout,
        QWidget,
    )
    API_NAME = "PyQt5"
