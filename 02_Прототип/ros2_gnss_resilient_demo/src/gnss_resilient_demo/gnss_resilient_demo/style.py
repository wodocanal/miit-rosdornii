"""Industrial operator-console visual language."""

COLORS = {
    "bg": "#10161b",
    "panel": "#172129",
    "panel_alt": "#1d2a33",
    "grid": "#2b3a43",
    "text": "#e5edf1",
    "muted": "#82949f",
    "amber": "#ffb547",
    "green": "#51d88a",
    "red": "#ff626d",
    "cyan": "#54cfe5",
    "orange": "#ff984f",
    "purple": "#b28cff",
    "truth": "#d5dce0",
}

APP_STYLESHEET = """
QWidget {
  background: #10161b; color: #e5edf1; font-family: "Ubuntu"; font-size: 28px;
}
QMainWindow { background: #0c1115; }
QFrame#card, QGroupBox {
  background: #172129; border: 1px solid #30404a; border-radius: 4px;
}
QGroupBox { margin-top: 38px; padding: 34px 14px 14px 14px; font-weight: 700; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; color: #ffb547; font-size: 30px; }
QPushButton, QComboBox {
  background: #24333d; border: 1px solid #40535f; border-radius: 5px;
  padding: 7px 14px; min-height: 58px;
}
QPushButton:hover { border-color: #ffb547; }
QPushButton:checked { background: #6a4a1e; border-color: #ffb547; color: #fff4de; }
QCheckBox { spacing: 13px; min-height: 58px; }
QCheckBox::indicator { width: 30px; height: 30px; }
QCheckBox::indicator:unchecked { border: 1px solid #61747f; background: #10161b; }
QCheckBox::indicator:checked { border: 1px solid #ffb547; background: #ffb547; }
QHeaderView::section { background: #22313a; color: #aebdc5; padding: 8px; border: 0; font-size: 24px; }
QTableWidget, QListWidget { background: #111920; border: 1px solid #30404a; gridline-color: #283740; }
QTableWidget { font-size: 24px; }
QListWidget { font-size: 24px; }
QScrollBar:vertical { background: #111920; width: 15px; }
QScrollBar::handle:vertical { background: #40535f; min-height: 24px; }
QLabel#kicker { color: #ffb547; font-family: "DejaVu Sans Mono"; font-size: 30px; font-weight: 700; }
QLabel#value { font-family: "DejaVu Sans Mono"; font-size: 38px; font-weight: 700; }
QLabel#muted { color: #82949f; }
QProgressBar {
  background: #10161b; border: 1px solid #40535f; border-radius: 4px;
  min-height: 42px; text-align: center; font-size: 24px; font-weight: 700;
}
QProgressBar::chunk { background: #51d88a; border-radius: 3px; }
"""
