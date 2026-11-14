"""Quiet industrial learning UI: semantic colors, readable type and clear focus."""
import re

COLORS = {
    'canvas': '#F3F6FA', 'surface': '#FFFFFF', 'ink': '#172B4D',
    'muted': '#52647C', 'border': '#DCE4EE', 'accent': '#176BDB',
    'accent_hover': '#1056B6', 'nav': '#142238', 'danger': '#AF2534',
}


def stylesheet(scale=1.0):
    css = '''
    QMainWindow { background: $canvas; }
    QWidget { color: $ink; font-family: "Noto Sans CJK SC"; font-size: 15px; }
    QFrame#sidebar { background: $nav; }
    #sidebar QLabel { color: #EDF3FC; background: transparent; }
    QLabel[role="brand"] { font-size: 12px; color: #9EC6FF; font-weight: 700; }
    QLabel[role="sideTitle"] { font-size: 23px; font-weight: 700; }
    #sidebar QLabel[role="sideMuted"] { color: #B0BFD3; font-size: 12px; }
    QLabel[role="title"] { font-size: 26px; font-weight: 700; }
    QLabel[role="section"] { font-size: 16px; font-weight: 700; }
    QLabel[role="eyebrow"] { color: $muted; font-size: 12px; }
    QLabel[role="muted"] { color: $muted; font-size: 13px; }
    QLabel[role="badge"] { background: #E7EFFC; color: #1A529E; border: 1px solid #CDDCF5;
        border-radius: 16px; padding: 8px 16px; font-size: 13px; font-weight: 600; }
    QLabel[role="notice"] { background: #EAF0F8; color: #3C5575; padding: 10px 14px;
        border-radius: 8px; font-size: 13px; }
    QFrame#card { background: $surface; border: 1px solid $border; border-radius: 12px; }
    QLabel#viewport { background: #152238; color: #C1CFE3; border-radius: 10px;
        padding: 8px; font-size: 14px; }
    QPushButton { background: $surface; border: 2px solid #CDD8E6;
        border-radius: 7px; padding: 7px 12px; font-weight: 500; }
    QPushButton:hover { background: #EDF3FD; border-color: #8CAEE0; }
    QPushButton:pressed { background: #DAE7FA; }
    QPushButton:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus,
    QLineEdit:focus, QTabBar::tab:focus { border: 2px solid $accent; }
    QPushButton[role="primary"] { background: $accent; border-color: $accent;
        color: white; font-weight: 600; }
    QPushButton[role="primary"]:hover { background: $accent_hover; }
    QPushButton[role="danger"] { color: $danger; background: #FFF5F5; border-color: #EDBBC1; }
    QPushButton[role="danger"]:hover { background: #FCE5E8; }
    QPushButton:disabled { background: #EEF2F7; color: #7A889D; border-color: #E0E6EF; }
    QPushButton#nav { color: #C0CDE0; background: transparent; text-align: left;
        border: 2px solid transparent; padding: 11px 9px; font-size: 14px; }
    QPushButton#nav:hover { background: #213650; }
    QPushButton#nav:checked { background: #25456C; color: #FFFFFF; border-color: #345A87; }
    QPushButton#nav:focus { border: 2px solid #9EC6FF; }
    QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit { background: #F8FAFD;
        border: 1px solid #CCD7E5; border-radius: 6px; padding: 8px; min-height: 24px; }
    QComboBox QAbstractItemView { background: $surface; selection-background-color: #DBE9FD;
        selection-color: $ink; padding: 4px; }
    QTabWidget::pane { border: none; background: transparent; }
    QTabBar::tab { background: #E6ECF4; color: $muted; border: 1px solid transparent;
        padding: 10px 12px; min-height: 22px; font-size: 14px; }
    QTabBar::tab:selected { background: $surface; color: $accent; border-bottom: 3px solid $accent; }
    QTabBar::tab:hover { background: #F4F8FE; }
    QScrollArea, QScrollArea > QWidget > QWidget { background: transparent; }
    QScrollBar:vertical { background: #EDF1F6; width: 10px; margin: 0; border-radius: 5px; }
    QScrollBar::handle:vertical { background: #B8C7DB; min-height: 30px; border-radius: 5px; }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
    QPlainTextEdit, QTextBrowser { background: #FBFCFE; border: none; padding: 8px; }
    QPlainTextEdit#log { background: #152238; color: #DFE8F6; border-radius: 8px;
        font-family: monospace; font-size: 13px; }
    QPlainTextEdit#code { font-family: monospace; font-size: 14px; }
    QTableWidget { border: none; background: $surface; gridline-color: #E8EEF6; font-size: 13px;
        selection-background-color: #DFEBFC; selection-color: $ink; }
    QHeaderView::section { font-size: 13px; background: #F0F4FA; color: $muted;
        border: none; padding: 8px; }
    QSplitter::handle { background: #DFE6F0; width: 6px; height: 6px; }
    QToolTip { background: #142238; color: #FFFFFF; border: none; padding: 8px; }
    '''
    for name in sorted(COLORS, key=len, reverse=True):
        css = css.replace('$' + name, COLORS[name])
    # Keep dimensions and fonts consistent when the user chooses a larger UI.
    return re.sub(r'(\d+)px', lambda m: f'{round(int(m.group(1)) * scale)}px', css)
