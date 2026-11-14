"""Qt 学习工作台：管理本软件拥有的仿真，不接管外部 ROS 会话。"""
from __future__ import annotations

from datetime import datetime
import base64
import binascii
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

import yaml

from PyQt5.QtCore import (QEvent, QObject, QRunnable, QSettings, Qt, QThreadPool, QTimer,
                          QUrl, pyqtSignal)
from PyQt5.QtGui import QColor, QDesktopServices, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFrame, QGridLayout,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QMessageBox,
    QPlainTextEdit, QPushButton, QScrollArea, QSizePolicy, QSpinBox, QSplitter,
    QStackedWidget, QTabWidget,
    QTableWidget, QTableWidgetItem, QTextBrowser, QVBoxLayout, QWidget,
)

from . import __version__
from .theme import stylesheet
from .ros_bridge import PREFIX
from .runtime import ProcessManager, check_environment, ros_command, simulation_conflicts

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
STATE_NAMES = {"idle": "待出题", "armed": "题目就绪", "running": "本轮运行中", "finished": "本轮结束"}
# ros_bridge monitor reports joint feedback once per second and considers it
# valid for <4 s. Keep this UI deadman freshness limit aligned with that source.
JOINT_FEEDBACK_MAX_AGE = 4.0


def _display_scale() -> float:
    """Estimate the desktop UI scale, including GNOME fractional scaling.

    On this machine X11 reports a 4384×2466 virtual desktop for a 4K panel and
    Qt reports 192 DPI. Fixed pixel fonts therefore look tiny unless the UI
    applies a matching scale. Keep a conservative cap so very high-DPI panels
    remain usable rather than producing oversized controls.
    """
    override = os.environ.get("WORKBENCH_UI_SCALE", "").strip()
    if override:
        try:
            return max(1.0, min(1.75, float(override)))
        except ValueError:
            pass
    screen = QApplication.primaryScreen()
    dpi = float(screen.logicalDotsPerInch()) if screen else 96.0
    return max(1.0, min(1.75, dpi / 96.0))


class _X11WindowEmbedder:
    """Reparent a foreign X11 top-level window into a native Qt host window."""

    def __init__(self):
        import ctypes
        import ctypes.util

        library = ctypes.util.find_library("X11")
        if not library:
            raise RuntimeError("找不到 X11 客户端库")
        self.x11 = ctypes.cdll.LoadLibrary(library)
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        display_name = os.environ.get("DISPLAY")
        self.display = self.x11.XOpenDisplay(display_name.encode() if display_name else None)
        if not self.display:
            raise RuntimeError("无法连接 X11 DISPLAY")
        self.x11.XDefaultScreen.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultScreen.restype = ctypes.c_int
        self.screen = self.x11.XDefaultScreen(self.display)
        self.x11.XRootWindow.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XRootWindow.restype = ctypes.c_ulong
        self.x11.XWithdrawWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int]
        self.x11.XReparentWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_int, ctypes.c_int]
        self.x11.XResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_uint, ctypes.c_uint]
        self.x11.XMapWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                       ctypes.POINTER(ctypes.c_ulong),
                                       ctypes.POINTER(ctypes.c_ulong),
                                       ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
                                       ctypes.POINTER(ctypes.c_uint)]
        self.x11.XQueryTree.restype = ctypes.c_int
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XFree.restype = ctypes.c_int
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.restype = ctypes.c_int
        self.window_id = None
        self.host_id = None

    def _parent(self, window_id: int) -> int:
        import ctypes

        root = ctypes.c_ulong()
        parent = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        ok = self.x11.XQueryTree(self.display, window_id, ctypes.byref(root),
                                 ctypes.byref(parent), ctypes.byref(children),
                                 ctypes.byref(count))
        if children:
            self.x11.XFree(children)
        if not ok:
            raise RuntimeError(f"无法查询 Gazebo X11 窗口 {window_id:#x}")
        return parent.value

    def embed(self, window_id: int, host_id: int, width: int, height: int):
        # XWithdrawWindow is asynchronous with respect to the window manager.
        # Reparenting immediately races Mutter, which can put Gazebo back into
        # its decoration frame. Wait until the WM has actually withdrawn it.
        if self._parent(window_id) != host_id:
            self.x11.XWithdrawWindow(self.display, window_id, self.screen)
            self.x11.XSync(self.display, 0)
            deadline = time.monotonic() + 1.0
            root = self.x11.XRootWindow(self.display, self.screen)
            while time.monotonic() < deadline:
                if self._parent(window_id) == root:
                    break
                time.sleep(0.01)
            else:
                raise RuntimeError("窗口管理器未释放 Gazebo 窗口，无法嵌入")
            self.x11.XReparentWindow(self.display, window_id, host_id, 0, 0)
            self.x11.XSync(self.display, 0)
            if self._parent(window_id) != host_id:
                raise RuntimeError("Gazebo 窗口重挂失败（父窗口不匹配）")
            self.window_id, self.host_id = window_id, host_id
        self.x11.XResizeWindow(self.display, window_id, max(1, width), max(1, height))
        self.x11.XMapWindow(self.display, window_id)
        self.x11.XFlush(self.display)

    def close(self):
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None
            self.window_id = None
            self.host_id = None


class _Signals(QObject):
    result = pyqtSignal(object)


class _CheckTask(QRunnable):
    def __init__(self, root: Path, mode: str, backend: str = "gazebo"):
        super().__init__()
        self.root, self.mode, self.backend = root, mode, backend
        self.signals = _Signals()

    def run(self):
        try:
            payload = {"checks": check_environment(self.root, self.mode, self.backend),
                       "conflicts": simulation_conflicts()}
        except Exception as exc:
            payload = {"checks": [{"name": "环境检查", "level": "error", "detail": str(exc)}],
                       "conflicts": []}
        self.signals.result.emit(payload)


def _button(text, handler, primary=False, danger=False):
    button = QPushButton(text)
    button.setCursor(Qt.PointingHandCursor)
    button.setMinimumHeight(round(44 * _display_scale()))
    button.setAccessibleName(text)
    if primary:
        button.setProperty("role", "primary")
    if danger:
        button.setProperty("role", "danger")
    button.clicked.connect(handler)
    return button


def _label(text, role=None):
    label = QLabel(text)
    label.setWordWrap(True)
    if role:
        label.setProperty("role", role)
    return label


def _card(title):
    frame = QFrame()
    frame.setObjectName("card")
    layout = QVBoxLayout(frame)
    scale = _display_scale()
    layout.setContentsMargins(round(18 * scale), round(15 * scale),
                              round(18 * scale), round(15 * scale))
    layout.setSpacing(round(12 * scale))
    if title:
        layout.addWidget(_label(title, "section"))
    return frame, layout


class WorkbenchWindow(QMainWindow):
    def __init__(self, root: Path, data_dir: Path, demo=False, mode="basic"):
        super().__init__()
        self.root, self.data_dir, self.demo = Path(root), Path(data_dir), bool(demo)
        self._pose_init_error = ""
        for name in ("logs", "results", "programs", "config"):
            (self.data_dir / name).mkdir(parents=True, exist_ok=True)
        self._ensure_pose_config()
        self.manager = ProcessManager(self.data_dir / "logs")
        self.settings = QSettings(str(self.data_dir / "preferences.ini"), QSettings.IniFormat)
        self.ready = False
        self.run_state = "idle"
        self.scene = {}
        self.score = None
        self.joint_positions = {}
        self.joint_sample_at = 0.0
        self._pose_inflight = False
        self._jog_held = None
        self._jog_result_received = False
        self._jog_fault = False
        self.last_status = 0.0
        self.stack_started = 0.0
        self._checking = False
        self._start_after_check = False
        self._check_task = None
        self._checks = []
        self._check_mode = mode
        self._service_inflight = False
        self._service_name = ""
        self._service_success = False
        self._service_payload = None
        self._service_queue = []
        self._program_inflight = False
        self._auto_start_reference = False
        self._reference_wait_at = 0.0
        self._closing_stack = False
        self._gazebo_window_id = None
        self._gazebo_refresh_at = 0.0
        self._gazebo_embedder = None
        self._shutdown_done = False
        self._demo_stack = False
        self._demo_running = False
        self._demo_step = 0
        self._demo_epoch = 0
        self.setWindowTitle("基础组学习工作台 · JAKA Mini 2")
        self.ui_scale = _display_scale()
        screen = QApplication.primaryScreen()
        available = screen.availableGeometry() if screen else None
        self.setMinimumSize(960, 620)
        self.resize(min(round(1440 * self.ui_scale), available.width() if available else 1440),
                    min(round(920 * self.ui_scale), available.height() if available else 920))
        self._build_ui()
        if self._pose_init_error:
            self._log(self._pose_init_error)
        self.mode_combo.setCurrentIndex(max(0, self.mode_combo.findData(mode)))
        self._style()
        self._refresh_results()
        self.timer = QTimer(self)
        self.timer.setInterval(150)
        self.timer.timeout.connect(self._tick)
        self.timer.start()
        self._log("工作台已打开。不会自动停止外部仿真，也不会修改现有抓取算法。")
        if self.demo:
            self._log("演示模式：所有状态和结果均为示例，不会启动 ROS 或移动机械臂。")
        else:
            QTimer.singleShot(100, self.check_now)
        self._update_buttons()

    @property
    def mode(self):
        return self.mode_combo.currentData() or "basic"

    def _build_ui(self):
        outer = QWidget()
        row = QHBoxLayout(outer)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        self.sidebar = QFrame()
        self.sidebar.setObjectName("sidebar")
        self.sidebar.setFixedWidth(round(176 * self.ui_scale))
        side = QVBoxLayout(self.sidebar)
        side.setContentsMargins(16, 24, 16, 20)
        side.setSpacing(10)
        side.addWidget(_label("JAKA / LEARN", "brand"))
        self.sidebar_title = _label("机器人学习\n工作台", "sideTitle")
        side.addWidget(self.sidebar_title)
        side.addWidget(_label("Mini 2 · 基础组", "sideMuted"))
        side.addSpacing(22)
        self.nav_buttons = []
        for index, title in enumerate(("01   练习控制台", "02   我的程序", "03   运行记录", "04   入门指南")):
            button = _button(title, lambda checked=False, i=index: self._navigate(i))
            button.setCheckable(True)
            button.setObjectName("nav")
            self.nav_buttons.append(button)
            side.addWidget(button)
        side.addStretch()
        side.addWidget(_label("SIMULATION ONLY", "sideMuted"))
        side.addWidget(_label("仿真学习 · 不连接真机", "sideMuted"))
        side.addWidget(_label(f"v{__version__}  /  {'DEMO' if self.demo else 'LOCAL'}", "sideMuted"))
        row.addWidget(self.sidebar)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(24, 20, 24, 14)
        layout.setSpacing(12)
        heading = QHBoxLayout()
        title_col = QVBoxLayout()
        title_col.setSpacing(4)
        self.page_title = _label("练习控制台", "title")
        self.page_subtitle = _label("先运行基础模型，再调好每一个姿态。", "muted")
        title_col.addWidget(self.page_title)
        title_col.addWidget(self.page_subtitle)
        heading.addLayout(title_col, 1)
        self.status_badge = _label("演示模式" if self.demo else "尚未启动", "badge")
        self.status_badge.setWordWrap(False)
        heading.addWidget(self.status_badge)
        layout.addLayout(heading)
        self.notice = _label("演示模式：不启动 ROS；画面、动画与成绩均非实际仿真。" if self.demo else
                             "仅用于仿真学习。Gazebo 与腕部相机同时启动；真机使用前必须重新标定。", "notice")
        layout.addWidget(self.notice)
        self.content_splitter = QSplitter(Qt.Vertical)
        self.pages = QStackedWidget()
        self.pages.addWidget(self._home_page())
        self.pages.addWidget(self._code_page())
        self.pages.addWidget(self._results_page())
        self.pages.addWidget(self._guide_page())
        self.content_splitter.addWidget(self.pages)
        self.log_frame, log_layout = _card("")
        log_head = QHBoxLayout()
        log_head.addWidget(_label("运行日志", "section"))
        log_head.addStretch()
        log_head.addWidget(_button("打开目录", lambda: self._open(self.data_dir / "logs")))
        log_head.addWidget(_button("清空显示", lambda: self.log_view.clear()))
        log_layout.addLayout(log_head)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1800)
        self.log_view.setObjectName("log")
        log_layout.addWidget(self.log_view)
        self.log_frame.setMinimumHeight(140)
        self.content_splitter.addWidget(self.log_frame)
        self.content_splitter.setChildrenCollapsible(False)
        self.content_splitter.setStretchFactor(0, 4)
        self.content_splitter.setStretchFactor(1, 1)
        self.log_frame.hide()
        layout.addWidget(self.content_splitter, 1)
        footer_row = QHBoxLayout()
        self.footer = _label("本地仿真学习 · 配置和程序自动保存在本机", "muted")
        self.footer.setToolTip(f"工程：{self.root}\n学习数据：{self.data_dir}")
        footer_row.addWidget(self.footer, 1)
        self.log_toggle = _button("展开日志", self._toggle_logs)
        self.log_toggle.setCheckable(True)
        footer_row.addWidget(self.log_toggle)
        layout.addLayout(footer_row)
        row.addWidget(content, 1)
        self.setCentralWidget(outer)
        self._navigate(0)

    def _toggle_logs(self):
        visible = self.log_toggle.isChecked()
        self.log_frame.setVisible(visible)
        self.log_toggle.setText("收起日志" if visible else "展开日志")
        if visible:
            self.content_splitter.setSizes([max(300, self.height() - 340), 180])

    def _scroll_panel(self, widget):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(widget)
        return scroll

    def _home_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        self.workspace_splitter = QSplitter(Qt.Horizontal)
        self.workspace_splitter.setChildrenCollapsible(False)
        self.gazebo_view, view_layout = _card("")
        viewport_heading = QHBoxLayout()
        viewport_heading.addWidget(_label("仿真现场", "section"))
        viewport_heading.addStretch()
        viewport_heading.addWidget(_label("GAZEBO  /  RGB-D", "eyebrow"))
        view_layout.addLayout(viewport_heading)
        self.view_split = QSplitter(Qt.Vertical)
        self.view_split.setChildrenCollapsible(False)
        self.gazebo_placeholder = QLabel("等待仿真启动")
        self.gazebo_placeholder.setToolTip("Gazebo 内嵌视图：机械臂、固定工位与台面")
        self.gazebo_placeholder.setWordWrap(True)
        self.gazebo_placeholder.setAttribute(Qt.WA_NativeWindow, True)
        self.gazebo_placeholder.setAttribute(Qt.WA_DontCreateNativeAncestors, False)
        self.gazebo_placeholder.setAlignment(Qt.AlignCenter)
        self.gazebo_placeholder.setScaledContents(False)
        self.gazebo_placeholder.setMinimumSize(160, 120)
        self.gazebo_placeholder.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.gazebo_placeholder.setObjectName("viewport")
        self.view_split.addWidget(self.gazebo_placeholder)
        camera_panel = QWidget()
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(0, 8, 0, 0)
        camera_layout.setSpacing(8)
        self.camera_title = _label("腕部相机（彩色）", "section")
        camera_layout.addWidget(self.camera_title)
        self.camera_placeholder = QLabel("等待腕部相机")
        self.camera_placeholder.setToolTip("腕部相机彩色画面；与仿真同时启动")
        self.camera_placeholder.setWordWrap(True)
        self.camera_placeholder.setAlignment(Qt.AlignCenter)
        self.camera_placeholder.setMinimumSize(160, 100)
        self.camera_placeholder.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_placeholder.setObjectName("viewport")
        camera_layout.addWidget(self.camera_placeholder, 1)
        self.recognition_status = _label("基础组识别：等待图像", "muted")
        self.recognition_status.setVisible(False)
        camera_layout.addWidget(self.recognition_status)
        self.view_split.addWidget(camera_panel)
        self.view_split.setSizes([440, 220])
        self.view_split.setStretchFactor(0, 2)
        self.view_split.setStretchFactor(1, 1)
        view_layout.addWidget(self.view_split, 1)
        self.gazebo_view.setMinimumWidth(260)
        self.workspace_splitter.addWidget(self.gazebo_view)
        self.inspector = QTabWidget()
        self.inspector.setObjectName("inspector")
        self.inspector.setMinimumWidth(round(320 * self.ui_scale))
        practice = QWidget()
        practice_layout = QVBoxLayout(practice)
        practice_layout.setContentsMargins(0, 12, 0, 0)
        practice_layout.setSpacing(12)
        pose_page = QWidget()
        pose_column = QVBoxLayout(pose_page)
        pose_column.setContentsMargins(0, 12, 0, 0)
        environment = QWidget()
        environment_layout = QVBoxLayout(environment)
        environment_layout.setContentsMargins(0, 12, 0, 0)
        environment_layout.setSpacing(12)
        card, cfg = _card("练习设置")
        grid = QGridLayout()
        grid.setSpacing(8)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("基础练习 · Gazebo 仿真", "basic")
        self.mode_combo.addItem("基础组识别 · 几何体", "recognition")
        self.mode_combo.addItem("视觉练习 · Gazebo / 相机", "vision")
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self.seed_spin = QSpinBox()
        self.seed_spin.setRange(-1, 1073741823)
        self.seed_spin.setValue(246135)
        self.backend_combo = QComboBox()
        self.backend_combo.addItem("Gazebo 主视图", "gazebo")
        self.backend_combo.addItem("轻量假硬件 / RViz", "mock")
        self.backend_combo.currentIndexChanged.connect(self._backend_changed)
        self.velocity_spin = QDoubleSpinBox()
        self.velocity_spin.setRange(0.1, 1.0)
        self.velocity_spin.setSingleStep(0.1)
        self.velocity_spin.setValue(0.5)
        self.velocity_spin.setDecimals(1)
        self.velocity_spin.setSuffix(" ×")
        for row, (title, widget) in enumerate((("运行模式", self.mode_combo),
                                               ("题目种子（−1 随机）", self.seed_spin),
                                               ("基础组显示", self.backend_combo),
                                               ("参考速度", self.velocity_spin))):
            widget.setAccessibleName(title)
            grid.addWidget(_label(title, "muted"), row * 2, 0)
            grid.addWidget(widget, row * 2 + 1, 0)
        cfg.addLayout(grid)
        self.mode_description = _label("基础组 Gazebo 仿真；固定位置放置六类物件，二维码只给出抓取顺序。工作台内置相机用于观察，不承担基础组物体分类。", "muted")
        cfg.addWidget(self.mode_description)
        environment_layout.addWidget(card)
        pose_card, pose_layout = _card("基础组固定姿态调参")
        pose_layout.addWidget(_label(
            "工位固定，二维码决定抓取顺序。读取关节角、调整并记录；仅用于仿真，真机须重新标定。", "muted"))
        pose_grid = QGridLayout()
        pose_grid.setSpacing(8)
        pose_grid.addWidget(_label("编号", "muted"), 0, 0)
        self.pose_station = QSpinBox()
        self.pose_station.setAccessibleName("固定工位编号")
        self.pose_station.setRange(1, 6)
        self.pose_station.setValue(1)
        self.pose_station.valueChanged.connect(self._pose_phase_changed)
        pose_grid.addWidget(self.pose_station, 0, 1)
        pose_grid.addWidget(_label("姿态阶段", "muted"), 1, 0, 1, 4)
        self.pose_phase = QComboBox()
        for label, value in (("预抓取（编号）", "object:approach"),
                             ("抓取（编号）", "object:grasp"),
                             ("抬升（编号）", "object:lift"),
                             ("抓取后转场路点（编号）", "object:transfer"),
                             ("投放前（料盒）", "place:approach"),
                             ("投放（料盒）", "place:release"),
                             ("退出（料盒）", "place:retreat")):
            self.pose_phase.addItem(label, value)
        self.pose_phase.setAccessibleName("姿态阶段")
        self.pose_phase.currentIndexChanged.connect(self._pose_phase_changed)
        pose_grid.addWidget(self.pose_phase, 2, 0, 1, 4)
        pose_grid.addWidget(_label("关节单位", "muted"), 3, 0, 1, 2)
        self.joint_unit = QComboBox()
        self.joint_unit.addItem("弧度 rad", "rad")
        self.joint_unit.addItem("角度 °", "deg")
        self.joint_unit.setAccessibleName("关节角度单位")
        self.joint_unit.currentIndexChanged.connect(self._change_joint_unit)
        pose_grid.addWidget(self.joint_unit, 3, 2, 1, 2)
        self._joint_unit = "rad"
        self.joint_spins = []
        for i in range(6):
            pose_grid.addWidget(_label(f"J{i + 1}", "muted"), 4 + (i // 2) * 2, (i % 2) * 2, 1, 2)
            spin = QDoubleSpinBox()
            spin.setRange(-6.2832, 6.2832)
            spin.setDecimals(4)
            spin.setSingleStep(0.05)
            spin.setSuffix(" rad")
            spin.setAccessibleName(f"J{i + 1} 关节角（弧度）")
            self.joint_spins.append(spin)
            pose_grid.addWidget(spin, 5 + (i // 2) * 2, (i % 2) * 2, 1, 2)
        pose_layout.addLayout(pose_grid)
        pose_layout.addWidget(_label("关节点动（仅仿真）", "section"))
        pose_layout.addWidget(_label("步长：按一次移动一次；连续：鼠标左键长按方向键才会移动，松开请求停止；短按请用步长模式。仅仿真，软件停止不是急停。", "muted"))
        self.jog_mode = QComboBox()
        self.jog_mode.setAccessibleName("关节点动控制方式")
        self.jog_mode.addItem("步长", "step")
        self.jog_mode.addItem("连续（长按不松开）", "continuous")
        self.jog_mode.currentIndexChanged.connect(self._release_jog)
        pose_layout.addWidget(self.jog_mode)
        self.jog_step = QComboBox()
        self.jog_step.setAccessibleName("关节点动步长")
        for step in (1, 2, 3):
            self.jog_step.addItem(f"步长 {step}°", step)
        pose_layout.addWidget(self.jog_step)
        jog_grid = QGridLayout()
        self.jog_buttons = []
        for i in range(6):
            jog_grid.addWidget(_label(f"J{i + 1}", "muted"), i, 0)
            for column, direction, text in ((1, -1, "−"), (2, 1, "+")):
                button = _button(text, lambda: None)
                button.pressed.connect(lambda j=i, d=direction, b=button: self._press_jog(j, d, b))
                button.released.connect(self._release_jog)
                button.installEventFilter(self)
                button.setAccessibleName(f"J{i + 1} {'负' if direction < 0 else '正'}方向点动")
                jog_grid.addWidget(button, i, column)
                self.jog_buttons.append(button)
        pose_layout.addLayout(jog_grid)
        pose_layout.addWidget(_label("连续点动为关节运动，并非末端直线运动；停止需等待确认。", "muted"))
        pose_buttons = QVBoxLayout()
        pose_buttons.addWidget(_button("读取当前关节角", self._read_current_joints))
        pose_buttons.addWidget(_button("移动到输入姿态", self._move_to_pose))
        pose_buttons.addWidget(_button("记录此姿态", self._record_pose, primary=True))
        self.pose_status = _label("等待仿真关节状态。", "muted")
        pose_buttons.addWidget(self.pose_status, 1)
        pose_layout.addLayout(pose_buttons)
        waypoint_buttons = QVBoxLayout()
        waypoint_buttons.addWidget(_button("追加阶段前路点", self._append_waypoint))
        waypoint_buttons.addWidget(_button("清空阶段前路点", self._clear_waypoints))
        self.waypoint_status = _label("中间路点按顺序在所选阶段前执行；抓取后转场在抬升后执行。", "muted")
        waypoint_buttons.addWidget(self.waypoint_status, 1)
        pose_layout.addLayout(waypoint_buttons)
        example_buttons = QVBoxLayout()
        example_buttons.addWidget(_button("载入仿真基础姿态", self._load_simulation_example))
        example_buttons.addWidget(_label("仅默认仿真布局；覆盖前备份。不能用于真机。", "muted"), 1)
        pose_layout.addLayout(example_buttons)
        self.pose_file_label = _label("配置自动保存在本机学习数据目录。", "muted")
        self.pose_file_label.setToolTip(str(self._pose_path()))
        pose_layout.addWidget(self.pose_file_label)
        pose_column.addWidget(pose_card)
        pose_column.addStretch()
        control, actions = _card("操作")
        action_grid = QGridLayout()
        action_grid.setSpacing(8)
        self.check_button = _button("环境自检", self.check_now)
        self.launch_button = _button("启动环境", self.start_environment, primary=True)
        self.generate_button = _button("生成题目", self.generate)
        self.reference_button = _button("运行官方基础模型", self.run_reference, primary=True)
        self.start_button = _button("发令 / 开始计时", self.start_round)
        self.stop_round_button = _button("停止本轮", self.stop_round, danger=True)
        self.reset_button = _button("重置场景", self.reset_scene)
        self.stop_stack_button = _button("关闭环境", self.stop_environment)
        for row, (label, button) in enumerate((("01  准备环境", self.launch_button),
                                                ("02  获取顺序", self.generate_button),
                                                ("03  执行抓取", self.reference_button))):
            action_grid.addWidget(_label(label, "muted"), row * 2, 0, 1, 2)
            action_grid.addWidget(button, row * 2 + 1, 0, 1, 2)
        action_grid.addWidget(self.start_button, 6, 0, 1, 2)
        action_grid.addWidget(self.reset_button, 7, 0, 1, 2)
        self.stop_round_button.setToolTip("软件停止，不是真机安全急停；只用于本工作台的仿真。")
        actions.addLayout(action_grid)
        self.round_label = _label("等待启动环境", "section")
        self.scene_label = _label("题目：尚未生成", "muted")
        self.score_label = _label("完成：—    用时：—    分数：—", "muted")
        actions.addWidget(self.scene_label)
        actions.addWidget(self.score_label)
        practice_layout.addWidget(control)
        quick_start, quick_layout = _card("第一次练习")
        quick_layout.addWidget(_label("先载入示例姿态，再生成题目和运行基础模型。官方模型准备完成后自动发令。", "muted"))
        quick_layout.addWidget(_button("载入仿真基础姿态", self._load_simulation_example))
        quick_layout.addWidget(_button("打开姿态调参", lambda: self.inspector.setCurrentIndex(1)))
        practice_layout.addWidget(quick_start)
        practice_layout.addStretch()
        environment_actions = QHBoxLayout()
        environment_actions.addWidget(self.check_button)
        environment_actions.addWidget(self.stop_stack_button)
        environment_layout.addLayout(environment_actions)
        checks, check_layout = _card("就绪检查")
        checks.setMinimumHeight(round(280 * self.ui_scale))
        self.check_table = QTableWidget(0, 3)
        self.check_table.setHorizontalHeaderLabels(["检查项", "状态", "说明"])
        self.check_table.verticalHeader().hide()
        self.check_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.check_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.check_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.check_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.check_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        check_layout.addWidget(self.check_table)
        environment_layout.addWidget(checks)
        environment_layout.addStretch()
        self.inspector.addTab(self._scroll_panel(practice), "练习流程")
        self.inspector.addTab(self._scroll_panel(pose_page), "姿态调参")
        self.inspector.addTab(self._scroll_panel(environment), "环境设置")
        self.inspector.currentChanged.connect(lambda index: self._release_jog() if index != 1 else None)
        inspector_panel = QWidget()
        inspector_layout = QVBoxLayout(inspector_panel)
        inspector_layout.setContentsMargins(0, 0, 0, 0)
        inspector_layout.setSpacing(8)
        inspector_layout.addWidget(self.inspector, 1)
        # Keep software stop reachable while scrolling or editing poses/settings.
        inspector_layout.addWidget(self.round_label)
        inspector_layout.addWidget(self.stop_round_button)
        self.workspace_splitter.addWidget(inspector_panel)
        self.workspace_splitter.setSizes([800, round(370 * self.ui_scale)])
        self.workspace_splitter.setStretchFactor(0, 3)
        self.workspace_splitter.setStretchFactor(1, 0)
        layout.addWidget(self.workspace_splitter, 1)
        return page

    def _code_page(self):
        page, layout = _card("我的程序")
        layout.addWidget(_label("示例读取生成的二维码 PNG，并使用已知工位坐标。它不是实时相机识别；代码在本机运行，没有安全沙箱。", "notice"))
        self.program_path = QLineEdit(str(self.settings.value("program_path", "")))
        self.program_path.setPlaceholderText("选择自己编写的 .py 文件")
        self.program_path.setAccessibleName("学生程序路径")
        self.program_path.setReadOnly(True)
        layout.addWidget(self.program_path)
        buttons = QGridLayout()
        buttons.setSpacing(8)
        buttons.addWidget(_button("选择程序", self.choose_program), 0, 0)
        buttons.addWidget(_button("新建示例副本", self.new_program), 0, 1)
        buttons.addWidget(_button("用系统编辑器打开", self.edit_program), 1, 0)
        self.student_button = _button("运行我的程序", self.run_student, primary=True)
        buttons.addWidget(self.student_button, 1, 1)
        layout.addLayout(buttons)
        layout.addWidget(_label("先生成题目 → 运行程序 → 回控制台发令", "muted"))
        self.code_preview = QPlainTextEdit()
        self.code_preview.setMinimumHeight(round(180 * self.ui_scale))
        self.code_preview.setReadOnly(True)
        self.code_preview.setObjectName("code")
        layout.addWidget(self.code_preview, 1)
        self._load_preview()
        return self._scroll_panel(page)

    def _results_page(self):
        page, layout = _card("运行记录")
        layout.addWidget(_label("本机成绩受电脑性能及现有判分逻辑影响，只用于练习，不用于正式排名。演示结果带有 DEMO 标记。", "notice"))
        buttons = QHBoxLayout()
        buttons.addWidget(_button("刷新记录", self._refresh_results))
        buttons.addWidget(_button("打开成绩目录", lambda: self._open(self.data_dir / "results")))
        buttons.addStretch()
        layout.addLayout(buttons)
        self.results_table = QTableWidget(0, 5)
        self.results_table.setHorizontalHeaderLabels(["记录", "类型", "完成", "用时 / s", "总分"])
        self.results_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.results_table.verticalHeader().hide()
        self.results_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.results_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.results_table.cellDoubleClicked.connect(self._open_result)
        layout.addWidget(self.results_table, 1)
        return page

    def _guide_page(self):
        page, layout = _card("入门与使用边界")
        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setHtml("""
        <h2>第一次使用</h2>
        <ol><li>选择「基础练习」，运行环境自检。</li>
        <li>点击「启动环境」，等待关节、夹爪、三组控制器、MoveIt 和出题服务就绪。</li>
        <li>点击「生成题目」，再点击「运行参考程序」。工作台等参考程序收到题目后自动发令。</li>
        <li>在控制台 Gazebo 视图观察机械臂、夹爪与场地；RViz 默认不启动，可按需切换轻量模式。停止本轮后可以重置场景。</li>
        <li>在「我的程序」创建示例副本、修改、运行，再手动发令。</li></ol>
        <h3>三种模式有什么区别？</h3>
        <p><b>基础练习：</b>二维码给出六个固定工位的顺序（例如 432165），不再依赖六个物体的类别识别。
        编号固定为 1 长方体、2 正方体、3 圆柱体、4 球体、5 三棱柱、6 六棱柱。
        在控制台右侧「姿态调参」标签中调整关节角并记录姿态；全部记录完成后官方基础模型自动复用。
        Gazebo 姿态是仿真参考，真机 JAKA Mini 2 必须重新校准。</p>
        <p><b>基础组识别：</b>启动专用 Gazebo 几何体场景、移动腕部相机到识别位姿，并运行现有深度轮廓分类器；标注画面与识别清单显示在工作台相机栏，不弹出单独识别窗口。该识别演示不替代正式比赛判分。</p>
        <p><b>视觉练习：</b>复用现有 Gazebo 腕部相机 / YOLO 练习。
        启动后原程序会自动识别并抓取，不使用控制台的出题/发令按钮，不产出基础组正式成绩。
        工件跟随也不是物理夹持证明；相机渲染仍有图形要求。</p>
        <h3>停止、重置与关闭</h3>
        <p>「停止本轮」停止本工作台的程序，并请求取消仿真轨迹；它不是真机安全急停。
        「关闭环境」只关闭工作台自己启动的进程。发现外部仿真时只报告，不接管、不强杀。</p>
        <p>重置场景不会删除学生代码。更换题目种子需关闭环境再启动；程序速度可在每次启动程序前调整。</p>
        <h3>当前版本</h3>
        <p>Ubuntu 源码原型，尚非完整离线安装包或 Windows 软件，不含加密狗。
        本地程序以当前用户权限执行，没有安全沙箱；只运行自己信任的代码。</p>
        """)
        layout.addWidget(browser, 1)
        return page

    def _style(self):
        font = QFont("Noto Sans CJK SC")
        font.setPixelSize(round(15 * self.ui_scale))
        self.setFont(font)
        self.setStyleSheet(stylesheet(self.ui_scale))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "sidebar"):
            compact = self.width() < round(1180 * self.ui_scale)
            self.sidebar.setFixedWidth(round((144 if compact else 176) * self.ui_scale))
            self.sidebar_title.setText("学习\n工作台" if compact else "机器人学习\n工作台")
            labels = ("练习控制台", "我的程序", "运行记录", "入门指南")
            for i, (button, label) in enumerate(zip(self.nav_buttons, labels)):
                button.setText(label if compact else f"{i + 1:02}   {label}")
        if hasattr(self, "view_split"):
            # Stack the live images in VM windows; wide screens can show both side by side.
            orientation = Qt.Horizontal if self.width() >= round(1700 * self.ui_scale) else Qt.Vertical
            if self.view_split.orientation() != orientation:
                self.view_split.setOrientation(orientation)
                self.view_split.setSizes([600, 320])

    def _navigate(self, index):
        if index != 0:
            self._release_jog()
        self.pages.setCurrentIndex(index)
        titles = (("练习控制台", "先运行基础模型，再调好每一个姿态。"),
                  ("我的程序", "从官方示例开始，逐步优化自己的抓取流程。"),
                  ("运行记录", "回看每轮顺序、用时与成绩。"),
                  ("入门指南", "了解操作步骤与仿真使用边界。"))
        self.page_title.setText(titles[index][0])
        self.page_subtitle.setText(titles[index][1])
        for i, button in enumerate(self.nav_buttons):
            button.setChecked(i == index)

    def _log(self, text):
        clean = ANSI.sub("", str(text)).rstrip()
        if clean:
            self.log_view.appendPlainText(f"{datetime.now():%H:%M:%S}  {clean}")

    def _show_checks(self, checks):
        self.check_table.setRowCount(len(checks))
        labels = {"ok": "通过", "warning": "等待 / 注意", "error": "未通过"}
        colors = {"ok": "#147d64", "warning": "#9c7025", "error": "#b44741"}
        for row, item in enumerate(checks):
            for column, value in enumerate((item["name"], labels.get(item["level"], item["level"]), item["detail"])):
                cell = QTableWidgetItem(str(value))
                cell.setToolTip(str(value))
                if column == 1:
                    cell.setForeground(QColor(colors.get(item["level"], "#66788b")))
                self.check_table.setItem(row, column, cell)
        self.check_table.resizeRowsToContents()

    def check_now(self):
        if self._checking:
            return
        if self.demo:
            self._checks = [{"name": "界面演示", "level": "warning", "detail": "未检测 ROS；示例状态不能证明仿真可用"}]
            self._show_checks(self._checks)
            self._log("演示模式：环境检查仅显示示例提示。")
            return
        self._checking = True
        self._check_mode = self.mode
        self._check_task = _CheckTask(self.root, self.mode, self.backend_combo.currentData() or "gazebo")
        self._check_task.signals.result.connect(self._checked)
        QThreadPool.globalInstance().start(self._check_task)
        self._update_buttons()

    def _checked(self, payload):
        self._checking = False
        self._checks = payload["checks"]
        if not self.manager.alive("stack"):
            self._show_checks(self._checks)
        conflicts = payload.get("conflicts", [])
        if conflicts and not self.manager.alive("stack"):
            pids = "、".join(str(item["pid"]) for item in conflicts[:6])
            self.notice.setText(
                f"启动被阻止：发现外部仿真或残留进程（PID {pids}）。"
                "即使 Gazebo/RViz 窗口已关闭，后台 ROS 仍可能运行。"
                "请在原启动终端按 Ctrl+C，确认退出后再点「启动环境」；工作台不会自动清理。")
            for item in conflicts[:6]:
                self._log(f"外部进程 PID {item['pid']}：{item['command'][:220]}")
        failed = [c for c in self._checks if c["level"] == "error"]
        self._log(f"环境检查完成：{len(failed)} 项未通过。")
        should_start = self._start_after_check
        self._start_after_check = False
        if should_start:
            if failed:
                self.notice.setText("运行依赖尚未齐全，请查看右侧「环境设置」中的自检结果。工作台没有修改系统。")
                self._navigate(0)
                self.inspector.setCurrentIndex(2)
            elif conflicts:
                self._log("已阻止重复启动：上方列出了冲突进程 PID 和命令，不会清理或接管外部仿真。")
                self._log("处理方法：回到原仿真启动终端按 Ctrl+C；仅关闭 Gazebo/RViz 窗口可能留下 ROS 后台。确认进程退出后，再点「启动环境」重新检查。")
            else:
                self._launch_stack()
        self._update_buttons()

    def _mode_changed(self):
        # 构造期间部分控件尚未创建。
        if not hasattr(self, "mode_description"):
            return
        descriptions = {
            "basic": "基础组比赛练习；Gazebo 与腕部相机同时运行，图像显示在工作台内。工件场景为规划场景镜像。",
            "recognition": "基础组几何体识别：启动专用识别场景、腕部相机与六类形状分类器，在工作台内显示标注图像和识别结果。",
            "vision": "视觉抓取练习：启动 Gazebo 腕部相机与现有 YOLO 识别抓取流程，不产出基础组比赛成绩。",
        }
        self.mode_description.setText(descriptions.get(self.mode, ""))
        if self.mode in ("vision", "recognition") and self.backend_combo.currentData() != "gazebo":
            self.backend_combo.setCurrentIndex(0)
        self.backend_combo.setEnabled(self.mode == "basic" and not self._has_stack())
        self.camera_title.setText("基础组识别标注" if self.mode == "recognition" else "腕部相机（彩色）")
        self.recognition_status.setVisible(self.mode == "recognition")
        self.gazebo_view.setVisible(self.mode != "basic" or self.backend_combo.currentData() == "gazebo")
        self._checks = []
        if hasattr(self, "check_table"):
            self._show_checks([])
        if hasattr(self, "student_button"):
            self._update_buttons()

    def _backend_changed(self):
        if hasattr(self, "gazebo_view"):
            self.gazebo_view.setVisible(self.backend_combo.currentData() == "gazebo")
        self._update_buttons()

    def _detach_gazebo_window(self, message="等待 Gazebo 窗口"):
        self._gazebo_window_id = None
        if hasattr(self, "gazebo_placeholder"):
            self.gazebo_placeholder.clear()
            self.gazebo_placeholder.setText(message)

    def _clear_camera_view(self, message="等待腕部相机图像"):
        if hasattr(self, "camera_placeholder"):
            self.camera_placeholder.clear()
            self.camera_placeholder.setText(message)
            self.camera_placeholder.setPixmap(QPixmap())
        if hasattr(self, "recognition_status"):
            self.recognition_status.setText("基础组识别：等待图像")

    def _refresh_gazebo_view(self):
        if not hasattr(self, "gazebo_view") or (self.mode == "basic" and self.backend_combo.currentData() != "gazebo") or not self._has_stack():
            if self._gazebo_window_id is not None:
                self._detach_gazebo_window("Gazebo 已停止")
            return
        # Decide from Qt's actual platform plugin, not XDG_SESSION_TYPE: a
        # Wayland desktop can still run this app through XWayland (xcb), where
        # both Qt's host and Gazebo's X11 window can be reparented.
        if QApplication.platformName().lower() not in ("xcb", "x11"):
            self._detach_gazebo_window(
                f"当前图形后端 {QApplication.platformName()} 不支持 X11 原生窗口嵌入；请使用独立 Gazebo 窗口。")
            return
        try:
            tree = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True,
                                  timeout=0.6, check=True).stdout
            own = self.manager.owned_pids("stack")
            selected = None
            for line in tree.splitlines():
                m = re.search(r"(0x[0-9a-fA-F]+)\s+\"([^\"]*Gazebo[^\"]*)\"", line)
                if not m:
                    continue
                wid, title = m.groups()
                prop = subprocess.run(["xprop", "-id", wid, "_NET_WM_PID"], capture_output=True,
                                      text=True, timeout=0.5).stdout
                pm = re.search(r"=\s*(\d+)", prop)
                if pm and int(pm.group(1)) in own:
                    selected = (wid, title)
                    break
            if selected is None:
                self._detach_gazebo_window("等待本工作台启动的 Gazebo；不会读取其他终端的窗口。")
                return
            wid, title = selected
            if self._gazebo_embedder is None:
                self._gazebo_embedder = _X11WindowEmbedder()
            self.gazebo_placeholder.setPixmap(QPixmap())
            self.gazebo_placeholder.setText("")
            self.gazebo_placeholder.setToolTip(
                f"{title} 原生窗口已嵌入；直接在此视口内操作 Gazebo。")
            self._gazebo_embedder.embed(
                int(wid, 16), int(self.gazebo_placeholder.winId()),
                self.gazebo_placeholder.width(), self.gazebo_placeholder.height())
            self._gazebo_window_id = wid
        except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
            self._detach_gazebo_window(f"原生 Gazebo 嵌入暂不可用，请使用独立窗口：{exc}")

    def start_environment(self):
        if self._has_stack() or self._checking or self._closing_stack:
            return
        if self.demo:
            self._demo_stack = True
            self.ready = True
            self.run_state = "idle"
            self._show_checks([{"name": "演示环境", "level": "warning", "detail": "只演示交互，没有启动仿真"}])
            self._log("[DEMO] 环境已打开（仅界面演示）。")
            self._update_buttons()
            return
        self._start_after_check = True
        self.check_now()

    def _launch_stack(self):
        self.ready = False
        self.scene, self.score = {}, None
        self.run_state = "idle"
        self.last_status = 0.0
        self.joint_sample_at = 0.0
        self.joint_positions = {}
        self.stack_started = time.monotonic()
        self._closing_stack = False
        backend = self.backend_combo.currentData() or "gazebo"
        if self.mode == "basic":
            use_rviz = "true" if backend == "mock" else "false"
            command = ["ros2", "launch", str(self.root / "jaka_competition_kit/launch/workbench.launch.py"),
                       f"backend:={backend}", f"use_rviz:={use_rviz}", f"seed:={self.seed_spin.value()}",
                       f"output_dir:={self.data_dir / 'results'}"]
        elif self.mode == "recognition":
            command = ["ros2", "launch", "jaka_competition_kit", "gemini305_preview.launch.py",
                       "learn:=false", "rviz:=false", f"output_dir:={self.data_dir / 'recognition'}"]
        else:
            command = ["ros2", "launch", "jaka_competition_kit", "gemini305_preview.launch.py",
                       "learn:=true", "rviz:=false", f"output_dir:={self.data_dir / 'vision'}"]
        try:
            self.manager.launch("stack", ros_command(self.root, command), self.root, self._env())
            monitor_args = ["monitor", "--mode", self.mode]
            if self.mode == "vision":
                monitor_args.append("--camera")
            elif self.mode == "recognition":
                monitor_args.extend(("--camera", "--recognition"))
            elif backend == "gazebo":
                monitor_args.append("--camera")
            self.manager.launch("monitor", self._bridge(*monitor_args), self.root, self._env())
            notices = {
                "basic": "正在等待机械臂、控制器与比赛服务反馈；相机也会同步显示在工作台内。",
                "recognition": "正在启动基础组识别场景；等待相机、分类器与识别结果就绪。",
                "vision": "Gazebo 与腕部相机已同时启动；视觉程序会自动识别抓取。",
            }
            self.notice.setText(notices.get(self.mode, "正在启动环境。"))
            self._log("已启动本工作台环境，等待就绪。")
        except Exception as exc:
            self._log(f"启动失败：{exc}")
            self.notice.setText(f"启动失败：{exc}。请展开日志查看详情。")
            if not self.log_toggle.isChecked():
                self.log_toggle.click()
            self.manager.stop_all()
        self._update_buttons()

    def _bridge(self, *args):
        return ros_command(self.root, ["/usr/bin/python3", "-u", "-m", "workbench.ros_bridge", *args])

    def _env(self):
        return {"JAKA_GRIPPER": "1", "PYTHONUNBUFFERED": "1", "ROS_LOCALHOST_ONLY": "1"}

    def _has_stack(self):
        return self._demo_stack if self.demo else self.manager.alive("stack")

    def _runner_active(self):
        return self._demo_running if self.demo else (self._program_inflight or self.manager.alive("program"))

    def _pose_busy(self):
        # Keep the lock until the process exit event is consumed, even if the
        # child has exited; otherwise a delayed protocol result can be lost.
        return self._jog_fault or self._pose_inflight or self.manager.alive("pose")

    def _service_busy(self):
        # OS 进程死亡不等于 GUI 已消费该请求的结果/退出事件。
        return self._service_inflight or self.manager.alive("service")

    def _can_start_program(self):
        return (self.ready and self._has_stack() and self.mode == "basic"
                and self.run_state == "armed" and not self._runner_active()
                and not self._service_busy() and not self._pose_busy() and not self._closing_stack)

    def start_round(self):
        if (not self.ready or not self._has_stack() or self.mode != "basic"
                or self.run_state != "armed" or self._closing_stack
                or self._service_busy() or self._pose_busy() or self._auto_start_reference):
            return
        if not self.demo and not self._runner_active():
            self._log("请先启动学生程序，确认程序正在等待发令；参考程序会自动发令。")
            return
        self._service("start")

    def generate(self):
        if (not self.ready or not self._has_stack() or self.mode != "basic"
                or self._runner_active() or self.run_state == "running"
                or self._service_busy() or self._pose_busy() or self._closing_stack):
            return
        if self.demo:
            self.scene = {"seed": self.seed_spin.value(), "object_count": 6}
            self.run_state = "armed"
            self.score = None
            self._log("[DEMO] 示例题目已准备，共六个工位；未生成实际二维码。")
            self._update_buttons()
            return
        self._service("generate")

    def _service(self, name):
        if self.demo:
            self._demo_service(name)
            return
        if not self.manager.alive("stack") or self._service_busy() or self._closing_stack:
            return
        try:
            self._service_inflight = True
            self._service_name = name
            self._service_success = False
            self._service_payload = None
            self.manager.launch("service", self._bridge("service", "--name", name), self.root, self._env())
            self._log(f"请求：{name}")
        except Exception as exc:
            self._service_inflight = False
            self._service_name = ""
            self._service_queue = []
            self._log(f"操作失败：{exc}")
        self._update_buttons()

    def run_reference(self):
        if not self._can_start_program():
            return
        if self.demo:
            self._demo_service("start")
            return
        try:
            self._auto_start_reference = True
            self._reference_wait_at = time.monotonic()
            self._program_inflight = True
            self.manager.launch("program", self._bridge("reference", "--velocity", str(self.velocity_spin.value()),
                                                           "--fixed-poses", str(self._pose_path())),
                                self.root, self._env())
            self._log("等待参考程序连接 MoveIt 并收到题目，随后自动发令。")
        except Exception as exc:
            self._program_inflight = False
            self._auto_start_reference = False
            self._log(f"参考程序启动失败：{exc}")
        self._update_buttons()

    def stop_round(self):
        if self.demo:
            self._demo_epoch += 1
            self._demo_running = False
            self.run_state = "finished"
            self._log("[DEMO] 本轮已停止。")
        else:
            self._auto_start_reference = False
            self.manager.stop("program")
            self._service_queue = []
            if self._service_busy():
                self.manager.stop("service")
                self._service_queue = ["halt"]
            else:
                self._service("halt")
        self._update_buttons()

    def reset_scene(self):
        if (not self.ready or not self._has_stack() or self.mode != "basic"
                or self._runner_active() or self.run_state == "running"
                or self._service_busy() or self._pose_busy() or self._closing_stack):
            return
        if self.demo:
            self._demo_epoch += 1
            self.scene, self.score = {}, None
            self.run_state = "idle"
            self._log("[DEMO] 示例场景已重置，学生代码保留。")
        else:
            self._service_queue = ["scene-reset"]
            self._service("run-reset")
        self._update_buttons()

    def stop_environment(self):
        self._release_jog()
        self._auto_start_reference = False
        self._service_queue = []
        self.ready = False
        if self.demo:
            self._demo_epoch += 1
            self._demo_stack = self._demo_running = False
            self.scene, self.score = {}, None
            self.run_state = "idle"
        else:
            self._closing_stack = True
            self.manager.stop_all()
        self._log("正在关闭本工作台拥有的进程；不处理外部会话。")
        self._update_buttons()

    def _handle_protocol(self, name, payload):
        kind = payload.get("type")
        if kind == "status" and name == "monitor":
            self.last_status = time.monotonic()
            self.ready = bool(payload.get("ready")) and self.manager.alive("stack") and not self._closing_stack
            self.run_state = payload.get("run_state", "idle")
            self.scene = payload.get("scene") or {}
            joint_age = payload.get("joint_age_s")
            positions = payload.get("joint_positions")
            if isinstance(positions, dict) and isinstance(joint_age, (int, float)) and math.isfinite(joint_age) and joint_age >= 0:
                self.joint_positions = positions
                self.joint_sample_at = self.last_status - joint_age
            else:
                self.joint_sample_at = 0.0
            if isinstance(joint_age, (int, float)) and joint_age > 4.0:
                self.pose_status.setText(f"关节状态已过期（{joint_age:.1f}s），暂不建议记录姿态。")
            new_score = payload.get("score")
            if new_score and new_score != self.score:
                self._log(f"本轮结束，完成 {new_score.get('collected_count', 0)}/6；成绩仅用于学习反馈。")
                self._refresh_results()
            self.score = new_score
            self._show_checks(payload.get("checks", []))
            if self.mode == "recognition" and "recognition" in payload:
                self._show_recognition(payload["recognition"])
        elif (kind == "service" and name == "service" and self._service_inflight
              and payload.get("name") == self._service_name):
            self._service_success = bool(payload.get("success"))
            self._service_payload = payload
            self._log(f"{payload.get('name')}：{payload.get('message', '')}")
        elif kind == "jog-result" and name == "pose":
            self._jog_result_received = True
            self._jog_held = None
            outcome = payload.get("outcome")
            if outcome == "uncertain" or outcome not in ("stopped", "completed", "failed", "rejected"):
                self._jog_fault = True
                self.pose_status.setText("停止状态未确认：已锁定运动，请关闭仿真环境检查。 " + str(payload.get("detail", "")))
            else:
                self.pose_status.setText("连续点动：" + str(payload.get("detail", "")))
        elif kind == "pose-move" and name == "pose":
            self.pose_status.setText("姿态移动完成。" if payload.get("success") else
                                     f"姿态移动失败：{payload.get('detail', '')}")
        elif kind == "program-ready" and name == "program" and self._auto_start_reference:
            self._auto_start_reference = False
            if self.ready and self.run_state == "armed":
                self._service("start")
            else:
                self._log("环境或题目状态已变化，已取消自动发令。")
                self.manager.stop("program")
        elif kind == "error":
            self._log(payload.get("message", "后台错误"))

    def _show_recognition(self, result):
        if not hasattr(self, "recognition_status"):
            return
        if not isinstance(result, dict):
            self.recognition_status.setText("基础组识别：等待相机与识别结果")
            return
        objects = result.get("objects", [])
        names = "、".join(item.get("name", item.get("id", "未知")) for item in objects)
        located = result.get("located", 0)
        count = result.get("count", len(objects))
        self.recognition_status.setText(
            f"基础组识别：{count}/6 个 · 位置匹配 {located}/6"
            + (f"\n{names}" if names else "\n等待识别到物体"))

    def _show_camera_image(self, payload):
        if not hasattr(self, "camera_placeholder"):
            return
        try:
            raw = base64.b64decode(payload.get("jpeg", ""), validate=True)
            image = QImage.fromData(raw, "JPG")
            if image.isNull():
                return
            pixmap = QPixmap.fromImage(image)
            self.camera_placeholder.setPixmap(
                pixmap.scaled(self.camera_placeholder.contentsRect().size(), Qt.KeepAspectRatio,
                              Qt.SmoothTransformation))
            self.camera_placeholder.setText("")
            self.camera_placeholder.setToolTip("来自 Gazebo 腕部 RGB-D 相机的实时彩色图像")
        except (ValueError, TypeError, binascii.Error):
            self._log("相机图像解码失败")

    def _tick(self):
        if self.demo:
            return
        for event in self.manager.poll():
            name = event.get("name", "")
            if event.get("kind") == "output":
                text = event.get("text", "")
                if text.startswith(PREFIX):
                    try:
                        payload = json.loads(text[len(PREFIX):])
                        if isinstance(payload, dict):
                            if payload.get("type") == "image" and name == "monitor":
                                self._show_camera_image(payload)
                            else:
                                self._handle_protocol(name, payload)
                    except ValueError:
                        self._log(f"[{name}] 无法解析后台状态")
                else:
                    self._log(f"[{name}] {text}")
            elif event.get("kind") == "exited":
                self._log(f"[{name}] 已退出，返回码 {event.get('returncode')}")
                if name == "stack":
                    self.ready = False
                    self._closing_stack = True
                    self.manager.stop("monitor")
                    self.manager.stop("program")
                    self.manager.stop("service")
                    self._auto_start_reference = False
                    self._service_queue = []
                elif name == "pose":
                    if getattr(self, "_jog_session", False) and not self._jog_result_received:
                        self._jog_fault = True
                        self.pose_status.setText("连续点动进程异常结束，运动状态未知；请关闭仿真环境检查。")
                    self._jog_session = False
                    self._jog_held = None
                    self.manager.close_input("pose")
                    self._pose_inflight = False
                    if event.get("returncode") not in (0, 130) and not self._jog_result_received:
                        self.pose_status.setText("姿态移动进程异常退出；请检查仿真状态。")
                elif name == "program":
                    self._program_inflight = False
                    self._auto_start_reference = False
                    if event.get("returncode") not in (0, 130) and not self._closing_stack:
                        self._log("程序异常退出；请检查上方日志。若本轮仍在计时，可点击「停止本轮」。")
                elif name == "service":
                    success = self._service_success and event.get("returncode") == 0
                    previous = self._service_name
                    self._service_inflight = False
                    self._service_name = ""
                    self._service_success = False
                    self._service_payload = None
                    if success and previous in ("run-reset", "scene-reset"):
                        self.scene, self.score = {}, None
                        self.run_state = "idle"
                    if self._service_queue:
                        if success or self._service_queue == ["halt"]:
                            next_name = self._service_queue.pop(0)
                            self._service(next_name)
                        else:
                            self._service_queue = []
            elif event.get("kind") == "error":
                self._log(f"[{name}] {event.get('text', '')}")
        now = time.monotonic()
        if self._jog_held:
            if (not self.ready or self.run_state == "running" or self._runner_active()
                    or not self.joint_sample_at or now - self.joint_sample_at >= JOINT_FEEDBACK_MAX_AGE
                    or not self.manager.write_input("pose", f"H{time.monotonic():.9f}\n".encode())):
                self._release_jog()
        if self.last_status and now - self.last_status > 5:
            self.ready = False
            self.status_badge.setText("状态反馈中断")
        if self._auto_start_reference and now - self._reference_wait_at > 35:
            self._auto_start_reference = False
            self.manager.stop("program")
            self._log("参考程序准备超时，已取消自动发令。检查 MoveIt 和题目状态。")
        if self._closing_stack and not self.manager.active_names():
            self._closing_stack = False
            self._jog_fault = False
            self._clear_camera_view("仿真已停止")
            self.scene, self.score = {}, None
            self.run_state = "idle"
        if time.monotonic() - self._gazebo_refresh_at > 1.2:
            self._gazebo_refresh_at = time.monotonic()
            self._refresh_gazebo_view()
        self._update_buttons()

    def _update_buttons(self):
        stack = self._has_stack()
        runner = self._runner_active()
        busy = False if self.demo else self._service_busy()
        basic = self.mode == "basic"
        armed = self.run_state == "armed"
        running = self.run_state == "running"
        pose_busy = self._pose_busy()
        idle_controls = not stack and not self._checking and not self._closing_stack
        self.mode_combo.setEnabled(idle_controls)
        self.seed_spin.setEnabled(idle_controls and basic)
        self.velocity_spin.setEnabled(not runner and basic and not self._checking)
        self.backend_combo.setEnabled(idle_controls and basic)
        self.check_button.setEnabled(not self._checking and not self._closing_stack and not stack)
        self.launch_button.setEnabled(idle_controls)
        self.generate_button.setEnabled(stack and self.ready and basic and not runner and not running and not busy and not pose_busy)
        self.reference_button.setEnabled(stack and self.ready and basic and armed and not runner and not busy and not pose_busy)
        self.start_button.setEnabled(stack and self.ready and basic and armed and not busy and not pose_busy
                                     and not self._auto_start_reference and (self.demo or runner))
        self.student_button.setEnabled(stack and self.ready and basic and armed and not runner and not busy and not pose_busy
                                       and bool(self.program_path.text()))
        self.stop_round_button.setEnabled(stack and basic and (running or runner))
        self.reset_button.setEnabled(stack and self.ready and basic and not runner and not running and not busy and not pose_busy)
        self.stop_stack_button.setEnabled(stack and not self._closing_stack)
        if self._closing_stack:
            badge = "正在关闭"
        elif self._checking:
            badge = "正在自检"
        elif not stack:
            badge = "演示模式" if self.demo else "尚未启动"
        elif self.ready:
            badge = "DEMO · 示例就绪" if self.demo else "环境就绪"
        else:
            badge = "等待就绪反馈"
        if not (self.last_status and time.monotonic() - self.last_status > 5 and stack):
            self.status_badge.setText(badge)
        self.round_label.setText(STATE_NAMES.get(self.run_state, self.run_state) if stack else "等待启动环境")
        if self.mode == "vision" and stack:
            self.round_label.setText("视觉练习 · 原程序自动执行")
        elif self.mode == "recognition" and stack:
            self.round_label.setText("基础组几何体识别")
        if self.scene:
            order = self.scene.get("qr_payload") or ("".join(map(str, self.scene.get("qr_order", []))))
            self.scene_label.setText(f"题目：seed={self.scene.get('seed')} · 二维码抓取顺序 {order or '待扫描'} · {self.scene.get('object_count', 0)} 件")
        else:
            self.scene_label.setText("题目：尚未生成")
        if self.score:
            self.score_label.setText(f"完成：{self.score.get('collected_count', 0)}/6    "
                                     f"用时：{self.score.get('elapsed_s', '—')} s    分数：{self.score.get('total', '—')}")
        else:
            self.score_label.setText("完成：—    用时：—    分数：—")

    def _demo_service(self, name):
        if name == "start" and self._demo_stack and self.run_state == "armed":
            self.run_state = "running"
            self._demo_running = True
            self._demo_step = 0
            self._demo_epoch += 1
            epoch = self._demo_epoch
            self._log("[DEMO] 开始六工位示例；无实际机械臂运动。")
            QTimer.singleShot(400, lambda: self._demo_advance(epoch))
        self._update_buttons()

    def _demo_advance(self, epoch):
        if not self._demo_running or epoch != self._demo_epoch:
            return
        self._demo_step += 1
        self._log(f"[DEMO] 示例工件 {self._demo_step}/6 已完成。")
        if self._demo_step < 6:
            QTimer.singleShot(400, lambda: self._demo_advance(epoch))
        else:
            self._demo_running = False
            self.run_state = "finished"
            self.score = {"demo": True, "collected_count": 6, "elapsed_s": 2.4,
                          "total": 100, "finish_reason": "仅界面演示，无真实仿真"}
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            try:
                (self.data_dir / "results" / f"demo_{stamp}.json").write_text(
                    json.dumps(self.score, ensure_ascii=False, indent=2), encoding="utf-8")
            except OSError as exc:
                self._log(f"[DEMO] 无法保存示例记录：{exc}")
            self._refresh_results()
        self._update_buttons()

    def _pose_phase_changed(self):
        self._load_saved_pose_into_editor()

    def _pose_template_path(self) -> Path:
        return self.root / "jaka_competition_kit" / "config" / "track1_fixed_poses.yaml"

    def _pose_path(self) -> Path:
        # Participant calibration belongs to the workbench data directory, not
        # the source checkout. This keeps the official template reproducible and
        # allows each team/profile to maintain its own persisted poses.
        return self.data_dir / "config" / "track1_fixed_poses.yaml"

    def _ensure_pose_config(self):
        destination = self._pose_path()
        if destination.exists():
            return
        template = self._pose_template_path()
        try:
            if template.exists():
                shutil.copyfile(template, destination)
            else:
                destination.write_text(
                    "version: 2\nenabled: false\nbackend: simulation\n"
                    "device_profile: jaka_mini_2_gemini305_gripper_sim\nobjects: {}\nplace: {}\n",
                    encoding="utf-8")
        except OSError as exc:
            # The editor will surface a precise error when a record is attempted.
            self._pose_init_error = f"固定姿态模板初始化失败：{exc}"

    def _pose_key(self):
        kind, phase = str(self.pose_phase.currentData()).split(":", 1)
        return kind, phase

    def _fixed_pose_api(self):
        package_root = str(self.root / "jaka_competition_kit")
        if package_root not in sys.path:
            sys.path.insert(0, package_root)
        from jaka_competition_kit import fixed_poses
        return fixed_poses

    def _load_saved_pose_into_editor(self):
        try:
            data = self._fixed_pose_api().load_fixed_poses(str(self._pose_path()))
            kind, phase = self._pose_key()
            target = (data.get("objects", {}).get(str(self.pose_station.value()), {})
                      if kind == "object" else data.get("place", {}))
            values = target.get(f"{phase}_joints") if isinstance(target, dict) else None
            points = self._fixed_pose_api().waypoints_for(target, phase)
            if hasattr(self, "waypoint_status"):
                self.waypoint_status.setText(
                    f"所选阶段前路点：{len(points)} 个，按录入顺序执行；需完整回放验收。")
                self.waypoint_status.setToolTip("\n".join(
                    f"{i}: " + ", ".join(f"{v:.4f}" for v in point)
                    for i, point in enumerate(points, 1)) or "暂无路点")
            if isinstance(values, list) and len(values) == 6:
                self._set_joint_values(values)
                self.pose_status.setText("已载入已保存姿态；不会自动移动机械臂。")
            elif phase == "transfer":
                self.pose_status.setText("转场只使用路点；输入框保留原值，请确认后再追加。")
            else:
                self.pose_status.setText("此阶段尚未记录；输入框保留原值，请确认后再移动或记录。")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.pose_status.setText(f"姿态配置读取失败：{exc}")

    def _joint_values(self):
        values = [spin.value() for spin in self.joint_spins]
        return [math.radians(v) for v in values] if self._joint_unit == "deg" else values

    def _set_joint_values(self, values):
        for spin, value in zip(self.joint_spins, values):
            spin.setValue(math.degrees(float(value)) if self._joint_unit == "deg" else float(value))

    def _change_joint_unit(self):
        if not hasattr(self, "joint_spins"):
            return
        values = self._joint_values()
        self._joint_unit = self.joint_unit.currentData()
        degrees = self._joint_unit == "deg"
        for i, spin in enumerate(self.joint_spins):
            spin.blockSignals(True)
            spin.setDecimals(8)
            limit = math.degrees(6.2832) if degrees else 6.2832
            spin.setRange(-limit, limit)
            spin.setSingleStep(1 if degrees else 0.05)
            spin.setSuffix(" °" if degrees else " rad")
            spin.setAccessibleName(f"J{i + 1} 关节角（{'角度' if degrees else '弧度'}）")
        self._set_joint_values(values)
        for spin in self.joint_spins:
            spin.blockSignals(False)

    def eventFilter(self, watched, event):
        if (watched is self._jog_held and event.type() in
                (QEvent.Leave, QEvent.FocusOut, QEvent.Hide, QEvent.UngrabMouse)):
            self._release_jog()
        return super().eventFilter(watched, event)

    def event(self, event):
        if event.type() in (QEvent.WindowDeactivate, QEvent.Hide) and hasattr(self, "_jog_held"):
            self._release_jog()
        return super().event(event)

    def _release_jog(self, *_):
        if self._jog_held is not None:
            self._jog_held = None
            self.manager.close_input("pose")
            self.pose_status.setText("已请求停止连续点动，等待运动终态确认……")

    def _press_jog(self, joint, direction, button):
        if self.jog_mode.currentData() == "step":
            self._jog_once(joint, direction)
            return
        if self.demo:
            self.pose_status.setText("演示模式不移动真实/仿真机械臂。")
            return
        if (self._pose_busy() or self._runner_active() or self.run_state == "running"
                or self._service_busy() or self._closing_stack or self.mode != "basic"
                or not self._has_stack() or not self.ready):
            self.pose_status.setText("环境未就绪、其他操作正在执行或停止状态未确认，不能连续点动。")
            return
        values = [self.joint_positions.get(f"joint_{i}") for i in range(1, 7)]
        if (not self.joint_sample_at or time.monotonic() - self.joint_sample_at >= JOINT_FEEDBACK_MAX_AGE
                or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)):
            self.pose_status.setText("关节反馈不完整或已过期，不能连续点动。")
            return
        try:
            self.manager.launch("pose", self._bridge("continuous-jog", "--joint", str(joint),
                                "--direction", str(direction), "--velocity",
                                str(min(0.1, self.velocity_spin.value())), "--backend", "simulation"),
                                self.root, self._env(), input_pipe=True)
            self._pose_inflight = True
            self._jog_session = True
            self._jog_result_received = False
            self._jog_held = button
            if not self.manager.write_input("pose", f"H{time.monotonic():.9f}\n".encode()):
                self._release_jog()
            else:
                self.pose_status.setText(f"J{joint + 1} 连续点动已启动，正在准备运动；请保持长按，松开请求停止。")
            self._update_buttons()
        except Exception as exc:
            self.pose_status.setText(f"连续点动启动失败：{exc}")

    def _jog_once(self, joint, direction):
        if self.demo:
            self.pose_status.setText("演示模式不移动真实/仿真机械臂。")
            return
        if self._runner_active() or self.run_state == "running" or self._pose_busy():
            self.pose_status.setText("本轮或姿态运动正在执行，不能点动。")
            return
        if self._service_busy() or self._closing_stack or self.mode != "basic":
            self.pose_status.setText("环境正在操作或不在基础组仿真模式，不能点动。")
            return
        if not self._has_stack() or not self.ready:
            self.pose_status.setText("请先启动并等待仿真环境就绪。")
            return
        if not self.joint_sample_at or time.monotonic() - self.joint_sample_at >= JOINT_FEEDBACK_MAX_AGE:
            self.pose_status.setText("关节反馈已过期，不能点动；请等待最新状态。")
            return
        values = [self.joint_positions.get(f"joint_{i}") for i in range(1, 7)]
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            self.pose_status.setText("关节反馈不完整，不能点动。")
            return
        values[joint] += direction * math.radians(self.jog_step.currentData())
        if abs(values[joint]) > 6.2832:
            self.pose_status.setText("目标超出关节输入范围，已拒绝点动。")
            return
        try:
            self.manager.launch("pose", self._bridge("move-joints", "--joints", *[format(v, ".12f") for v in values],
                                "--velocity", str(self.velocity_spin.value()), "--backend", "simulation"),
                                self.root, self._env())
            self._pose_inflight = True
            self.joint_sample_at = 0.0
            self._update_buttons()
            self.pose_status.setText(f"J{joint + 1} 正在执行单次点动；模型限位由运动规划器检查。")
        except Exception as exc:
            self.pose_status.setText(f"点动失败：{exc}")

    def _read_current_joints(self):
        values = [self.joint_positions.get(f"joint_{i}") for i in range(1, 7)]
        if (not self.joint_sample_at or time.monotonic() - self.joint_sample_at >= JOINT_FEEDBACK_MAX_AGE
                or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)):
            self.pose_status.setText("关节反馈不完整或已过期，请等待最新状态。")
            return
        self._set_joint_values(values)
        self.pose_status.setText("已读取当前机械臂姿态。")

    def _move_to_pose(self):
        if self._runner_active() or self.run_state == "running" or self._pose_busy():
            self.pose_status.setText("本轮或姿态运动正在执行；请先停止，不能并发发送姿态运动。")
            return
        values = self._joint_values()
        if self.demo:
            self.pose_status.setText("演示模式不移动真实/仿真机械臂。")
            return
        if self._service_busy() or self._closing_stack or self.mode != "basic":
            self.pose_status.setText("环境正在操作或不在基础组仿真模式，不能移动。")
            return
        if not self._has_stack() or not self.ready:
            self.pose_status.setText("请先启动并等待环境就绪。")
            return
        try:
            self.manager.launch("pose", self._bridge("move-joints", "--joints", *[format(v, ".12f") for v in values],
                                                     "--velocity", str(self.velocity_spin.value()),
                                                     "--backend", "simulation"),
                                self.root, self._env())
            self._pose_inflight = True
            self._update_buttons()
            self.pose_status.setText("正在移动到输入姿态……")
        except Exception as exc:
            self.pose_status.setText(f"移动失败：{exc}")

    def _record_pose(self):
        values = self._joint_values()
        path = self._pose_path()
        try:
            kind, phase = self._pose_key()
            if phase == "transfer":
                self.pose_status.setText("抓取后转场请使用‘追加阶段前路点’按钮记录，可依次追加多个路点。")
                return
            api = self._fixed_pose_api()
            data = api.update_pose(
                str(path), station=self.pose_station.value() if kind == "object" else None,
                phase=phase, joints=values, backend="simulation",
                device_profile="jaka_mini_2_gemini305_gripper_sim")
            complete = api.is_complete(data)
            self.pose_status.setText("已记录；" + ("官方基础模型将使用固定姿态。"
                                                    if complete else "请继续记录其它阶段。"))
            self._log(f"已记录固定姿态：{kind} {phase} -> {path}")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.pose_status.setText(f"保存失败：{exc}")


    def _load_simulation_example(self):
        if self._runner_active() or self.run_state == "running" or self._pose_busy():
            self.pose_status.setText("请先停止本轮和姿态运动，再载入仿真基础姿态。")
            return
        if QMessageBox.question(
                self, "载入仿真基础姿态",
                "此示例仅适用于默认基础组场景和仿真夹爪。\n"
                "它使用规划场景附着，不代表物理接触抓取或真机标定。\n"
                "将备份当前姿态并启用示例供下一轮仿真回放；不会立即移动机械臂。继续？",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        # Modal dialogs process Qt events: a program can start while the
        # confirmation is open, so recheck before replacing any configuration.
        if self._runner_active() or self.run_state == "running" or self._pose_busy():
            self.pose_status.setText("确认期间已开始运行，未替换姿态配置；请先停止。")
            return
        try:
            source = self._pose_template_path().with_name("track1_simulation_example.yaml")
            _, backup = self._fixed_pose_api().import_simulation_poses(
                str(source), str(self._pose_path()), enable=True)
            self._load_saved_pose_into_editor()
            self.pose_status.setText("仿真基础姿态已启用；未移动机械臂，下一轮仍需完整回放验收。")
            self._log(f"载入仿真基础姿态：{self._pose_path()}；原配置备份：{backup}")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.pose_status.setText(f"仿真基础姿态载入失败：{exc}")

    def _append_waypoint(self):
        self._edit_waypoints(clear=False)

    def _clear_waypoints(self):
        if QMessageBox.question(self, "清空中间路点",
                                "确认清空当前工位、当前阶段的全部中间路点？清空后须重新验收。") != QMessageBox.Yes:
            return
        self._edit_waypoints(clear=True)

    def _edit_waypoints(self, *, clear):
        try:
            kind, phase = self._pose_key()
            station = self.pose_station.value() if kind == "object" else None
            values = None if clear else self._joint_values()
            data = self._fixed_pose_api().update_waypoints(
                str(self._pose_path()), station=station, phase=phase, joints=values, clear=clear)
            target = data["objects"][str(station)] if station is not None else data["place"]
            count = len(self._fixed_pose_api().waypoints_for(target, phase))
            self.waypoint_status.setText(f"所选阶段前路点：{count} 个；配置已更改，须重新验收。")
            points = self._fixed_pose_api().waypoints_for(target, phase)
            self.waypoint_status.setToolTip("\n".join(
                f"{i}: " + ", ".join(f"{v:.4f}" for v in point)
                for i, point in enumerate(points, 1)) or "暂无路点")
            self._log(f"{'清空' if clear else '追加'}中间路点：{kind} {station or ''} {phase}")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.waypoint_status.setText(f"中间路点保存失败：{exc}")


    def choose_program(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择可信的本地 Python 程序",
                                            str(self.data_dir / "programs"), "Python (*.py)")
        if path:
            self.program_path.setText(path)
            self._load_preview()
            self._update_buttons()

    def new_program(self):
        source = self.root / "workbench/examples/student_track1.py"
        if not source.is_file():
            self._log("学生示例尚未安装。")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        dest = self.data_dir / "programs" / f"my_track1_{stamp}.py"
        try:
            shutil.copyfile(source, dest)
        except OSError as exc:
            self._log(f"无法创建学生程序副本：{exc}")
            return
        self.program_path.setText(str(dest))
        self._load_preview()
        self._log(f"已创建可编辑副本：{dest}；平台示例未修改。")
        self._update_buttons()

    def _load_preview(self):
        path = self.program_path.text()
        self.settings.setValue("program_path", path)
        if not path:
            self.code_preview.setPlainText("点击「新建示例副本」开始编程。")
            return
        try:
            with Path(path).open(encoding="utf-8") as stream:
                self.code_preview.setPlainText(stream.read(100000))
        except (OSError, UnicodeError) as exc:
            self.code_preview.setPlainText(f"无法预览程序：{exc}")

    def edit_program(self):
        path = Path(self.program_path.text())
        if path.is_file():
            self._open(path)
        else:
            self._log("请先选择程序或创建示例副本。")

    def run_student(self):
        if not self._can_start_program():
            return
        if self.demo:
            self._log("[DEMO] 不执行学生文件；参考程序按钮可演示交互。")
            return
        path = Path(self.program_path.text()).expanduser().resolve()
        if not path.is_file() or path.suffix.lower() != ".py":
            self._log("请选择存在的 .py 程序。")
            return
        answer = QMessageBox.question(self, "运行本地程序", "程序将以当前用户权限执行，没有安全沙箱。\n仅运行你信任的代码。\n\n是否运行？")
        if answer != QMessageBox.Yes:
            return
        # 模态确认框仍会处理 Qt 定时器；确认期间可能失去就绪或关闭环境。
        if not self._can_start_program():
            self._log("环境或本轮状态在确认期间已变化，已取消启动学生程序。")
            return
        try:
            args = ["/usr/bin/python3", "-u", str(path), "--ros-args", "-p", "gripper:=sim",
                    "-p", f"vel_scale:={self.velocity_spin.value()}"]
            self._program_inflight = True
            self.manager.launch("program", ros_command(self.root, args), self.root, self._env())
            self._log("学生程序已启动。收到题目并准备好后，请在控制台手动发令。")
        except Exception as exc:
            self._program_inflight = False
            self._log(f"学生程序启动失败：{exc}")
        self._update_buttons()

    def _refresh_results(self):
        if not hasattr(self, "results_table"):
            return
        records = []
        paths = sorted((self.data_dir / "results").glob("*.json"), reverse=True)[:100]
        for path in paths:
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(data, dict) and "collected_count" in data:
                    records.append((path, data))
            except (OSError, ValueError):
                continue
        self.results_table.setRowCount(len(records))
        for row, (path, data) in enumerate(records):
            values = [path.name, "DEMO · 示例" if data.get("demo") else "本机练习",
                      f"{data.get('collected_count', 0)}/6", data.get("elapsed_s", "—"), data.get("total", "—")]
            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setData(Qt.UserRole, str(path))
                self.results_table.setItem(row, col, item)

    def _open_result(self, row, _column):
        item = self.results_table.item(row, 0)
        if item:
            self._open(Path(item.data(Qt.UserRole)))

    def _open(self, path):
        if not QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(path).resolve()))):
            self._log(f"系统未能打开：{path}")

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._release_jog()
        self.timer.stop()
        self._demo_epoch += 1
        self.manager.close(timeout=7.0)
        if self._gazebo_embedder is not None:
            self._gazebo_embedder.close()
            self._gazebo_embedder = None
        self.settings.sync()

    def closeEvent(self, event):
        if not self.demo and self.manager.active_names():
            answer = QMessageBox.question(self, "关闭工作台", "关闭后将停止本工作台启动的仿真和程序，学生代码保留。\n是否继续？")
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        self.shutdown()
        event.accept()
