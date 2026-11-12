"""Qt 学习工作台：管理本软件拥有的仿真，不接管外部 ROS 会话。"""
from __future__ import annotations

from datetime import datetime
import base64
import binascii
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

from PyQt5.QtCore import (QObject, QRunnable, QSettings, Qt, QThreadPool, QTimer,
                          QUrl, pyqtSignal)
from PyQt5.QtGui import QColor, QDesktopServices, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFrame, QGridLayout,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QMessageBox,
    QPlainTextEdit, QPushButton, QSpinBox, QSplitter, QStackedWidget,
    QTableWidget, QTableWidgetItem, QTextBrowser, QVBoxLayout, QWidget,
)

from . import __version__
from .ros_bridge import PREFIX
from .runtime import ProcessManager, check_environment, ros_command, simulation_conflicts

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
STATE_NAMES = {"idle": "待出题", "armed": "题目就绪", "running": "本轮运行中", "finished": "本轮结束"}


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
    button.setMinimumHeight(round(36 * _display_scale()))
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
        for name in ("logs", "results", "programs"):
            (self.data_dir / name).mkdir(parents=True, exist_ok=True)
        self.manager = ProcessManager(self.data_dir / "logs")
        self.settings = QSettings(str(self.data_dir / "preferences.ini"), QSettings.IniFormat)
        self.ready = False
        self.run_state = "idle"
        self.scene = {}
        self.score = None
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
        self.resize(round(1240 * self.ui_scale), round(860 * self.ui_scale))
        self.setMinimumSize(round(960 * self.ui_scale), round(680 * self.ui_scale))
        self._build_ui()
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
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(round(210 * self.ui_scale))
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(round(20 * self.ui_scale), round(30 * self.ui_scale),
                                round(20 * self.ui_scale), round(22 * self.ui_scale))
        side.setSpacing(round(10 * self.ui_scale))
        side.addWidget(_label("JAKA / LEARN", "brand"))
        side.addWidget(_label("基础组\n仿真学习工作台", "sideTitle"))
        side.addSpacing(round(28 * self.ui_scale))
        self.nav_buttons = []
        for index, title in enumerate(("01  练习控制台", "02  我的程序", "03  运行记录", "04  入门与说明")):
            button = _button(title, lambda checked=False, i=index: self._navigate(i))
            button.setCheckable(True)
            button.setObjectName("nav")
            self.nav_buttons.append(button)
            side.addWidget(button)
        side.addStretch()
        side.addWidget(_label("本地 Ubuntu 原型\nCPU 优先 · 无云端依赖", "sideMuted"))
        side.addWidget(_label(f"v{__version__}  /  {'DEMO' if self.demo else 'LOCAL'}", "sideMuted"))
        row.addWidget(sidebar)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(round(26 * self.ui_scale), round(20 * self.ui_scale),
                                  round(26 * self.ui_scale), round(18 * self.ui_scale))
        layout.setSpacing(round(12 * self.ui_scale))
        heading = QHBoxLayout()
        title_col = QVBoxLayout()
        title_col.addWidget(_label("从第一轮抓取开始", "title"))
        title_col.addWidget(_label("出题、运行、观察、修改程序，在一个入口完成。", "muted"))
        heading.addLayout(title_col, 1)
        heading.addStretch()
        self.status_badge = _label("演示模式" if self.demo else "尚未启动", "badge")
        self.status_badge.setWordWrap(False)
        self.status_badge.setMinimumWidth(round(120 * self.ui_scale))
        heading.addWidget(self.status_badge)
        layout.addLayout(heading)
        self.notice = _label("演示模式：不启动 ROS；动画与成绩不是实际仿真结果。" if self.demo else
                             "基础组Gazebo 原生窗口与腕部相机均内置工作台；识别和视觉模式使用专用 Gazebo 场景。各模式不可同时运行。", "notice")
        layout.addWidget(self.notice)
        splitter = QSplitter(Qt.Vertical)
        self.pages = QStackedWidget()
        self.pages.addWidget(self._home_page())
        self.pages.addWidget(self._code_page())
        self.pages.addWidget(self._results_page())
        self.pages.addWidget(self._guide_page())
        splitter.addWidget(self.pages)
        log_frame, log_layout = _card("")
        log_head = QHBoxLayout()
        log_head.addWidget(_label("运行日志", "section"))
        log_head.addStretch()
        log_head.addWidget(_button("打开日志目录", lambda: self._open(self.data_dir / "logs")))
        log_head.addWidget(_button("清空显示", lambda: self.log_view.clear()))
        log_layout.addLayout(log_head)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1800)
        self.log_view.setObjectName("log")
        log_layout.addWidget(self.log_view)
        splitter.addWidget(log_frame)
        splitter.setSizes([round(540 * self.ui_scale), round(180 * self.ui_scale)])
        splitter.setChildrenCollapsible(False)
        layout.addWidget(splitter, 1)
        self.footer = _label(f"工程：{self.root}    ·    记录仅用于本机学习反馈", "muted")
        self.footer.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.footer)
        row.addWidget(content, 1)
        self.setCentralWidget(outer)
        self._navigate(0)

    def _home_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        self.gazebo_view = QFrame()
        self.gazebo_view.setObjectName("card")
        view_layout = QVBoxLayout(self.gazebo_view)
        view_layout.addWidget(_label("Gazebo 仿真视图", "section"))
        view_split = QSplitter(Qt.Horizontal)
        self.gazebo_placeholder = QLabel("启动仿真后，Gazebo 原生窗口将直接嵌入此区域。")
        self.gazebo_placeholder.setWordWrap(True)
        self.gazebo_placeholder.setAttribute(Qt.WA_NativeWindow, True)
        self.gazebo_placeholder.setAttribute(Qt.WA_DontCreateNativeAncestors, False)
        self.gazebo_placeholder.setAlignment(Qt.AlignCenter)
        self.gazebo_placeholder.setScaledContents(False)
        self.gazebo_placeholder.setMinimumHeight(250)
        self.gazebo_placeholder.setStyleSheet("background:#111827;color:#cbd5e1;border-radius:8px")
        view_split.addWidget(self.gazebo_placeholder)
        camera_panel = QWidget()
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(0, 0, 0, 0)
        self.camera_title = _label("腕部相机（彩色）", "section")
        camera_layout.addWidget(self.camera_title)
        self.camera_placeholder = QLabel("启动仿真后，腕部相机图像会在这里实时显示。")
        self.camera_placeholder.setWordWrap(True)
        self.camera_placeholder.setAlignment(Qt.AlignCenter)
        self.camera_placeholder.setMinimumWidth(260)
        self.camera_placeholder.setStyleSheet("background:#111827;color:#cbd5e1;border-radius:8px")
        camera_layout.addWidget(self.camera_placeholder, 1)
        self.recognition_status = QLabel("基础组识别：等待图像")
        self.recognition_status.setWordWrap(True)
        self.recognition_status.setObjectName("muted")
        self.recognition_status.setVisible(False)
        camera_layout.addWidget(self.recognition_status)
        view_split.addWidget(camera_panel)
        view_split.setSizes([700, 400])
        view_layout.addWidget(view_split, 1)
        self.gazebo_view.setMinimumWidth(520)
        card, cfg = _card("练习设置")
        grid = QGridLayout()
        grid.addWidget(_label("运行模式", "muted"), 0, 0)
        grid.addWidget(_label("题目种子（−1 随机）", "muted"), 0, 1)
        grid.addWidget(_label("基础组显示", "muted"), 0, 2)
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
        grid.addWidget(self.mode_combo, 1, 0)
        grid.addWidget(self.seed_spin, 1, 1)
        grid.addWidget(self.backend_combo, 1, 2)
        grid.addWidget(_label("参考速度", "muted"), 0, 3)
        grid.addWidget(self.velocity_spin, 1, 3)
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        cfg.addLayout(grid)
        self.mode_description = _label("基础组 Gazebo 仿真；工件场景为规划场景镜像，当前不代表真实接触/摩擦夹持。RViz 默认关闭，可选轻量模式。", "muted")
        cfg.addWidget(self.mode_description)
        layout.addWidget(card)
        middle = QHBoxLayout()
        control, actions = _card("操作")
        action_grid = QGridLayout()
        self.check_button = _button("环境自检", self.check_now)
        self.launch_button = _button("启动环境", self.start_environment, primary=True)
        self.generate_button = _button("生成题目", self.generate)
        self.reference_button = _button("运行参考程序", self.run_reference, primary=True)
        self.start_button = _button("发令 / 开始计时", self.start_round)
        self.stop_round_button = _button("停止本轮", self.stop_round, danger=True)
        self.reset_button = _button("重置场景", self.reset_scene)
        self.stop_stack_button = _button("关闭环境", self.stop_environment)
        for i, button in enumerate((self.check_button, self.launch_button, self.generate_button,
                                     self.reference_button, self.start_button, self.stop_round_button,
                                     self.reset_button, self.stop_stack_button)):
            action_grid.addWidget(button, i // 2, i % 2)
        self.stop_round_button.setToolTip("软件停止，不是真机安全急停；只用于本工作台的仿真。")
        actions.addLayout(action_grid)
        self.round_label = _label("等待启动环境", "section")
        self.scene_label = _label("题目：尚未生成", "muted")
        self.score_label = _label("完成：—    用时：—    分数：—", "muted")
        actions.addWidget(self.round_label)
        actions.addWidget(self.scene_label)
        actions.addWidget(self.score_label)
        actions.addStretch()
        middle.addWidget(control, 1)
        checks, check_layout = _card("就绪检查")
        self.check_table = QTableWidget(0, 3)
        self.check_table.setHorizontalHeaderLabels(["检查项", "状态", "说明"])
        self.check_table.verticalHeader().hide()
        self.check_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.check_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.check_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.check_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.check_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        check_layout.addWidget(self.check_table)
        middle.addWidget(checks, 2)
        middle.addWidget(self.gazebo_view, 3)
        layout.addLayout(middle, 1)
        return page

    def _code_page(self):
        page, layout = _card("我的程序")
        layout.addWidget(_label("示例读取生成的二维码 PNG，并使用已知工位坐标。它不是实时相机识别；代码在本机运行，没有安全沙箱。", "notice"))
        line = QHBoxLayout()
        self.program_path = QLineEdit(str(self.settings.value("program_path", "")))
        self.program_path.setPlaceholderText("选择自己编写的 .py 文件")
        self.program_path.setReadOnly(True)
        line.addWidget(self.program_path, 1)
        line.addWidget(_button("选择程序", self.choose_program))
        line.addWidget(_button("新建示例副本", self.new_program))
        layout.addLayout(line)
        buttons = QHBoxLayout()
        buttons.addWidget(_button("用系统编辑器打开", self.edit_program))
        self.student_button = _button("运行我的程序", self.run_student, primary=True)
        buttons.addWidget(self.student_button)
        buttons.addStretch()
        buttons.addWidget(_label("先生成题目 → 运行程序 → 回控制台发令", "muted"))
        layout.addLayout(buttons)
        self.code_preview = QPlainTextEdit()
        self.code_preview.setReadOnly(True)
        self.code_preview.setObjectName("code")
        layout.addWidget(self.code_preview, 1)
        self._load_preview()
        return page

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
        <p><b>基础练习：</b>Gazebo 主视图 + MoveIt 规划与基础组判分流程；可选轻量假硬件/RViz。
        当前工件由规划场景镜像跟随，抓取/判分并非 Gazebo 接触物理真值，不代表真实摩擦夹持。</p>
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
        # Scale the pixel-based stylesheet to match the desktop's logical DPI.
        # This is important on 4K + GNOME fractional scaling, where X11 exposes
        # a large virtual desktop but fixed 14px text remains physically tiny.
        font.setPixelSize(round(14 * self.ui_scale))
        self.setFont(font)
        stylesheet = """
        QMainWindow { background: #f2f4f7; color: #1d2939; }
        QWidget { color: #263445; font-family: "Noto Sans CJK SC"; font-size: 14px; }
        QFrame#sidebar { background: #142a36; }
        #sidebar QLabel { color: #d9e6ec; }
        QLabel[role="brand"] { font-size: 13px; letter-spacing: 2px; color: #75d4bf; font-weight: 700; }
        QLabel[role="sideTitle"] { font-size: 22px; font-weight: 700; margin-top: 10px; }
        QLabel[role="sideMuted"] { color: #8fa9b5; font-size: 11px; }
        QLabel[role="title"] { font-size: 23px; font-weight: 700; }
        QLabel[role="section"] { font-size: 14px; font-weight: 700; }
        QLabel[role="muted"] { color: #66788b; font-size: 12px; }
        QLabel[role="badge"] { background: #e2edf0; color: #245a64; border-radius: 14px; padding: 9px 15px; font-weight: 600; }
        QLabel[role="notice"] { background: #eaf0f7; color: #48627c; padding: 10px 13px; border-radius: 7px; font-size: 11px; }
        QFrame#card { background: white; border: 1px solid #e1e7ed; border-radius: 10px; }
        QPushButton { background: #f6f8fa; border: 1px solid #d9e2e9; border-radius: 6px; padding: 6px 11px; }
        QPushButton:hover { background: #e9f0f3; border-color: #9fb9c4; }
        QPushButton[role="primary"] { background: #167969; border-color: #167969; color: white; font-weight: 600; }
        QPushButton[role="primary"]:hover { background: #106455; }
        QPushButton[role="danger"] { color: #b44741; background: #fff5f4; border-color: #f0d2cf; }
        QPushButton:disabled { background: #f3f5f7; color: #aab4bf; border-color: #e5e9ee; }
        QPushButton#nav { color: #b9cbd5; background: transparent; text-align: left; border: none; padding: 12px 10px; }
        QPushButton#nav:checked { background: #26434f; color: #a4e6d5; }
        QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit { background: #f8fafb; border: 1px solid #dbe3ea; border-radius: 5px; padding: 7px; min-height: 22px; }
        QPlainTextEdit, QTextBrowser { background: #fbfcfd; border: none; padding: 5px; }
        QPlainTextEdit#log { background: #172b37; color: #d4e3e9; border-radius: 5px; font-family: monospace; font-size: 11px; }
        QPlainTextEdit#code { font-family: monospace; font-size: 12px; }
        QTableWidget { border: none; background: white; gridline-color: #edf1f5; font-size: 11px; }
        QHeaderView::section { font-size: 12px; background: #f5f7fa; color: #66788b; border: none; padding: 8px; text-align: left; }
        QSplitter::handle { background: #e4e9ef; height: 4px; }
        """
        stylesheet = re.sub(
            r"font-size:\s*(\d+)px",
            lambda match: f"font-size: {round(int(match.group(1)) * self.ui_scale)}px",
            stylesheet,
        )
        self.setStyleSheet(stylesheet)

    def _navigate(self, index):
        self.pages.setCurrentIndex(index)
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
                self.notice.setText("运行依赖尚未齐全，请按就绪检查的说明处理。工作台没有自动安装或修改系统。")
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

    def _service_busy(self):
        # OS 进程死亡不等于 GUI 已消费该请求的结果/退出事件。
        return self._service_inflight or self.manager.alive("service")

    def _can_start_program(self):
        return (self.ready and self._has_stack() and self.mode == "basic"
                and self.run_state == "armed" and not self._runner_active()
                and not self._service_busy() and not self._closing_stack)

    def start_round(self):
        if (not self.ready or not self._has_stack() or self.mode != "basic"
                or self.run_state != "armed" or self._closing_stack
                or self._service_busy() or self._auto_start_reference):
            return
        if not self.demo and not self._runner_active():
            self._log("请先启动学生程序，确认程序正在等待发令；参考程序会自动发令。")
            return
        self._service("start")

    def generate(self):
        if (not self.ready or not self._has_stack() or self.mode != "basic"
                or self._runner_active() or self.run_state == "running"
                or self._service_busy() or self._closing_stack):
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
            self.manager.launch("program", self._bridge("reference", "--velocity", str(self.velocity_spin.value())),
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
                or self._service_busy() or self._closing_stack):
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
                pixmap.scaled(self.camera_placeholder.size(), Qt.KeepAspectRatio,
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
        if self.last_status and now - self.last_status > 5:
            self.ready = False
            self.status_badge.setText("状态反馈中断")
        if self._auto_start_reference and now - self._reference_wait_at > 35:
            self._auto_start_reference = False
            self.manager.stop("program")
            self._log("参考程序准备超时，已取消自动发令。检查 MoveIt 和题目状态。")
        if self._closing_stack and not self.manager.active_names():
            self._closing_stack = False
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
        idle_controls = not stack and not self._checking and not self._closing_stack
        self.mode_combo.setEnabled(idle_controls)
        self.seed_spin.setEnabled(idle_controls and basic)
        self.velocity_spin.setEnabled(not runner and basic and not self._checking)
        self.backend_combo.setEnabled(idle_controls and basic)
        self.check_button.setEnabled(not self._checking and not self._closing_stack and not stack)
        self.launch_button.setEnabled(idle_controls)
        self.generate_button.setEnabled(stack and self.ready and basic and not runner and not running and not busy)
        self.reference_button.setEnabled(stack and self.ready and basic and armed and not runner and not busy)
        self.start_button.setEnabled(stack and self.ready and basic and armed and not busy
                                     and not self._auto_start_reference and (self.demo or runner))
        self.student_button.setEnabled(stack and self.ready and basic and armed and not runner and not busy
                                       and bool(self.program_path.text()))
        self.stop_round_button.setEnabled(stack and basic and (running or runner))
        self.reset_button.setEnabled(stack and self.ready and basic and not runner and not running and not busy)
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
        self.scene_label.setText(f"题目：seed={self.scene.get('seed')} · {self.scene.get('object_count', 0)} 件"
                                 if self.scene else "题目：尚未生成")
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
