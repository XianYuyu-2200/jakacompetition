"""不要求预先 source ROS 的工作台入口。"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="JAKA 基础组仿真学习工作台")
    parser.add_argument("--demo", action="store_true", help="只演示界面，不启动 ROS 或机械臂")
    parser.add_argument("--root", type=Path, help="工程根目录（默认自动定位）")
    parser.add_argument("--data-dir", type=Path, default=Path(os.environ.get(
        "XDG_DATA_HOME", str(Path.home() / ".local/share"))) / "jaka-workbench")
    parser.add_argument("--check", action="store_true", help="输出 JSON 环境检查，不启动界面")
    parser.add_argument("--vm-check", action="store_true", help="只读教学虚拟机检查（含 OpenGL 探测），输出 JSON，不启动仿真")
    parser.add_argument("--mode", choices=("basic", "recognition", "vision"), default="basic")
    args = parser.parse_args(argv)
    from .runtime import check_environment, discover_root, simulation_conflicts
    try:
        root = discover_root(args.root) if args.root else discover_root()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.check or args.vm_check:
        checks = check_environment(root, args.mode)
        report = {"root": str(root), "mode": args.mode, "checks": checks,
                  "conflicts": simulation_conflicts()}
        if args.vm_check:
            from .vm_check import collect_vm_report
            report["vm"] = collect_vm_report(root)
            checks.extend(report["vm"]["checks"])
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1 if any(c["level"] == "error" for c in checks) else 0
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
                               or os.environ.get("QT_QPA_PLATFORM")):
        print("没有图形桌面。请在 Ubuntu 桌面终端运行，或使用 --check 自检。", file=sys.stderr)
        return 2
    try:
        from PyQt5.QtWidgets import QApplication
        from .app import WorkbenchWindow
    except ImportError as exc:
        print(f"工作台界面依赖未就绪：{exc}\n可安装 python3-pyqt5 后重试。", file=sys.stderr)
        return 2
    # X11 WM_CLASS is a legacy STRING, not UTF8_STRING. Keep both its
    # instance (argv[0]) and class (applicationName) ASCII; use the Unicode
    # display name and matching desktop entry for the user-facing Chinese name.
    # Do not pass workbench CLI options through to Qt.
    app = QApplication(["jaka-competition-workbench"])
    app.setApplicationName("jaka-competition-workbench")
    app.setApplicationDisplayName("基础组学习工作台")
    app.setDesktopFileName("jaka-competition-workbench")
    app.setOrganizationName("JAKACompetitionLearning")
    window = WorkbenchWindow(root, args.data_dir.expanduser().resolve(),
                             demo=args.demo, mode=args.mode)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
