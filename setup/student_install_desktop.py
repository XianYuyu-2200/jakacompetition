#!/usr/bin/env python3
"""Standard-library-only desktop installer entry, usable before Qt/ROS exist."""
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = '一键安装工作台.desktop'

# %k is provided by the desktop launcher. Pass it as data, never interpolate it
# into shell code. The HOME fallback supports copying the icon to the desktop.
BOOTSTRAP = '''import runpy, sys
from pathlib import Path
from urllib.parse import urlsplit, unquote
status = 1
try:
    value = sys.argv[1]
    uri = urlsplit(value)
    if value.startswith('file:'):
        if uri.netloc not in ('', 'localhost'):
            raise RuntimeError('请先将安装包复制到虚拟机本地目录。')
        location = Path(unquote(uri.path))
    else:
        location = Path(value)
    roots = (location.resolve().parent, Path.home() / 'jaka-student')
    script = next((root / 'setup/student_install_desktop.py' for root in roots
                   if (root / 'setup/student_install_desktop.py').is_file()), None)
    if script is None:
        raise RuntimeError('找不到安装包。请在 Ubuntu 内将安装包解压至主文件夹，确认 ~/jaka-student/setup 存在。不要只复制这个图标。')
    entry = runpy.run_path(str(script), run_name='jaka_desktop_entry')
    status = entry['install']()
except (Exception, KeyboardInterrupt) as exc:
    print('安装未完成：' + str(exc), flush=True)
if sys.stdin.isatty():
    try:
        input('\\n按回车关闭此窗口……')
    except (EOFError, KeyboardInterrupt):
        pass
sys.exit(status)
'''


def desktop_arg(value: str) -> str:
    # Desktop Entry string unescaping happens before Exec argument unescaping.
    escaped = value.replace('\\', '\\\\').replace('"', '\\"')
    escaped = escaped.replace('`', '\\`').replace('$', '\\$')
    return '"' + escaped.replace('\\', '\\\\').replace('%', '%%') + '"'


def desktop_content() -> str:
    code = 'exec(' + repr(BOOTSTRAP) + ')'
    command = '/usr/bin/python3 -c ' + desktop_arg(code) + ' %k'
    return ('[Desktop Entry]\nType=Application\nVersion=1.0\n'
            'Name=一键安装基础组工作台\n'
            'Comment=在 Ubuntu 虚拟机中安装 ROS、Gazebo、工作台及桌面快捷方式\n'
            'Icon=system-software-install\nTerminal=true\nStartupNotify=false\n'
            'Categories=Education;\n'
            f'Exec={command}\n')


def install(root: Path = ROOT) -> int:
    print('=== 基础组学习工作台 · 一键安装 ===\n'
          '适用 Ubuntu 22.04 amd64 桌面虚拟机。\n'
          '即将联网安装依赖并编译；提示时输入当前账户的 sudo 密码。\n'
          '输入密码时终端不显示字符，输入后按回车即可。\n'
          '不要关闭此窗口。不会启动仿真或连接真实设备。\n', flush=True)
    script = root / 'setup/install-student.sh'
    if not script.is_file():
        print('安装包不完整：缺少 setup/install-student.sh，请重新解压完整安装包。', flush=True)
        return 1
    result = subprocess.run(['/bin/bash', str(script), '--install'], cwd=root, check=False)
    if result.returncode == 0:
        print('\n安装和预检已完成。请从应用菜单打开「基础组学习工作台」。\n'
              '仍需检查 Gazebo、相机和六工位流程，不代表仿真验收通过。', flush=True)
    else:
        print(f'\n安装未完成（退出码 {result.returncode}）。请保留上方错误和日志，修复后可再次双击安装。', flush=True)
    return result.returncode if result.returncode >= 0 else 128 - result.returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-launcher', action='store_true', help='仅生成可移植的桌面启动文件，不安装')
    args = parser.parse_args(argv)
    if args.write_launcher:
        path = ROOT / LAUNCHER
        path.write_text(desktop_content(), encoding='utf-8')
        path.chmod(0o755)
        print(f'已生成：{path}')
        return 0
    parser.print_help()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
