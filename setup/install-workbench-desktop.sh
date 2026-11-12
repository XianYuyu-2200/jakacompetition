#!/usr/bin/env bash
# Install launchers for this user only. No sudo, dependency installation or ROS startup.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PYTHON="${WORKBENCH_PYTHON:-/usr/bin/python3}"
if ! command -v -- "$PYTHON" >/dev/null 2>&1; then
  printf '找不到 Python：%s\n' "$PYTHON" >&2
  exit 127
fi

exec "$PYTHON" - "$SCRIPT_DIR/run-workbench.sh" <<'PY'
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

NAME = "jaka-competition-workbench.desktop"
MARKER = "X-Jaka-Workbench=true"


def exec_arg(value):
    # Desktop Entry Exec is NOT shell syntax. Escape its quoted argument first,
    # then escape the backslashes for the desktop-file string layer.
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    escaped = escaped.replace("`", "\\`").replace("$", "\\$")
    return '"' + escaped.replace("\\", "\\\\").replace("%", "%%") + '"'


def install(path, content):
    if path.is_symlink():
        raise RuntimeError(f"拒绝覆盖符号链接：{path}")
    if path.exists():
        if not path.is_file() or MARKER not in path.read_text(encoding="utf-8").splitlines():
            raise RuntimeError(f"同名文件不是本工作台的快捷方式，未覆盖：{path}")
    fd, temporary = tempfile.mkstemp(prefix=f".{NAME}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fchmod(stream.fileno(), 0o755)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    print(f"已安装：{path}")


def main():
    launcher = Path(sys.argv[1]).absolute()
    if not launcher.is_file() or not os.access(launcher, os.X_OK):
        raise RuntimeError(f"启动脚本不存在或不可执行：{launcher}")
    if any(ord(char) < 32 for char in str(launcher)):
        raise RuntimeError("项目路径不能包含换行或控制字符")
    home = Path.home()
    data = Path(os.environ.get("XDG_DATA_HOME") or home / ".local/share")
    if not data.is_absolute():
        raise RuntimeError("XDG_DATA_HOME 必须为绝对路径")
    applications = data / "applications"
    applications.mkdir(parents=True, exist_ok=True)
    # Keep a fixed executable in Exec: GIO checks it before expanding %% in
    # field-code arguments. env passes the absolute script path without a shell.
    content = (
        "[Desktop Entry]\n"
        "Type=Application\n"
        "Version=1.0\n"
        "Name=基础组学习工作台\n"
        "Comment=Ubuntu 机械臂仿真学习工作台\n"
        f"Exec=/usr/bin/env {exec_arg(str(launcher))}\n"
        "Icon=applications-education\n"
        "Terminal=false\n"
        "Categories=Education;Science;\n"
        "StartupNotify=true\n"
        f"{MARKER}\n"
    )
    install(applications / NAME, content)
    desktop_tool = shutil.which("xdg-user-dir")
    if desktop_tool:
        try:
            result = subprocess.run([desktop_tool, "DESKTOP"], capture_output=True,
                                    text=True, timeout=5, check=True)
            desktop = Path(result.stdout.strip())
            if (desktop.is_absolute() and desktop.is_dir()
                    and desktop.resolve() not in (home.resolve(), applications.resolve())):
                install(desktop / NAME, content)
            else:
                print("未找到独立的桌面目录，已仅安装应用菜单入口。")
        except (OSError, subprocess.SubprocessError) as exc:
            print(f"无法查询桌面目录，已仅安装应用菜单入口：{exc}", file=sys.stderr)
    print("若桌面首次提示不可信，请右键选择「允许启动」。移动源码目录后请重新运行安装脚本。")


try:
    main()
except (OSError, RuntimeError, UnicodeError) as exc:
    print(f"快捷方式安装失败：{exc}", file=sys.stderr)
    sys.exit(1)
PY
