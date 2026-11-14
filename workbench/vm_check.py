"""Read-only teaching-VM diagnostics, not simulation or hardware acceptance.

Only explicitly requested --vm-check runs external probes. No shell, sudo,
package changes, ROS nodes, robot services or automatic graphics overrides.
"""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess


GIB = 1024 ** 3


def probe(argv, timeout=8):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                                check=False, env={**os.environ, 'LC_ALL': 'C'})
        return {'command': argv, 'returncode': result.returncode,
                'stdout': result.stdout[-16000:], 'stderr': result.stderr[-2000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'command': argv, 'returncode': None, 'stdout': '', 'stderr': str(exc)}


def graphics_checks(result):
    """A usable GL context is necessary, never sufficient for Ogre/camera use."""
    def item(level, detail):
        return {'name': 'OpenGL 实际上下文', 'level': level, 'detail': detail}
    if result['returncode'] != 0:
        return [item('error', 'glxinfo 无法创建/查询上下文；检查桌面会话和虚拟机 3D 加速')]
    text = result['stdout']
    # Prefer core profile; legacy compatibility can advertise a lower version.
    match = re.search(r'OpenGL core profile version string:\s*(\d+)\.(\d+)', text)
    if match is None:
        match = re.search(r'OpenGL version string:\s*(\d+)\.(\d+)', text)
    if match is None:
        return [item('warning', '无法解析 OpenGL 版本，不能确认图形前置条件')]
    version = tuple(map(int, match.groups()))
    checks = [item('error' if version < (3, 3) else 'warning' if version < (4, 3) else 'ok',
                   f'OpenGL {version[0]}.{version[1]}；版本不证明 Ogre 扩展或相机渲染可用')]
    if re.search(r'llvmpipe|softpipe|software rasterizer', text, re.I):
        checks.append({'name': '软件渲染', 'level': 'warning',
                       'detail': '检测到 CPU 软件渲染；须实测帧率与整轮耗时，不自动改驱动或版本号'})
    return checks


def collect_vm_report(root):
    root = Path(root)
    checks, probes = [], {}
    def add(name, level, detail):
        checks.append({'name': name, 'level': level, 'detail': detail})
    try:
        release = platform.freedesktop_os_release()
    except OSError:
        release = {}
    target = release.get('ID') == 'ubuntu' and release.get('VERSION_ID') == '22.04'
    add('教学镜像系统', 'ok' if target else 'error',
        f"{release.get('PRETTY_NAME', '未知系统')}；当前教学镜像目标固定为 Ubuntu 22.04")
    machine = platform.machine()
    add('教学镜像架构', 'ok' if machine == 'x86_64' else 'error',
        f'{machine}；本交付候选仅面向 amd64，不代表 ARM 已验收')
    cpus = os.cpu_count() or 0
    add('可见逻辑 CPU', 'ok' if cpus >= 4 else 'warning',
        f'{cpus}；4 vCPU 是测试起点，不是已测最低配置')
    try:
        memory = int(re.search(r'^MemTotal:\s+(\d+)', Path('/proc/meminfo').read_text(), re.M)[1]) / 1024
        add('客体内存', 'ok' if memory >= 7500 else 'warning',
            f'{memory / 1024:.1f} GiB；建议起点 8GB，运行性能仍需验收')
    except (OSError, TypeError, ValueError):
        add('客体内存', 'warning', '无法读取 /proc/meminfo')
    try:
        free = shutil.disk_usage(root).free / GIB
        add('工程所在磁盘剩余空间', 'ok' if free >= 20 else 'warning',
            f'{free:.1f} GiB；低于 20GiB 时先检查编译和日志空间，不自动清理')
    except OSError as exc:
        add('工程所在磁盘剩余空间', 'warning', str(exc))
    tool = shutil.which('systemd-detect-virt')
    if tool:
        result = probes['virtualization'] = probe([tool])
        add('虚拟化检测', 'ok' if result['returncode'] == 0 else 'warning',
            (result['stdout'].strip() or '未识别虚拟化') + '；不代替宿主机型号与虚拟机版本记录')
    else:
        add('虚拟化检测', 'warning', '缺少 systemd-detect-virt，环境类型未知')
    display = bool(os.environ.get('DISPLAY'))
    add('X11 图形会话', 'ok' if display else 'error',
        '已设置 DISPLAY；下面单独检查连接' if display else '无 DISPLAY，无法验收当前工作台窗口嵌入')
    if os.environ.get('WAYLAND_DISPLAY'):
        add('Wayland 会话', 'warning', '窗口嵌入需要实测；教学镜像优先在 Ubuntu on Xorg 会话验收')
    glx = shutil.which('glxinfo')
    if not glx:
        add('OpenGL 检查工具', 'error', '缺少 glxinfo；由教师安装 mesa-utils 后重测')
    elif display:
        result = probes['opengl'] = probe([glx, '-B'])
        checks.extend(graphics_checks(result))
    overrides = {name: os.environ[name] for name in (
        'LIBGL_ALWAYS_SOFTWARE', 'LIBGL_DRI3_DISABLE', 'MESA_GL_VERSION_OVERRIDE',
        'MESA_GLSL_VERSION_OVERRIDE', 'QT_QPA_PLATFORM') if name in os.environ}
    if overrides:
        add('图形环境覆盖', 'warning', f'检测到 {overrides}；记录原值，不能将伪装版本号视为能力验证')
    for module in ('numpy', 'cv2', 'yaml', 'qrcode', 'pyzbar', 'PIL'):
        try:
            available = importlib.util.find_spec(module) is not None
        except (ImportError, ValueError, AttributeError):
            available = False
        add(f'教学 Python 依赖 {module}', 'ok' if available else 'error',
            '可发现模块；实际二维码解码仍需回放验证' if available else '当前 Python 未找到模块，请由教师补齐依赖')
    # Full package versions allow teachers to compare imported images without
    # publishing the rest of the environment, usernames, keys or network config.
    dpkg = shutil.which('dpkg-query')
    if dpkg:
        probes['packages'] = probe([dpkg, '-W', '-f=${binary:Package}\t${Version}\n',
            'ros-humble-ros-base', 'ros-humble-ros-gz', 'ignition-fortress',
            'python3-pyqt5', 'python3-pyzbar', 'python3-qrcode', 'mesa-utils'])
    git = shutil.which('git')
    if git:
        probes['revision'] = probe([git, '-C', str(root), 'rev-parse', 'HEAD'])
        probes['worktree'] = probe([git, '-C', str(root), 'status', '--porcelain', '--untracked-files=no'])
    return {'scope': 'read-only teaching VM preflight; no robot commands',
            'simulation_acceptance': 'not_run', 'hardware_acceptance': 'not_run',
            'checks': checks, 'probes': probes}
