#!/usr/bin/env python3
"""Provision a clean Jammy desktop, never launch simulation or robot processes."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import tempfile
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
ROS_VERSION = '1.3.0'
ROS_SHA256 = '110b9a462d55252decb8b7c816f61c2ba0d9890ce5fb93ac504e97cae5860d76'
ROS_URL = ('https://github.com/ros-infrastructure/ros-apt-source/releases/download/'
           f'{ROS_VERSION}/ros2-apt-source_{ROS_VERSION}.jammy_all.deb')
GAZEBO_URL = 'https://packages.osrfoundation.org/gazebo.gpg'
GAZEBO_FINGERPRINT = 'D2486D2DD83DB69272AFE98867170598AF249743'
PLAN = '''学生工作台联网安装（Ubuntu 22.04 amd64 桌面版）：
1. 检查普通用户、系统、源码、可用空间（至少 15 GiB），申请 sudo。
2. 启用 Ubuntu universe；配置 ROS 官方软件源（固定版本及 SHA-256），
   配置 Gazebo 官方签名软件源（校验密钥指纹）；已有源不覆盖。
3. 安装 ROS 2 Humble、Fortress、MoveIt、控制器、Qt、相机/二维码依赖。
4. 本机低并发编译，创建当前用户的桌面/应用菜单快捷方式。
5. 保存依赖版本、安装日志和只读预检；不启动仿真、不连接真机。
需要联网下载数 GB；不会执行整机 apt upgrade，不覆盖用户姿态和标定。
不会修改全局 MoveIt Python 文件、.bashrc 或图形版本变量。
默认仅显示计划；确认后加 --install。此包不是离线包或已验收的 OVA。
'''


def validate_host(root: Path):
    if os.geteuid() == 0:
        raise RuntimeError('请使用学生普通账户运行，不要 sudo 整个安装脚本。')
    info = platform.freedesktop_os_release()
    if info.get('ID') != 'ubuntu' or info.get('VERSION_ID') != '22.04' or platform.machine() != 'x86_64':
        raise RuntimeError('仅支持 Ubuntu 22.04 amd64 桌面版；ARM/其他 Ubuntu 版本不能混装 Humble。')
    if shutil.disk_usage(root).free < 15 * 1024 ** 3:
        raise RuntimeError('源码所在磁盘可用空间不足 15 GiB，请先扩容。')
    if shutil.disk_usage('/').free < 10 * 1024 ** 3:
        raise RuntimeError('系统磁盘可用空间不足 10 GiB，无法安装依赖。')
    if not shutil.which('sudo'):
        raise RuntimeError('缺少 sudo，请使用具有管理员权限的 Ubuntu 桌面普通账户。')
    if any(c.isspace() for c in str(root)):
        raise RuntimeError('ROS/CMake 源码路径不能包含空格，请解压到 ~/jaka-student 等路径。')
    for name in ('setup/build.sh', 'setup/install-deps.sh', 'setup/run-workbench.sh',
                 'jaka_ros2/src/jaka_minicobo_moveit_config/package.xml',
                 'jaka_competition_kit/package.xml'):
        if not (root / name).is_file():
            raise RuntimeError(f'安装包不完整，缺少 {name}')
    link = root / 'jaka_ros2/src/jaka_competition_kit'
    if not link.is_symlink() or link.resolve() != root / 'jaka_competition_kit':
        raise RuntimeError('比赛套件相对符号链接丢失；请在 Ubuntu 内使用 tar 解压原始安装包。')


def verify_sha256(path: Path, expected: str):
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise RuntimeError(f'下载文件 SHA-256 不匹配，拒绝安装：{path.name}')


def verify_gazebo_key(colons: str):
    # Only the primary public key fingerprint is trusted, not a matching subkey.
    lines = colons.splitlines()
    primary = [lines[i + 1].split(':')[9] for i, line in enumerate(lines[:-1])
               if line.startswith('pub:') and lines[i + 1].startswith('fpr:')]
    if primary != [GAZEBO_FINGERPRINT]:
        raise RuntimeError('Gazebo 签名密钥指纹不匹配；拒绝导入，请由维护者核对官方密钥轮换。')


def source_exists(marker: str, apt_root=Path('/etc/apt')) -> bool:
    paths = [apt_root / 'sources.list']
    for pattern in ('*.list', '*.sources'):
        paths += list((apt_root / 'sources.list.d').glob(pattern))
    for path in paths:
        if path.is_file():
            active = '\n'.join(line for line in path.read_text().splitlines()
                               if not line.lstrip().startswith('#'))
            if marker in active:
                return True
    return False


class Installer:
    def __init__(self, root: Path, log_dir: Path):
        self.root, self.log_dir = root, log_dir
        self.log = (log_dir / 'install.log').open('w', encoding='utf-8')
        self.env = os.environ.copy()
        # Do not build against the developer's overlay, Conda or another ROS distro.
        for key in list(self.env):
            if key.startswith(('ROS_', 'AMENT_', 'COLCON_', 'CONDA', 'VIRTUAL_ENV')) or key in {
                    'PYTHONPATH', 'PYTHONHOME', 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'MAKEFLAGS'}:
                self.env.pop(key, None)
        self.env.update(PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
                        LANG='C.UTF-8', LC_ALL='C.UTF-8', ROS_DISTRO='humble',
                        WORKBENCH_PYTHON='/usr/bin/python3', CMAKE_BUILD_PARALLEL_LEVEL='2',
                        MAKEFLAGS='-j2')

    def run(self, args, *, capture=False, check=True):
        args = [str(a) for a in args]
        print('+ ' + shlex.join(args), flush=True)
        self.log.write('\n+ ' + shlex.join(args) + '\n')
        self.log.flush()
        if capture:
            p = subprocess.run(args, cwd=self.root, env=self.env, text=True,
                               stdout=subprocess.PIPE, stderr=self.log, check=False)
            self.log.write(p.stdout)
        else:
            p = subprocess.run(args, cwd=self.root, env=self.env, text=True,
                               stdout=self.log, stderr=subprocess.STDOUT, check=False)
        self.log.flush()
        if check and p.returncode:
            raise RuntimeError(f'命令失败（退出码 {p.returncode}）：{shlex.join(args)}')
        return p

    def apt(self, *args):
        return self.run(['sudo', 'apt-get', '-o', 'Acquire::Retries=3', *args])

    def download(self, url: str, path: Path):
        self.run(['curl', '--fail', '--location', '--show-error', '--silent',
                  '--proto', '=https', '--proto-redir', '=https', '--retry', '3',
                  '--connect-timeout', '20', '--max-time', '300', '--output', path, url])

    def configure_sources(self, temp: Path):
        if not source_exists('packages.ros.org/ros2/ubuntu'):
            deb = temp / 'ros2-apt-source.deb'
            self.download(ROS_URL, deb)
            verify_sha256(deb, ROS_SHA256)
            self.run(['sudo', 'dpkg', '-i', deb])
        else:
            print('保留已有 ROS 2 软件源；若过期或冲突，apt 将报错并停止，不自动删除。')
        if not source_exists('packages.osrfoundation.org/gazebo/ubuntu'):
            key = temp / 'gazebo.gpg'
            self.download(GAZEBO_URL, key)
            keys = self.run(['gpg', '--batch', '--homedir', temp, '--show-keys', '--with-colons', key],
                            capture=True).stdout
            verify_gazebo_key(keys)
            target = Path('/usr/share/keyrings/jaka-student-gazebo.gpg')
            source = Path('/etc/apt/sources.list.d/jaka-student-gazebo.list')
            # Never replace an unrelated/symlinked source file.
            content = (f'deb [arch=amd64 signed-by={target}] '
                       'https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main\n')
            if source.is_symlink() or (source.exists() and source.read_text() != content):
                raise RuntimeError(f'软件源文件冲突，未覆盖：{source}')
            if target.is_symlink() or (target.exists() and target.read_bytes() != key.read_bytes()):
                raise RuntimeError(f'签名密钥文件冲突，未覆盖：{target}')
            self.run(['sudo', 'install', '-m', '644', key, target])
            entry = temp / 'gazebo.list'
            entry.write_text(content)
            self.run(['sudo', 'install', '-m', '644', entry, source])
        else:
            print('保留已有 Gazebo 软件源；使用 apt 的签名验证。')

    def install(self):
        self.apt('update')
        self.apt('install', '-y', 'ca-certificates', 'curl', 'gnupg',
                 'software-properties-common', 'build-essential', 'cmake', 'pkg-config')
        self.run(['sudo', 'add-apt-repository', '-y', 'universe'])
        with tempfile.TemporaryDirectory(prefix='jaka-student-') as tmp:
            self.configure_sources(Path(tmp))
        self.apt('update')
        # ROS docs warn old Jammy systemd/udev packages may remove desktop dependencies.
        self.apt('install', '-y', '--only-upgrade', 'systemd', 'systemd-sysv', 'udev')
        self.apt('install', '-y', 'ros-humble-desktop')
        self.run(['bash', self.root / 'setup/install-deps.sh'])
        self.run(['bash', self.root / 'setup/build.sh', '--student'])
        self.run(['bash', self.root / 'setup/install-workbench-desktop.sh'])
        versions = self.run(['dpkg-query', '-W', '-f=${binary:Package}\t${Version}\n'], capture=True)
        (self.log_dir / 'packages.tsv').write_text(versions.stdout)
        self.run(['bash', self.root / 'setup/run-workbench.sh', '--check'])
        report = self.run(['bash', self.root / 'setup/run-workbench.sh', '--vm-check'],
                          capture=True, check=False)
        (self.log_dir / 'vm-preflight.json').write_text(report.stdout)
        if report.returncode:
            raise RuntimeError('依赖/编译完成，但 VM 预检未通过；检查 vm-preflight.json（图形终端/Xorg/3D），修复后重跑。')
        print('安装和前置检查完成。请打开「JAKA 比赛工作台」；仿真运动与相机仍需实际验收。')


def main(argv=None):
    parser = argparse.ArgumentParser(description=PLAN,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--plan', action='store_true', help='只输出计划（默认）')
    group.add_argument('--install', action='store_true', help='联网安装、编译和创建快捷方式')
    args = parser.parse_args(argv)
    if not args.install:
        print(PLAN)
        return 0
    log_dir = None
    installer = None
    try:
        validate_host(ROOT)
        import fcntl
        state = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')) / 'jaka-workbench'
        state.mkdir(parents=True, exist_ok=True)
        with (state / 'installer.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('当前账户已有安装任务运行，请勿重复点击。')
            subprocess.run(['sudo', '-v'], check=True)
            log_dir = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('install-%Y%m%d-%H%M%S-'), dir=state))
            print(f'日志目录：{log_dir}\n耗时步骤的详细输出可另开终端查看：tail -f {shlex.quote(str(log_dir / "install.log"))}', flush=True)
            installer = Installer(ROOT, log_dir)
            installer.install()
            (log_dir / 'result.json').write_text(json.dumps({
                'installation': 'passed', 'simulation_acceptance': 'not_run',
                'hardware_acceptance': 'not_run'}, indent=2) + '\n')
        return 0
    except (RuntimeError, OSError, subprocess.SubprocessError, KeyboardInterrupt) as exc:
        print(f'安装未完成：{exc}', flush=True)
        if log_dir:
            (log_dir / 'result.json').write_text(json.dumps({
                'installation': 'failed', 'error': str(exc),
                'simulation_acceptance': 'not_run', 'hardware_acceptance': 'not_run'}, ensure_ascii=False, indent=2) + '\n')
            print(f'请保留日志：{log_dir}。修复原因后重跑同一命令；不要删除个人姿态文件。')
        return 1
    finally:
        if installer:
            installer.log.close()


if __name__ == '__main__':
    raise SystemExit(main())
