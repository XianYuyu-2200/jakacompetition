#!/usr/bin/env python3
"""Create an auditable source installer, not an offline package or VM image."""
from __future__ import annotations
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import re
import subprocess
import tarfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
# Only reviewed repository files are exported. Never glob a developer's workspace.
PREFIXES = ('setup/', 'workbench/', 'jaka_ros2/src/', 'jaka_competition_kit/',
            'perception/track1_sim/', 'perception/track2_yolo/', 'design/', 'docs/', 'verification/')
EXACT = {'一键安装工作台.desktop', 'README.md', 'competition.md', 'jaka_ros2/launches.py', 'jaka_ros2/README.md',
         'verification/accept_workbench_fixed_poses.py'}
# These are the new reviewed files for an uncommitted installer candidate.
ADDITIONS = {'setup/install-student.sh', 'setup/student_install.py',
             'setup/package-student.py', 'setup/test/test_student_install.py',
             'docs/学生工作台一键安装.md', '一键安装工作台.desktop',
             'setup/student_install_desktop.py', 'setup/test/test_student_desktop.py',
             'workbench/theme.py', 'workbench/test/test_ui_layout.py',
             'docs/工作台界面改版与验收.md'}
BLOCKED_PARTS = {'.git', '.venv', '__pycache__', 'build', 'install', 'log', 'runs',
                 'weights', 'dataset', '.vendor', '.fontcache', 'gemini305_preview',
                 'track1_recognize', 'results', 'screenshots'}


def selected(name):
    p = Path(name)
    return (not p.is_absolute() and '..' not in p.parts
            and not (set(p.parts) & BLOCKED_PARTS)
            and p.suffix not in {'.pyc', '.pt', '.xwd'}
            and (name in EXACT or name.startswith(PREFIXES)))


def collect(root):
    tracked = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z']).decode().split('\0')
    names = sorted({n for n in tracked if selected(n)} | ADDITIONS)
    for name in names:
        path = root / name
        if not path.is_file() and not path.is_symlink():
            raise ValueError(f'缺少分发文件：{name}')
        if path.is_symlink():
            target = os.readlink(path)
            if os.path.isabs(target) or not path.resolve().is_relative_to(root.resolve()):
                raise ValueError(f'拒绝打包外部/绝对符号链接：{name}')
            relative = path.resolve().relative_to(root.resolve()).as_posix()
            if relative not in names and not any(n.startswith(relative + '/') for n in names):
                raise ValueError(f'符号链接目标未打包：{name}')
    return names


def write_package(root, output, version):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]{0,100}', version):
        raise ValueError('版本只能包含字母、数字、点、下划线和短横线。')
    names = collect(root)
    output.mkdir(parents=True, exist_ok=True)
    archive = output / f'jaka-student-{version}.tar.gz'
    if archive.exists():
        raise ValueError(f'拒绝覆盖已有安装包：{archive}')
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = bool(subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain']))
    metadata = {'format': 'source-online-installer', 'version': version, 'base_commit': revision,
                'working_tree_modified': dirty, 'created_utc': datetime.now(timezone.utc).isoformat(),
                'student_vm_acceptance': 'not_run', 'offline_dependencies_included': False, 'files': {}}
    for name in names:
        p = root / name
        metadata['files'][name] = ({'symlink': os.readlink(p)} if p.is_symlink() else
                                 {'sha256': hashlib.sha256(p.read_bytes()).hexdigest()})
    readme = '''JAKA 基础组学生工作台：联网源码安装包（不是 OVA / 离线镜像）
支持 Ubuntu 22.04 amd64 桌面虚拟机，无需预装 ROS。
1. 在 Ubuntu 内解压到用户目录，不要放在共享文件夹，不要改坏符号链接。
2. 推荐鼠标安装：将本目录解压为主文件夹中的 jaka-student，将
   「一键安装工作台.desktop」复制到 Ubuntu 桌面，右键「允许启动」，然后双击。
   会打开安装终端，提示时输入 sudo 密码，无需输入安装命令。
   命令行备用入口：bash setup/install-student.sh --install
   使用普通用户，不要 sudo 整个脚本。这不是 Windows 的 exe，须在 Ubuntu 虚拟机中打开。
3. 安装后从应用菜单打开「基础组学习工作台」。解压目录就是安装目录，请保留。
完整说明：docs/学生工作台一键安装.md
此候选包尚未通过干净学生 VM 的安装/图形/全流程验收。
'''
    temporary = archive.with_suffix('.tar.gz.partial')
    try:
        with tarfile.open(temporary, 'w:gz', format=tarfile.PAX_FORMAT, dereference=False) as tar:
            prefix = 'jaka-student/'
            for name in names:
                info = tar.gettarinfo(str(root / name), arcname=prefix + name)
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                info.mode = 0o777 if info.issym() else (0o755 if info.mode & 0o111 else 0o644)
                if info.isfile():
                    with (root / name).open('rb') as f:
                        tar.addfile(info, f)
                else:
                    tar.addfile(info)
            for name, data in {'INSTALL.txt': readme,
                               'PACKAGE-MANIFEST.json': json.dumps(metadata, ensure_ascii=False, indent=2) + '\n'}.items():
                encoded = data.encode()
                info = tarfile.TarInfo(prefix + name)
                info.size, info.mode = len(encoded), 0o644
                tar.addfile(info, io.BytesIO(encoded))
        temporary.rename(archive)
    finally:
        temporary.unlink(missing_ok=True)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_name(archive.name + '.sha256').write_text(f'{digest}  {archive.name}\n')
    print(f'已生成：{archive}\nSHA-256：{digest}\n文件数：{len(names)}；仅为联网安装候选包，学生 VM 验收未运行。')
    return archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'dist')
    parser.add_argument('--version', default=datetime.now().strftime('%Y%m%d-%H%M%S-candidate'))
    args = parser.parse_args()
    try:
        write_package(ROOT, args.output, args.version)
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        parser.exit(1, f'打包失败：{exc}\n')


if __name__ == '__main__':
    main()
