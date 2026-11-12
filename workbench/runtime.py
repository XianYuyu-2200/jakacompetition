"""Standard-library-only runtime services for the Ubuntu learning workbench.

No ROS/Qt import, simulation startup, package installation, or external-process
termination happens during inspection. ProcessManager owns only jobs it starts.
Its UI-facing stop() is nonblocking; call poll() regularly to advance escalation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import importlib.util
import os
from pathlib import Path
import queue
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid
from typing import Mapping, Sequence, TextIO


_ROS_SETUP = Path('/opt/ros/humble/setup.bash')
_ROS_EXECUTABLE = Path('/opt/ros/humble/bin/ros2')


def discover_root(start: Path | None = None) -> Path:
    """Find the repository above *start* (or this module), not above cwd."""
    origin = Path(start) if start is not None else Path(__file__)
    origin = origin.expanduser().resolve()
    if origin.is_file():
        origin = origin.parent
    for candidate in (origin, *origin.parents):
        if ((candidate / 'jaka_competition_kit').is_dir()
                and (candidate / 'jaka_ros2').is_dir()):
            return candidate
    raise RuntimeError(f'未找到仓库根目录（需要 jaka_competition_kit/ 和 jaka_ros2/）：{origin}')


def check_environment(root: Path, mode: str = 'basic', backend: str = 'gazebo') -> list[dict]:
    """Read-only prerequisite checks; OK never means ROS controllers are ready.

    Uses only metadata, import specifications and PATH lookup. No subprocess is
    executed. Check ROS topics/controllers separately after starting a job.
    """
    root = Path(root)
    results: list[dict] = []

    def add(name: str, level: str, detail: str) -> None:
        results.append({'name': name, 'level': level, 'detail': detail})

    def file_check(name: str, path: Path, executable: bool = False) -> None:
        exists = path.is_file() and (not executable or os.access(path, os.X_OK))
        detail = str(path) if exists else f'缺失或不可访问：{path}'
        add(name, 'ok' if exists else 'error', detail)

    linux = sys.platform.startswith('linux')
    add('操作系统', 'ok' if linux else 'error',
        'Linux；仅检查运行前置条件，不代表仿真已就绪' if linux else '当前平台不是 Linux；本版本需要 Ubuntu/Linux')
    if mode not in ('basic', 'vision', 'recognition'):
        add('运行模式', 'error', f'不支持的模式：{mode!r}，请选择 basic、recognition 或 vision')
    file_check('ROS Humble 环境脚本', _ROS_SETUP)
    file_check('ROS 2 可执行文件', _ROS_EXECUTABLE, executable=True)
    file_check('工作区环境脚本', root / 'jaka_ros2/install/setup.bash')
    add('运行就绪状态', 'warning', '以上脚本存在仅表示安装文件可见；尚未检查控制器、MoveIt 或相机是否就绪')
    try:
        qt_available = importlib.util.find_spec('PyQt5') is not None
    except (ImportError, ValueError, AttributeError):
        qt_available = False
    add('PyQt5', 'ok' if qt_available else 'error',
        '当前 Python 可发现 PyQt5（未导入 GUI）' if qt_available else f'当前 Python 找不到 PyQt5：{sys.executable}')
    display = os.environ.get('DISPLAY', '')
    add('图形会话 DISPLAY', 'ok' if display else 'warning',
        f'DISPLAY={display}（尚未验证连接或 OpenGL）' if display else '未设置 DISPLAY，RViz/Gazebo 图形窗口可能无法打开')
    for relative in (
        'jaka_competition_kit/package.xml',
        'jaka_competition_kit/launch/round.launch.py',
        'jaka_competition_kit/launch/workbench.launch.py',
        'jaka_competition_kit/config/workbench.rviz',
        'jaka_ros2/src/wheeltec_gripper/package.xml',
        'jaka_ros2/src/wheeltec_gripper/config/gripper.yaml',
        'jaka_ros2/src/wheeltec_gripper/urdf/wheeltec_gripper.urdf.xacro',
        'jaka_competition_kit/jaka_competition_kit/reference.py',
        'jaka_competition_kit/jaka_competition_kit/executor.py',
        'jaka_competition_kit/jaka_competition_kit/scorer.py',
        'jaka_competition_kit/config/arena.yaml',
        'jaka_competition_kit/config/rules.yaml',
    ):
        file_check(f'基础文件 {Path(relative).name}', root / relative)
    if (mode == 'basic' and backend == 'gazebo') or mode == 'recognition':
        ign = shutil.which('ign')
        add('Gazebo Fortress', 'ok' if ign else 'error', ign or 'PATH 中未找到 ign；基础组 Gazebo 模式需要 Gazebo Fortress')
        for tool in ('xwininfo', 'xprop'):
            found = shutil.which(tool)
            add(f'窗口嵌入工具 {tool}', 'ok' if found else 'warning', found or '缺失；Gazebo 将以独立窗口运行')
    if mode == 'recognition':
        for module, name in (('cv2', 'OpenCV Python'), ('numpy', 'NumPy Python')):
            try:
                available = importlib.util.find_spec(module) is not None
            except (ImportError, ValueError, AttributeError):
                available = False
            add(name, 'ok' if available else 'error',
                f'{sys.executable} 可发现 {module}' if available else
                f'{sys.executable} 找不到 {module}')
        for name, relative in (
            ('基础组识别启动文件', 'jaka_competition_kit/launch/gemini305_preview.launch.py'),
            ('基础组几何体分类器', 'jaka_competition_kit/jaka_competition_kit/track1_recognize.py'),
            ('识别场景关节配置', 'jaka_competition_kit/config/gemini305_preview_joints.yaml'),
        ):
            file_check(name, root / relative)
    if mode == 'vision':
        ign = shutil.which('ign')
        add('Gazebo ign', 'ok' if ign else 'error', ign or 'PATH 中未找到 ign；没有启动 Gazebo 进行验证')
        for name, relative in (
            ('视觉启动文件', 'jaka_competition_kit/launch/gemini305_preview.launch.py'),
            ('基础组视觉模型', 'perception/track1_sim/runs/track1/weights/best.pt'),
            ('YOLO 虚拟环境 Python', 'perception/track2_yolo/.venv/bin/python'),
        ):
            file_check(name, root / relative, executable=name == 'YOLO 虚拟环境 Python')
    return results


@dataclass(frozen=True)
class _Proc:
    pid: int
    ppid: int
    pgrp: int
    session: int
    start: int
    state: str


def _read_proc(pid: int) -> _Proc | None:
    try:
        # /proc/PID/stat's comm may contain spaces and parentheses.
        raw = Path(f'/proc/{pid}/stat').read_text(encoding='utf-8', errors='replace')
        tail = raw[raw.rfind(')') + 2:].split()
        return _Proc(pid, int(tail[1]), int(tail[2]), int(tail[3]), int(tail[19]), tail[0])
    except (OSError, ValueError, IndexError):
        return None


def _process_snapshot() -> dict[int, _Proc]:
    try:
        with os.scandir('/proc') as entries:
            pids = [int(entry.name) for entry in entries if entry.name.isdigit()]
    except OSError:
        return {}
    return {pid: proc for pid in pids if (proc := _read_proc(pid)) is not None}


def _simulation_argv(argv: Sequence[str]) -> bool:
    """Match executable/argument tokens, never shell -c or python -c text."""
    if not argv:
        return False
    first = Path(argv[0]).name
    # ROS Python console scripts have their interpreter in argv[0].
    if re.fullmatch(r'python(?:\d+(?:\.\d+)*)?', first):
        if len(argv) < 2 or argv[1].startswith('-'):
            return False
        tokens = list(argv[1:])
    else:
        tokens = list(argv)
    executable = Path(tokens[0]).name
    args = tokens[1:]
    if executable == 'ros2':
        if len(args) >= 2 and args[0] == 'launch':
            launch_files = {
                'round.launch.py', 'gemini305_preview.launch.py', 'workbench.launch.py'
            }

            def launch_file(token: str) -> bool:
                # A launch assignment such as seed:=/tmp/workbench.launch.py
                # must never be interpreted as a file, even if basename matches.
                return (not token.startswith('-') and ':=' not in token
                        and Path(token).name in launch_files)

            # ros2 launch /absolute/or/relative/workbench.launch.py seed:=...
            # (a bare filename is also a valid file-form invocation).
            if launch_file(args[1]):
                return True
            # ros2 launch PACKAGE FILE seed:=... . Only a package-like first
            # positional may shift the filename to args[2]; an unrelated path
            # followed by arguments must not be mistaken for package form.
            return (len(args) >= 3
                    and re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*', args[1]) is not None
                    and launch_file(args[2]))
        if len(args) >= 3 and args[0] == 'run':
            return args[1] == 'jaka_competition_kit' and args[2] in {
                'scorer', 'competition_scorer', 'scene_generator', 'gz_scene'
            }
    if executable in {'ros2_control_node', 'move_group', 'competition_scorer',
                      'scene_generator', 'gz_scene'}:
        return True
    if executable == 'scorer':
        return 'jaka_competition_kit' in Path(tokens[0]).parts
    if executable == 'ign' or executable.startswith('ignition-gazebo'):
        return executable != 'ign' or 'gazebo' in args
    # Gazebo's Ruby launcher may exec the versioned ign-gazebo script.
    if executable.startswith('ign-gazebo') or executable.startswith('gz-sim'):
        return True
    if executable.startswith('ruby') and args:
        return Path(args[0]).name.startswith(('ign', 'gz')) and 'gazebo' in args[1:]
    return False


def simulation_conflicts() -> list[dict]:
    """Inspect Linux processes without signalling or spawning anything.

    Excludes this process and all its ancestors. Returned commands are diagnostic
    text, not shell commands to be executed. No substring match in shell payloads.
    """
    snapshot = _process_snapshot()
    ancestors: set[int] = set()
    pid = os.getpid()
    while pid > 0 and pid not in ancestors:
        ancestors.add(pid)
        proc = snapshot.get(pid)
        if proc is None:
            break
        pid = proc.ppid
    conflicts = []
    for pid, proc in sorted(snapshot.items()):
        if pid in ancestors or proc.state == 'Z':
            continue
        try:
            raw = Path(f'/proc/{pid}/cmdline').read_bytes()
        except OSError:
            continue
        argv = [token.decode('utf-8', 'replace') for token in raw.split(b'\0') if token]
        if _simulation_argv(argv):
            conflicts.append({'pid': pid, 'command': shlex.join(argv)})
    return conflicts


def ros_command(root: Path, argv: Sequence[str]) -> list[str]:
    """Build a shell environment wrapper without interpolating any user data."""
    if not argv or isinstance(argv, (str, bytes)):
        raise ValueError('argv 必须是非空的参数序列，不能是 shell 字符串')
    # All paths/arguments are positional parameters; bash sources quote them.
    script = 'source "$1" && source "$2" && shift 2 && exec "$@"'
    return ['/bin/bash', '--noprofile', '--norc', '-c', script, 'workbench-ros',
            str(_ROS_SETUP), str(Path(root) / 'jaka_ros2/install/setup.bash'),
            *(os.fspath(arg) for arg in argv)]


@dataclass
class _Job:
    name: str
    process: subprocess.Popen
    log_path: Path
    log_file: TextIO
    owned: dict[int, int] = field(default_factory=dict)
    reader: threading.Thread | None = None
    reader_done: threading.Event = field(default_factory=threading.Event)
    stopping: bool = False
    stage: int = 0
    deadline: float = 0.0
    exited: bool = False
    returncode: int | None = None
    signalled: set[int] = field(default_factory=set)
    token: str = ''
    start: int = 0


class ProcessManager:
    """Manage isolated job sessions, UTF-8 logs and queued UI notifications.

    Linux /proc tracks descendants, including children that create new sessions.
    Every signal revalidates PID plus kernel start time to avoid PID-reuse kills.
    A lightweight observer follows ancestry between UI polls. A unique inherited
    environment marker recovers ordinary detached/reparented children when their
    parent exits too fast to observe. This is not a security boundary: a child
    that removes that marker AND escapes between observations may be untracked.
    No global process killing, shared-memory deletion, or ROS imports are used.
    """

    INT_TIMEOUT = 1.5
    TERM_TIMEOUT = 1.0
    KILL_RETRY = 0.2
    TRACK_INTERVAL = 0.04

    def __init__(self, log_dir: Path):
        if not sys.platform.startswith('linux'):
            raise RuntimeError('ProcessManager 当前仅支持 Linux')
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._jobs: dict[str, _Job] = {}
        self._events: queue.Queue[dict] = queue.Queue()
        self._lock = threading.RLock()
        self._closed = False
        self._observer_stop = threading.Event()
        self._observer = threading.Thread(target=self._observe, name='workbench-process-observer', daemon=True)
        self._observer.start()

    def _observe(self) -> None:
        while not self._observer_stop.wait(self.TRACK_INTERVAL):
            with self._lock:
                if not any(not job.exited for job in self._jobs.values()):
                    continue
            snapshot = _process_snapshot()
            with self._lock:
                for job in self._jobs.values():
                    if not job.exited:
                        self._track(job, snapshot)

    def _track(self, job: _Job, snapshot: dict[int, _Proc]) -> None:
        known = {pid for pid, start in job.owned.items()
                 if pid in snapshot and snapshot[pid].start == start}
        # Recover fast double-forks/new sessions after ancestry is lost. Only
        # children of this job inherit its unique marker; never match generic
        # ROS executable names or unverified process-group IDs for ownership.
        leader = snapshot.get(job.process.pid)
        if job.token and (job.stopping or leader is None or leader.state == 'Z'):
            marker = ('_COMPETITION_WORKBENCH_JOB=' + job.token).encode()
            for pid, proc in snapshot.items():
                if pid in known or proc.start < job.start or proc.state in ('Z', 'X'):
                    continue
                try:
                    with open(f'/proc/{pid}/environ', 'rb') as stream:
                        environ = stream.read(1024 * 1024)
                except OSError:
                    continue
                if marker in environ.split(b'\0'):
                    job.owned[pid] = proc.start
                    known.add(pid)
        # Include all generations, irrespective of PID order or session changes.
        remaining = dict(snapshot)
        while True:
            children = {pid: proc for pid, proc in remaining.items()
                        if pid not in known and proc.ppid in known}
            if not children:
                break
            for pid, proc in children.items():
                job.owned[pid] = proc.start
                known.add(pid)
                remaining.pop(pid, None)

    def _live(self, job: _Job) -> list[_Proc]:
        result = []
        for pid, start in job.owned.items():
            proc = _read_proc(pid)
            if proc is not None and proc.start == start and proc.state not in ('Z', 'X'):
                result.append(proc)
        return result

    def _read_output(self, job: _Job) -> None:
        try:
            assert job.process.stdout is not None
            for line in job.process.stdout:
                job.log_file.write(line)
                job.log_file.flush()
                self._events.put({'kind': 'output', 'name': job.name, 'text': line.rstrip('\r\n')})
        except (OSError, ValueError) as exc:
            self._events.put({'kind': 'error', 'name': job.name, 'text': f'读取/记录日志失败：{exc}'})
        finally:
            if job.process.stdout is not None:
                job.process.stdout.close()
            job.log_file.close()
            # Every output put happens before this event is set. _finish waits
            # for it, then puts exited into the SAME FIFO, so WB_JSON service
            # results (including a final unterminated line) precede exited.
            job.reader_done.set()

    def launch(self, name: str, argv: Sequence[str], cwd: Path,
               env: Mapping[str, str] | None = None) -> int:
        if not name or not isinstance(name, str):
            raise ValueError('任务名必须是非空字符串')
        if not argv or isinstance(argv, (str, bytes)):
            raise ValueError('argv 必须是非空参数序列')
        with self._lock:
            if self._closed:
                raise RuntimeError('ProcessManager 已关闭')
            previous = self._jobs.get(name)
            if previous is not None and (not previous.exited):
                # A finished job must be polled before reuse, so its output and
                # one-shot exit event cannot be lost behind a same-name launch.
                if previous.process.poll() is None or self._live(previous):
                    raise RuntimeError(f'任务已在运行：{name}')
                self._finish(previous)
                if not previous.exited:
                    raise RuntimeError(f'任务正在收尾，请稍后重试：{name}')
            safe_name = re.sub(r'[^\w.-]+', '_', name, flags=re.UNICODE)[:80].strip('.') or 'job'
            stamp = f'{time.time_ns()}-{os.getpid()}'
            log_path = self.log_dir / f'{safe_name}-{stamp}.log'
            log_file = log_path.open('x', encoding='utf-8', buffering=1)
            child_env = dict(os.environ)
            if env is not None:
                child_env.update(env)
            token = uuid.uuid4().hex
            child_env['_COMPETITION_WORKBENCH_JOB'] = token
            try:
                start_tick = int(float(Path('/proc/uptime').read_text().split()[0]) * os.sysconf('SC_CLK_TCK')) - 1
            except (OSError, ValueError, IndexError):
                start_tick = 0
            try:
                process = subprocess.Popen(
                    list(argv), cwd=os.fspath(cwd), env=child_env,
                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                    errors='replace', bufsize=1, start_new_session=True,
                )
            except BaseException:
                log_file.close()
                raise
            job = _Job(name, process, log_path, log_file, token=token, start=start_tick)
            proc = _read_proc(process.pid)
            if proc is not None:
                job.owned[proc.pid] = proc.start
                job.start = proc.start
            self._jobs[name] = job
            job.reader = threading.Thread(target=self._read_output, args=(job,),
                                          name=f'workbench-log-{safe_name}', daemon=True)
            job.reader.start()
            self._track(job, _process_snapshot())
            return process.pid

    def alive(self, name: str) -> bool:
        with self._lock:
            job = self._jobs.get(name)
            return bool(job and not job.exited and
                        (job.process.poll() is None or self._live(job)))

    def active_names(self) -> list[str]:
        with self._lock:
            return [name for name in self._jobs if self.alive(name)]

    def owned_pids(self, name: str) -> set[int]:
        """Return live, PID/start-time-verified descendants of one owned job."""
        with self._lock:
            job = self._jobs.get(name)
            if job is None:
                return set()
            return {proc.pid for proc in self._live(job)
                    if proc.state not in ('Z', 'X')}

    def _signal(self, job: _Job, sig: int, only: set[int] | None = None) -> None:
        # Signal only start-time-verified PIDs. Unlike an unqualified killpg,
        # this cannot hit a recycled group after the original leader exits.
        # New-session children are handled identically to normal descendants.
        live = self._live(job)
        for proc in sorted(live, key=lambda proc: proc.pid == job.process.pid):
            if only is not None and proc.pid not in only:
                continue
            current = _read_proc(proc.pid)
            if current is None or current.start != proc.start or current.state in ('Z', 'X'):
                continue
            try:
                os.kill(proc.pid, sig)
                job.signalled.add(proc.pid)
            except ProcessLookupError:
                pass
            except PermissionError as exc:
                self._events.put({'kind': 'error', 'name': job.name,
                                  'text': f'无法停止本任务子进程 {proc.pid}：{exc}'})

    def stop(self, name: str) -> None:
        """Request SIGINT immediately, returning without waiting for exit."""
        with self._lock:
            job = self._jobs.get(name)
            if job is None or job.exited or job.stopping:
                return
            self._track(job, _process_snapshot())
            job.stopping = True
            job.stage = 1
            job.deadline = time.monotonic() + self.INT_TIMEOUT
            self._signal(job, signal.SIGINT)

    def stop_all(self) -> None:
        with self._lock:
            for name in list(self._jobs):
                self.stop(name)

    def _finish(self, job: _Job) -> None:
        if job.exited:
            return
        code = job.process.poll()
        if code is None or self._live(job) or not job.reader_done.is_set():
            return
        job.returncode = code
        job.exited = True
        self._events.put({'kind': 'exited', 'name': job.name,
                          'text': f'任务已退出，返回码 {code}；日志：{job.log_path}',
                          'returncode': code})

    def poll(self) -> list[dict]:
        """Drain FIFO notifications and advance nonblocking stop escalation.

        A job's output events are returned before its exited event, either in
        this batch or an earlier poll. The GUI must process each returned list
        in order; process exit alone is not a substitute for its result output.
        """
        with self._lock:
            if any(not job.exited for job in self._jobs.values()):
                snapshot = _process_snapshot()
                now = time.monotonic()
                for job in self._jobs.values():
                    if job.exited:
                        continue
                    self._track(job, snapshot)
                    code = job.process.poll()
                    # A job's surviving descendants are not free background
                    # services: stop them even if their original parent exits.
                    if code is not None and self._live(job) and not job.stopping:
                        self.stop(job.name)
                    if job.stopping and now >= job.deadline:
                        if job.stage == 1:
                            job.stage = 2
                            job.deadline = now + self.TERM_TIMEOUT
                            self._signal(job, signal.SIGTERM)
                        else:
                            job.stage = 3
                            job.deadline = now + self.KILL_RETRY
                            self._signal(job, signal.SIGKILL)
                    elif job.stopping:
                        # Newly observed descendants get the current stage,
                        # including ones forked while SIGINT was being handled.
                        sig = (signal.SIGINT if job.stage == 1 else
                               signal.SIGTERM if job.stage == 2 else signal.SIGKILL)
                        newly_live = [proc for proc in self._live(job)
                                      if proc.pid not in job.signalled]
                        if newly_live:
                            self._signal(job, sig, {proc.pid for proc in newly_live})
                    self._finish(job)
            events = []
            while True:
                try:
                    events.append(self._events.get_nowait())
                except queue.Empty:
                    return events

    def close(self, timeout: float = 6.0) -> None:
        """Bounded shutdown; SIGKILL is forced before the supplied deadline.

        Normal UI shutdown should pass the default 6 seconds. poll() remains
        available afterwards to retrieve final output/exit notifications.
        """
        timeout = max(0.0, float(timeout))
        deadline = time.monotonic() + timeout
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self.stop_all()
        while time.monotonic() < deadline:
            # Preserve queued UI events: close must not consume poll's results.
            with self._lock:
                events = self.poll()
                for event in events:
                    self._events.put(event)
                pending = [job for job in self._jobs.values() if not job.exited]
                if not pending:
                    break
                if time.monotonic() >= deadline - min(0.5, timeout / 2):
                    for job in pending:
                        self._track(job, _process_snapshot())
                        self._signal(job, signal.SIGKILL)
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        with self._lock:
            for job in self._jobs.values():
                if not job.exited:
                    self._track(job, _process_snapshot())
                    self._signal(job, signal.SIGKILL)
                    # Nonblocking reap; reader thread closes descriptors at EOF.
                    job.process.poll()
                    self._finish(job)
        self._observer_stop.set()
        remaining = max(0.0, deadline - time.monotonic())
        self._observer.join(timeout=remaining)
        for job in list(self._jobs.values()):
            remaining = max(0.0, deadline - time.monotonic())
            if job.reader is not None:
                job.reader.join(timeout=remaining)
            with self._lock:
                self._finish(job)
