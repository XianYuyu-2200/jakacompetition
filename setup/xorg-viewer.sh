#!/usr/bin/env bash
# 把 :0(独显 Xorg)上的画面投到当前桌面, 可交互(能自己拖拽转 Gazebo 视角)。
#
# 为什么需要它: Gazebo 的 3D 区在 :1001(NoMachine 的虚拟 X server)上只画一帧
# 就不重绘, 而 :0 在 NoMachine 里根本看不到。VNC 是唯一既能看、又能操作的通道。
# 服务端只听 127.0.0.1(-localhost), 不对外开端口。
#
# 用法:
#   bash setup/xorg-viewer.sh start            # 起 VNC 服务 + 打开查看窗口
#   bash setup/xorg-viewer.sh start --no-gui   # 只起 VNC 服务(给别的机器连)
#   bash setup/xorg-viewer.sh stop
#
# 本机没装 x11vnc / gvncviewer 时, 脚本会自己 `apt-get download` 解到
# ~/.local/opt/vnc(不需要 sudo, 只在 ~/.local 里放二进制)。
# 想装进系统: sudo apt install x11vnc tigervnc-viewer
#
# 环境变量: SRC_DISPLAY(默认 :0) ZOOM(默认 88, 1080p 用 88 刚好放得下)
set -euo pipefail

SRC_DISPLAY="${SRC_DISPLAY:-:0}"
ZOOM="${ZOOM:-88}"
PREFIX="${HOME}/.local/opt/vnc"
LIBDIR="${PREFIX}/usr/lib/x86_64-linux-gnu"
PKGS=(x11vnc gvncviewer libgtk-vnc-2.0-0 libgvnc-1.0-0)
LOGFILE="${TMPDIR:-/tmp}/x11vnc.log"

ensure_bins() {
    if [ -x "${PREFIX}/usr/bin/x11vnc" ] && [ -x "${PREFIX}/usr/bin/gvncviewer" ]; then
        return
    fi
    echo "[*] 本机没有 x11vnc/gvncviewer, 解包到 ${PREFIX} (不需要 sudo)"
    local tmp
    tmp="$(mktemp -d)"
    (cd "${tmp}" && apt-get download "${PKGS[@]}")
    mkdir -p "${PREFIX}"
    for deb in "${tmp}"/*.deb; do dpkg -x "${deb}" "${PREFIX}"; done
    echo "[*] 完成: $(ls "${PREFIX}/usr/bin" | tr '\n' ' ')"
}

start_vnc() {
    if pgrep -f "x11vnc -display ${SRC_DISPLAY}" >/dev/null; then
        echo "[=] x11vnc 已经在跑"
        return
    fi
    # -localhost: 只监听回环, 别把桌面暴露到网络上
    # -nopw: 回环连接不需要密码(要对外连请自己加 -rfbauth)
    # env -u WAYLAND_DISPLAY: 本机这个变量被设成了 "no", x11vnc 会当成 Wayland
    #   会话直接退出("Wayland display server detected")
    env -u WAYLAND_DISPLAY -u XDG_SESSION_TYPE \
        LD_LIBRARY_PATH="${LIBDIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        "${PREFIX}/usr/bin/x11vnc" -display "${SRC_DISPLAY}" \
        -localhost -forever -shared -nopw -bg -o "${LOGFILE}"
    echo "[OK] x11vnc 已起, 日志 ${LOGFILE}"
}

start_gui() {
    echo "[*] 打开查看窗口 (${SRC_DISPLAY} -> ${DISPLAY:-<未设置>}, zoom ${ZOOM}%)"
    echo "    Gazebo 里: 左键拖=转, 中键拖=平移, 滚轮=缩放; F8 退出全屏"
    LD_LIBRARY_PATH="${LIBDIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        "${PREFIX}/usr/bin/gvncviewer" -z "${ZOOM}" "localhost:0"
}

case "${1:-start}" in
    start)
        ensure_bins
        start_vnc
        if [ "${2:-}" = "--no-gui" ]; then
            exit 0
        fi
        start_gui
        ;;
    stop)
        pkill -f "x11vnc -display ${SRC_DISPLAY}" && echo "[OK] x11vnc 已停" || echo "[=] 没有在跑的 x11vnc"
        pkill -f "gvncviewer" && echo "[OK] 查看窗口已关" || true
        ;;
    *)
        echo "用法: $0 {start [--no-gui]|stop}" >&2
        exit 2
        ;;
esac
