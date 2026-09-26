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
#   bash setup/xorg-viewer.sh fill             # 把 Gazebo 铺满 :0 并置顶
#   bash setup/xorg-viewer.sh cam [模型名]      # 把 Gazebo 相机对到某个模型(default: table)
#   bash setup/xorg-viewer.sh view             # 只重开查看窗口(拖不动的时候用这个)
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

# `:0` 上没有窗口管理器, 于是有两件事会让人以为"Gazebo 黑屏":
#   1) 后启动的 RViz 和 Gazebo 都在 (0,0), 没有 WM 就没人管叠放, RViz 盖住它;
#   2) 根窗口是黑的, 窗口又只占屏幕一块, 剩下的大片区域就是纯黑。
# 把 Gazebo 拉到 (0,0) 铺满并置顶, 两个问题一起解决(VNC 里看到的就是 Gazebo)。
arrange_gazebo() {
    local wid best="" best_area=0 w h area
    for wid in $(DISPLAY="${SRC_DISPLAY}" xdotool search --name 'Gazebo' 2>/dev/null); do
        w=$(DISPLAY="${SRC_DISPLAY}" xwininfo -id "${wid}" 2>/dev/null \
            | awk '/^  Width/{print $2}')
        h=$(DISPLAY="${SRC_DISPLAY}" xwininfo -id "${wid}" 2>/dev/null \
            | awk '/^  Height/{print $2}')
        [ -n "${w}" ] && [ -n "${h}" ] || continue
        area=$((w * h))
        if [ "${area}" -gt "${best_area}" ]; then best_area="${area}"; best="${wid}"; fi
    done
    if [ -z "${best}" ]; then
        echo "[=] ${SRC_DISPLAY} 上还没有 Gazebo 窗口 —— 等 Gazebo 起来后再执行:"
        echo "    bash setup/xorg-viewer.sh fill"
        return
    fi
    DISPLAY="${SRC_DISPLAY}" xdotool windowmove "${best}" 0 0
    DISPLAY="${SRC_DISPLAY}" xdotool windowsize "${best}" 100% 100%
    DISPLAY="${SRC_DISPLAY}" xdotool windowraise "${best}"
    echo "[OK] 已把 Gazebo(${best})铺满 ${SRC_DISPLAY} 并置顶"
}

start_gui() {
    echo "[*] 打开查看窗口 (${SRC_DISPLAY} -> ${DISPLAY:-<未设置>}, zoom ${ZOOM}%)"
    echo "    Gazebo 里: 左键拖=转, 中键拖=平移, 滚轮=缩放"
    echo "    拖不动就换一个: bash $0 view   (滚轮是围着光标缩的, 缩飞了用 cam 复位)"
    LD_LIBRARY_PATH="${LIBDIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        "${PREFIX}/usr/bin/gvncviewer" -z "${ZOOM}" "localhost:0"
}

# 把 Gazebo 的相机对到某个模型上。
#
# 为什么需要: gvncviewer 的缩放是"围着鼠标光标"缩的, 光标没落在赛场上时连缩几下
# 镜头就飘到地面里, 看起来像"画面卡住"; 另外查看窗口偶尔会卡在弹菜单的状态, 那时
# 鼠标事件根本传不到 :0(实测: 指针在 :0 上纹丝不动), 重启查看窗口即可。
set_camera() {
    local target="${1:-table}" cli="" mtype=""
    if command -v ign >/dev/null 2>&1; then cli=ign; mtype=ignition.msgs.StringMsg
    elif command -v gz >/dev/null 2>&1; then cli=gz; mtype=gz.msgs.StringMsg
    else
        echo "[!] 找不到 ign/gz 命令行 —— 先 source /opt/ros/humble/setup.bash" >&2
        return 1
    fi
    if "${cli}" service -s /gui/move_to --reqtype "${mtype}" \
            --reptype "${mtype%StringMsg}Boolean" --timeout 3000 \
            --req "data: \"${target}\"" >/dev/null 2>&1; then
        echo "[OK] Gazebo 相机已对到 ${target}"
    else
        echo "[!] 对到 ${target} 失败 —— Gazebo 起了吗? (ros2 launch ... world:=gazebo)" >&2
        return 1
    fi
}

case "${1:-start}" in
    start)
        ensure_bins
        start_vnc
        arrange_gazebo
        if [ "${2:-}" = "--no-gui" ]; then
            exit 0
        fi
        start_gui
        ;;
    fill)
        ensure_bins
        arrange_gazebo
        ;;
    view)
        ensure_bins
        pkill -f "bin/gvncviewer" 2>/dev/null || true
        sleep 1
        start_gui
        ;;
    cam)
        set_camera "${2:-table}"
        ;;
    stop)
        pkill -f "x11vnc -display ${SRC_DISPLAY}" && echo "[OK] x11vnc 已停" || echo "[=] 没有在跑的 x11vnc"
        pkill -f "gvncviewer" && echo "[OK] 查看窗口已关" || true
        ;;
    *)
        echo "用法: $0 {start [--no-gui]|fill|view|cam [模型名]|stop}" >&2
        exit 2
        ;;
esac
