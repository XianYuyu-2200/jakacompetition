#!/usr/bin/env bash
# 起一个"无头" Xorg 显示, 让 OpenGL 真正走独显。
#
# 为什么需要它(2026-09 实测):
#   这台机器有 RTX 5060 Ti(驱动 580.178.04), 但平时用的 :1001 是 NoMachine
#   自己实现的虚拟 X server —— nxnode.bin 直接持有 /tmp/.X11-unix/X1001,
#   不是 Xorg。NVIDIA 的 GLX 客户端库没法往这个 X server 上呈现画面:
#   强行 `__GLX_VENDOR_LIBRARY_NAME=nvidia` 的结果是**所有 GL 窗口全黑**
#   (glxgears 对照组实测: 默认能画出齿轮, 加了这个变量就是纯黑窗口)。
#   所以那块卡在远程桌面会话里是用不上的, 只能走 Mesa llvmpipe。
#
#   重新起一个由 NVIDIA 驱动的 Xorg 就行 —— GPU 上没接显示器也能起,
#   靠 AllowEmptyInitialConfiguration + UseDisplayDevice none 走 NoScanout:
#     OpenGL renderer string: NVIDIA GeForce RTX 5060 Ti/PCIe/SSE2
#     direct rendering: Yes            (OpenGL 4.6 / 16311 MB 显存)
#
# 用法:
#   bash setup/xorg-gpu.sh start [:0]     # 起, 默认 :0
#   bash setup/xorg-gpu.sh status [:0]    # 看当前是谁在渲染
#   bash setup/xorg-gpu.sh stop  [:0]     # 停
#
#   DISPLAY=:0 ros2 launch jaka_competition_kit round.launch.py world:=gazebo
#
# 注意: 这个 Xorg 是 -ac(不校验 xauth)的无头显示, 只适合本机单人使用。
#
# 要 sudo 密码。想免交互: SUDO="sudo -S" bash setup/xorg-gpu.sh start :0
set -euo pipefail

# 想免交互(比如 CI / 脚本里喂密码)就 SUDO="sudo -S" bash setup/xorg-gpu.sh ...
SUDO="${SUDO:-sudo}"
DISPLAY_NUM="${2:-:0}"
N="${DISPLAY_NUM#:}"
CONF="/tmp/jaka-xorg-gpu.conf"
LOG="/tmp/jaka-xorg-gpu.log"

bus_id() {
  command -v nvidia-smi >/dev/null || { echo "没装 nvidia-smi"; exit 1; }
  # 00000000:01:00.0 -> PCI:1:0:0(用 bash 的 $((16#..)) 转, 别用 gawk 的 strtonum,
  # 本机 awk 是 mawk 没有那个函数)
  local raw bdf b d f
  raw="$(nvidia-smi --query-gpu=pci.bus_id --format=csv,noheader | head -1)"
  bdf="${raw#*:}"
  IFS=':.' read -r b d f <<< "$bdf"
  printf 'PCI:%d:%d:%d' "$((16#$b))" "$((16#$d))" "$((16#$f))"
}

case "${1:-}" in
  start)
    if [ -e "/tmp/.X11-unix/X$N" ]; then
      echo ": $DISPLAY_NUM 已经在了, 先 stop 或者换一个号"; exit 1
    fi
    BID="$(bus_id)"
    cat > "$CONF" <<EOF
Section "ServerLayout"
    Identifier "Layout0"
    Screen 0 "Screen0" 0 0
EndSection
Section "Module"
    Load "glx"
EndSection
Section "Device"
    Identifier "Dev0"
    Driver "nvidia"
    BusID "$BID"
    Option "AllowEmptyInitialConfiguration" "True"
    Option "ConnectedMonitor" "DFP-0"
    Option "UseDisplayDevice" "none"
EndSection
Section "Screen"
    Identifier "Screen0"
    Device "Dev0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1920x1080"
        Virtual 1920 1080
    EndSubSection
EndSection
EOF
    echo "> 起 Xorg $DISPLAY_NUM(设备 $BID), 需要 sudo"
    # 先把密码问掉: bash 非交互时后台任务的 stdin 会被换成 /dev/null,
    # 那样 sudo 就再也拿不到密码了。
    $SUDO -v
    $SUDO Xorg "$DISPLAY_NUM" -config "$CONF" -ac -noreset -logfile "$LOG" \
        >/dev/null 2>&1 </dev/null &
    for _ in $(seq 1 20); do
      [ -e "/tmp/.X11-unix/X$N" ] && break
      sleep 1
    done
    sleep 1
    if [ ! -e "/tmp/.X11-unix/X$N" ]; then
      echo "!! 起不来, 看 $LOG"; exit 1
    fi
    echo "> 好了, 当前渲染器:"
    DISPLAY="$DISPLAY_NUM" glxinfo -B 2>/dev/null | grep -E "renderer|direct rendering" || true
    ;;

  status)
    if [ ! -e "/tmp/.X11-unix/X$N" ]; then echo ": $DISPLAY_NUM 没人"; exit 0; fi
    DISPLAY="$DISPLAY_NUM" glxinfo -B 2>/dev/null | grep -E "renderer|direct rendering" || true
    ;;

  stop)
    # 只认命令行以 "Xorg :0" 开头的进程, 免得把 ps/grep/这个脚本自己也算进去
    PIDS="$(ps -eo pid=,args= \
      | grep -E "^ *[0-9]+ ([^ ]*/)?Xorg ${DISPLAY_NUM}( |$)" \
      | awk '{print $1}' || true)"
    if [ -z "$PIDS" ]; then echo ": $DISPLAY_NUM 没在跑"; exit 0; fi
    # shellcheck disable=SC2086
    $SUDO kill $PIDS
    echo ": $DISPLAY_NUM 已停"
    ;;

  *)
    sed -n '2,30p' "$0"; exit 1
    ;;
esac
