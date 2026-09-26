#!/usr/bin/env bash
# 停掉上一轮仿真留下的进程, 并清掉 FastRTPS 的共享内存段。
#
# 为什么需要它: round.launch.py 起的节点名是固定的(controller_manager /
# robot_state_publisher / move_group / ign gazebo)。`ros2 launch` **发完令
# 跑完一轮也不会自己退**, 上一套还活着时再起一套, 第二套的 spawner 会连到
# 第一套的 controller_manager 上, 报 "Failed to configure controller",
# 接着 rviz2 段错误(-11)、move_group 段错误, 35 秒内整套崩掉。
# 现象和"命令写错了"很像, 所以这里做成一条命令。详见 docs/Gazebo仿真.md 第 5 节。
#
# 用法:
#   bash setup/sim-clean.sh          # 只清本项目的仿真栈(默认)
#   bash setup/sim-clean.sh --all    # 再狠一点: 所有 ros2 / ign / rviz 进程
#
# 注意: pkill 的匹配串写成 "[r]os2" 这种带方括号的形式, 是为了让 pkill -f
# 匹配不到本脚本自己的命令行(否则会把自己也杀掉)。
set -uo pipefail

ALL=0
[[ "${1:-}" == "--all" ]] && ALL=1

PATS=(
  "[r]os2 launch jaka_competition_kit"
  "[j]aka_competition_kit/lib/jaka_competition_kit/"
  "[r]os2_control_node"
  "[r]obot_state_publisher"
  "[c]ontroller_manager/spawner"
  "[m]oveit_ros_move_group/move_group"
  "[r]viz2"
  "[r]os_gz_sim"
  "[i]gn gazebo"
  "[r]uby /usr/bin/ign"
  "[p]arameter_bridge"
  "[g]z_scene"
)
if [[ "${ALL}" == "1" ]]; then
  PATS+=("[r]os2 " "[r]os2-" "[r]os2cli" "[i]gn " "[r]uby")
fi

killed=0
for p in "${PATS[@]}"; do
  if pkill -9 -f "$p" 2>/dev/null; then
    killed=1
  fi
done
[[ "${killed}" == "1" ]] && sleep 4

# FastRTPS 共享内存段: 留着会让新起的节点互相看不见 / 卡在发现阶段
before=$(ls /dev/shm 2>/dev/null | wc -l)
find /dev/shm -maxdepth 1 \( -name 'fastrtps*' -o -name 'sem.fastrtps*' \) -delete 2>/dev/null
rm -f /dev/shm/*.fastrtps 2>/dev/null
after=$(ls /dev/shm 2>/dev/null | wc -l)

left=$(ps -eo pid,cmd | grep -E "$(IFS='|'; echo "${PATS[*]}")" | grep -v 'grep' | wc -l)
if [[ "${left}" == "0" ]]; then
  echo "[OK] 仿真栈已清空 (共享内存段 ${before} -> ${after})"
  exit 0
fi

echo "[!] 还剩 ${left} 个进程没清掉:"
ps -eo pid,cmd | grep -E "$(IFS='|'; echo "${PATS[*]}")" | grep -v 'grep' | sed 's/\(.\{100\}\).*/\1/'
exit 1
