#!/usr/bin/env bash
# 限时标定工具 —— 反复跑官方参考实现, 输出每轮用时, 供"中位数 × 1.4"定限时。
#
#   bash design/calibrate.sh 1 3        # 赛道一跑 3 轮
#   bash design/calibrate.sh 2 2        # 赛道二跑 2 轮
#
# 前置: 已在另一个终端起好 MoveIt 仿真 (demo.launch.py use_rviz_sim:=true)
#       和判分侧  (judge.launch.py track:=<赛道> seed:=<种子>)
set +u   # ROS 的 setup.bash 会引用未定义变量, 不能开 -u

TRACK="${1:-1}"
ROUNDS="${2:-3}"
SEED="${3:-246135}"

source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../jaka_ros2/install/setup.bash"

OUT="$HOME/.ros/jaka_competition"
declare -a TIMES=()

for i in $(seq 1 "$ROUNDS"); do
  echo "===================  第 $i/$ROUNDS 轮 (赛道 $TRACK)  ==================="
  ros2 run jaka_competition_kit "ref_track${TRACK}" \
      --ros-args -p vel_scale:=1.0 > "/tmp/ref_track${TRACK}_$i.log" 2>&1 &
  REF_PID=$!
  sleep 3
  ros2 service call /competition/generate std_srvs/srv/Trigger "{}" >/dev/null
  ros2 service call /competition/start    std_srvs/srv/Trigger "{}" >/dev/null
  wait "$REF_PID"

  SHEET="$(ls -t "$OUT"/score_*.json | head -1)"
  read -r ELAPSED OK < <(python3 -c "
import json,sys
s=json.load(open('$SHEET'))
print(s['elapsed_s'], s['collected_count'])
")
  TIMES+=("$ELAPSED")
  echo ">>> 第 $i 轮: ${ELAPSED}s, 入盒 ${OK} 件   ($SHEET)"
done

echo
echo "===================  汇总 (赛道 $TRACK)  ==================="
printf '%s\n' "${TIMES[@]}" | sort -n | awk '
  {a[NR]=$1; s+=$1}
  END {
    n=NR
    med = (n%2) ? a[(n+1)/2] : (a[n/2]+a[n/2+1])/2
    printf "样本: %s\n", n
    printf "最快: %.1fs   中位数: %.1fs   最慢: %.1fs   均值: %.1fs\n", a[1], med, a[n], s/n
    printf "建议限时 (中位数 × 1.4): %.0fs\n", med*1.4
    printf "建议效率满分线 (中位数 × 1.05): %.0fs\n", med*1.05
  }'
