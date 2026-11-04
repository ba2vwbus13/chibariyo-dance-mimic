#!/bin/bash
# 1方式を指定秒(sim時間)だけ走らせて summary を確定させる
METHOD="${1:-reid}"
TARGET="${2:-180}"
MAXWALL="${3:-2400}"
EXTRA="${4:-}"   # 例: "hist_min_sim:=0.15 accept_threshold:=0.35"
OUT=/home/ubuntu/ros2_ws/metrics
export HOME=/home/ubuntu
export DISPLAY=:1
export XAUTHORITY=/home/ubuntu/.Xauthority
source /opt/ros/humble/setup.bash
[ -f /opt/ros_gz_ws/install/setup.bash ] && source /opt/ros_gz_ws/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
cd /home/ubuntu/ros2_ws
mkdir -p /home/ubuntu/runlogs

TAG="${TAG:-$METHOD}"
MARK=/home/ubuntu/runlogs/mark_${TAG}; : > "$MARK"; sleep 1
LOG=/home/ubuntu/runlogs/run_${TAG}.log; rm -f "$LOG"
setsid ros2 launch wheelchair_gazebo crowd_demo.launch.py \
  method:=${METHOD} out_dir:=${OUT} label:=${TAG} ${EXTRA} > "$LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p $LPID | tr -d ' ')
echo "launched pid=$LPID pgid=$PGID method=$METHOD target=${TARGET}s(sim)"

newest() { find "$OUT" -maxdepth 1 -name "${1}_${TAG}_*.${2}" -newer "$MARK" -print 2>/dev/null | sort | tail -1; }

CSV=""; start=$(date +%s); t=0
while :; do
  now=$(date +%s); wall=$((now-start))
  [ -z "$CSV" ] && CSV=$(newest metrics csv)
  if [ -n "$CSV" ] && [ -s "$CSV" ]; then
    t=$(tail -1 "$CSV" | cut -d, -f1); t=${t%%.*}; [ -z "$t" ] && t=0
  fi
  if [ "${t:-0}" -ge "$TARGET" ]; then echo "REACHED sim_t=${t}s wall=${wall}s"; break; fi
  if [ "$wall" -ge "$MAXWALL" ]; then echo "TIMEOUT wall=${wall}s sim_t=${t}"; break; fi
  if ! kill -0 $LPID 2>/dev/null; then echo "LAUNCH_DIED wall=${wall}s sim_t=${t}"; break; fi
  sleep 5
done

kill -INT -$PGID 2>/dev/null
for i in $(seq 1 40); do kill -0 $LPID 2>/dev/null || break; sleep 1; done
kill -TERM -$PGID 2>/dev/null; sleep 3
kill -KILL -$PGID 2>/dev/null; sleep 2
pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f "ruby.*ign" 2>/dev/null; sleep 2
echo "=== wall=$(( $(date +%s) - start ))s ==="
S=$(newest summary txt)
echo "SUMMARY_FILE=$S"; echo "CSV_FILE=$CSV"; [ -n "$S" ] && cat "$S"
