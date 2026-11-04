#!/bin/bash
# follow_test.world で追従方式を 1 本走らせ、metrics_logger の summary を出す
#   run_follow.sh <method> <タグ> [sim秒=180] [demo.launch.py の追加引数…]
METHOD=$1; TAG=$2; TARGET=${3:-180}; shift 3
OUT=/home/ubuntu/ros2_ws/metrics_pose
export HOME=/home/ubuntu DISPLAY=:1 XAUTHORITY=/home/ubuntu/.Xauthority
source /opt/ros/humble/setup.bash
source /opt/ros_gz_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/runlogs $OUT
cd ~/ros2_ws   # yolov8n.pt はここにある
MARK=~/runlogs/mark_$TAG; : > $MARK; sleep 1
setsid bash -c "
ros2 launch wheelchair_gazebo demo.launch.py method:=$METHOD $* &
sleep 6
ros2 run ros_gz_bridge parameter_bridge /model/wheelchair/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry --ros-args -r /model/wheelchair/odometry:=/wheelchair/ground_truth -p use_sim_time:=true &
sleep 4
ros2 run wheelchair_follower metrics_logger --ros-args -p use_sim_time:=true -p label:=$TAG -p out_dir:=$OUT &
wait" > ~/runlogs/follow_$TAG.log 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p $LPID | tr -d ' ')
newest() { find $OUT -maxdepth 1 -name "${1}_${TAG}_*.${2}" -newer $MARK | sort | tail -1; }
CSV=""; start=$(date +%s); t=0
while :; do
  wall=$(( $(date +%s) - start ))
  [ -z "$CSV" ] && CSV=$(newest metrics csv)
  if [ -n "$CSV" ] && [ -s "$CSV" ]; then t=$(tail -1 $CSV | cut -d, -f1); t=${t%%.*}; [ -z "$t" ] && t=0; fi
  [ "${t:-0}" -ge "$TARGET" ] && { echo "REACHED sim_t=${t}s wall=${wall}s"; break; }
  [ $wall -ge 2400 ] && { echo "TIMEOUT sim_t=$t"; break; }
  sleep 5
done
kill -INT -$PGID; for i in $(seq 1 30); do kill -0 $LPID 2>/dev/null || break; sleep 1; done
kill -KILL -$PGID 2>/dev/null; sleep 2
pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f "ruby.*ign" 2>/dev/null; pkill -9 -f parameter_bridge; sleep 2
S=$(newest summary txt); echo "SUMMARY=$S CSV=$CSV"; [ -n "$S" ] && cat $S
grep -E '推論|実寸|Error|error' ~/runlogs/follow_$TAG.log | tail -5
