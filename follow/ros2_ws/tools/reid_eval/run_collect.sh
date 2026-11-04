#!/bin/bash
# 引数: world名(拡張子なし) 秒数(sim時間ではなく実時間)
W=$1; SEC=${2:-150}
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
export DISPLAY=:1 XAUTHORITY=/home/ubuntu/.Xauthority HOME=/home/ubuntu
cd ~/ros2_ws
setsid ros2 launch wheelchair_gazebo crowd_demo.launch.py world:=${W}.world label:=collect out_dir:=/tmp/collect_m > /tmp/collect_launch_${W}.log 2>&1 &
LP=$!
sleep 20
OUT=/home/ubuntu/ros2_ws/metrics/collect/${W}
rm -rf $OUT
cd /tmp
timeout $SEC python3 /home/ubuntu/ros2_ws/tools/reid_eval/collect.py $OUT --ros-args -r __node:=collect -p use_sim_time:=true -r cmd_vel:=/collect/cmd_vel -r /follower/status:=/collect/status -r /follower/debug:=/collect/debug > /tmp/collect_node_${W}.log 2>&1
kill -INT -$(ps -o pgid= -p $LP | tr -d ' ') 2>/dev/null; sleep 6
pkill -9 -f "[i]gn gazebo"; pkill -9 -f "[r]uby.*ign"
pkill -9 -f "[y]olo_reid_follower|[m]etrics_logger|[p]erson_mover|[p]arameter_bridge|[r]obot_state_publisher"
sleep 2
echo "$W: $(wc -l < $OUT/log.jsonl) crops"
