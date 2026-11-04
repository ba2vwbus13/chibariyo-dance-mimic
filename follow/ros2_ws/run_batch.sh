#!/bin/bash
# 複数条件を順に計測する。引数: "TAG|METHOD|EXTRA" を1件ずつ
for spec in "$@"; do
  IFS='|' read -r tag method extra <<< "$spec"
  echo "##### $tag ($method) $extra"
  TAG="$tag" /home/ubuntu/ros2_ws/run_measure.sh "$method" 180 2400 "$extra" | grep -E "REACHED|TIMEOUT|DIED|ID維持|別人|見失い率"
done
echo "##### BATCH_DONE"
