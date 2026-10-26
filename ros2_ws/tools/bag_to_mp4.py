# ros2 bag に記録した検出画面の画像（/finger_image/compressed または /pose_image/compressed）を、
# 時刻どおりの MP4 にする（コンテナの中で実行する）
#   python3 ~/ros2_ws/tools/bag_to_mp4.py <bag のフォルダ> [出力.mp4]
# ・画像の時刻は camera.html が付けた時刻（Mac の時計）を使う。抜けた時間は直前の画像を出し続ける（30 fps）
# ・その時刻の速度の指令（/cmd_vel）を画像の下に書き込む
# ・最初の画像の時刻を <出力>.json に書く（iPhone の動画と時刻を合わせるのに使う）

import json
import os
import sys
from datetime import datetime, timedelta, timezone

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

FPS = 30
JST = timezone(timedelta(hours=9))


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    frames, cmds = [], []
    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        if topic in ('/finger_image/compressed', '/pose_image/compressed'):
            msg = deserialize_message(data, get_message(types[topic]))
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frames.append((t, bytes(msg.data)))
        elif topic == '/cmd_vel':
            msg = deserialize_message(data, get_message(types[topic]))
            tw = msg.twist if hasattr(msg, 'twist') else msg
            cmds.append((t_rec * 1e-9, tw.linear.x, tw.angular.z))
    frames.sort()
    return frames, cmds


def main():
    bag = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(bag.rstrip('/')), os.path.basename(bag.rstrip('/')) + '_finger.mp4')
    frames, cmds = read(bag)
    if not frames:
        sys.exit('/finger_image/compressed も /pose_image/compressed も記録されていません')
    t0, t1 = frames[0][0], frames[-1][0]
    first = cv2.imdecode(np.frombuffer(frames[0][1], np.uint8), cv2.IMREAD_COLOR)
    h, w = first.shape[:2]
    bar = 40
    writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (w, h + bar))
    i, j, img = 0, 0, first
    lin, ang = 0.0, 0.0
    n = int((t1 - t0) * FPS) + 1
    for k in range(n):
        t = t0 + k / FPS
        while i + 1 < len(frames) and frames[i + 1][0] <= t:
            i += 1
            img = cv2.imdecode(np.frombuffer(frames[i][1], np.uint8), cv2.IMREAD_COLOR)
        while j < len(cmds) and cmds[j][0] <= t:
            _, lin, ang = cmds[j]
            j += 1
        canvas = np.zeros((h + bar, w, 3), np.uint8)
        canvas[:h] = img
        clock = datetime.fromtimestamp(t, JST).strftime('%H:%M:%S.%f')[:-4]
        cv2.putText(canvas, f'{clock}   cmd: v={lin:+.3f} m/s  w={ang:+.2f} rad/s', (8, h + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        writer.write(canvas)
    writer.release()
    with open(os.path.splitext(out)[0] + '.json', 'w') as f:
        json.dump({'start_epoch': t0, 'start_jst': datetime.fromtimestamp(t0, JST).isoformat(),
                   'duration': t1 - t0, 'frames': len(frames)}, f, ensure_ascii=False, indent=1)
    print(f'{out}：{t1 - t0:.1f} 秒、画像 {len(frames)} 枚、開始 {datetime.fromtimestamp(t0, JST):%H:%M:%S}（JST）')


if __name__ == '__main__':
    main()
