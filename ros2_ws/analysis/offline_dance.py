# 動画を、実行時と同じ処理（640x480 に収める → 33関節の検出 → features → DanceMapper → Follower）にかけ、
# エイサーの判定動作名・関節座標・特徴量・車いすの速度指令を CSV に出力する（コンテナで実行）。
# 使い方:
#   python3 ~/ros2_ws/analysis/offline_dance.py ~/ros2_ws/videos/mirukumunari.mp4 ~/ros2_ws/analysis/mirukumunari_dance.csv
#   コンテナの外（Windows・Mac）でも、モデルをリポジトリの local/ に置けば動く（または環境変数 POSE_MODEL で場所を指定）
import csv
import os
import sys
import time

# MediaPipe / TensorFlow Lite の起動時 C++ 警告ログ (W0000 / INFO) を非表示にする
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['GLOG_minloglevel'] = '2'

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))   # リポジトリのいちばん上
sys.path.insert(0, '/home/ubuntu/ros2_ws/src/finger_dance')
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'finger_dance'))   # コンテナの外（Windows・Mac）で動かすとき
import finger_dance.pose_mapping as pm
from finger_dance.pose_mapping import DanceMapper, Follower, features


def letterbox(img, width=640, height=480):
    h, w = img.shape[:2]
    s = min(width / w, height / h)
    nw, nh = int(w * s), int(h * s)
    out = np.zeros((height, width, 3), dtype=np.uint8)
    out[(height - nh) // 2:(height - nh) // 2 + nh, (width - nw) // 2:(width - nw) // 2 + nw] = cv2.resize(img, (nw, nh))
    return out

MODEL_PATHS = [
    os.environ.get('POSE_MODEL', ''),                                  # 場所を指定するとき
    '/opt/finger_dance/pose_landmarker_full.task',
    os.path.join(ROOT, 'local', 'pose_landmarker_full.task'),          # コンテナの外では、リポジトリの local/ に置く
    '/home/ubuntu/ros2_ws/models/pose_landmarker_lite.task',
    '/opt/finger_dance/pose_landmarker_lite.task',
]
MODEL = next((p for p in MODEL_PATHS if p and os.path.exists(p)), MODEL_PATHS[1])

src = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + '_dance.csv'

cap = cv2.VideoCapture(src)
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
step = max(1, round(fps / 10))  # 1 秒に約 10 枚 (10Hz)
total_steps = max(1, total_video_frames // step)
print(f'[*] 動画を解析中です: {os.path.basename(src)} (全 {total_video_frames} フレーム / 約 {total_video_frames/fps:.1f} 秒)...', flush=True)

lm = vision.PoseLandmarker.create_from_options(
    vision.PoseLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=MODEL),
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.25,
        min_pose_presence_confidence=0.25,
        min_tracking_confidence=0.25,
    )
)

mapper, fol = DanceMapper(), Follower()
pm._DETECT_MAPPER.reset()
rows = []
i = 0
sim_clock = 1000.0

# whill_bridge.py と同一のパラメータ・状態変数
FRONT_CAP = 55
SIDE_CAP = 70
LINEAR_MAX = 0.35
ANGULAR_MAX = 3.0
BACK_GAIN = 1.3
BACK_CAP = 70.0
SIDE_SIGN = -1
RAMP = 500.0
whill_front = 0.0
whill_side = 0.0


def clip(x, lo, hi):
    return max(lo, min(hi, x))


while True:
    ok, img = cap.read()
    if not ok:
        break
    i += 1
    if (i - 1) % step:
        continue
    t = (i - 1) / fps
    dt = step / fps
    sim_clock += dt

    img = letterbox(img)
    h, w = img.shape[:2]
    infer_img = cv2.resize(img, (320, int(h * 320 / w)), interpolation=cv2.INTER_NEAREST) if w > 320 else img

    res = lm.detect_for_video(
        mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(infer_img, cv2.COLOR_BGR2RGB)),
        int(t * 1000) + 1,
    )

    f = None
    pts = None
    if res.pose_landmarks:
        pts = [(p.x, p.y, p.visibility) for p in res.pose_landmarks[0]]
        world = [(p.x, p.y, p.z) for p in res.pose_world_landmarks[0]]
        pm._DETECT_LAST_T = time.monotonic() - dt
        f = features(pts, world, w, h)

    if f:
        ht, xt = mapper.update(f, dt)
        action = mapper.action_name
    else:
        # 人が一時的に検出できないフレームも停止せず、次の認識へ滑らかに繋げる
        mapper.groove_phase = getattr(mapper, 'groove_phase', 0.0) + dt * 3.2
        ht = 8.0 * np.sin(mapper.groove_phase)
        xt = 0.04 * np.cos(mapper.groove_phase * 0.5)
        action = 'つなぎ動作（次動作へ補間中）'

    for _ in range(2):  # 指令は 0.05 秒ごと（画像 1 枚あたり 2 回）
        v, wz = fol.step(ht, xt, dt / 2)

    # whill_bridge.py (10Hz, period=dt) と全く同じジョイスティック倒し量 [%] の計算
    tf = clip(v / LINEAR_MAX, -1.0, 1.0) * FRONT_CAP
    if tf < 0:
        tf = max(-BACK_CAP, tf * BACK_GAIN)
    ts = SIDE_SIGN * clip(wz / ANGULAR_MAX, -1.0, 1.0) * SIDE_CAP
    r_step = 250.0 * dt
    f_step = r_step * max(1.0, BACK_GAIN) if tf < 0 else r_step
    if tf * whill_front < 0 or abs(tf) < 1.0:
        whill_front = tf
    else:
        whill_front += clip(tf - whill_front, -f_step, f_step) if abs(tf) > abs(whill_front) else (tf - whill_front)
    if ts * whill_side < 0 or abs(ts) < 1.0:
        whill_side = ts
    else:
        whill_side += clip(ts - whill_side, -r_step, r_step) if abs(ts) > abs(whill_side) else (ts - whill_side)
    whill_f_int = int(round(whill_front))
    whill_s_int = int(round(whill_side))

    rows.append({
        't': round(t, 2),
        'ok': int(f is not None),
        'action_name': action,
        'whill_front_pct': whill_f_int,
        'whill_side_pct': whill_s_int,
        'whill_front_raw': round(whill_front, 2),
        'whill_side_raw': round(whill_side, 2),
        'v': round(v, 4),
        'w': round(wz, 4),
        'heading_target': round(ht, 2) if f else 0.0,
        'forward_target': round(xt, 4) if f else 0.0,
        'heading': round(fol.heading, 2),
        'forward': round(fol.forward, 4),
        'yaw': round(f['yaw'], 2) if f else '',
        'dproj_a': round(f.get('dproj_a', 0.0), 4) if f else '',
        'squat': int(bool(f and f.get('squat'))),
        'turn': int(bool(f and f.get('turn'))),
        'l_knee_lift': int(bool(f and f.get('l_knee_lift'))),
        'r_knee_lift': int(bool(f and f.get('r_knee_lift'))),
        'r_step_fwd': int(bool(f and f.get('r_step_fwd'))),
        'l_step_back': int(bool(f and f.get('l_step_back'))),
        'l_up': int(bool(f and f.get('l_up'))),
        'r_up': int(bool(f and f.get('r_up'))),
        'hip_x': round(f['hip_x'] / w, 4) if f else '',
        'hip_y': round(f['hip_y'] / h, 4) if f else '',
        'r_knee_x': round(pts[26][0], 4) if pts else '',
        'r_knee_y': round(pts[26][1], 4) if pts else '',
        'l_knee_x': round(pts[25][0], 4) if pts else '',
        'l_knee_y': round(pts[25][1], 4) if pts else '',
        'r_ankle_x': round(pts[28][0], 4) if pts else '',
        'r_ankle_y': round(pts[28][1], 4) if pts else '',
        'l_ankle_x': round(pts[27][0], 4) if pts else '',
        'l_ankle_y': round(pts[27][1], 4) if pts else '',
    })

    if len(rows) % 100 == 0 or len(rows) == total_steps:
        pct = min(100.0, len(rows) * 100.0 / total_steps)
        print(f'  進捗: {len(rows)}/{total_steps} フレーム ({pct:.0f}%) - 現在時刻 {t:.1f}s [{action}]   ', end='\r', flush=True)

fieldnames = [
    't', 'ok', 'action_name',
    'whill_front_pct', 'whill_side_pct', 'whill_front_raw', 'whill_side_raw',
    'v', 'w',
    'heading_target', 'forward_target', 'heading', 'forward',
    'yaw', 'dproj_a',
    'squat', 'turn', 'l_knee_lift', 'r_knee_lift', 'r_step_fwd', 'l_step_back', 'l_up', 'r_up',
    'hip_x', 'hip_y',
    'r_knee_x', 'r_knee_y', 'l_knee_x', 'l_knee_y',
    'r_ankle_x', 'r_ankle_y', 'l_ankle_x', 'l_ankle_y',
]

# Excel で日本語（動作名）が文字化けしないよう UTF-8 (BOM付き) で保存
with open(out, 'w', newline='', encoding='utf-8-sig') as fo:
    wr = csv.DictWriter(fo, fieldnames=fieldnames)
    wr.writeheader()
    wr.writerows(rows)

det_count = sum(r['ok'] for r in rows)
print(
    f'\n{os.path.basename(src)} -> {out}\n'
    f'  全 {len(rows)} フレーム (10Hz) / 検出 {det_count} フレーム\n'
    f'  しこ立ち: {sum(r["squat"] for r in rows)}回, '
    f'ターン: {sum(r["turn"] for r in rows)}回, '
    f'足上げ(左/右): {sum(r["l_knee_lift"] for r in rows)}/{sum(r["r_knee_lift"] for r in rows)}回, '
    f'ステップ(右/左): {sum(r["r_step_fwd"] for r in rows)}/{sum(r["l_step_back"] for r in rows)}回, '
    f'手上げ: {sum(r["l_up"] or r["r_up"] for r in rows)}回'
)
lm.close()
