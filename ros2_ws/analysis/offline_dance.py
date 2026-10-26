# 動画を、実行時と同じ処理（640x480 に収める → 体の検出 → features → DanceMapper → Follower）にかけ、
# ロボットの動きを試算する（コンテナで実行）。python3 offline_dance.py <動画> [<出力.csv>]
import csv, sys
import cv2, mediapipe as mp, numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
sys.path.insert(0, '/home/ubuntu/ros2_ws/src/finger_dance')
from finger_dance.pose_mapping import features, DanceMapper, Follower
from finger_dance.video_source import VideoSource
src = sys.argv[1]; out = sys.argv[2] if len(sys.argv) > 2 else None
cap = cv2.VideoCapture(src); fps = cap.get(cv2.CAP_PROP_FPS); step = max(1, round(fps / 10))   # 1 秒に約 10 枚
lm = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
    base_options=mp_python.BaseOptions(model_asset_path='/opt/finger_dance/pose_landmarker_full.task'), running_mode=vision.RunningMode.VIDEO, num_poses=1))
mapper, fol = DanceMapper(), Follower()
rows, i, prev_t = [], 0, None
while True:
    ok, img = cap.read()
    if not ok: break
    i += 1
    if (i - 1) % step: continue
    t = (i - 1) / fps; img = VideoSource.letterbox(img); h, w = img.shape[:2]
    res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), int(t * 1000))
    f = None
    if res.pose_landmarks:
        f = features([(p.x, p.y, p.visibility) for p in res.pose_landmarks[0]], [(p.x, p.y, p.z) for p in res.pose_world_landmarks[0]], w, h)
    dt = step / fps
    if f:
        ht, xt = mapper.update(f, dt)
    for _ in range(2):   # 指令は 0.05 秒ごと（画像 1 枚あたり 2 回）
        v, wz = fol.step(ht, xt, dt / 2) if f else (0.0, 0.0)
    rows.append((t, int(f is not None), ht if f else '', xt if f else '', fol.heading, fol.forward, v, wz, int(bool(f and f['l_up'])), int(bool(f and f['r_up']))))
a = np.array([[r[4], r[5], r[6], r[7]] for r in rows], float)
print(f'{src.split("/")[-1]}: 画像 {len(rows)} 枚、検出 {sum(r[1] for r in rows)}、向き {a[:,0].min():.0f}〜{a[:,0].max():.0f}°、前後 {a[:,1].min()*100:.1f}〜{a[:,1].max()*100:.1f} cm、'
      f'ω が上限 {np.mean(np.abs(a[:,3])>=0.999)*100:.0f}%、手を上げた {sum(r[8] or r[9] for r in rows)} 枚')
if out:
    with open(out, 'w', newline='') as fo:
        wr = csv.writer(fo); wr.writerow(['t', 'ok', 'heading_target', 'forward_target', 'heading', 'forward', 'v', 'w', 'l_up', 'r_up']); wr.writerows(rows)

lm.close()
