# 動画の全フレームで体の 33 点を求め、時系列を CSV に書く（コンテナで実行。踊りの対応を決めるための分析）
#   python3 pose_series.py <動画.mp4> <出力.csv>
import csv, math, sys
import cv2, mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
MODEL = '/tmp/pose_full.task'
src, out = sys.argv[1], sys.argv[2]
cap = cv2.VideoCapture(src); fps = cap.get(cv2.CAP_PROP_FPS)
lm = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
    base_options=mp_python.BaseOptions(model_asset_path=MODEL), running_mode=vision.RunningMode.VIDEO, num_poses=1))
rows, i = [], 0
while True:
    ok, img = cap.read()
    if not ok: break
    h, w = img.shape[:2]
    res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), int(i * 1000 / fps))
    t = i / fps; i += 1
    if not res.pose_landmarks:
        rows.append({'t': t, 'ok': 0}); continue
    p = res.pose_landmarks[0]; q = res.pose_world_landmarks[0]
    P = lambda k: (p[k].x * w, p[k].y * h)
    # 11/12 肩（左/右）、23/24 腰、15/16 手首、27/28 足首。左右は本人の左右
    sh_w = math.dist(P(11), P(12)); hip_c = ((P(23)[0] + P(24)[0]) / 2, (P(23)[1] + P(24)[1]) / 2)
    ank_y = (P(27)[1] + P(28)[1]) / 2; nose = P(0)
    body_h = abs(ank_y - nose[1]) or 1
    # 体の向き：世界座標（m、腰が原点）で、右肩→左肩の向きの水平角。正面で 0
    yaw = math.degrees(math.atan2(q[11].z - q[12].z, q[11].x - q[12].x))
    lean = math.degrees(math.atan2((P(11)[0] + P(12)[0]) / 2 - hip_c[0], hip_c[1] - (P(11)[1] + P(12)[1]) / 2))
    rows.append({'t': t, 'ok': 1, 'hip_x': hip_c[0] / w, 'hip_y': hip_c[1] / h, 'sh_w': sh_w / body_h, 'body_h': body_h / h,
                 'yaw': yaw, 'lean': lean, 'lwrist_up': (P(11)[1] - P(15)[1]) / body_h, 'rwrist_up': (P(12)[1] - P(16)[1]) / body_h,
                 'vis_l': p[11].visibility, 'vis_r': p[12].visibility})
keys = ['t', 'ok', 'hip_x', 'hip_y', 'sh_w', 'body_h', 'yaw', 'lean', 'lwrist_up', 'rwrist_up', 'vis_l', 'vis_r']
with open(out, 'w', newline='') as f:
    wr = csv.DictWriter(f, keys); wr.writeheader(); wr.writerows(rows)
print(out, len(rows), 'frames', f'{fps:.2f} fps', 'detected', sum(r['ok'] for r in rows))
