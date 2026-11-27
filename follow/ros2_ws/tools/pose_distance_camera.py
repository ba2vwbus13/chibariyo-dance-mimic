#!/usr/bin/env python3
"""本物のカメラで、骨格から人の方向と距離を求める（Issue #1 段階 2。ロボットへは何も送らない）。

追従ノード（pose_follower_node.py）と同じ計算（pose_follow_core.py の measure・TargetTracker・calibrate）を、
ROS を使わずに PC のカメラの映像で動かし、画面に骨格・距離・方向・追っている人を出す。
人が巻尺で測った距離に立ったときの値を記録して、推定の誤差を確かめるのに使う。

  python3 follow/ros2_ws/tools/pose_distance_camera.py                 # カメラ 0 番
  python3 follow/ros2_ws/tools/pose_distance_camera.py --camera 1 --hfov-deg 78
  python3 follow/ros2_ws/tools/pose_distance_camera.py --url http://<スマホのIP>:4747/video   # スマホの DroidCam
  python3 follow/ros2_ws/tools/pose_distance_camera.py --video 動画.mp4 --no-window --record-at 3   # 動作の確認

キー（映像のウィンドウを選んでから押す）:
  1〜9 … 人がカメラから 1〜9 m に立っているとして、30 枚の推定距離を記録する（平均・ばらつき・誤差を表示）
  c    … 追っている人が --calib-distance [m] に立っているとして、体の実寸（胴の長さ・肩幅）を測り直す
  r    … ロックオンを外す（いちばん近い人に付け直す）
  q    … 終わる

記録は --out の CSV（既定 local/pose_distance_<日時>.csv。local/ はコミットされない）に、数値だけを残す。
画像は保存しない（顔が写るため）。

距離は「焦点距離 × 体の実寸 ÷ 画像上の長さ」で求める。焦点距離は水平画角 --hfov-deg から出すので、
画角が分からないカメラでも、c で実寸を測り直せば、その距離での誤差は画角に依らない
（実寸 = 画像上の長さ × 既知の距離 ÷ 焦点距離 なので、距離 = 既知の距離 × 測り直したときの長さ ÷ 今の長さ になる）。

必要なもの: mediapipe（Tasks API）、opencv-python、体のモデル pose_landmarker_full.task
（--model で指定。無ければ local/、~/mp/、/opt/mediapipe/ を探す。配布元は README の「使わせていただいたもの」の MediaPipe）。
"""
import argparse
import csv
import math
import os
import statistics
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'wheelchair_follower'))
from wheelchair_follower.pose_follow_core import TargetTracker, calibrate, measure  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
MODEL_CANDIDATES = ['~/mp/pose_landmarker_full.task', '/opt/mediapipe/pose_landmarker_full.task']
# 骨格の線（MediaPipe Pose の 33 点のうち、体の主なところ）
BONES = [(11, 12), (11, 23), (12, 24), (23, 24), (11, 13), (13, 15), (12, 14), (14, 16),
         (23, 25), (25, 27), (24, 26), (26, 28), (0, 11), (0, 12)]
SAMPLES = 30   # 1 回の記録で集める枚数


def parse_args():
    ap = argparse.ArgumentParser(description='本物のカメラで、骨格から人の方向と距離を求める（ロボットへは送らない）')
    ap.add_argument('--camera', type=int, default=0, help='カメラの番号')
    ap.add_argument('--video', help='カメラの代わりに読む動画（動作の確認用）')
    ap.add_argument('--url', help='ネットワークの映像の URL（DroidCam: http://<スマホのIP>:4747/video）。--camera より優先')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--hfov-deg', type=float, default=70.0, help='カメラの水平画角 [度]（分からなければ c で測り直す）')
    ap.add_argument('--model', default='', help='pose_landmarker_full.task の場所')
    ap.add_argument('--num-poses', type=int, default=2)
    ap.add_argument('--min-visibility', type=float, default=0.5)
    ap.add_argument('--torso-m', type=float, default=0.62, help='胴の長さの実寸 [m]（既定は Gazebo の人のモデル）')
    ap.add_argument('--shoulder-m', type=float, default=0.42, help='肩幅の実寸 [m]（既定は Gazebo の人のモデル）')
    ap.add_argument('--calib-distance', type=float, default=2.0, help='c で測り直すときに人が立つ距離 [m]')
    ap.add_argument('--out', default='', help='記録の CSV（既定 local/pose_distance_<日時>.csv）')
    ap.add_argument('--no-window', action='store_true', help='画面を出さず、1 秒ごとに値を表示する')
    ap.add_argument('--frames', type=int, default=0, help='この枚数で終わる（0 = 終わらない。動作の確認用）')
    ap.add_argument('--record-at', type=float, default=0.0, help='始めからこの距離 [m] として記録する（キーを押せないとき）')
    return ap.parse_args()


def find_model(path):
    local = os.path.join(ROOT, 'local', 'pose_landmarker_full.task')   # リポジトリの local/（コミットされない）
    for p in ([path] if path else []) + [local] + MODEL_CANDIDATES:
        p = os.path.expanduser(p)
        if p and os.path.exists(p):
            return p
    sys.exit('体のモデル pose_landmarker_full.task が見つからない。--model で場所を指定する')


def open_source(args):
    url = getattr(args, 'url', None)
    if url:
        # ネットワークの映像（DroidCam なら http://<スマホのIP>:4747/video）。古い画像がたまって遅れないように、たまりは 1 枚
        cap = cv2.VideoCapture(url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            sys.exit('映像を開けない: --url（スマホの DroidCam が起動しているか、PC と同じ Wi-Fi か、URL の IP と番号を確かめる）')
        return cap
    if args.video:
        cap = cv2.VideoCapture(args.video)
    elif sys.platform == 'win32':
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)   # Windows は DirectShow の方が早く開く
    else:
        cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit(f'映像を開けない: {args.video or "カメラ " + str(args.camera)}')
    if not args.video:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    return cap


def draw(img, poses, ms_by_pose, target, info):
    h, w = img.shape[:2]
    for lm, m in zip(poses, ms_by_pose):
        is_target = target is not None and m is target
        color = (0, 200, 255) if is_target else (200, 200, 200)
        for a, b in BONES:
            if lm[a].visibility >= 0.5 and lm[b].visibility >= 0.5:
                cv2.line(img, (int(lm[a].x * w), int(lm[a].y * h)), (int(lm[b].x * w), int(lm[b].y * h)), color, 2)
        if m is not None:
            x = int(m.cx)
            y = int(min(p.y for p in lm) * h)
            label = f'{m.distance:.2f} m  {math.degrees(m.bearing):+.0f} deg'
            cv2.putText(img, label, (max(0, x - 70), max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    for i, line in enumerate(info):
        cv2.putText(img, line, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)


def main():
    args = parse_args()
    from mediapipe.tasks import python as mp_python   # import に時間がかかるので、引数を確かめてから
    from mediapipe.tasks.python import vision
    import mediapipe as mp

    opts = vision.PoseLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=find_model(args.model)),
        running_mode=vision.RunningMode.VIDEO, num_poses=args.num_poses)
    detector = vision.PoseLandmarker.create_from_options(opts)
    cap = open_source(args)
    hfov = math.radians(args.hfov_deg)
    tracker = TargetTracker()
    torso_m, shoulder_m = args.torso_m, args.shoulder_m

    out = args.out or os.path.join(ROOT, 'local', time.strftime('pose_distance_%Y%m%d_%H%M%S.csv'))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    log = open(out, 'w', newline='', encoding='utf-8')
    writer = csv.writer(log)
    writer.writerow(['time', 'true_distance_m', 'n', 'est_mean_m', 'est_std_m', 'error_m', 'error_pct',
                     'torso_px', 'shoulder_px', 'bearing_deg', 'torso_m', 'shoulder_m', 'hfov_deg'])
    print(f'記録: {out}')

    t_ms = 0
    last_seen = None
    recording = (args.record_at, []) if args.record_at > 0 else None   # (真の距離, [BodyMeasure…])
    n_frames, t_infer, t_print = 0, 0.0, 0.0
    try:
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            h, w = bgr.shape[:2]
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            t_ms = max(t_ms + 1, int(time.monotonic() * 1000))   # VIDEO モードは時刻が増え続ける必要がある
            t0 = time.monotonic()
            r = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), t_ms)
            t_infer += time.monotonic() - t0
            n_frames += 1

            ms_by_pose = [measure(lm, w, h, hfov, torso_m, shoulder_m, args.min_visibility) for lm in r.pose_landmarks]
            ms = [m for m in ms_by_pose if m is not None]
            t_lost = 0.0 if last_seen is None else time.monotonic() - last_seen
            target = tracker.select(ms, t_lost)
            if target is not None:
                last_seen = time.monotonic()

            if recording is not None and target is not None:
                recording[1].append(target)
                if len(recording[1]) >= SAMPLES:
                    true_d, got = recording
                    ds = [m.distance for m in got]
                    mean = statistics.mean(ds)
                    sd = statistics.pstdev(ds)
                    tp = [m.torso_px for m in got if m.torso_px]
                    sp = [m.shoulder_px for m in got if m.shoulder_px]
                    row = [time.strftime('%H:%M:%S'), true_d, len(ds), round(mean, 3), round(sd, 3),
                           round(mean - true_d, 3), round(100 * (mean - true_d) / true_d, 1),
                           round(statistics.mean(tp), 1) if tp else '', round(statistics.mean(sp), 1) if sp else '',
                           round(math.degrees(statistics.mean(m.bearing for m in got)), 1),
                           round(torso_m, 3), round(shoulder_m, 3), args.hfov_deg]
                    writer.writerow(row)
                    log.flush()
                    print(f'記録 {true_d:.0f} m: 推定 {mean:.2f} m（ばらつき {sd:.2f} m）、誤差 {mean - true_d:+.2f} m（{row[6]:+.1f} %）')
                    recording = None

            state = '追従中' if target is not None else (f'見失い {t_lost:.1f} s' if last_seen else '人を待っている')
            info = [f'people {len(r.pose_landmarks)}  measured {len(ms)}  infer {1000 * t_infer / n_frames:.0f} ms',
                    f'torso {torso_m:.2f} m  shoulder {shoulder_m:.2f} m  hfov {args.hfov_deg:.0f} deg']
            if recording is not None:
                info.append(f'REC {recording[0]:.0f} m  {len(recording[1])}/{SAMPLES}')
            if args.no_window:
                if time.monotonic() - t_print >= 1.0:
                    t_print = time.monotonic()
                    d = f'{target.distance:.2f} m {math.degrees(target.bearing):+.0f}°' if target else '-'
                    print(f'{n_frames:5d} 枚 {state:10s} 追う人 {d:14s} ' + info[0])
            else:
                draw(bgr, r.pose_landmarks, ms_by_pose, target, info)
                cv2.imshow('pose_distance_camera (q: quit)', bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if ord('1') <= key <= ord('9'):
                    recording = (float(key - ord('0')), [])
                    print(f'{key - ord("0")} m の記録を始める（{SAMPLES} 枚）')
                elif key == ord('c'):
                    if target is None:
                        print('追っている人がいないので、測り直せない')
                    else:
                        t_m, s_m = calibrate(target.torso_px, target.shoulder_px, args.calib_distance, w, hfov)
                        torso_m, shoulder_m = t_m or torso_m, s_m or shoulder_m
                        tracker.last = (args.calib_distance, target.bearing)
                        print(f'実寸を測り直した（{args.calib_distance:.1f} m）: 胴 {torso_m:.3f} m / 肩幅 {shoulder_m:.3f} m')
                elif key == ord('r'):
                    tracker.last = None
                    last_seen = None
                    print('ロックオンを外した')
            if args.frames and n_frames >= args.frames:
                break
    finally:
        cap.release()
        log.close()
        cv2.destroyAllWindows()
        if n_frames:
            print(f'{n_frames} 枚、推論 平均 {1000 * t_infer / n_frames:.0f} ms/枚')


if __name__ == '__main__':
    main()
