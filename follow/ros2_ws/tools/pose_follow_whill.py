#!/usr/bin/env python3
"""本物のカメラの骨格追従の速度の指令を、WHILL Model CR の橋渡し whill/whill_bridge.py へ渡す（試験用）。

※ 実機を動かすプログラム。whill/手順.md の安全の決まりを守る。先生がいるときだけ動かす。
   最初は whill_bridge.py --dry-run、次にタイヤを浮かせて旋回だけ（前後 0%）、の順に試す。

追従の計算は追従ノード（pose_follower_node.py）と同じ pose_follow_core.py（measure・TargetTracker・follow_cmd）を使う。
ROS は使わず、whill_bridge.py が待っている rosbridge と同じ形の WebSocket（ws://127.0.0.1:9091、/cmd_vel）を立てて送る。

  ターミナル 1: python3 follow/ros2_ws/tools/pose_follow_whill.py --camera 0
        （スマホの DroidCam なら --url http://<スマホのIP>:4747/video。hfov は使うカメラに合わせて --hfov-deg）
  ターミナル 2: cd whill && python3 whill_bridge.py --dry-run              （確かめたら --port <装置名>）

止める順: ターミナル 2（whill_bridge.py）を Ctrl + C → ターミナル 1 で q。
whill_bridge.py は WebSocket が切れてつながり直すと、人の確認なしに再開するので、先に止める。

whill_bridge.py の安全の仕組み（倒し量の上限 旋回 10%・前後 0% が既定、1 秒に 20% まで、指令が 0.3 秒届かないと 0、
Enter で停止）に、こちらでも次を重ねる:
  - c で体の大きさを測り直すまでは 0 しか送らない（既定の大きさでは近くで遠く見積もることがある）
  - 追う人がいないとき、距離や方向が NaN・Inf のときは 0。見失ったときの旋回の探索は --search を付けたときだけで、--search-max 秒まで
  - r でロックオンを外したら、また長く見失った後に付き直したとき（別の人かもしれない）も、c で測り直すまで 0
  - スペース・r を押したら、次の画像を待たずにすぐ 0 を送る
  - スペースで一時停止（0 を送る）。終わるときは 0 を数回送ってから閉じる
  - 近すぎの判定（body_cut.py）: 追う人の頭（鼻・目・耳）か膝が画面の上下の端で切れている・確からしさが低いときは前進 0（旋回は残す）。
    近づくと頭や脚が切れて胴が短く写り、距離を遠く見積もって前進してしまうため（1.2 m に立つと 1.7 m と出た）。
    切れていない状態が --release-s 秒（既定 0.5）続いてから前進を許す。見つけ直した直後も同じ
  - 映像が遅れた（前の画像から --max-frame-gap 秒、既定 0.5 より間があいた）ときも前進 0。映像が止まって読み込みが返らないときは、
    指令の更新が止まるので 0.3 秒で 0 を送る（下）
  - 近すぎの判定は左右も見る: 肩・腰の点が画面の左右の端にかかったら切れている（横にはみ出すと肩幅が短く写るため）
  - NaN は 0。前後は --max-linear（既定 0.10 m/s）、旋回は --max-angular（既定 0.3 rad/s、最大 1.0）で切る（探索の旋回も同じ）。
    指令の更新が 0.3 秒止まったら 0 を送る。--max-angular は whill_bridge.py の --angular-max と同じ値にする
    --max-linear は whill_bridge.py の --linear-max と同じ値にする（小さければ遅くなるだけで安全側。大きいと橋渡しが頭打ちにする）
  - 距離のならし（--distance-median N）: 速さの計算には「直近 N 枚の中央値」と「今の 1 枚」の小さい方を使う。
    1 枚の遠い跳びは抑え、近づくときは今の値がそのまま効く（遅れて前進が出すぎない）。近すぎの判定はならさない。
    追う人がいない・付き直し・c（測り直し）・r でためた値を捨てる。限界: 途切れずに別の人へ乗り換えると混ざる
    （今の 1 枚より遠くは使わない）。近い側の跳びは素通りするので、止まって再び加速することがある
  - 同じ PC（127.0.0.1）にだけ開く。数値の引数は有限で決まった範囲の値だけを受け付ける
  - 限界: 5 秒より短い見失いで別の人に入れ替わると、前の人の体の大きさのまま距離を出す（追う人の選び方の性質）。
    試験は画面に入るのを被験者 1 人だけにして行う。一定の遅れ（古い画像が間をあけずに届き続ける）は見分けられない

キー（映像のウィンドウを選んでから）: c = 測り直す（人が --calib-distance に立つ） / r = ロックオンを外す / スペース = 一時停止 / q = 終わる
  1〜9 = 「いま人が巻尺で ○ m に立っている」の印を --log-follow の mark_m 列に付ける（0 で消す。指令には関係しない）
c は、追う人が本当に --calib-distance（既定 2 m）に立っているときに押す。違う距離で押すと、距離をずっと誤る。
--no-window ではキーが効かないので、止めるのは Ctrl + C だけになる（動作の確認用）。
"""
import argparse
import collections
import json
import math
import os
import statistics
import sys
import threading
import time

import cv2
from websockets.sync.server import serve

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'wheelchair_follower'))
from wheelchair_follower.pose_follow_core import TargetTracker, calibrate, follow_cmd, measure  # noqa: E402
from pose_distance_camera import draw, find_model, open_source  # noqa: E402
from body_cut import NearGuard, body_cut  # noqa: E402

WHILL_MAX_LINEAR = 0.10    # whill_bridge.py の --linear-max の既定。この速さが前後の倒し量の上限に当たる [m/s]
WHILL_MAX_ANGULAR = 0.30   # whill_bridge.py の --angular-max の既定 [rad/s]
STALE_SEC = 0.3


def clamp(x, lim):
    if x is None or not math.isfinite(x):
        return 0.0
    return max(-lim, min(lim, x))


class CmdServer:
    """rosbridge と同じ形の最小の WebSocket サーバ。subscribe してきた whill_bridge.py に /cmd_vel を一定周期で送る"""

    def __init__(self, port, rate_hz=20.0, max_linear=WHILL_MAX_LINEAR, max_angular=WHILL_MAX_ANGULAR):
        self.port = port
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.period = 1.0 / rate_hz
        self.target = (0.0, 0.0)
        self.set_at = None
        self.subs = {}
        self.lock = threading.Lock()
        self.stop_ev = threading.Event()
        self.server = None
        self.threads = []

    def set(self, v, w):
        with self.lock:
            self.target = (clamp(v, self.max_linear), clamp(w, self.max_angular))
            self.set_at = time.monotonic()

    def current(self):
        with self.lock:
            if self.set_at is None or time.monotonic() - self.set_at > STALE_SEC:
                return 0.0, 0.0
            return self.target

    def n_subs(self):
        with self.lock:
            return len(self.subs)

    def _handler(self, ws):
        try:
            for raw in ws:
                try:
                    d = json.loads(raw)
                except (TypeError, ValueError):
                    continue
                if isinstance(d, dict) and d.get('op') == 'subscribe':
                    topic = d.get('topic') if isinstance(d.get('topic'), str) else '/cmd_vel'
                    with self.lock:
                        self.subs[ws] = topic
        except Exception:
            pass
        finally:
            with self.lock:
                self.subs.pop(ws, None)

    def _send(self, v, w):
        with self.lock:
            subs = list(self.subs.items())
        for ws, topic in subs:
            msg = {'op': 'publish', 'topic': topic,
                   'msg': {'linear': {'x': v, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': w}}}
            try:
                ws.send(json.dumps(msg))
            except Exception:
                with self.lock:
                    self.subs.pop(ws, None)

    def _loop(self):
        next_t = time.monotonic()
        while not self.stop_ev.is_set():
            self._send(*self.current())
            next_t += self.period
            time.sleep(max(0.0, next_t - time.monotonic()))

    def start(self):
        self.server = serve(self._handler, '127.0.0.1', self.port)   # ポートが使えなければここで止まる
        self.threads = [threading.Thread(target=self.server.serve_forever, daemon=True),
                        threading.Thread(target=self._loop, daemon=True)]
        for t in self.threads:
            t.start()

    def stop(self):
        self.stop_ev.set()
        if len(self.threads) > 1:
            self.threads[1].join(timeout=1.0)
        for _ in range(5):
            self._send(0.0, 0.0)
            time.sleep(0.02)
        with self.lock:
            conns = list(self.subs)
        for ws in conns:
            try:
                ws.close()
            except Exception:
                pass
        if self.server is not None:
            self.server.shutdown()


def parse_args():
    ap = argparse.ArgumentParser(description='本物のカメラの骨格追従の指令を whill_bridge.py へ渡す（試験用）')
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--video', help='カメラの代わりに読む動画（動作の確認用）')
    ap.add_argument('--url', help='ネットワークの映像の URL（DroidCam: http://<スマホのIP>:4747/video）。--camera より優先')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--hfov-deg', type=float, default=70.0)
    ap.add_argument('--model', default='')
    ap.add_argument('--num-poses', type=int, default=2)
    ap.add_argument('--min-visibility', type=float, default=0.5)
    ap.add_argument('--torso-m', type=float, default=0.62)
    ap.add_argument('--shoulder-m', type=float, default=0.42)
    ap.add_argument('--calib-distance', type=float, default=2.0, help='c で測り直すときに人が立つ距離 [m]')
    ap.add_argument('--calib-on-lock', action='store_true',
                    help='最初にロックオンしたとき、人が --calib-distance にいるとして測り直す（追従ノードの calib_distance と同じ）')
    ap.add_argument('--target-distance', type=float, default=1.2)
    ap.add_argument('--stop-distance', type=float, default=0.8)
    ap.add_argument('--lost-timeout', type=float, default=1.5)
    ap.add_argument('--search', action='store_true', help='見失ったら、最後に見えた側へその場で旋回して探す')
    ap.add_argument('--search-angular', type=float, default=0.5)
    ap.add_argument('--search-max', type=float, default=5.0, help='探索の旋回を続ける上限 [秒]。過ぎたら 0')
    ap.add_argument('--port', type=int, default=9091, help='whill_bridge.py の --rosbridge の番号（既定 ws://localhost:9091）')
    ap.add_argument('--distance-median', type=int, default=1,
                    help='速さの計算に使う距離を、直近この枚数の中央値と今の 1 枚の小さい方にする（1〜15、既定 1 = ならさない）。'
                         '近すぎの判定・ロックオン・--log-follow の distance_m はならす前の値')
    ap.add_argument('--max-angular', type=float, default=WHILL_MAX_ANGULAR,
                    help='旋回の上限 [rad/s]（0 より大きく 1.0 以下、既定 0.3）。whill_bridge.py の --angular-max と同じ値にする'
                         '（1.0 なら whill_bridge.py は --angular-max 1.0 --side-cap 90 --cap-limit-side 90。無人の開発用）')
    ap.add_argument('--max-linear', type=float, default=WHILL_MAX_LINEAR,
                    help='前進の上限 [m/s]（0 より大きく 0.7 以下）。whill_bridge.py の --linear-max と同じ値にする'
                         '（0.7 なら whill_bridge.py は --linear-max 0.7 --front-cap 42 --cap-limit-front 45。無人の開発用）')
    ap.add_argument('--edge', type=float, default=0.03, help='画面の上下の端から、この割合より内側にない点は切れているとみなす')
    ap.add_argument('--need-lower', choices=['knee', 'ankle', 'none'], default='knee',
                    help='写っていないと「近すぎ」とみなす下半身の点（既定は膝）')
    ap.add_argument('--release-s', type=float, default=0.5, help='切れていない状態がこの秒数続いてから前進を許す')
    ap.add_argument('--max-frame-gap', type=float, default=0.5,
                    help='前の画像からこの秒数より間があいて届いた画像では前進しない（ネットワークの映像が遅れたとき）')
    ap.add_argument('--log-follow', help='時刻・状態・推定距離・方向・近すぎの判定・送った v と ω を、この CSV に残す')
    ap.add_argument('--no-window', action='store_true')
    ap.add_argument('--frames', type=int, default=0)
    return ap.parse_args()


def main():
    args = parse_args()
    for name, lo, hi in (('max_frame_gap', 0.05, 5.0), ('release_s', 0.0, 10.0), ('edge', 0.0, 0.2),
                         ('lost_timeout', 0.1, 30.0), ('search_max', 0.0, 30.0), ('target_distance', 0.5, 5.0),
                         ('stop_distance', 0.3, 5.0), ('hfov_deg', 20.0, 170.0), ('calib_distance', 0.5, 10.0)):
        val = getattr(args, name)
        if not (math.isfinite(val) and lo <= val <= hi):   # nan・inf や範囲の外では、遅れの監視などが働かなくなる
            sys.exit(f'中止: --{name.replace("_", "-")} は {lo}〜{hi} の数で指定する（{val}）')
    if not 0.0 < args.max_linear <= 0.7:
        sys.exit('中止: --max-linear は 0 より大きく 0.7 以下（0.7 は追従の式の上限、先生の設計値）')
    if not 1 <= args.distance_median <= 15:
        sys.exit('中止: --distance-median は 1〜15')
    if not (math.isfinite(args.max_angular) and 0.0 < args.max_angular <= 1.0):
        sys.exit('中止: --max-angular は 0 より大きく 1.0 以下')
    server = CmdServer(args.port, max_linear=args.max_linear, max_angular=args.max_angular)
    server.start()   # カメラを開く前に、ポートが使えるかを確かめる
    print(f'ws://127.0.0.1:{args.port} で待っている。別のターミナルで: cd whill && python3 whill_bridge.py --dry-run')

    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision
    import mediapipe as mp
    detector = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=find_model(args.model)),
        running_mode=vision.RunningMode.VIDEO, num_poses=args.num_poses))
    cap = open_source(args)
    hfov = math.radians(args.hfov_deg)
    tracker = TargetTracker()
    torso_m, shoulder_m = args.torso_m, args.shoulder_m
    calibrated = False
    auto_calibrated = False
    dist_buf = collections.deque(maxlen=args.distance_median)   # 追っている人の直近の距離（速さの計算用にならす）
    paused = False
    last_seen = None
    search_dir = 1.0
    t_ms, n_frames, t_print = 0, 0, 0.0
    guard = NearGuard(args.release_s, time.monotonic())
    flog = None
    t_start = time.monotonic()
    t_frame = None
    mark = 0   # 数字キーで付ける「巻尺の距離」の印 [m]（記録だけに使う）
    try:
        if args.log_follow:
            flog = open(args.log_follow, 'w', buffering=1, encoding='utf-8')
            flog.write('time_s,state,distance_m,bearing_rad,cut,cut_reason,v_mps,w_radps,mark_m,distance_used_m\n')
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            now_f = time.monotonic()
            frame_late = t_frame is not None and now_f - t_frame > args.max_frame_gap
            t_frame = now_f
            h, w = bgr.shape[:2]
            t_ms = max(t_ms + 1, int(time.monotonic() * 1000))
            r = detector.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)), t_ms)
            n_frames += 1
            ms_by_pose = [measure(lm, w, h, hfov, torso_m, shoulder_m, args.min_visibility) for lm in r.pose_landmarks]
            ms = [m for m in ms_by_pose if m is not None]
            t_lost = 0.0 if last_seen is None else time.monotonic() - last_seen
            was_locked = tracker.last is not None
            target = tracker.select(ms, t_lost)
            if target is not None and calibrated and last_seen is not None and t_lost > tracker.relock_after:
                # 長く見失った後の付き直し: 別の人かもしれないので、体の大きさを測り直すまで 0 を送る
                calibrated = False
                print('長く見失った後に付き直した（別の人かもしれない）。c で測り直すまで 0 を送る')

            if target is not None and not was_locked and args.calib_on_lock and not calibrated and not auto_calibrated:
                auto_calibrated = True   # 自動の測り直しは最初の 1 回だけ（付き直しのたびに 2 m とみなさない）
                t_m, s_m = calibrate(target.torso_px, target.shoulder_px, args.calib_distance, w, hfov)
                torso_m, shoulder_m = t_m or torso_m, s_m or shoulder_m
                target.distance = args.calib_distance
                tracker.last = (args.calib_distance, target.bearing)
                calibrated = True
                print(f'実寸を測り直した（ロックオン時、{args.calib_distance:.1f} m）: 胴 {torso_m:.3f} m / 肩幅 {shoulder_m:.3f} m')

            v, wz = 0.0, 0.0
            if target is not None and not (math.isfinite(target.distance) and math.isfinite(target.bearing)):
                target = None   # NaN・Inf の距離や方向では動かさない（follow_cmd は NaN の方向を最大の旋回にしてしまう）
            cut, cut_reason = True, '追う人がいない'
            if target is not None:
                lm = next((r.pose_landmarks[i] for i, m in enumerate(ms_by_pose) if m is target), None)
                cut, cut_reason = body_cut(lm, args.min_visibility, args.edge, args.need_lower)
            if frame_late:
                cut, cut_reason = True, '映像が遅れた'   # 前の 1 枚から間があいた: 古い映像で前進しない（--release-s の待ちも数え直す）
            allow_forward = guard.update(cut, time.monotonic())
            if target is None or not was_locked:
                dist_buf.clear()   # 追う人がいない・付き直した: 前の人の距離を混ぜない
            d_used = None
            if target is not None:
                dist_buf.append(target.distance)
                # 直近 --distance-median 枚の中央値と今の 1 枚の小さい方: 1 枚の遠い跳びは抑え、近づくときは遅れない（安全側）
                d_used = min(statistics.median(dist_buf), target.distance)
                last_seen = time.monotonic()
                search_dir = 1.0 if target.bearing >= 0.0 else -1.0
                v, wz = follow_cmd(d_used, target.bearing, args.target_distance, args.stop_distance)
                state = '追従中'
                if not allow_forward:
                    v = 0.0   # 体が切れている（近すぎ）か、切れていない状態がまだ --release-s 秒続いていない。旋回は残す
                    state = (f'前進 0（{cut_reason}）' if cut_reason == '映像が遅れた'
                             else f'近すぎ: 前進 0（{cut_reason}）' if cut else '前進を待っている')
            elif last_seen is not None and t_lost > args.lost_timeout:
                state = f'見失い {t_lost:.1f} s'
                if args.search and t_lost <= args.lost_timeout + args.search_max:
                    wz = args.search_angular * search_dir
            else:
                state = '人を待っている' if last_seen is None else f'見失い {t_lost:.1f} s'
            if not calibrated:
                v, wz, state = 0.0, 0.0, state + '（c で測り直すまで 0）'
            if paused:
                v, wz, state = 0.0, 0.0, '一時停止（スペースで再開）'
            if v > 0.0 and time.monotonic() - now_f > args.max_frame_gap:
                # 画像を受け取ってから骨格の計算などで間があいた: 古い画像の結果で前進しない（待ちも数え直す）
                v = 0.0
                guard.update(True, time.monotonic())
                state = '前進 0（処理が遅れた）'
            server.set(v, wz)
            sv, sw = server.current()
            if flog is not None:
                dist = f'{target.distance:.3f}' if target else ''
                brg = f'{target.bearing:.3f}' if target else ''
                dused = f'{d_used:.3f}' if d_used is not None else ''
                flog.write(f'{time.monotonic() - t_start:.3f},{state},{dist},{brg},{int(cut)},{cut_reason},{sv:.3f},{sw:.3f},'
                           f'{mark if mark else ""},{dused}\n')

            d = f'{target.distance:.2f} m {math.degrees(target.bearing):+.0f} deg' if target else '-'
            if args.no_window:
                if time.monotonic() - t_print >= 1.0:
                    t_print = time.monotonic()
                    print(f'{n_frames:5d} 枚 {state:12s} 人 {d:15s} 送る v={sv:+.3f} w={sw:+.2f}  橋渡し {server.n_subs()}')
            else:
                info = [f'{state}  person {d}',
                        f'send v={sv:+.3f} m/s  w={sw:+.2f} rad/s  bridge {server.n_subs()}',
                        f'torso {torso_m:.2f} m  shoulder {shoulder_m:.2f} m  calibrated {calibrated}',
                        'NEAR: forward 0' if (target is not None and not allow_forward) else 'near-guard ok',
                        f'mark {mark} m' if mark else 'mark -']
                draw(bgr, r.pose_landmarks, ms_by_pose, target, info)
                cv2.imshow('pose_follow_whill (q: quit, space: pause)', bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord(' '):
                    paused = not paused
                    if paused:
                        server.set(0.0, 0.0)   # 次の画像を待たずに、すぐ 0 を送る
                    print('一時停止' if paused else '再開')
                elif key == ord('c'):
                    if target is None:
                        print('追っている人がいないので、測り直せない')
                    else:
                        t_m, s_m = calibrate(target.torso_px, target.shoulder_px, args.calib_distance, w, hfov)
                        torso_m, shoulder_m = t_m or torso_m, s_m or shoulder_m
                        tracker.last = (args.calib_distance, target.bearing)
                        calibrated = True
                        dist_buf.clear()   # 尺度が変わったので、古い尺度の距離を混ぜない
                        print(f'実寸を測り直した（{args.calib_distance:.1f} m）: 胴 {torso_m:.3f} m / 肩幅 {shoulder_m:.3f} m')
                elif ord('0') <= key <= ord('9'):
                    mark = key - ord('0')
                    print(f'印: {mark} m' if mark else '印を消した')
                elif key == ord('r'):
                    tracker.last = None
                    last_seen = None
                    calibrated = False   # 別の人に付き直すので、体の大きさも測り直すまで 0 を送る
                    dist_buf.clear()
                    server.set(0.0, 0.0)
                    print('ロックオンを外した（c で測り直すまで 0 を送る）')
            if args.frames and n_frames >= args.frames:
                break
    finally:
        server.set(0.0, 0.0)
        server.stop()
        if flog is not None:
            flog.close()
        cap.release()
        cv2.destroyAllWindows()
        print('0 を送って終わった')


if __name__ == '__main__':
    main()
