# 踊りや指の操縦のプログラムの速度の指令（/cmd_vel）を rosbridge で受け取り、WHILL Model CR のジョイスティックの指令に変えて送る
# （MacBook などで直接実行する。Docker は使わない）
#   pip install whill websocket-client
#   python3 whill_bridge.py --port /dev/cu.usbserial-XXXX
#   python3 whill_bridge.py --dry-run          … 車いすにつながず、送るはずの値を表示するだけ（試験用）
#   python3 whill_bridge.py --port ... --log finger.csv … 送った倒し量と左右のモータの速さを CSV に残す
#
# 安全のための仕組み
#   ・ジョイスティックの倒し量を小さく制限する（旋回は既定 10%、前後は既定 0% = 前後に動かない。どちらも最大 20%）。
#     無人の追従の開発で速く動かすときだけ、--cap-limit-front（最大 45）・--cap-limit-side（最大 90）で最大を毎回指定して上げる
#     （先生の許可 2026-11-06、旋回の 90% は 2026-11-16。後退の最大 20% は変えない）。指定しなければ最大は 20% のまま
#     速さは車いすの速度モードにも従う。前後の揺れを試すときは --front-cap 5 などを付ける
#     指の操縦（finger_dance robot:=whill）で前後に走らせるときは --front-cap 8 から始める
#     Model CR は同じ倒し量でも後退が前進の 1/3 ほどの速さなので、後ろ向きは --back-gain 倍（既定 3）にして釣り合わせる
#   ・倒し量は 1 秒に 20% までしか増やさない（急に動き出さない）。止めるときはすぐ 0 にする。
#     向きが逆になるとき（左回り → 右回りなど）は、一度 0 にしてから増やす
#     （後退は倒し量を back-gain 倍にしているので、増やす速さも同じ倍。車いすの速さの上がり方は前進と同じになる）
#   ・指令が 0.3 秒届かない・踊りのプログラムが止まった・rosbridge が切れたら止める
#   ・Enter キーで「停止」と「再開」を切り替える（停止中は 0 を送り続ける）
#   ・Ctrl + C で終わるときは、止めてから車いすのジョイスティックを使えるように戻す
#   ・送っている間は車いすのジョイスティックが効かない。いざというときは車いすの電源ボタンを押す
import argparse
import json
import math
import sys
import threading
import time

ap = argparse.ArgumentParser()
ap.add_argument('--port', help='USB-RS232C の装置名（ls /dev/cu.usbserial* で調べる）')
ap.add_argument('--dry-run', action='store_true', help='車いすにつながず、送るはずの値を表示する')
ap.add_argument('--rosbridge', default='ws://localhost:9091', help='rosbridge の場所')
ap.add_argument('--topic', default='/cmd_vel', help='速度の指令のトピック（Twist または TwistStamped）')
ap.add_argument('--linear-max', type=float, default=0.1, help='この前後の速さ [m/s] を、倒し量の上限に当てる')
ap.add_argument('--angular-max', type=float, default=0.3, help='この旋回の速さ [rad/s] を、倒し量の上限に当てる')
ap.add_argument('--front-cap', type=int, default=0, help='前後の倒し量の上限 [パーセント]（既定 0 = 前後に動かない。最大は --cap-limit-front、既定 20）')
ap.add_argument('--side-cap', type=int, default=10, help='旋回の倒し量の上限 [パーセント]（最大は --cap-limit-side、既定 20）')
ap.add_argument('--cap-limit-front', type=int, default=20,
                help='--front-cap の最大 [パーセント]（既定 20、最大 45）。無人の開発用。前進だけで、後退の最大 20%% は変えない')
ap.add_argument('--cap-limit-side', type=int, default=20, help='--side-cap の最大 [パーセント]（既定 20、最大 90）。無人の開発用')
ap.add_argument('--back-gain', type=float, default=3.0,
                help='後ろ向きの倒し量を何倍にするか。Model CR は同じ倒し量だと後退が前進の 1/3 ほどの速さなので 3（最大 20%%）')
ap.add_argument('--side-sign', type=int, default=-1, choices=(-1, 1),
                help='ROS の左回り（+）を、車いすの side の符号に直す。pywhill では side + が右回りなので -1')
ap.add_argument('--ramp', type=float, default=20.0, help='倒し量を増やす速さの上限 [パーセント/秒]')
ap.add_argument('--timeout', type=float, default=0.3, help='指令がこの時間 [秒] 届かなければ止める')
ap.add_argument('--rate', type=float, default=10.0, help='車いすに指令を送る回数 [回/秒]')
ap.add_argument('--log', help='送った倒し量と左右のモータの速さを、この CSV ファイルに残す（報告に使う）')
ap.add_argument('--log-motor', help='車いすから状態が届くたびに、左右のモータの角度・速さと車いすの計測の時刻を、この CSV ファイルに残す'
                '（指令には関係しない。速さの値は精度が低いので、正確な速さは角度と時刻から出す）')
ap.add_argument('--stream-ms', type=int, default=100, help='車いすが状態を送る間隔 [ms]（10〜1000。既定 100。--log-motor で細かく見るときは 20 など）')
args = ap.parse_args()
CAP_LIMIT_FRONT = max(0, min(45, args.cap_limit_front))
CAP_LIMIT_SIDE = max(0, min(90, args.cap_limit_side))
FRONT_CAP = max(0, min(CAP_LIMIT_FRONT, args.front_cap))
SIDE_CAP = max(0, min(CAP_LIMIT_SIDE, args.side_cap))
if FRONT_CAP != args.front_cap or SIDE_CAP != args.side_cap:
    print(f'注意: 倒し量の上限を最大で切った（前後 {args.front_cap}→{FRONT_CAP}%、旋回 {args.side_cap}→{SIDE_CAP}%）')
if not args.dry_run and not args.port:
    sys.exit('--port を指定してください（車いすにつながないで試すときは --dry-run）')

import websocket   # noqa: E402  （websocket-client）

state = {'v': 0.0, 'w': 0.0, 't': 0.0, 'stopped': False, 'connected': False}
lock = threading.Lock()


def on_message(_ws, message):
    d = json.loads(message)
    if d.get('op') != 'publish':
        return
    m = d.get('msg', {})
    tw = m.get('twist', m)          # TwistStamped なら twist の中、Twist ならそのまま
    v = float(tw.get('linear', {}).get('x', 0.0))
    w = float(tw.get('angular', {}).get('z', 0.0))
    if not (math.isfinite(v) and math.isfinite(w)):
        v = w = 0.0                  # NaN・Inf は止める（そのままだと clip が上限いっぱいの倒し量を返す）
    with lock:
        state['v'] = v
        state['w'] = w
        state['t'] = time.monotonic()


def ros_thread():
    # rosbridge につなぎ、切れたらつなぎ直す（切れている間は指令が届かないので、止まる）
    while True:
        try:
            ws = websocket.create_connection(args.rosbridge, timeout=5)
            ws.send(json.dumps({'op': 'subscribe', 'topic': args.topic, 'throttle_rate': 0, 'queue_length': 1}))
            with lock:
                state['connected'] = True
            ws.settimeout(1.0)
            while True:
                try:
                    on_message(ws, ws.recv())
                except websocket.WebSocketTimeoutException:
                    continue
        except Exception:
            with lock:
                state['connected'] = False
            time.sleep(1.0)


def key_thread():
    for _ in sys.stdin:
        with lock:
            state['stopped'] = not state['stopped']
            print('\n' + ('■ 停止（Enter で再開）' if state['stopped'] else '▶ 再開'))


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def approach(cur, target, step):
    # 倒し量を target に近づける。増やすときは 1 回に step まで、減らすときはすぐ。
    # 向きが逆になるとき（チョキの踊りの左右の入れ替えなど）は、一度 0 にしてから増やす
    if cur * target < 0:
        cur = 0.0
    return cur + clip(target - cur, -step, step) if abs(target) > abs(cur) else target


whill = None
if not args.dry_run:
    from whill import ComWHILL
    whill = ComWHILL(port=args.port)
    whill.start_data_stream(max(10, min(1000, args.stream_ms)))

threading.Thread(target=ros_thread, daemon=True).start()
threading.Thread(target=key_thread, daemon=True).start()
print(f'rosbridge {args.rosbridge} の {args.topic} を受け取り、倒し量 前 {FRONT_CAP}%・後ろ {min(20, round(FRONT_CAP * args.back_gain))}%・旋回 ±{SIDE_CAP}% で送ります。'
      f'Enter で停止・再開、Ctrl + C で終了。')

log = None
if args.log:
    log = open(args.log, 'w', buffering=1)
    # 速さは m/s にそろえる（left_mps・right_mps）。昨日までの記録と比べられるように km/h の列も残す
    log.write('time_s,state,v_mps,w_radps,front_pct,side_pct,left_kmh,right_kmh,battery_pct,left_mps,right_mps\n')
start = time.monotonic()

# モータの記録（--log-motor）: 状態が届くたびに 1 行。時刻 time_s は --log と同じ起点
# 角度は 0.001 rad（±π で巻き戻る）、counter は車いすの計測の時刻（Model CR は ms、200 で 0 に戻る）
motor_log = None
if args.log_motor and whill is not None:
    motor_log = open(args.log_motor, 'w', buffering=1)
    motor_log.write(f'# stream_ms={max(10, min(1000, args.stream_ms))}\n')
    motor_log.write('time_s,seq,counter,left_angle_rad,right_angle_rad,left_kmh,right_kmh,left_mps,right_mps\n')

    def on_data_set_1():
        motor_log.write(f'{time.monotonic() - start:.4f},{whill.seq_data_set_1},{whill.timestamp_current},'
                        f'{whill.left_motor["angle"]:.3f},{whill.right_motor["angle"]:.3f},'
                        f'{whill.left_motor["speed"]:.3f},{whill.right_motor["speed"]:.3f},'
                        f'{whill.left_motor["speed"] / 3.6:.4f},{whill.right_motor["speed"] / 3.6:.4f}\n')
    whill.register_callback('data_set_1', on_data_set_1)

front = side = 0.0
period = 1.0 / args.rate
try:
    while True:
        t0 = time.monotonic()
        with lock:
            v, w, t, stopped, connected = state['v'], state['w'], state['t'], state['stopped'], state['connected']
        fresh = connected and (t0 - t) <= args.timeout
        if stopped or not fresh:
            front = side = 0.0                       # 止めるときはすぐ 0
            why = '停止中' if stopped else ('rosbridge なし' if not connected else '指令なし')
        else:
            tf = clip(v / args.linear_max, -1, 1) * FRONT_CAP
            if tf < 0:
                tf = max(-20.0, tf * args.back_gain)  # 後退は遅いので倒し量を増やす（20% を超えない）
            ts = args.side_sign * clip(w / args.angular_max, -1, 1) * SIDE_CAP
            step = args.ramp * period                # 増やすときはゆっくり
            # 後退は倒し量を back-gain 倍にしているので、増やす速さも同じ倍にする（車いすの速さの上がり方は前進と同じ）
            fstep = step * max(1.0, args.back_gain) if tf < 0 else step
            front = approach(front, tf, fstep)
            side = approach(side, ts, step)
            why = '動いている'
        f, s = int(round(front)), int(round(side))
        info = ''
        if whill is not None:
            whill.send_joystick(front=f, side=s)
            whill.refresh()
            info = (f'  車輪 左 {whill.left_motor["speed"] / 3.6:+.3f} 右 {whill.right_motor["speed"] / 3.6:+.3f} m/s'
                    f'  電池 {whill.battery["level"]}%')
        if log is not None:
            lm = rm = bat = lms = rms = ''
            if whill is not None:
                lm, rm, bat = whill.left_motor['speed'], whill.right_motor['speed'], whill.battery['level']
                lms, rms = f'{lm / 3.6:.4f}', f'{rm / 3.6:.4f}'
            log.write(f'{t0 - start:.2f},{why},{v:.3f},{w:.3f},{f},{s},{lm},{rm},{bat},{lms},{rms}\n')
        print(f'{why:10s} 指令 v={v:+.3f} m/s ω={w:+.2f} rad/s → 前後 {f:+3d}% 旋回 {s:+3d}%{info}   ', end='\r', flush=True)
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
except KeyboardInterrupt:
    pass
finally:
    print('\n止めて終わります')
    if whill is not None:
        for _ in range(5):
            whill.send_joystick(front=0, side=0)
            time.sleep(0.05)
        whill.release_joystick()     # 車いすのジョイスティックを使えるように戻す
        # pywhill 1.5.0 の stop_data_stream() は長さの表に STOP がなく KeyError になるので、STOP（長さ 1）を直接送る
        whill.com.write(bytes([0xAF, 0x02, 0x01, 0xAF ^ 0x02 ^ 0x01]))
    if log is not None:
        log.close()
        print(f'記録：{args.log}')
    if motor_log is not None:
        whill.register_callback('data_set_1', None)
        motor_log.close()
        print(f'モータの記録：{args.log_motor}')
