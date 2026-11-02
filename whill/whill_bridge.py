# 踊りのプログラムの速度の指令（/cmd_vel）を rosbridge で受け取り、WHILL Model CR のジョイスティックの指令に変えて送る
# （MacBook などで直接実行する。Docker は使わない）
#   pip install whill websocket-client
#   python3 whill_bridge.py --port /dev/cu.usbserial-XXXX
#   python3 whill_bridge.py --dry-run          … 車いすにつながず、送るはずの値を表示するだけ（試験用）
#
# 安全のための仕組み
#   ・ジョイスティックの倒し量を小さく制限する（旋回は既定 10%、前後は既定 0% = 前後に動かない。どちらも最大 20%）。
#     速さは車いすの速度モードにも従う。前後の揺れを試すときは --front-cap 5 などを付ける
#   ・倒し量は 1 秒に 20% までしか増やさない（急に動き出さない）。止めるときはすぐ 0 にする
#   ・指令が 0.3 秒届かない・踊りのプログラムが止まった・rosbridge が切れたら止める
#   ・Enter キーで「停止」と「再開」を切り替える（停止中は 0 を送り続ける）
#   ・Ctrl + C で終わるときは、止めてから車いすのジョイスティックを使えるように戻す
#   ・送っている間は車いすのジョイスティックが効かない。いざというときは車いすの電源ボタンを押す
import argparse
import json
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
ap.add_argument('--front-cap', type=int, default=0, help='前後の倒し量の上限 [パーセント]（既定 0 = 前後に動かない。最大 20）')
ap.add_argument('--side-cap', type=int, default=10, help='旋回の倒し量の上限 [パーセント]（最大 20）')
ap.add_argument('--side-sign', type=int, default=-1, choices=(-1, 1),
                help='ROS の左回り（+）を、車いすの side の符号に直す。pywhill では side + が右回りなので -1')
ap.add_argument('--ramp', type=float, default=20.0, help='倒し量を増やす速さの上限 [パーセント/秒]')
ap.add_argument('--timeout', type=float, default=0.3, help='指令がこの時間 [秒] 届かなければ止める')
ap.add_argument('--rate', type=float, default=10.0, help='車いすに指令を送る回数 [回/秒]')
args = ap.parse_args()
FRONT_CAP = max(0, min(20, args.front_cap))
SIDE_CAP = max(0, min(20, args.side_cap))
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
    with lock:
        state['v'] = float(tw.get('linear', {}).get('x', 0.0))
        state['w'] = float(tw.get('angular', {}).get('z', 0.0))
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


whill = None
if not args.dry_run:
    from whill import ComWHILL
    whill = ComWHILL(port=args.port)
    whill.start_data_stream(100)

threading.Thread(target=ros_thread, daemon=True).start()
threading.Thread(target=key_thread, daemon=True).start()
print(f'rosbridge {args.rosbridge} の {args.topic} を受け取り、倒し量 前後 ±{FRONT_CAP}%・旋回 ±{SIDE_CAP}% で送ります。'
      f'Enter で停止・再開、Ctrl + C で終了。')

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
            ts = args.side_sign * clip(w / args.angular_max, -1, 1) * SIDE_CAP
            step = args.ramp * period                # 増やすときはゆっくり
            front += clip(tf - front, -step, step) if abs(tf) > abs(front) else (tf - front)
            side += clip(ts - side, -step, step) if abs(ts) > abs(side) else (ts - side)
            why = '踊っている'
        f, s = int(round(front)), int(round(side))
        info = ''
        if whill is not None:
            whill.send_joystick(front=f, side=s)
            whill.refresh()
            info = (f'  左 {whill.left_motor["speed"]:+.2f} 右 {whill.right_motor["speed"]:+.2f} km/h'
                    f'  電池 {whill.battery["level"]}%')
        print(f'{why:10s} 指令 v={v:+.3f} ω={w:+.2f} → 前後 {f:+3d}% 旋回 {s:+3d}%{info}   ', end='\r', flush=True)
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
