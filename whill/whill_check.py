# WHILL Model CR とつながるかを確かめる（MacBook などで直接実行する。Docker は使わない）
#   pip install whill
#   python3 whill_check.py --port /dev/cu.usbserial-XXXX              … 動かさずに、電池・速度モード・モータの速さを 5 秒表示する
#   python3 whill_check.py --port /dev/cu.usbserial-XXXX --test-move  … ごく小さく前進・旋回して、向きと速さを確かめる
# 動かす試験の前に：人を乗せない、周りに 1 m 以上の空きを作る、すぐ電源ボタンを押せる人がそばにいる
# 指令を送っている間は、車いすのジョイスティックが効かない（pywhill がジョイスティックを無効にして指令する）。
# 止めるときは車いすの電源ボタンを押す。このプログラムは終わるときに止めてからジョイスティックを戻す
import argparse
import time

from whill import ComWHILL

ap = argparse.ArgumentParser()
ap.add_argument('--port', required=True, help='USB-RS232C の装置名（ls /dev/cu.usbserial* で調べる）')
ap.add_argument('--test-move', action='store_true', help='ジョイスティック 8 パーセントで 1 秒ずつ前進・左右に旋回する')
ap.add_argument('--percent', type=int, default=8, help='動かす試験のジョイスティックの倒し量 [パーセント]（最大 15）')
args = ap.parse_args()
pct = max(1, min(15, args.percent))

w = ComWHILL(port=args.port)
w.start_data_stream(100)   # 0.1 秒ごとに状態を送ってもらう


def show(label, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        w.refresh()
        print(f'{label:8s} 電池 {w.battery["level"]:3d}%  速度モード {w.speed_mode_indicator}  '
              f'左 {w.left_motor["speed"]:+.2f} km/h  右 {w.right_motor["speed"]:+.2f} km/h', end='\r')
        time.sleep(0.1)
    print()


def drive(front, side, sec, label):
    # 0.1 秒ごとにジョイスティックの指令を送り続け、終わったら 0 にして止める
    t0 = time.time()
    while time.time() - t0 < sec:
        w.send_joystick(front=front, side=side)
        w.refresh()
        print(f'{label:8s} 左 {w.left_motor["speed"]:+.2f} km/h  右 {w.right_motor["speed"]:+.2f} km/h', end='\r')
        time.sleep(0.1)
    for _ in range(5):
        w.send_joystick(front=0, side=0)
        time.sleep(0.05)
    print()


try:
    show('状態', 5.0)
    if args.test_move:
        if input(f'ジョイスティック {pct}% で 1 秒ずつ動かします。人が乗っていない・周りが空いているなら yes：') != 'yes':
            raise SystemExit('中止しました')
        drive(pct, 0, 1.0, '前進')
        show('停止', 1.0)
        drive(0, pct, 1.0, 'side +')
        show('停止', 1.0)
        drive(0, -pct, 1.0, 'side -')
        show('停止', 1.0)
        print('pywhill の説明では side + は右回り。違ったら whill_bridge.py に --side-sign 1 を付けてください。')
finally:
    for _ in range(5):
        w.send_joystick(front=0, side=0)
        time.sleep(0.05)
    w.release_joystick()   # 車いすのジョイスティックを使えるように戻す
    w.stop_data_stream()
