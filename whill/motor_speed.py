# whill_bridge.py --log-motor の記録から、モータの角度と車いすの計測の時刻で車輪の速さを出し、
# --log の指令の区間ごとに「指令との比・動き出し（90%）・止まるまで」を表にする（車いすにはつながない）
#   python3 motor_speed.py <--log の CSV> <--log-motor の CSV> [--tread 0.49] [--radius 0.145] [--out 速さの CSV]
#
# 速さの出し方（Model CR の通信仕様書: 速さの値は精度が低いので、正確な速さは角度と時刻から出す）
#   角度: 0.001 rad、±π で巻き戻る → 差を −π〜π に直す
#   時刻: 車いすの計測の時刻 counter（Model CR は ms で 200 前後で 0 に戻る。pywhill は 0〜200 の周期 201 として扱う）
#         → 周期で割った余りを差にする。counter が 0 のまま・通し番号が飛んだ（データの抜け）ときは PC の時刻を使う
#   車輪の速さ [m/s] = 角度の差 / 時間の差 × 車輪の半径。左は前進で +、右は前進で −（記録で確かめた向き）
#   前後の速さ v = (左 − 右) / 2、旋回の速さ ω = (−左 − 右) / トレッド（左回りが +）
import argparse
import csv
import math
import statistics


def read_rows(path, enc):
    with open(path, 'rb') as fp:
        text = fp.read().decode(enc)
    return list(csv.DictReader(line for line in text.splitlines() if not line.startswith('#')))


def wheel_speeds(motor_rows, radius):
    out = []
    prev = None
    # counter の周期: pywhill は 0〜200 の 201 として扱う。記録に 200 が出れば 201、出なければ 200
    period = 201 if any(int(r['counter']) >= 200 for r in motor_rows) else 200
    gaps = 0
    for r in motor_rows:
        cur = (float(r['time_s']), int(r['counter']), float(r['left_angle_rad']), float(r['right_angle_rad']),
               int(r['seq']))
        if prev is not None and cur[1] == prev[1] and cur[1] != 0:
            continue                           # 同じ測定がもう一度送られてきた（counter が同じ）: 読み飛ばし、前の行を基準に残す
        if prev is not None:
            dt = ((cur[1] - prev[1]) % period) / 1000.0
            if cur[4] - prev[4] != 1 and dt <= 0:
                gaps += 1                      # 通し番号が飛んだ（データが抜けた）: counter は 1 周以上回ったかもしれない
                dt = cur[0] - prev[0]
            elif dt <= 0 or prev[1] == cur[1] == 0:
                dt = cur[0] - prev[0]          # counter が使えないときは PC の時刻
            if dt > 0:
                dl = math.atan2(math.sin(cur[2] - prev[2]), math.cos(cur[2] - prev[2]))
                dr = math.atan2(math.sin(cur[3] - prev[3]), math.cos(cur[3] - prev[3]))
                out.append((cur[0], dl / dt * radius, dr / dt * radius))
        prev = cur
    if gaps:
        print(f'注意: データの抜けが {gaps} か所（そこは PC の時刻で計算した）')
    return out


def segments(cmd_rows):
    """--log の「動いている」で指令が 0 でない区間: (始め, 終わり, v, w)"""
    segs, cur = [], None
    for r in cmd_rows:
        t, v, w = float(r['time_s']), float(r['v_mps']), float(r['w_radps'])
        key = (v, w) if r['state'] == '動いている' and (v or w) else None
        if cur is None or key != cur[2]:
            if cur is not None and cur[2] is not None:
                segs.append((cur[0], t, *cur[2]))
            cur = [t, t, key]
    if cur is not None and cur[2] is not None:
        segs.append((cur[0], float(cmd_rows[-1]['time_s']), *cur[2]))
    return segs


def main():
    ap = argparse.ArgumentParser(description='モータの角度から車輪の速さを出し、指令の区間ごとに比と時間を出す')
    ap.add_argument('cmd_log')
    ap.add_argument('motor_log')
    ap.add_argument('--tread', type=float, default=0.49, help='左右の車輪の間隔 [m]')
    ap.add_argument('--radius', type=float, default=0.145, help='車輪の半径 [m]（直径 約 29 cm の実測）')
    ap.add_argument('--enc', default='cp932', help='--log の CSV の文字コード（Windows で作ったものは cp932）')
    ap.add_argument('--out', help='時刻ごとの v・ω を書き出す CSV')
    args = ap.parse_args()

    with open(args.motor_log, encoding='utf-8') as fp:
        head = fp.readline()
    if head.startswith('# stream_ms='):
        ms = int(head.split('=')[1])
        print(f'送る間隔 {ms} ms')
        if ms >= 200:
            print('注意: 送る間隔が counter の周期（約 200 ms）以上なので、counter の時刻は使えない（PC の時刻で計算する）')
    speeds = wheel_speeds(read_rows(args.motor_log, 'utf-8'), args.radius)
    vw = [(t, (l - r) / 2, (-l - r) / args.tread) for t, l, r in speeds]
    if args.out:
        with open(args.out, 'w', newline='', encoding='utf-8') as fp:
            w = csv.writer(fp)
            w.writerow(['time_s', 'v_mps', 'w_radps'])
            w.writerows([(f'{t:.4f}', f'{v:.4f}', f'{o:.4f}') for t, v, o in vw])

    print(f'{"指令":>10} {"実際":>8} {"比":>5} {"動き始め10%":>11} {"動き出し90%":>11} {"止まるまで":>9}')
    for t0, t1, v, w in segments(read_rows(args.cmd_log, args.enc)):
        kind, cmd = ('v', v) if v else ('w', w)
        pick = (lambda x: x[1]) if kind == 'v' else (lambda x: x[2])
        inside = [x for x in vw if t0 <= x[0] < t1]
        late = [pick(x) for x in inside if x[0] >= t1 - 2.5]
        if not late:
            continue
        m = statistics.mean(late)
        rise = next((x[0] - t0 for x in inside if abs(pick(x)) >= 0.9 * abs(m)), None)
        start10 = next((x[0] - t0 for x in inside if abs(pick(x)) >= 0.1 * abs(m)), None)
        stop = next((x[0] - t1 for x in vw if x[0] >= t1 and abs(pick(x)) <= 0.1 * abs(m)), None)
        f = lambda s: '-' if s is None else f'{s:.2f} s'
        print(f'{kind}{cmd:+9.3f} {m:+8.3f} {m / cmd:5.2f} {f(start10):>11} {f(rise):>11} {f(stop):>9}')


if __name__ == '__main__':
    main()
