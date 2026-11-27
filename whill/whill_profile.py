# WHILL Model CR の速度プロファイル（速度モードごとの最高速度・加速・減速）を読む・書く・元に戻す
# （MacBook などで直接実行する。whill_bridge.py・whill_check.py と同時には使えない。シリアルは 1 つのプログラムだけが開ける）
#   pip install whill
#   python3 whill_profile.py read --port <装置名>                          … 全モード（0〜5）の今の値を表示し、CSV ではなく JSON に保存する（車いすは動かない）
#   python3 whill_profile.py set --port <装置名> --mode 0 --backup <read の JSON> --forward-accel 40 --forward-decel 120 …
#                                                                            … 加速・減速だけを書き換え、読み直して確かめる
#   python3 whill_profile.py restore --port <装置名> --mode 0 --backup <read の JSON>
#                                                                            … read で保存した元の値に戻し、読み直して確かめる
#
# ※ 本体の加速・減速を変える。強い加速・減速は人を乗せない開発用で、人を乗せる前に必ず元に戻す（restore）。
#    書き換えた値が電源を切っても残るかは確かめていない。試験の後は必ず restore し、read で元の値に戻ったことを確かめる。
#    先生や別の学生が使う前に元に戻す。
# ※ 最高速度（forward/reverse/turn_speed）は変えない。変えると、倒し量と速さの対応が変わるため。
# 値の範囲（Model CR の通信仕様書 Rev 10 §3.2.5。前進の加速だけ pywhill の説明（CR2 の 10〜64）と違う）: 前進 加速 10〜90・減速 40〜160、後退 加速 10〜50・減速 40〜80、旋回 加速 10〜60・減速 40〜160
import argparse
import json
import os
import sys
import time

from whill import ComWHILL

KEYS = ['forward_speed', 'forward_acceleration', 'forward_deceleration',
        'reverse_speed', 'reverse_acceleration', 'reverse_deceleration',
        'turn_speed', 'turn_acceleration', 'turn_deceleration']
RANGE = {'forward_acceleration': (10, 90), 'forward_deceleration': (40, 160),
         'reverse_acceleration': (10, 50), 'reverse_deceleration': (40, 80),
         'turn_acceleration': (10, 60), 'turn_deceleration': (40, 160)}
SPEED_RANGE = {'forward_speed': (8, 60), 'reverse_speed': (8, 30), 'turn_speed': (8, 35)}
STOP_PACKET = bytes([0xAF, 0x02, 0x01, 0xAF ^ 0x02 ^ 0x01])   # pywhill 1.5.0 の stop_data_stream() は KeyError になるので直接送る


def read_mode(w, mode, timeout=2.0):
    """speed_mode の値を読み出す（データセット 0）。読めなければ None"""
    w.com.reset_input_buffer()                 # 前の残りを読まない
    for k in KEYS:
        w.speed_profile[mode][k] = -1          # このモードの値が届いたかを見分ける印
    w.start_data_stream(100, data_set_number=0, speed_mode=mode)
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            w.refresh()
            if all(w.speed_profile[mode][k] != -1 for k in KEYS):
                return {k: int(w.speed_profile[mode][k]) for k in KEYS}
            time.sleep(0.02)
        return None
    finally:
        w.com.write(STOP_PACKET)
        time.sleep(0.2)
        w.com.reset_input_buffer()


def write_mode(w, mode, prof):
    w.set_speed_profile(mode, *[prof[k] for k in KEYS])
    time.sleep(0.3)


def show(mode, prof):
    if prof is None:
        print(f'モード {mode}: 読めなかった')
        return
    print(f'モード {mode}: 前進 {prof["forward_speed"]}/{prof["forward_acceleration"]}/{prof["forward_deceleration"]}  '
          f'後退 {prof["reverse_speed"]}/{prof["reverse_acceleration"]}/{prof["reverse_deceleration"]}  '
          f'旋回 {prof["turn_speed"]}/{prof["turn_acceleration"]}/{prof["turn_deceleration"]}  （最高速度/加速/減速）')


def check_ranges(prof):
    for k, (lo, hi) in {**RANGE, **SPEED_RANGE}.items():
        if not lo <= prof[k] <= hi:
            sys.exit(f'中止: {k} = {prof[k]} は範囲 {lo}〜{hi} の外')


def main():
    ap = argparse.ArgumentParser(description='WHILL の速度プロファイルを読む・書く・元に戻す')
    ap.add_argument('action', choices=['read', 'set', 'restore'])
    ap.add_argument('--port', required=True, help='USB-RS232C の装置名')
    ap.add_argument('--mode', type=int, choices=range(0, 6), help='書き換える・戻す速度モード（set・restore で必要）')
    ap.add_argument('--backup', help='read で保存した JSON（set・restore で必要）')
    ap.add_argument('--out', default='', help='read の保存先（既定 whill_profile_<日時>.json）')
    for k in RANGE:
        ap.add_argument('--' + k.replace('_acceleration', '-accel').replace('_deceleration', '-decel').replace('_', '-'),
                        dest=k, type=int, help=f'{k}（範囲 {RANGE[k][0]}〜{RANGE[k][1]}）')
    args = ap.parse_args()

    if args.action in ('set', 'restore'):
        if args.mode is None or not args.backup:
            sys.exit('中止: set・restore には --mode と --backup（read で保存した JSON）が要る')
        with open(args.backup, encoding='utf-8') as fp:
            backup = json.load(fp)
        if backup.get(str(args.mode)) is None:
            sys.exit(f'中止: {args.backup} にモード {args.mode} の元の値がない')

    w = ComWHILL(port=args.port)
    try:
        if args.action == 'read':
            profs = {m: read_mode(w, m) for m in range(6)}
            for m, p in profs.items():
                show(m, p)
            out = args.out or time.strftime('whill_profile_%Y%m%d_%H%M%S.json')
            if os.path.exists(out):
                sys.exit(f'中止: {out} はもうある（元の値の記録を上書きしない）。別の名前を --out で指定する')
            with open(out, 'w', encoding='utf-8') as fp:
                json.dump({str(m): p for m, p in profs.items()}, fp, ensure_ascii=False, indent=1)
            print(f'保存: {os.path.abspath(out)}')
            return

        now = read_mode(w, args.mode)
        if now is None:
            sys.exit('中止: 今の値を読めなかった')
        if args.action == 'restore':
            new = {k: int(backup[str(args.mode)][k]) for k in KEYS}
        else:
            changes = {k: getattr(args, k) for k in RANGE if getattr(args, k) is not None}
            if not changes:
                sys.exit('中止: 変える値が指定されていない（--forward-accel など）')
            if any(now[k] != backup[str(args.mode)][k] for k in ('forward_speed', 'reverse_speed', 'turn_speed')):
                sys.exit('中止: 今の最高速度が保存した元の値と違う（別の人が変えた可能性）。read からやり直す')
            new = dict(now)
            new.update(changes)
        check_ranges(new)
        print('今:  ', end='')
        show(args.mode, now)
        print('書く:', end='')
        show(args.mode, new)
        if args.action == 'set' and input('人を乗せていない・タイヤを浮かせているなら yes: ') != 'yes':
            sys.exit('中止しました')
        write_mode(w, args.mode, new)
        got = read_mode(w, args.mode)
        print('読み直し:', end='')
        show(args.mode, got)
        if got != new:
            sys.exit('注意: 読み直した値が書いた値と違う。restore で元の値に戻し、read で確かめる')
        print('書いた値と一致した' + ('（元に戻した）' if args.action == 'restore' else '。試験の後は restore で元に戻す'))
    finally:
        try:
            w.com.write(STOP_PACKET)
        except Exception:
            pass


if __name__ == '__main__':
    main()
