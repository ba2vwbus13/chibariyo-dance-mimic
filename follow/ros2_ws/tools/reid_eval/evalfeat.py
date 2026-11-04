"""色特徴のオフライン比較(collect.py で集めた切り抜きを使う)。
使い方: python3 evalfeat.py <collectのディレクトリ> crowd_test crowd_swap ...

各ワールドで、本人の切り抜き(距離1.5-3m)から原本を5枚選び、原本と
以降の本人(self)・通行人(other)の類似度を計算する。重なり(2人の方位差<0.3rad)は除外。"""
import glob, json, math, os, sys
import cv2
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from label import labeled

def h_hs(img, hb, sb, blur=0):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [hb, sb], [0, 180, 0, 256])
    if blur:
        # 色相は循環するので端を巻き込んでぼかす
        pad = np.concatenate([h[-blur:], h, h[:blur]], axis=0)
        pad = cv2.GaussianBlur(pad, (2 * blur + 1, 2 * blur + 1), 0)
        h = pad[blur:-blur]
    h = h.astype(np.float32); h /= max(h.sum(), 1e-9)
    return h

def h_hsv(img, hb, sb, vb):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1, 2], None, [hb, sb, vb], [0, 180, 0, 256, 0, 256])
    h = h.astype(np.float32); h /= max(h.sum(), 1e-9)
    return h

def bhat(a, b):
    return max(0.0, 1.0 - float(cv2.compareHist(a, b, cv2.HISTCMP_BHATTACHARYYA)))

def lab_mean(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2LAB).reshape(-1, 3).astype(np.float32).mean(0)

FEATS = {
    'H30xS32(平滑なし)': (lambda im: h_hs(im, 30, 32), bhat),
    'H16xS4': (lambda im: h_hs(im, 16, 4), bhat),
    'H30xS32+平滑(採用)': (lambda im: h_hs(im, 30, 32, blur=2), bhat),
    'H12xS4xV4': (lambda im: h_hsv(im, 12, 4, 4), bhat),
    'H8xS3xV3': (lambda im: h_hsv(im, 8, 3, 3), bhat),
    'Lab平均': (lab_mean, lambda a, b: math.exp(-float(np.linalg.norm(a - b)) / 20.0)),
}

def auc(s, o):
    s = np.array(s); o = np.array(o)
    return float((s[:, None] > o[None, :]).mean() + 0.5 * (s[:, None] == o[None, :]).mean())

def main(root, worlds):
    print('| 特徴 | ' + ' | '.join(worlds) + ' |')
    print('|---|' + '---|' * len(worlds))
    for fname, (feat, sim) in FEATS.items():
        cells = []
        for w in worlds:
            d = f'{root}/{w}'
            rows = [x for x in labeled(d) if not x['overlap']]
            me = [x for x in rows if x['label2'] == 'person']
            ot = [x for x in rows if x['label2'] == 'person2']
            anchors = [x for x in me if 1.5 <= x['dist'] <= 3.0][:5]
            if not anchors or not ot:
                cells.append('-'); continue
            img = lambda x: cv2.imread(f"{d}/{x['name']}.png")
            S, O = [], []
            for a in anchors:
                fa = feat(img(a))
                S += [sim(fa, feat(img(x))) for x in me if x is not a and x['dist'] >= 0.6]
                O += [sim(fa, feat(img(x))) for x in ot if x['dist'] >= 0.6]
            th = float(np.percentile(S, 5))  # 本人の棄却を5%に抑える閾値
            pas = 100 * float(np.mean(np.array(O) >= th))
            cells.append(f'AUC {auc(S, O):.2f} / 通過{pas:.0f}%')
        print(f'| {fname} | ' + ' | '.join(cells) + ' |')

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
