#!/usr/bin/env python3
"""解説資料の図(データから描くもの)を docs/img/ に生成する。

使い方(プロジェクトのルートで): python3 docs/build_figs.py
入力は ros2_ws/metrics/ の計測結果(git管理外)。手で描く構成図は guide.html に直接 SVG で書いてある。
"""
import csv
import glob
import json
import math
import os
import re
import statistics as st

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
M = os.path.join(ROOT, 'ros2_ws', 'metrics')
OUT = os.path.join(ROOT, 'docs', 'img')
os.makedirs(OUT, exist_ok=True)

# 色は対象ごとに固定する(本人=青、通行人=橙、車椅子=濃い灰)
C_ME, C_OTHER, C_CHAIR = '#2a78d6', '#eb6834', '#3d3d3a'
C_GRAY, C_GRID, C_SUB = '#b4b2a9', '#e4e2dc', '#52514e'
plt.rcParams.update({
    'font.family': ['Hiragino Sans'],
    'font.size': 9, 'axes.edgecolor': C_SUB, 'axes.labelcolor': '#0b0b0b',
    'xtick.color': C_SUB, 'ytick.color': C_SUB, 'axes.spines.top': False,
    'axes.spines.right': False, 'svg.fonttype': 'path', 'axes.grid': False,
})


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), bbox_inches='tight', pad_inches=0.04)
    if os.environ.get('REVIEW_DIR'):  # 見た目の確認用
        fig.savefig(os.path.join(os.environ['REVIEW_DIR'], name.replace('.svg', '.png')), dpi=110, bbox_inches='tight')
    plt.close(fig)
    print('wrote', name)


def summaries(prefix):
    keys = {'id': r'ID維持率\(本人を追従\): ([\d.]+)', 'wrong': r'別人追従: ([\d.]+)',
            'lost': r'見失い率: ([\d.]+)'}
    out = {k: [] for k in keys}
    for f in sorted(glob.glob(os.path.join(M, f'summary_{prefix}[0-9]_*.txt'))):
        t = open(f).read()
        for k, p in keys.items():
            out[k].append(float(re.search(p, t).group(1)))
    return out


def match(c, g, b_tol=0.15):
    return g is not None and abs(c[1] - g[1]) < b_tol and abs(c[0] + 0.3 - g[0]) < 0.6


# ---------- 評価シナリオの見取り図 ----------
def fig_map():
    target = [3.5, 0.0, 3.0, 2.0, 1.5, 2.8, 0.0, 2.0, -0.5, 0.0, 0.0, -2.0, 1.5, -2.8, 3.0, -2.0]
    tx, ty = target[0::2] + target[:1], target[1::2] + target[1:2]
    rows = list(csv.DictReader(open(glob.glob(os.path.join(M, 'metrics_f_reid1_*.csv'))[0])))
    rows = [r for r in rows if 20.0 <= float(r['t']) <= 60.0]
    fig, ax = plt.subplots(figsize=(4.6, 4.3))
    ax.add_patch(plt.Rectangle((-6, -6), 12, 12, fill=False, lw=3, ec=C_GRAY))
    ax.plot(tx, ty, ls='--', lw=1.2, color=C_ME, alpha=.55)
    ax.plot([1.8, 1.8], [-3, 3], ls='--', lw=1.2, color=C_OTHER, alpha=.55)
    ax.plot([float(r['person_x']) for r in rows], [float(r['person_y']) for r in rows], lw=2, color=C_ME, label='本人(0.4 m/s)')
    ax.plot([float(r['person2_x']) for r in rows], [float(r['person2_y']) for r in rows], lw=2, color=C_OTHER, label='通行人(0.6 m/s)')
    ax.plot([float(r['robot_x']) for r in rows], [float(r['robot_y']) for r in rows], lw=2, color=C_CHAIR, label='車椅子')
    ax.plot([-1], [0], marker='s', ms=8, color=C_CHAIR, mec='white', mew=1.5)
    ax.annotate('車椅子の出発位置\n(-1, 0)', (-1, 0), (-4.3, -1.3), fontsize=8, color=C_SUB,
                arrowprops=dict(arrowstyle='-', color=C_SUB, lw=.6))
    ax.annotate('本人の周回コース\n1周 約15 m', (0, 2), (-4.6, 3.3), fontsize=8, color=C_ME,
                arrowprops=dict(arrowstyle='-', color=C_ME, lw=.6))
    ax.annotate('通行人の往復\nx = 1.8 m, y = -3〜3 m', (1.8, -3), (2.2, -4.9), fontsize=8, color=C_OTHER,
                arrowprops=dict(arrowstyle='-', color=C_OTHER, lw=.6))
    ax.set_xlim(-6.5, 6.5); ax.set_ylim(-6.5, 6.5); ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.legend(loc='upper right', fontsize=8, frameon=False)
    save(fig, 'map.svg')


# ---------- 4方式の比較 ----------
def fig_results():
    conds = [('reid(改良後)', 'f_reid'), ('reid(8/23 設定)', 'f_old'), ('YOLO 最近傍', 'f_yolo'), ('LiDAR 最近傍', 'f_lidar')]
    data = [summaries(p) for _, p in conds]
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.2), sharey=True)
    for ax, (key, title) in zip(axes, [('id', 'ID 維持率 [%]'), ('wrong', '別人追従 [%]'), ('lost', '見失い率 [%]')]):
        for i, d in enumerate(data):
            v = d[key]; med = st.median(v)
            col = C_ME if i == 0 else C_GRAY
            ax.barh(i, med, height=.55, color=col)
            ax.plot([min(v), max(v)], [i, i], color=C_CHAIR, lw=1)
            for x in (min(v), max(v)):
                ax.plot([x, x], [i - .12, i + .12], color=C_CHAIR, lw=1)
            ax.text(max(v) + 2, i, f'{med:.1f}', va='center', fontsize=8, color='#0b0b0b')
        ax.set_title(title, fontsize=9, loc='left')
        ax.set_xlim(0, 108); ax.invert_yaxis()
        ax.xaxis.grid(True, color=C_GRID, lw=.6); ax.set_axisbelow(True)
    axes[0].set_yticks(range(len(conds))); axes[0].set_yticklabels([c for c, _ in conds])
    save(fig, 'results.svg')


# ---------- 色の記憶が通行人に染まる様子 ----------
def fig_contam():
    rows = [json.loads(l) for l in open(glob.glob(os.path.join(M, 'debug_reidbase1_*.jsonl'))[0])]
    csvr = list(csv.DictReader(open(glob.glob(os.path.join(M, 'metrics_reidbase1_*.csv'))[0])))
    fig, ax = plt.subplots(figsize=(7.2, 2.4))
    # 別人を追っていた時間帯を薄く塗る
    on = None
    for r in csvr:
        t = float(r['t'])
        if not 8 <= t <= 45: continue
        if r['followed'] == 'person2' and on is None: on = t
        if r['followed'] != 'person2' and on is not None:
            ax.axvspan(on, t, color=C_OTHER, alpha=.12, lw=0); on = None
    for r in rows:
        if not 8 <= r['t'] <= 45: continue
        for c in r['cands']:
            if match(c, r['gt_person']): col = C_ME
            elif match(c, r['gt_person2']): col = C_OTHER
            else: continue
            ax.plot(r['t'], c[2], 'o', ms=4.5, mfc=col if r['accepted'] else 'white', mec=col, mew=1.2)
    ax.plot([], [], 'o', ms=4.5, color=C_ME, label='本人の候補'); ax.plot([], [], 'o', ms=4.5, color=C_OTHER, label='通行人の候補')
    ax.plot([], [], 'o', ms=4.5, mfc='white', mec=C_SUB, label='棄却されたフレーム(白抜き)')
    ax.fill_between([], [], color=C_OTHER, alpha=.12, label='通行人を追従していた時間')
    ax.set_xlim(8, 45); ax.set_ylim(0, 1.0)
    ax.set_xlabel('計測開始からの時間 [s]'); ax.set_ylabel('記憶との色類似度')
    ax.yaxis.grid(True, color=C_GRID, lw=.6); ax.set_axisbelow(True)
    ax.legend(loc='lower left', fontsize=7.5, frameon=False, ncol=2)
    save(fig, 'contam.svg')


# ---------- 本人と通行人の類似度の分布(色領域の比較) ----------
def sims(prefix):
    S, O = [], []
    for f in glob.glob(os.path.join(M, f'debug_{prefix}*.jsonl')):
        for l in open(f):
            r = json.loads(l); gp, g2 = r['gt_person'], r['gt_person2']
            if gp is None or g2 is None or abs(gp[1] - g2[1]) < 0.3: continue
            for c in r['cands']:
                if match(c, gp): S.append(c[2])
                elif match(c, g2): O.append(c[2])
    return S, O


def fig_simdist():
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.3), sharey=False)
    bins = np.linspace(0, 1, 26)
    for ax, (pre, title) in zip(axes, [('reidv2', 'ボックス比率の色領域(改良前)'), ('reidv4', '胴体帯の色領域(改良後)')]):
        S, O = sims(pre)
        ax.hist(S, bins=bins, color=C_ME, alpha=.85, weights=np.ones(len(S)) / len(S) * 100, label=f'本人 {len(S)} 件')
        ax.hist(O, bins=bins, color=C_OTHER, alpha=.85, weights=np.ones(len(O)) / len(O) * 100, label=f'通行人 {len(O)} 件')
        ax.axvline(0.55, color=C_CHAIR, lw=1, ls='--')
        ax.text(0.56, ax.get_ylim()[1] * .92, '下限 0.55', fontsize=7.5, color=C_CHAIR)
        ax.set_title(title, fontsize=9, loc='left'); ax.set_xlabel('原本との色類似度'); ax.set_xlim(0, 1)
        ax.legend(fontsize=7.5, frameon=False, loc='upper left')
    axes[0].set_ylabel('割合 [%]')
    save(fig, 'simdist.svg')


# ---------- 赤い服の彩度のゆれ ----------
def hs_hist(img, blur):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
    if blur:
        pad = np.concatenate([h[-blur:], h, h[:blur]], axis=0)
        h = cv2.GaussianBlur(pad, (2 * blur + 1, 2 * blur + 1), 0)[blur:-blur]
    h = np.ascontiguousarray(h, dtype=np.float32); h /= h.sum()
    return h


def bhat(a, b):
    return max(0.0, 1.0 - float(cv2.compareHist(a, b, cv2.HISTCMP_BHATTACHARYYA)))


def fig_sat():
    import sys
    sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'tools', 'reid_eval'))
    from label import labeled
    d = os.path.join(M, 'collect', 'crowd_swap')
    me = [x for x in labeled(d) if x['label2'] == 'person' and not x['overlap'] and 1.0 <= x['dist'] <= 3.0]
    imgs = [cv2.imread(f"{d}/{x['name']}.png") for x in me]
    # 赤い画素(彩度>100)の平均彩度が低い方・高い方から1割の位置の2枚を選ぶ
    def smean(im):
        s_ = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)[..., 1]; return s_[s_ > 100].mean()
    order = sorted(range(len(imgs)), key=lambda i: smean(imgs[i]))
    a, b = imgs[order[len(order) // 10]], imgs[order[-len(order) // 10 - 1]]
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.2), sharey=True)
    for ax, blur, title in [(axes[0], 0, '平滑化なし'), (axes[1], 2, '平滑化あり(幅2ビン)')]:
        ha, hb = hs_hist(a, blur).sum(0), hs_hist(b, blur).sum(0)
        x = np.arange(32)
        ax.bar(x - .2, ha * 100, width=.4, color=C_ME, label=f'フレーム A(平均彩度 {smean(a):.0f})')
        ax.bar(x + .2, hb * 100, width=.4, color='#8fb8e8', label=f'フレーム B(平均彩度 {smean(b):.0f})')
        s_ = bhat(hs_hist(a, blur), hs_hist(b, blur))
        ax.set_title(f'{title}  A と B の類似度 {s_:.2f}', fontsize=9, loc='left')
        ax.set_xlim(15.5, 31.5); ax.set_xticks(range(16, 32, 2)); ax.set_xticklabels([f'{i*8}' for i in range(16, 32, 2)])
        ax.set_xlabel('彩度ビンの下端 [0-255]')
        ax.legend(fontsize=7.5, frameon=False, loc='upper left')
    axes[0].set_ylabel('画素の割合 [%]')
    save(fig, 'sat.svg')


# ---------- 通行人との相互デッドロック ----------
def fig_deadlock():
    rows = list(csv.DictReader(open(glob.glob(os.path.join(M, 'metrics_reidv3c2_*.csv'))[0])))
    t = [float(r['t']) for r in rows]
    d2 = [float(r['dist_to_person2']) for r in rows]
    yaw = np.degrees(np.unwrap([float(r['robot_yaw']) for r in rows]))
    # 旋回の速さ[度/s](1秒の幅で差分)
    turn = [abs(yaw[min(len(yaw) - 1, i + 5)] - yaw[max(0, i - 5)]) / (t[min(len(t) - 1, i + 5)] - t[max(0, i - 5)]) for i in range(len(t))]
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 2.6), sharex=True)
    lost_on = None
    for ax in axes:
        for r in rows:
            tt = float(r['t'])
            if not r['followed'] and lost_on is None: lost_on = tt
            if r['followed'] and lost_on is not None:
                if tt - lost_on > 3: ax.axvspan(lost_on, tt, color=C_GRAY, alpha=.25, lw=0)
                lost_on = None
        if lost_on is not None: ax.axvspan(lost_on, t[-1], color=C_GRAY, alpha=.25, lw=0); lost_on = None
    axes[0].plot(t, d2, color=C_OTHER, lw=1.5); axes[0].set_ylabel('通行人まで\n[m]'); axes[0].set_ylim(0, 5)
    axes[1].plot(t, turn, color=C_CHAIR, lw=1.5); axes[1].set_ylabel('車椅子の旋回\n[度/s]'); axes[1].set_ylim(0, 60)
    axes[1].set_xlabel('計測開始からの時間 [s](灰色は3秒以上の見失い)')
    axes[0].set_xlim(100, 187)
    save(fig, 'deadlock.svg')


# ---------- 服装違いのワールドの模式図 ----------
def fig_clothes_schematic():
    worlds = [('test', '元の条件', [], [(0.975, 1.325, '#cc0d0d')]),
              ('swap', '本人が赤', [(0.975, 1.325, '#cc0d0d')], []),
              ('high', '赤帯が肩寄り', [], [(1.20, 1.50, '#cc0d0d')]),
              ('pants', '赤帯がズボン', [], [(0.45, 0.85, '#cc0d0d')]),
              ('similar', '水色と紺', [(0.975, 1.325, '#408cf2')], [(0.975, 1.325, '#0d1a66')]),
              ('identical', '同じ服', [(0.975, 1.325, '#cc0d0d')], [(0.975, 1.325, '#cc0d0d')])]
    fig, ax = plt.subplots(figsize=(7.2, 2.6))
    for i, (key, title, me, ot) in enumerate(worlds):
        x0 = i * 1.6
        ax.axhspan(0.95, 1.35, xmin=0, xmax=1, color='#e4c5f0', alpha=.35, lw=0) if i == 0 else None
        for j, (bands, who) in enumerate([(me, '本人'), (ot, '通行人')]):
            cx = x0 + j * 0.62
            ax.add_patch(plt.Rectangle((cx - .14, 0.0), .28, 1.55, color='#d8d6cf', lw=0))  # 体
            ax.add_patch(plt.Circle((cx, 1.67), .11, color='#d8d6cf', lw=0))  # 頭
            for z0, z1, col in bands:
                ax.add_patch(plt.Rectangle((cx - .19, z0), .38, z1 - z0, color=col, lw=0))
            ax.text(cx, -0.13, who, ha='center', va='top', fontsize=7.5, color=C_SUB)
        ax.text(x0 + .31, 1.95, f'{key}\n{title}', ha='center', va='bottom', fontsize=8)
    ax.text(-0.45, 1.15, '色領域\n0.95〜1.35 m', ha='right', va='center', fontsize=7.5, color='#7a3d99')
    ax.set_xlim(-1.5, 9.2); ax.set_ylim(-0.4, 2.45); ax.set_aspect('equal'); ax.axis('off')
    save(fig, 'clothes.svg')


# ---------- 服装違いの色領域の切り抜き ----------
def fig_crops():
    import sys
    sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'tools', 'reid_eval'))
    from label import labeled
    worlds = ['test', 'swap', 'high', 'pants', 'similar', 'identical']
    fig, axes = plt.subplots(2, 6, figsize=(7.2, 2.7))
    for k, w in enumerate(worlds):
        d = os.path.join(M, 'collect', f'crowd_{w}')
        rows = [x for x in labeled(d) if not x['overlap'] and 1.4 <= x['dist'] <= 2.6]
        for r, lab in enumerate(('person', 'person2')):
            c = [x for x in rows if x['label2'] == lab]
            im = cv2.cvtColor(cv2.imread(f"{d}/{c[len(c) // 2]['name']}.png"), cv2.COLOR_BGR2RGB)
            ax = axes[r, k]; ax.imshow(im); ax.set_xticks([]); ax.set_yticks([])
            for sp in ax.spines.values(): sp.set_visible(False)
            if r == 0: ax.set_title(w, fontsize=9)
            if k == 0: ax.set_ylabel('本人' if r == 0 else '通行人', fontsize=9)
    save(fig, 'crops.svg')


# ---------- カメラ画像・Gazebo画面 ----------
def photos():
    cam = os.path.join(M, 'docshot', 'crowd_test', 'cam')
    for src, dst in (('00201.png', 'cam_two.png'), ('00003.png', 'cam_near.png')):
        Image.open(os.path.join(cam, src)).save(os.path.join(OUT, dst))
    gz = sorted(glob.glob(os.path.join(M, 'docshot', 'gz', '2026-10-20T08:03*.png')))[1]
    im = Image.open(gz); w, h = im.size
    im.crop((0, int(h * .17), w, int(h * .62))).save(os.path.join(OUT, 'gazebo.png'))
    print('wrote photos')


if __name__ == '__main__':
    fig_map(); fig_results(); fig_contam(); fig_simdist(); fig_sat()
    fig_deadlock(); fig_clothes_schematic(); fig_crops(); photos()
