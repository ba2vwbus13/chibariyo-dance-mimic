# 設計書・操作手順書の図（SVG）を作る。docs フォルダで python3 make_figures.py を実行すると img/ に書き出す
import math
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img')
FONT = "font-family:'Hiragino Sans','Hiragino Kaku Gothic ProN',sans-serif"
INK, SUB, LINE = '#1f2328', '#57606a', '#8c959f'
BLUE, BLUE_L = '#0b6aa8', '#dbeaf6'
TEAL, TEAL_L = '#0f6e56', '#d9f2ea'
AMBER, AMBER_L = '#854f0b', '#faeeda'
CORAL, CORAL_L = '#993c1d', '#fae6df'
GRAY_L = '#eef0f2'


def svg(name, w, h, body):
    s = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" style="{FONT}">\n'
         '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M1 1L9 5L1 9z" fill="{INK}"/></marker>'
         '<marker id="ab" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M1 1L9 5L1 9z" fill="{BLUE}"/></marker>'
         '<marker id="at" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M1 1L9 5L1 9z" fill="{TEAL}"/></marker>'
         '<marker id="aa" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
         f'<path d="M1 1L9 5L1 9z" fill="{AMBER}"/></marker></defs>\n'
         + body + '\n</svg>\n')
    with open(os.path.join(OUT, name), 'w', encoding='utf-8') as f:
        f.write(s)


def text(x, y, s, size=13, color=INK, anchor='middle', weight='normal'):
    return f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" text-anchor="{anchor}" dominant-baseline="central" font-weight="{weight}">{s}</text>'


def box(x, y, w, h, title, sub=None, fill=GRAY_L, stroke=LINE, tcolor=INK, rx=6, size=13):
    s = f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="1"/>'
    if sub:
        s += text(x + w / 2, y + h / 2 - 9, title, size, tcolor, weight='bold') + text(x + w / 2, y + h / 2 + 10, sub, 11, SUB)
    else:
        s += text(x + w / 2, y + h / 2, title, size, tcolor, weight='bold')
    return s


def arrow(x1, y1, x2, y2, color=INK, marker='a', dash=None, width=1.4):
    d = f' stroke-dasharray="{dash}"' if dash else ''
    return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}" marker-end="url(#{marker})"{d}/>'


def path_arrow(d, color=INK, marker='a', dash=None):
    ds = f' stroke-dasharray="{dash}"' if dash else ''
    return f'<path d="{d}" fill="none" stroke="{color}" stroke-width="1.4" marker-end="url(#{marker})"{ds}/>'


# ---------- 全体の流れ（設計書 図 1-1、操作手順書 図 1-1） ----------
def overview():
    b = ''
    b += box(20, 60, 130, 64, '人', '手の形を見せる', fill='#fff', stroke=LINE)
    b += box(190, 60, 130, 64, 'カメラ', 'Studio Display', fill=BLUE_L, stroke=BLUE, tcolor=BLUE)
    b += f'<rect x="360" y="40" width="150" height="104" rx="6" fill="{BLUE_L}" stroke="{BLUE}"/>'
    b += text(435, 60, 'Mac', 13, BLUE, weight='bold')
    b += text(435, 86, '手の検出', 11, SUB) + text(435, 104, '形の判定', 11, SUB) + text(435, 122, '速度の指令', 11, SUB)
    b += box(550, 60, 130, 64, 'ロボット', '走る・回る・踊る', fill=TEAL_L, stroke=TEAL, tcolor=TEAL)
    b += arrow(150, 92, 188, 92) + arrow(320, 92, 358, 92) + arrow(510, 92, 548, 92, TEAL, 'at')
    b += text(169, 76, '映る', 11, SUB) + text(339, 76, '映像', 11, SUB) + text(529, 76, 'Wi-Fi', 11, TEAL)
    svg('fig_overview.svg', 700, 170, b)


# ---------- 機器とネットワーク（設計書 図 2-1） ----------
def network():
    b = ''
    b += box(250, 110, 200, 90, 'Mac', 'Mac Studio', fill=BLUE_L, stroke=BLUE, tcolor=BLUE)
    b += box(270, 10, 160, 52, 'カメラ', 'Studio Display', fill='#fff')
    b += arrow(350, 62, 350, 108)
    b += box(20, 125, 160, 60, '学校の LAN', 'インターネット', fill='#fff')
    b += f'<line x1="180" y1="155" x2="248" y2="155" stroke="{INK}" stroke-width="2.5"/>'
    b += text(214, 140, '有線', 11, SUB)
    b += box(520, 125, 160, 60, '研究用ルータ', '&lt;研究用ルータのSSID&gt;', fill=TEAL_L, stroke=TEAL, tcolor=TEAL)
    b += f'<line x1="450" y1="155" x2="518" y2="155" stroke="{TEAL}" stroke-width="2" stroke-dasharray="5 4"/>'
    b += text(484, 140, 'Wi-Fi', 11, TEAL)
    b += box(520, 250, 160, 60, 'ロボット', 'chibamouse1', fill=TEAL_L, stroke=TEAL, tcolor=TEAL)
    b += f'<line x1="600" y1="185" x2="600" y2="248" stroke="{TEAL}" stroke-width="2" stroke-dasharray="5 4"/>'
    b += text(640, 217, 'Wi-Fi', 11, TEAL)
    b += text(350, 215, '&lt;ルータが配るIP&gt;（自動）', 11, SUB)
    b += text(600, 325, '&lt;ロボットのIP&gt;', 11, SUB)
    b += f'<rect x="500" y="100" width="200" height="240" rx="10" fill="none" stroke="{TEAL}" stroke-dasharray="4 4"/>'
    b += text(600, 352, '研究用のネットワーク（2.4 GHz）', 11, TEAL)
    svg('fig_network.svg', 720, 370, b)


# ---------- ソフトウェアの構成（設計書 図 2-2） ----------
def software():
    b = ''
    b += f'<rect x="10" y="10" width="420" height="330" rx="10" fill="#f6f8fa" stroke="{BLUE}"/>'
    b += text(30, 30, 'Mac（macOS）', 13, BLUE, 'start', 'bold')
    b += box(30, 48, 380, 44, 'ブラウザ（Chrome）', 'カメラページ camera.html', fill='#fff')
    b += f'<rect x="30" y="108" width="380" height="216" rx="8" fill="{BLUE_L}" stroke="{BLUE}"/>'
    b += text(45, 126, 'コンテナ finger_dance（Ubuntu 24.04、ROS 2 Jazzy）', 12, BLUE, 'start', 'bold')
    b += box(45, 142, 170, 40, 'rosbridge_websocket', fill='#fff', size=12)
    b += box(225, 142, 170, 40, 'rqt_image_view', fill='#fff', size=12)
    b += box(45, 192, 170, 40, 'finger_detect', fill='#fff', size=12)
    b += box(225, 192, 170, 40, 'finger_dance', fill='#fff', size=12)
    b += box(45, 252, 350, 52, 'Zenoh ルータ（rmw_zenohd）', 'ロボットの Zenoh ルータにつなぐ', fill='#fff', size=12)
    b += f'<rect x="490" y="10" width="220" height="330" rx="10" fill="#f6f8fa" stroke="{TEAL}"/>'
    b += text(505, 30, 'ロボット（Raspberry Pi 3）', 13, TEAL, 'start', 'bold')
    b += text(505, 48, 'Ubuntu 24.04、ROS 2 Jazzy', 11, SUB, 'start')
    b += box(505, 252, 190, 52, 'Zenoh ルータ', '電源を入れると起動', fill=TEAL_L, stroke=TEAL, tcolor=TEAL, size=12)
    b += box(505, 162, 190, 52, 'raspimouse ノード', '/cmd_vel を受け取る', fill=TEAL_L, stroke=TEAL, tcolor=TEAL, size=12)
    b += box(505, 72, 190, 52, 'ドライバ rtmouse', 'モータ・光センサ', fill='#fff', size=12)
    b += f'<line x1="395" y1="278" x2="503" y2="278" stroke="{TEAL}" stroke-width="2" stroke-dasharray="5 4"/>'
    b += text(449, 264, 'Wi-Fi', 11, TEAL) + text(449, 292, 'TCP 7447', 11, SUB)
    b += arrow(600, 160, 600, 126) + arrow(600, 250, 600, 216)
    svg('fig_software.svg', 720, 350, b)


# ---------- ノードとトピック（設計書 図 3-1） ----------
def nodes():
    b = ''
    b += box(10, 30, 120, 52, 'カメラページ', 'ブラウザ', fill='#fff')
    b += box(170, 30, 150, 52, 'rosbridge', 'ブラウザとの橋渡し', fill=BLUE_L, stroke=BLUE, tcolor=BLUE)
    b += box(170, 140, 150, 52, 'finger_detect', '手の検出・判定', fill=BLUE_L, stroke=BLUE, tcolor=BLUE)
    b += box(420, 140, 150, 52, 'finger_dance', '速度の指令', fill=BLUE_L, stroke=BLUE, tcolor=BLUE)
    b += box(420, 260, 150, 52, 'raspimouse', 'ロボット', fill=TEAL_L, stroke=TEAL, tcolor=TEAL)
    b += box(170, 260, 150, 52, 'rqt_image_view', '検出画面', fill='#fff')
    b += box(10, 260, 120, 52, 'ros2 bag', '記録するとき', fill='#fff')
    b += arrow(130, 56, 168, 56)
    b += text(149, 18, 'WebSocket', 10, SUB) + text(149, 30, '9091', 10, SUB)
    b += arrow(245, 82, 245, 138)
    b += text(250, 110, '/image_raw/compressed', 11, BLUE, 'start')
    b += arrow(320, 166, 418, 166)
    b += text(369, 152, '/finger_gesture', 11, BLUE)
    b += arrow(245, 192, 245, 258)
    b += text(250, 225, '/finger_image', 11, BLUE, 'start')
    b += path_arrow('M200 192 L200 222 L70 222 L70 258', SUB, 'a', '4 3')
    b += text(12, 240, '/finger_image/compressed', 10, SUB, 'start')
    b += arrow(495, 192, 495, 258, TEAL, 'at')
    b += text(503, 216, '/cmd_vel', 11, TEAL, 'start') + text(503, 234, '/motor_power', 11, TEAL, 'start')
    b += f'<rect x="400" y="245" width="190" height="80" rx="8" fill="none" stroke="{TEAL}" stroke-dasharray="4 4"/>'
    b += text(610, 285, 'Wi-Fi の先', 11, TEAL, 'start')
    svg('fig_nodes.svg', 690, 335, b)


# ---------- 手の 21 点（設計書 図 4-2） ----------
def hand21():
    b = ''
    wrist = (200, 330)
    fingers = {  # 付け根 → 指先（手のひら側から見た右手を簡単にした形）
        'thumb': [(165, 300), (130, 270), (108, 240), (92, 214)],
        'index': [(150, 215), (145, 170), (142, 140), (140, 112)],
        'middle': [(190, 208), (190, 158), (190, 124), (190, 94)],
        'ring': [(230, 212), (234, 166), (237, 134), (240, 106)],
        'pinky': [(265, 225), (275, 190), (281, 166), (286, 144)],
    }
    num = {'thumb': 1, 'index': 5, 'middle': 9, 'ring': 13, 'pinky': 17}
    b += f'<path d="M{wrist[0]} {wrist[1]} L165 300 L150 215 L190 208 L230 212 L265 225 Z" fill="#f3e3d3" stroke="none"/>'
    for f, pts in fingers.items():
        prev = wrist
        for p in pts:
            b += f'<line x1="{prev[0]}" y1="{prev[1]}" x2="{p[0]}" y2="{p[1]}" stroke="#7d9" stroke-width="3"/>'
            prev = p
    for a, c in [((150, 215), (190, 208)), ((190, 208), (230, 212)), ((230, 212), (265, 225))]:
        b += f'<line x1="{a[0]}" y1="{a[1]}" x2="{c[0]}" y2="{c[1]}" stroke="#7d9" stroke-width="3"/>'
    used_pip = {6, 10, 14, 18}
    used_tip = {8, 12, 16, 20}
    b += f'<circle cx="{wrist[0]}" cy="{wrist[1]}" r="8" fill="{CORAL}"/>' + text(wrist[0] + 18, wrist[1] + 4, '0 手首', 12, CORAL, 'start', 'bold')
    for f, pts in fingers.items():
        for i, p in enumerate(pts):
            n = num[f] + i
            if n in used_tip:
                col, r = BLUE, 7
            elif n in used_pip:
                col, r = AMBER, 7
            elif n == 5:
                col, r = TEAL, 7
            else:
                col, r = '#9aa', 4.5
            b += f'<circle cx="{p[0]}" cy="{p[1]}" r="{r}" fill="{col}"/>'
            dx = -14 if f in ('thumb', 'index') else 14
            b += text(p[0] + dx, p[1], str(n), 10, SUB)
    # 凡例
    lx = 360
    for i, (col, s) in enumerate([(CORAL, '0：手首（距離を測る基準）'), (AMBER, '6・10・14・18：第 2 関節'),
                                  (BLUE, '8・12・16・20：指先'), (TEAL, '5：人差し指の付け根（向きの基準）'),
                                  ('#9aa', 'その他の点（親指 1〜4 は使わない）')]):
        b += f'<circle cx="{lx}" cy="{120 + i * 34}" r="7" fill="{col}"/>' + text(lx + 16, 120 + i * 34, s, 12, INK, 'start')
    svg('fig_hand21.svg', 680, 350, b)


# ---------- 指を伸ばしているかの判定（設計書 図 4-3） ----------
def extension():
    b = ''
    for k, (ox, title, tip, pip, ok) in enumerate([
            (40, '伸ばしている指', (110, 40), (110, 120), True),
            (360, '曲げている指', (440, 185), (430, 130), False)]):
        wrist = (ox + 70, 300)
        mcp = (ox + 70, 190)
        b += text(ox + 140, 20, title, 13, INK, weight='bold')
        if ok:
            pts = [wrist, mcp, (ox + 70, 120), (ox + 70, 75), (ox + 70, 40)]
        else:
            pts = [wrist, mcp, (ox + 75, 140), (ox + 105, 150), (ox + 110, 185)]
        for a, c in zip(pts, pts[1:]):
            b += f'<line x1="{a[0]}" y1="{a[1]}" x2="{c[0]}" y2="{c[1]}" stroke="#d9b38c" stroke-width="16" stroke-linecap="round"/>'
        w, p, t = pts[0], pts[2], pts[4]
        b += f'<circle cx="{w[0]}" cy="{w[1]}" r="7" fill="{CORAL}"/>' + text(w[0] - 14, w[1], '0', 11, CORAL)
        b += f'<circle cx="{p[0]}" cy="{p[1]}" r="6" fill="{AMBER}"/>'
        b += f'<circle cx="{t[0]}" cy="{t[1]}" r="6" fill="{BLUE}"/>'
        b += f'<line x1="{w[0]}" y1="{w[1]}" x2="{t[0]}" y2="{t[1]}" stroke="{BLUE}" stroke-width="2" stroke-dasharray="6 3"/>'
        b += f'<line x1="{w[0]}" y1="{w[1]}" x2="{p[0]}" y2="{p[1]}" stroke="{AMBER}" stroke-width="2" stroke-dasharray="2 3"/>'
        b += text(ox + 200, 90, '青の点線 = 手首〜指先', 11, BLUE, 'start') if False else ''
        b += text(ox + 175, 230, '手首〜指先', 11, BLUE, 'start') + text(ox + 175, 250, '手首〜第 2 関節', 11, AMBER, 'start')
        b += f'<line x1="{ox + 150}" y1="230" x2="{ox + 170}" y2="230" stroke="{BLUE}" stroke-width="2" stroke-dasharray="6 3"/>'
        b += f'<line x1="{ox + 150}" y1="250" x2="{ox + 170}" y2="250" stroke="{AMBER}" stroke-width="2" stroke-dasharray="2 3"/>'
        dt = math.dist(w, t)
        dp = math.dist(w, p)
        b += text(ox + 140, 330, f'{dt:.0f} ÷ {dp:.0f} = {dt / dp:.2f}', 12, INK)
        b += text(ox + 140, 350, '1.1 より大きい → 伸ばしている' if ok else '1.1 以下 → 曲げている', 12, TEAL if ok else CORAL, weight='bold')
    svg('fig_extension.svg', 680, 365, b)


# ---------- 指の向きの角度（設計書 図 4-4） ----------
def angle_def():
    b = ''
    cx, cy, r = 220, 175, 125
    b += f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="none" stroke="{LINE}" stroke-dasharray="3 4"/>'
    for deg, lab in [(0, '0°（真上）'), (90, '+90°'), (-90, '−90°'), (180, '±180°（真下）')]:
        a = math.radians(deg)
        x, y = cx - r * math.sin(a), cy - r * math.cos(a)   # + は本人から見て左 = 図の左に描く
        b += f'<line x1="{cx}" y1="{cy}" x2="{x:.1f}" y2="{y:.1f}" stroke="{LINE}"/>'
        ox = {0: 0, 90: -26, -90: 26, 180: 0}[deg]
        oy = {0: -16, 90: 0, -90: 0, 180: 16}[deg]
        b += text(x + ox, y + oy, lab, 12, INK)
    a = math.radians(30)
    x, y = cx - 115 * math.sin(a), cy - 115 * math.cos(a)
    b += f'<line x1="{cx}" y1="{cy}" x2="{x:.1f}" y2="{y:.1f}" stroke="{TEAL}" stroke-width="5" marker-end="url(#at)"/>'
    b += f'<path d="M{cx} {cy - 50} A50 50 0 0 0 {cx - 50 * math.sin(a):.1f} {cy - 50 * math.cos(a):.1f}" fill="none" stroke="{TEAL}" stroke-width="1.5"/>'
    b += text(cx + 10, cy - 80, 'θ = +30°', 12, TEAL, 'start', 'bold')
    b += f'<circle cx="{cx}" cy="{cy}" r="6" fill="{TEAL}"/>' + text(cx + 30, cy + 48, '人差し指の付け根', 11, SUB, 'start')
    b += text(cx - r, cy + 22, '本人から見て左', 12, BLUE) + text(cx + r, cy + 22, '本人から見て右', 12, BLUE)
    # 右側：カメラの画像との関係
    b += box(450, 60, 230, 60, 'カメラの画像', '本人の左は画像の右に写る', fill='#fff')
    b += box(450, 160, 230, 60, '鏡に映した見え方', '本人の左は左に見える', fill='#fff')
    b += text(565, 250, 'θ は本人から見た向きで決める', 12, INK)
    svg('fig_angle.svg', 700, 330, b)


# ---------- 指の向きと速度（設計書 図 5-1、操作手順書 図 4-2） ----------
def dial(name='fig_dial.svg'):
    b = ''
    cx, cy, R, r0 = 250, 230, 190, 40
    regions = [  # (始まり, 終わり, 色, 名前)  θ は + が左。図では左側に描く
        (-10, 10, BLUE_L, '前進'), (10, 70, '#cfe3f3', '前進＋左旋回'), (70, 110, TEAL_L, '左旋回'),
        (110, 170, '#e3dcf3', '後退＋左旋回'), (170, 190, CORAL_L, '後退'),
        (-70, -10, '#cfe3f3', '前進＋右旋回'), (-110, -70, TEAL_L, '右旋回'), (-170, -110, '#e3dcf3', '後退＋右旋回')]

    def pt(deg, rr):
        a = math.radians(deg)
        return cx - rr * math.sin(a), cy - rr * math.cos(a)
    for a0, a1, col, lab in regions:
        p0, p1 = pt(a0, R), pt(a1, R)
        q0, q1 = pt(a1, r0), pt(a0, r0)
        large = 1 if (a1 - a0) > 180 else 0
        b += (f'<path d="M{p0[0]:.1f} {p0[1]:.1f} A{R} {R} 0 {large} 0 {p1[0]:.1f} {p1[1]:.1f} L{q0[0]:.1f} {q0[1]:.1f} '
              f'A{r0} {r0} 0 {large} 1 {q1[0]:.1f} {q1[1]:.1f} Z" fill="{col}" stroke="#fff" stroke-width="2"/>')
        m = (a0 + a1) / 2
        tx, ty = pt(m, (R + r0) / 2 + (18 if abs(a1 - a0) < 30 else 0))
        b += text(tx, ty, lab, 12 if abs(a1 - a0) >= 30 else 11, INK, weight='bold')
    for deg in (0, 10, 70, 90, 110, 170, 180, -10, -70, -90, -110, -170):
        x, y = pt(deg, R + 16)
        lab = ('+' if 0 < deg < 180 else '') + (f'{deg}°' if deg != 180 else '±180°').replace('-', '−')
        b += text(x, y, lab, 10, SUB)
    b += f'<circle cx="{cx}" cy="{cy}" r="{r0 - 6}" fill="#fff" stroke="{LINE}"/>' + text(cx, cy, '付け根', 10, SUB)
    b += text(cx - R - 30, cy, '本人から見て左', 11, BLUE, 'end') if False else ''
    # 右側：式
    x0 = 480
    b += text(x0, 120, '前後の速さ  v = 0.05 × cos θ  [m/s]', 13, INK, 'start')
    b += text(x0, 150, '旋回の速さ  ω = 1.0 × sin θ  [rad/s]', 13, INK, 'start')
    b += text(x0, 190, '真上・真下から ±10° 以内 → ω = 0', 12, SUB, 'start')
    b += text(x0, 214, '真横から ±20° 以内 → v = 0', 12, SUB, 'start')
    b += text(x0, 254, 'θ の + は本人から見て左', 12, SUB, 'start')
    b += text(x0, 278, 'ω の + は左回り（反時計回り）', 12, SUB, 'start')
    svg(name, 800, 460, b)


# ---------- 踊りの角速度と向き（設計書 図 5-2） ----------
def dance():
    b = ''
    x0, y0, W = 70, 30, 560
    T = 2.0
    sx = W / T
    # 角速度
    b += text(30, y0 + 50, 'ω', 14, INK, weight='bold')
    b += f'<line x1="{x0}" y1="{y0 + 50}" x2="{x0 + W + 10}" y2="{y0 + 50}" stroke="{LINE}"/>'
    pts = []
    t = 0.0
    while t < T:
        left = ((t / 0.4) + 0.5) % 2.0 < 1.0
        pts.append((t, 1 if left else -1))
        t += 0.005
    d = ''
    for i, (t, v) in enumerate(pts):
        X, Y = x0 + t * sx, y0 + 50 - v * 35
        if i == 0:
            d = f'M{X:.1f} {Y:.1f}'
        else:
            if v != pts[i - 1][1]:
                d += f' L{X:.1f} {y0 + 50 - pts[i - 1][1] * 35:.1f}'
            d += f' L{X:.1f} {Y:.1f}'
    b += f'<path d="{d}" fill="none" stroke="{TEAL}" stroke-width="2"/>'
    b += text(x0 - 8, y0 + 15, '+1.0', 10, SUB, 'end') + text(x0 - 8, y0 + 85, '−1.0', 10, SUB, 'end')
    b += text(x0 + W + 14, y0 + 15, '左回り', 11, TEAL, 'start') + text(x0 + W + 14, y0 + 85, '右回り', 11, TEAL, 'start')
    # 向き
    yb = y0 + 190
    b += text(30, yb, '向き', 13, INK, weight='bold')
    b += f'<line x1="{x0}" y1="{yb}" x2="{x0 + W + 10}" y2="{yb}" stroke="{LINE}"/>'
    yaw, d = 0.0, ''
    for i, (t, v) in enumerate(pts):
        yaw += v * 1.0 * 0.005
        X, Y = x0 + t * sx, yb - math.degrees(yaw) * 3.2
        d += (f'M{X:.1f} {Y:.1f}' if i == 0 else f' L{X:.1f} {Y:.1f}')
    b += f'<path d="{d}" fill="none" stroke="{BLUE}" stroke-width="2"/>'
    b += text(x0 - 8, yb - 36.7, '+11.5°', 10, SUB, 'end') + text(x0 - 8, yb + 36.7, '−11.5°', 10, SUB, 'end')
    for tt in (0, 0.2, 0.6, 1.0, 1.4, 1.8):
        X = x0 + tt * sx
        b += f'<line x1="{X:.1f}" y1="{y0}" x2="{X:.1f}" y2="{yb + 50}" stroke="{LINE}" stroke-dasharray="2 4"/>'
        b += text(X, yb + 62, f'{tt:g}', 10, SUB)
    b += text(x0 + W / 2, yb + 82, '踊り始めてからの時間 [s]', 11, SUB)
    svg('fig_dance.svg', 700, 320, b)


# ---------- 走る・止まるの切り替え（設計書 図 5-3） ----------
def states():
    b = ''
    b += box(250, 20, 180, 60, '停止', 'v = 0、ω = 0', fill=CORAL_L, stroke=CORAL, tcolor=CORAL)
    b += box(40, 210, 200, 60, '走る', '指の向きで v・ω', fill=BLUE_L, stroke=BLUE, tcolor=BLUE)
    b += box(440, 210, 200, 60, '踊る', '左右に小刻みに振る', fill=TEAL_L, stroke=TEAL, tcolor=TEAL)
    b += path_arrow('M260 80 L150 208', BLUE, 'ab')
    b += text(118, 130, '人差し指を', 11, BLUE, 'start') + text(118, 146, '2 回続けて検出', 11, BLUE, 'start')
    b += path_arrow('M420 80 L530 208', TEAL, 'at')
    b += text(500, 130, 'チョキを', 11, TEAL, 'start') + text(500, 146, '2 回続けて検出', 11, TEAL, 'start')
    b += path_arrow('M190 208 L300 82', CORAL, 'a')
    b += path_arrow('M490 208 L380 82', CORAL, 'a')
    b += text(340, 140, 'グー・パー・その他・手なし', 11, CORAL) + text(340, 156, '→ すぐに停止', 11, CORAL)
    b += text(340, 184, '検出結果が 0.5 秒届かない → 停止', 11, CORAL)
    b += path_arrow('M240 250 L438 250', INK, 'a', '4 3')
    b += text(340, 236, 'チョキを 2 回', 10, SUB) + text(340, 266, '人差し指を 2 回', 10, SUB)
    b += path_arrow('M438 258 L242 258', INK, 'a', '4 3')
    svg('fig_states.svg', 680, 290, b)


# ---------- 操作の流れ（操作手順書 図 2-1） ----------
def flow():
    steps = [('ロボットの電源', '3.1'), ('Mac の Wi-Fi', '3.2'), ('ロボットのノード', '3.3'),
             ('指ダンスの起動', '3.4'), ('カメラページ', '3.5'), ('指で操作', '4 章'), ('終了', '5 章')]
    b = ''
    w, gap = 86, 12
    for i, (s, ref) in enumerate(steps):
        x = 10 + i * (w + gap)
        col, stc, tc = (TEAL_L, TEAL, TEAL) if i in (0, 2) else (BLUE_L, BLUE, BLUE)
        if i == 6:
            col, stc, tc = (GRAY_L, LINE, INK)
        b += f'<rect x="{x}" y="20" width="{w}" height="70" rx="6" fill="{col}" stroke="{stc}"/>'
        b += text(x + w / 2, 42, f'{i + 1}', 15, tc, weight='bold')
        b += text(x + w / 2, 64, s, 10.5, tc, weight='bold')
        b += text(x + w / 2, 104, ref, 10, SUB)
        if i < len(steps) - 1:
            b += arrow(x + w, 55, x + w + gap - 1, 55)
    b += f'<rect x="10" y="125" width="14" height="14" fill="{TEAL_L}" stroke="{TEAL}"/>' + text(30, 132, 'ロボットでの作業', 11, INK, 'start')
    b += f'<rect x="150" y="125" width="14" height="14" fill="{BLUE_L}" stroke="{BLUE}"/>' + text(170, 132, 'Mac での作業', 11, INK, 'start')
    svg('fig_flow.svg', 700, 150, b)



# ---------- 標準の通信方式が届かない理由（設計書 図 2-3） ----------
def dds_blocked():
    b = ''
    b += f'<rect x="10" y="10" width="400" height="250" rx="10" fill="#f6f8fa" stroke="{BLUE}"/>'
    b += text(25, 30, 'Mac（&lt;ルータが配るIP&gt;）', 13, BLUE, 'start', 'bold')
    b += f'<rect x="30" y="50" width="360" height="190" rx="8" fill="{BLUE_L}" stroke="{BLUE}" stroke-dasharray="5 3"/>'
    b += text(45, 68, 'Docker Desktop の Linux（仮想マシン）', 12, BLUE, 'start', 'bold')
    b += f'<rect x="60" y="90" width="200" height="110" rx="6" fill="#fff" stroke="{BLUE}"/>'
    b += text(160, 115, 'コンテナ', 13, BLUE, weight='bold')
    b += text(160, 145, 'IP は 172.17.0.x', 11, SUB) + text(160, 165, 'Mac の外からは見えない', 11, SUB)
    # 探す声（マルチキャスト）
    for r in (30, 50, 70):
        b += f'<path d="M{260 + r * 0.35:.0f} {150 - r * 0.9:.0f} A{r} {r} 0 0 1 {260 + r * 0.35:.0f} {150 + r * 0.9:.0f}" fill="none" stroke="{CORAL}" stroke-width="2"/>'
    b += f'<line x1="372" y1="96" x2="388" y2="112" stroke="{CORAL}" stroke-width="4"/><line x1="388" y1="96" x2="372" y2="112" stroke="{CORAL}" stroke-width="4"/>'
    b += text(210, 222, '「誰かいますか」が仮想マシンの外へ出ない', 11, CORAL)
    b += f'<line x1="410" y1="135" x2="488" y2="135" stroke="{TEAL}" stroke-width="2" stroke-dasharray="5 4"/>'
    b += text(449, 120, 'Wi-Fi', 11, TEAL)
    b += box(490, 90, 200, 90, 'ロボット', '&lt;ロボットのIP&gt;', fill=TEAL_L, stroke=TEAL, tcolor=TEAL)
    b += text(590, 205, 'コンテナを見つけられない', 12, CORAL, weight='bold')
    svg('fig_dds_blocked.svg', 700, 270, b)


# ---------- Zenoh ルータによる接続（設計書 図 2-4） ----------
def zenoh_route():
    b = ''
    b += f'<rect x="10" y="10" width="380" height="300" rx="10" fill="{BLUE_L}" stroke="{BLUE}"/>'
    b += text(25, 30, 'コンテナ', 13, BLUE, 'start', 'bold')
    for i, n in enumerate(['rosbridge', 'finger_detect', 'finger_dance', 'rqt_image_view']):
        x = 22 + i * 92
        b += box(x, 50, 86, 40, n, fill='#fff', size=10.5)
        if i in (0, 2):
            b += f'<line x1="{x + 43}" y1="90" x2="{195 + (i - 1.5) * 40:.0f}" y2="188" stroke="{BLUE}" stroke-width="1.2"/>'
    # 検出画面の画像の道筋：finger_detect → Zenoh ルータ → rqt_image_view（どちらもコンテナの中）
    b += f'<line x1="157" y1="92" x2="175" y2="186" stroke="{AMBER}" stroke-width="3" marker-end="url(#aa)"/>'
    b += f'<line x1="215" y1="188" x2="341" y2="94" stroke="{AMBER}" stroke-width="3" marker-end="url(#aa)"/>'
    b += box(90, 190, 210, 60, 'Zenoh ルータ', 'ノードはここにだけつながる', fill='#fff', stroke=BLUE, tcolor=BLUE)
    b += text(278, 168, '検出画面の画像', 11, AMBER, 'start', 'bold') + text(278, 184, '約 9 MB/秒', 11, AMBER, 'start')
    b += text(200, 272, '検出画面の画像は finger_detect から rqt_image_view へ', 11, AMBER)
    b += text(200, 290, 'コンテナの中だけで渡し、ロボットへは送らない', 11, AMBER)
    b += f'<rect x="530" y="10" width="180" height="300" rx="10" fill="{TEAL_L}" stroke="{TEAL}"/>'
    b += text(545, 30, 'ロボット', 13, TEAL, 'start', 'bold')
    b += box(545, 50, 150, 40, 'raspimouse', fill='#fff', size=12)
    b += f'<line x1="620" y1="90" x2="620" y2="188" stroke="{TEAL}" stroke-width="1.2"/>'
    b += box(545, 190, 150, 60, 'Zenoh ルータ', 'TCP 7447 で待つ', fill='#fff', stroke=TEAL, tcolor=TEAL)
    b += arrow(300, 220, 543, 220, TEAL, 'at', width=2.2)
    b += text(460, 158, '① コンテナから', 11, TEAL) + text(460, 174, 'つなぎに行く', 11, TEAL)
    b += text(460, 205, 'tcp/&lt;ロボットのIP&gt;:7447', 9.5, SUB)
    b += text(460, 246, '② ロボットへ送るのは', 11, TEAL) + text(460, 262, '速度の指令などだけ', 11, TEAL)
    svg('fig_zenoh_route.svg', 720, 320, b)


# ---------- robot_connect の働き（設計書 図 2-5） ----------
def connect_steps():
    steps = [('1', 'つなぎ先を保存', 'ファイル ~/.robot_ip に書く'),
             ('2', 'Zenoh ルータを起動', 'ロボットの 7447 番につなぐ'),
             ('3', '通信方式を切り替え', 'ノードが rmw_zenoh を使う'),
             ('4', '新しい端末にも反映', '開くたびに 1 を読んで 3 を行う')]
    b = text(350, 18, 'robot_connect &lt;ロボットのIP&gt; を実行すると', 13, INK, weight='bold')
    w, gap = 158, 14
    for i, (n, t, sub) in enumerate(steps):
        x = 10 + i * (w + gap)
        b += f'<rect x="{x}" y="36" width="{w}" height="78" rx="6" fill="{BLUE_L}" stroke="{BLUE}"/>'
        b += text(x + w / 2, 54, n, 14, BLUE, weight='bold')
        b += text(x + w / 2, 76, t, 12, BLUE, weight='bold')
        b += text(x + w / 2, 98, sub, 9.5, SUB)
        if i < 3:
            b += arrow(x + w, 75, x + w + gap - 1, 75)
    # ROS_DOMAIN_ID
    y = 150
    b += text(350, y, 'ROS_DOMAIN_ID が同じノードどうしだけが通信する', 13, INK, weight='bold')
    b += box(60, y + 20, 170, 50, 'finger_dance', 'ROS_DOMAIN_ID 30', fill=BLUE_L, stroke=BLUE, tcolor=BLUE, size=12)
    b += box(470, y + 20, 170, 50, 'raspimouse', 'ROS_DOMAIN_ID 30', fill=TEAL_L, stroke=TEAL, tcolor=TEAL, size=12)
    b += arrow(230, y + 45, 468, y + 45, TEAL, 'at', width=2)
    b += text(349, y + 33, '/cmd_vel が届く', 11, TEAL)
    b += box(60, y + 90, 170, 50, 'ほかのノード', 'ROS_DOMAIN_ID 0', fill='#fff', size=12)
    b += f'<line x1="230" y1="{y + 115}" x2="440" y2="{y + 115}" stroke="{LINE}" stroke-width="2" stroke-dasharray="5 4"/>'
    b += f'<line x1="440" y1="{y + 105}" x2="456" y2="{y + 125}" stroke="{CORAL}" stroke-width="3"/><line x1="456" y1="{y + 105}" x2="440" y2="{y + 125}" stroke="{CORAL}" stroke-width="3"/>'
    b += text(335, y + 103, '番号が違うので届かない', 11, CORAL)
    svg('fig_connect.svg', 700, 300, b)

if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for f in (overview, network, software, nodes, hand21, extension, angle_def, dial, dance, states, flow,
              dds_blocked, zenoh_route, connect_steps):
        f()
    print('ok')
