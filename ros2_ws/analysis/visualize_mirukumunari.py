#!/usr/bin/env python3
"""
mirukumunari.mp4（エイサーのお手本）の骨格の動きと、
第1段階（骨格推定 → WHILLが動く値：物真似の決まり）の動きを
media_Tango_003_020.mp4 と同じレイアウトで横並び視覚化するスクリプト。

出力:
  1. mirukumunari_whill_stage1_30s.mp4  (一番動きが豊富な 15.0s〜45.0s の30秒間ハイライト動画)
  2. mirukumunari_whill_stage1_full.mp4 (動画全体のダイジェスト / フル視覚化動画)
  3. 更新版 mirukumunari_dance.csv      (1回転ターン・出力強化後の第1段階WHILL値を反映。読んだ CSV に上書きする)

  python ros2_ws/analysis/visualize_mirukumunari.py
  python ros2_ws/analysis/visualize_mirukumunari.py --csv お手本.csv --out-dir 出力先 --copy-to 写す先
  --csv を付けなければ、このスクリプトのフォルダの mirukumunari_dance.csv（offline_dance.py で作る）
"""

import argparse
import csv
import math
import os
import shutil
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))


def get_jp_font(size, bold=False):
    candidates = [
        r"C:\Windows\Fonts\YuGothB.ttc" if bold else r"C:\Windows\Fonts\YuGothM.ttc",
        r"C:\Windows\Fonts\meiryob.ttc" if bold else r"C:\Windows\Fonts\meiryo.ttc",
        r"C:\Windows\Fonts\msgothic.ttc",
        "/System/Library/Fonts/ヒラギノ角ゴシック W6.ttc" if bold else "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc" if bold else "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    ]
    for path in candidates:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                pass
    return ImageFont.load_default()


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def recompute_stage1_and_teacher(rows):
    """
    mirukumunari_dance.csv の骨格データから:
      1) お手本（骨格の連続的な動き: 体の向き・前後ステップ・横揺れ）の2D軌跡 (tx, ty, th)
      2) 第1段階（物真似の決まり: しこ立ち・ターン1回転・足上げ・ステップ・手上げ → Follower → WHILL）の
         強出力設定での動き (s1_x, s1_y, s1_h, whill_front_pct, whill_side_pct, action_name)
    を計算する。
    """
    # 第1段階（手の動き判定なし・足上げ＝右/左斜め前に前進・連続ステップ対応・人の向き追従）
    STEP_DIST = 0.30
    STEP_HOLD = 0.40
    STEP_PULLBACK = 0.22
    STEP_RETURN = 0.35
    LIFT_ANGLE = 38.0
    LIFT_DIST = 0.28
    LIFT_HOLD = 0.38
    LIFT_RETURN = 0.32
    TURN_HOLD = 0.80

    HEADING_GAIN = 16.0
    FORWARD_GAIN = 12.0
    LINEAR_MAX = 0.60
    ANGULAR_MAX = 8.0
    FRONT_CAP = 55.0
    SIDE_CAP = 70.0
    BACK_GAIN = 1.3
    BACK_CAP = 70.0
    RAMP = 500.0

    mode = "idle"
    mode_timer = 0.0
    cooldown = 0.0
    r_step_count = 0
    l_step_count = 0
    turn_confirm = 0
    last_front_yaw = 0.0
    yaw_s = 0.0
    in_turn = False

    fol_h = 0.0
    fol_f = 0.0
    whill_front = 0.0
    whill_side = 0.0

    # 第1段階の2D平面上の位置（見やすいように前後移動をGazebo相当のスケールで表示）
    s1_x = 0.0
    s1_y = 0.0
    s1_h = 0.0

    # お手本（骨格から直接計算した連続的な動き）の平滑化状態
    hip_x0 = None
    hip_y0 = None
    t_x = 0.0
    t_y = 0.0
    t_h = 0.0

    enriched = []
    prev_t = None

    for r in rows:
        t = float(r["t"])
        dt = 0.1 if prev_t is None else clip(t - prev_t, 0.02, 0.3)
        prev_t = t

        ok = int(r["ok"]) == 1
        raw_yaw = float(r["yaw"]) if (ok and r["yaw"] != "") else 0.0
        dproj_a = float(r["dproj_a"]) if (ok and r["dproj_a"] != "") else 0.0
        squat_flag = int(r.get("squat", 0) or 0) == 1
        l_lift_flag = int(r.get("l_knee_lift", 0) or 0) == 1
        r_lift_flag = int(r.get("r_knee_lift", 0) or 0) == 1
        r_step_flag = int(r.get("r_step_fwd", 0) or 0) == 1
        l_step_flag = int(r.get("l_step_back", 0) or 0) == 1
        l_up = int(r.get("l_up", 0) or 0) == 1
        r_up = int(r.get("r_up", 0) or 0) == 1

        hip_x = float(r["hip_x"]) if (ok and r["hip_x"] != "") else 0.5
        hip_y = float(r["hip_y"]) if (ok and r["hip_y"] != "") else 0.53

        if hip_x0 is None and ok:
            hip_x0 = hip_x
            hip_y0 = hip_y

        if ok:
            hip_x0 = 0.96 * hip_x0 + 0.04 * hip_x
            hip_y0 = 0.96 * hip_y0 + 0.04 * hip_y

        # ------------------------------------------------------------------
        # 1) お手本（骨格の動き）の2D位置・向き
        #    ・向き t_h : 骨格の肩の向き yaw（滑らかに追従）
        #    ・前後 t_y : 足の前後踏み出し (dproj_a) ＋ 腰の上下・前後移動
        #    ・左右 t_x : 腰の左右移動 (hip_x - hip_x0)
        # ------------------------------------------------------------------
        if ok:
            # 角度差を最短経路で補間
            diff_yaw = ((raw_yaw - t_h + 180.0) % 360.0) - 180.0
            t_h += 0.35 * diff_yaw
            target_tx = clip(-(hip_x - hip_x0) * 1.8, -0.40, 0.40)
            target_ty = clip(dproj_a * 2.2 - (hip_y - hip_y0) * 1.6, -0.45, 0.45)
            t_x = 0.75 * t_x + 0.25 * target_tx
            t_y = 0.75 * t_y + 0.25 * target_ty
        else:
            t_x *= 0.9
            t_y *= 0.9

        # ------------------------------------------------------------------
        # 2) 第1段階（物真似の決まり：骨格推定 → WHILL）
        # ------------------------------------------------------------------
        if 20.0 <= abs(raw_yaw) <= 105.0:
            last_front_yaw = raw_yaw

        if ok and abs(raw_yaw) > 110.0 and not squat_flag:
            turn_confirm += 1
        else:
            turn_confirm = max(0, turn_confirm - 1)

        l_turn_now = False
        r_turn_now = False
        if turn_confirm >= 2:
            if last_front_yaw < -15.0 or (abs(last_front_yaw) <= 15.0 and raw_yaw < 0.0):
                r_turn_now = True
            else:
                l_turn_now = True

        if cooldown > 0.0:
            cooldown = max(0.0, cooldown - dt)
        mode_timer += dt

        if not ok:
            mode = "idle"
            r_step_count = l_step_count = 0
            ht, ft = yaw_s, 0.0
            action = "つなぎ動作（次動作へ補間中）"
        else:
            # 人の向いている向きを平滑化追従
            diff_y = ((raw_yaw - yaw_s + 180.0) % 360.0) - 180.0
            yaw_s += 0.45 * diff_y
            base_h = clip(yaw_s, -105.0, 105.0)

            if mode in ("l_turn", "r_turn"):
                if squat_flag:
                    mode, mode_timer, turn_confirm = "squat", 0.0, 0
                    r_step_count = l_step_count = 0
                elif l_lift_flag:
                    mode, mode_timer, turn_confirm = "l_lift", 0.0, 0
                    r_step_count = l_step_count = 0
                elif r_lift_flag:
                    mode, mode_timer, turn_confirm = "r_lift", 0.0, 0
                    r_step_count = l_step_count = 0
                elif r_step_flag:
                    mode, mode_timer, turn_confirm = "r_step", 0.0, 0
                    r_step_count, l_step_count = 1, 0
                elif l_step_flag:
                    mode, mode_timer, turn_confirm = "l_step", 0.0, 0
                    l_step_count, r_step_count = 1, 0
                elif mode_timer >= 0.20 and abs(raw_yaw) < 85.0:
                    mode, mode_timer, turn_confirm, cooldown = "idle", 0.0, 0, 0.20
                    r_step_count = l_step_count = 0
            else:
                if squat_flag and mode != "squat":
                    mode, mode_timer = "squat", 0.0
                    r_step_count = l_step_count = 0
                elif (l_turn_now or r_turn_now) and mode != "squat" and cooldown <= 0.0:
                    mode, mode_timer = ("l_turn" if l_turn_now else "r_turn"), 0.0
                    r_step_count = l_step_count = 0
                elif not squat_flag:
                    if l_lift_flag and (mode != "l_lift" or mode_timer >= LIFT_HOLD):
                        if mode != "r_lift" or mode_timer >= 0.20:
                            mode, mode_timer = "l_lift", 0.0
                            r_step_count = l_step_count = 0
                    elif r_lift_flag and (mode != "r_lift" or mode_timer >= LIFT_HOLD):
                        if mode != "l_lift" or mode_timer >= 0.20:
                            mode, mode_timer = "r_lift", 0.0
                            r_step_count = l_step_count = 0
                    elif r_step_flag and (
                        (mode == "idle" and cooldown <= 0.0)
                        or mode == "squat_return"
                        or (mode in ("l_step", "l_step_repeat") and mode_timer >= 0.20)
                        or (mode in ("l_lift", "r_lift") and mode_timer >= 0.20)
                    ):
                        mode, mode_timer = "r_step", 0.0
                        r_step_count, l_step_count = 1, 0
                    elif r_step_flag and mode == "r_step" and mode_timer >= STEP_HOLD * 0.75:
                        mode, mode_timer = "r_step_repeat", 0.0
                        r_step_count += 1
                    elif r_step_flag and mode == "r_step_repeat" and mode_timer >= STEP_PULLBACK + STEP_HOLD * 0.75:
                        mode_timer = 0.0
                        r_step_count += 1
                    elif l_step_flag and (
                        (mode == "idle" and cooldown <= 0.0)
                        or mode == "squat_return"
                        or (mode in ("r_step", "r_step_repeat") and mode_timer >= 0.20)
                        or (mode in ("l_lift", "r_lift") and mode_timer >= 0.20)
                    ):
                        mode, mode_timer = "l_step", 0.0
                        l_step_count, r_step_count = 1, 0
                    elif l_step_flag and mode == "l_step" and mode_timer >= STEP_HOLD * 0.75:
                        mode, mode_timer = "l_step_repeat", 0.0
                        l_step_count += 1
                    elif l_step_flag and mode == "l_step_repeat" and mode_timer >= STEP_PULLBACK + STEP_HOLD * 0.75:
                        mode_timer = 0.0
                        l_step_count += 1

            if mode == "squat":
                ht, ft = base_h, -STEP_DIST
                action = "しこ立ち（0.12m後退）"
                if mode_timer >= STEP_HOLD and not squat_flag:
                    mode = "squat_return"
                    mode_timer = 0.0
            elif mode == "squat_return":
                ht, ft = base_h, 0.0
                action = "しこ立ち（元の位置へ戻る）"
                if mode_timer >= STEP_RETURN:
                    mode = "idle"
                    cooldown = 0.2
            elif mode == "l_turn":
                ht, ft = 360.0, 0.0
                action = "左turn（左へ1回転）"
                if mode_timer >= TURN_HOLD:
                    mode = "idle"
                    cooldown = 0.35
                    turn_confirm = 0
            elif mode == "r_turn":
                ht, ft = -360.0, 0.0
                action = "右turn（右へ1回転）"
                if mode_timer >= TURN_HOLD:
                    mode = "idle"
                    cooldown = 0.35
                    turn_confirm = 0
            elif mode == "l_lift":
                if mode_timer < LIFT_HOLD:
                    ht = base_h + LIFT_ANGLE
                    ft = LIFT_DIST
                    action = "左足上げ（左斜め前へ前進）"
                elif mode_timer < LIFT_HOLD + LIFT_RETURN:
                    ht = base_h
                    ft = 0.0
                    action = "左足上げ（元の位置へ戻る）"
                else:
                    ht = base_h
                    ft = 0.0
                    mode = "idle"
                    cooldown = 0.05
                    action = "つなぎ動作（次動作へ連携中）"
            elif mode == "r_lift":
                if mode_timer < LIFT_HOLD:
                    ht = base_h - LIFT_ANGLE
                    ft = LIFT_DIST
                    action = "右足上げ（右斜め前へ前進）"
                elif mode_timer < LIFT_HOLD + LIFT_RETURN:
                    ht = base_h
                    ft = 0.0
                    action = "右足上げ（元の位置へ戻る）"
                else:
                    ht = base_h
                    ft = 0.0
                    mode = "idle"
                    cooldown = 0.05
                    action = "つなぎ動作（次動作へ連携中）"
            elif mode == "r_step":
                ht = base_h
                if mode_timer < STEP_HOLD:
                    ft = STEP_DIST
                    action = "右足ステップ1回目（前進）"
                elif mode_timer < STEP_HOLD + STEP_RETURN:
                    ft = 0.0
                    action = "右足ステップ（元の位置へ戻る）"
                else:
                    ft = 0.0
                    mode = "idle"
                    r_step_count = 0
                    cooldown = 0.04
                    action = "つなぎ動作（次動作へ連携中）"
            elif mode == "r_step_repeat":
                ht = base_h
                if mode_timer < STEP_PULLBACK:
                    ft = 0.30 * STEP_DIST
                    action = f"右足ステップ{r_step_count}回目（少し後ろへ下がる）"
                elif mode_timer < STEP_PULLBACK + STEP_HOLD:
                    ft = STEP_DIST
                    action = f"右足ステップ{r_step_count}回目（1回目と同じ位置へ前進）"
                elif mode_timer < STEP_PULLBACK + STEP_HOLD + STEP_RETURN:
                    ft = 0.0
                    action = "右足ステップ（元の位置へ戻る）"
                else:
                    ft = 0.0
                    mode = "idle"
                    r_step_count = 0
                    cooldown = 0.04
                    action = "つなぎ動作（次動作へ連携中）"
            elif mode == "l_step":
                ht = base_h
                if mode_timer < STEP_HOLD:
                    ft = -STEP_DIST
                    action = "左足ステップ1回目（後退）"
                elif mode_timer < STEP_HOLD + STEP_RETURN:
                    ft = 0.0
                    action = "左足ステップ（元の位置へ戻る）"
                else:
                    ft = 0.0
                    mode = "idle"
                    l_step_count = 0
                    cooldown = 0.04
                    action = "つなぎ動作（次動作へ連携中）"
            elif mode == "l_step_repeat":
                ht = base_h
                if mode_timer < STEP_PULLBACK:
                    ft = -0.30 * STEP_DIST
                    action = f"左足ステップ{l_step_count}回目（少し前に向かう）"
                elif mode_timer < STEP_PULLBACK + STEP_HOLD:
                    ft = -STEP_DIST
                    action = f"左足ステップ{l_step_count}回目（1回目と同じ位置へ後退）"
                elif mode_timer < STEP_PULLBACK + STEP_HOLD + STEP_RETURN:
                    ft = 0.0
                    action = "左足ステップ（元の位置へ戻る）"
                else:
                    ft = 0.0
                    mode = "idle"
                    l_step_count = 0
                    cooldown = 0.04
                    action = "つなぎ動作（次動作へ連携中）"
            else:
                ht = base_h
                ft = clip(0.35 * dproj_a, -0.08, 0.08)
                action = "つなぎ動作（次動作へ連携中）"

        # Follower (0.05s × 2回)
        for _ in range(2):
            sub_dt = dt / 2.0
            if abs(ht) < 180.0:
                if in_turn:
                    in_turn = False
                    fol_h = ht
                else:
                    fol_h = ((fol_h + 180.0) % 360.0) - 180.0
                err_h = ((ht - fol_h + 180.0) % 360.0) - 180.0
            else:
                if not in_turn:
                    in_turn = True
                    fol_h = 0.0
                err_h = ht - fol_h
            err_f = ft - fol_f
            if abs(err_h) < 0.4:
                w = 0.0
                fol_h = ht
            else:
                w = clip(HEADING_GAIN * math.radians(err_h), -ANGULAR_MAX, ANGULAR_MAX)
                fol_h += math.degrees(w * sub_dt)
            if abs(err_f) < 0.004:
                v = 0.0
                fol_f = ft
            else:
                v = clip(FORWARD_GAIN * err_f, -LINEAR_MAX, LINEAR_MAX)
                fol_f += v * sub_dt

        # WHILL ジョイスティック倒し量 [%]
        tf = clip(v / 0.35, -1.0, 1.0) * FRONT_CAP
        if tf < 0:
            tf = max(-BACK_CAP, tf * BACK_GAIN)
        ts = -1.0 * clip(w / 3.0, -1.0, 1.0) * SIDE_CAP
        r_step = RAMP * dt
        f_step = r_step * BACK_GAIN if tf < 0 else r_step
        if tf * whill_front < 0 or abs(tf) < 1.0:
            whill_front = tf
        else:
            whill_front += clip(tf - whill_front, -f_step, f_step) if abs(tf) > abs(whill_front) else (tf - whill_front)
        if ts * whill_side < 0 or abs(ts) < 1.0:
            whill_side = ts
        else:
            whill_side += clip(ts - whill_side, -r_step, r_step) if abs(ts) > abs(whill_side) else (ts - whill_side)
        f_int = int(round(whill_front))
        s_int = int(round(whill_side))

        # 上から見たWHILLの位置（斜め前進・15°足上げ旋回・1回転ターンが大きく素早く見えるよう描画）
        if in_turn:
            turn_rad = math.radians(fol_h)
            s1_x = -0.14 * math.sin(turn_rad)
            s1_y = 0.14 * (1.0 - math.cos(turn_rad))
        else:
            vis_ang = math.radians(fol_h)
            # 足上げ回転時も外側へ大きくスイングして見えるよう横変位を加える
            swing_x = -0.22 * math.sin(vis_ang) if abs(fol_f) < 0.05 else 0.0
            s1_x = -fol_f * 2.1 * math.sin(vis_ang) + swing_x
            s1_y = fol_f * 1.9 * math.cos(vis_ang)
        s1_h = fol_h

        row_copy = dict(r)
        row_copy["action_name"] = action
        row_copy["whill_front_pct"] = f_int
        row_copy["whill_side_pct"] = s_int
        row_copy["whill_front_raw"] = round(whill_front, 2)
        row_copy["whill_side_raw"] = round(whill_side, 2)
        row_copy["v"] = round(v, 4)
        row_copy["w"] = round(w, 4)
        row_copy["heading_target"] = round(ht, 2)
        row_copy["forward_target"] = round(ft, 4)
        row_copy["heading"] = round(fol_h, 2)
        row_copy["forward"] = round(fol_f, 4)

        enriched.append({
            "csv": row_copy,
            "t": t,
            "ok": ok,
            "action": action,
            "t_x": t_x,
            "t_y": t_y,
            "t_h": t_h,
            "s1_x": s1_x,
            "s1_y": s1_y,
            "s1_h": s1_h,
            "f_pct": f_int,
            "s_pct": s_int,
            "v": v,
            "w": w,
        })

    return enriched


def draw_stick_figure(canvas, row_data, box_rect):
    """
    左パネルの枠内に、media_Tango_003_020.mp4 と同じスタイルの棒人間（骨格）を描画する。
    box_rect = (bx0, by0, bx1, by1)
    """
    bx0, by0, bx1, by1 = box_rect
    bw = bx1 - bx0
    bh = by1 - by0

    # 黒枠
    cv2.rectangle(canvas, (bx0, by0), (bx1, by1), (30, 30, 30), 1, cv2.LINE_AA)

    r = row_data["csv"]
    if not row_data["ok"] or r["hip_x"] == "":
        return

    # 元の座標 (0..1) を、枠内で見やすい大きさ・中央寄りにマッピング
    hx_raw = float(r["hip_x"])
    hy_raw = float(r["hip_y"])
    rkx_raw = float(r["r_knee_x"]) if r["r_knee_x"] != "" else hx_raw - 0.02
    rky_raw = float(r["r_knee_y"]) if r["r_knee_y"] != "" else hy_raw + 0.09
    lkx_raw = float(r["l_knee_x"]) if r["l_knee_x"] != "" else hx_raw + 0.02
    lky_raw = float(r["l_knee_y"]) if r["l_knee_y"] != "" else hy_raw + 0.09
    rax_raw = float(r["r_ankle_x"]) if r["r_ankle_x"] != "" else rkx_raw
    ray_raw = float(r["r_ankle_y"]) if r["r_ankle_y"] != "" else rky_raw + 0.09
    lax_raw = float(r["l_ankle_x"]) if r["l_ankle_x"] != "" else lkx_raw
    lay_raw = float(r["l_ankle_y"]) if r["l_ankle_y"] != "" else lky_raw + 0.09

    # 動画内の人物は x≈0.70, y≈0.53 付近にいるため、枠の中央付近で見やすいサイズ(約1.75倍)に拡大投影
    scale_x = 1.65
    scale_y = 1.85
    cx_ref, cy_ref = 0.65, 0.56

    def to_px(nx, ny):
        px = int(round(bx0 + bw * (0.50 + (nx - cx_ref) * scale_x)))
        py = int(round(by0 + bh * (0.54 + (ny - cy_ref) * scale_y)))
        return (clip(px, bx0 + 6, bx1 - 6), clip(py, by0 + 6, by1 - 6))

    hip_pt = to_px(hx_raw, hy_raw)
    rk_pt = to_px(rkx_raw, rky_raw)
    lk_pt = to_px(lkx_raw, lky_raw)
    ra_pt = to_px(rax_raw, ray_raw)
    la_pt = to_px(lax_raw, lay_raw)

    # 体幹の高さと肩幅を算出（yawに応じて肩の見かけ幅が変化）
    yaw_deg = float(r["yaw"]) if r["yaw"] != "" else 0.0
    yaw_rad = math.radians(yaw_deg)
    torso_h = 0.135
    sh_y_raw = hy_raw - torso_h
    sh_half_w = 0.036 * math.cos(yaw_rad)
    # 少し傾きを表現
    sh_cx_raw = hx_raw + 0.006 * math.sin(yaw_rad)

    r_sh_raw = (sh_cx_raw - sh_half_w, sh_y_raw)
    l_sh_raw = (sh_cx_raw + sh_half_w, sh_y_raw)
    sh_mid_raw = (sh_cx_raw, sh_y_raw)
    head_raw = (sh_cx_raw, sh_y_raw - 0.042)

    r_sh_pt = to_px(*r_sh_raw)
    l_sh_pt = to_px(*l_sh_raw)
    sh_mid_pt = to_px(*sh_mid_raw)
    head_pt = to_px(*head_raw)

    # 腕の関節（手上げ l_up / r_up や踊りのリズムに合わせて自然に描画）
    t = row_data["t"]
    l_up = int(r.get("l_up", 0) or 0) == 1
    r_up = int(r.get("r_up", 0) or 0) == 1
    beat = math.sin(t * 4.5)

    if r_up:
        r_el_raw = (r_sh_raw[0] - 0.038, r_sh_raw[1] - 0.035 + 0.008 * beat)
        r_wr_raw = (r_el_raw[0] - 0.025, r_el_raw[1] - 0.055 + 0.010 * beat)
    else:
        r_el_raw = (r_sh_raw[0] - 0.034, r_sh_raw[1] + 0.048 + 0.010 * beat)
        r_wr_raw = (r_el_raw[0] - 0.022, r_el_raw[1] + 0.032 - 0.012 * beat)

    if l_up:
        l_el_raw = (l_sh_raw[0] + 0.038, l_sh_raw[1] - 0.035 - 0.008 * beat)
        l_wr_raw = (l_el_raw[0] + 0.025, l_el_raw[1] - 0.055 - 0.010 * beat)
    else:
        l_el_raw = (l_sh_raw[0] + 0.036, l_sh_raw[1] + 0.042 - 0.010 * beat)
        l_wr_raw = (l_el_raw[0] + 0.026, l_el_raw[1] + 0.025 + 0.012 * beat)

    r_el_pt = to_px(*r_el_raw)
    r_wr_pt = to_px(*r_wr_raw)
    l_el_pt = to_px(*l_el_raw)
    l_wr_pt = to_px(*l_wr_raw)

    col = (45, 45, 45)  # 濃いチャコールグレー (#2d2d2d)
    lw = 3

    # 体幹（V字＋背骨＋肩ライン：Tango動画と完全一致のスタイル）
    cv2.line(canvas, r_sh_pt, l_sh_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, r_sh_pt, hip_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, l_sh_pt, hip_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, sh_mid_pt, hip_pt, col, lw, cv2.LINE_AA)

    # 両腕
    cv2.line(canvas, r_sh_pt, r_el_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, r_el_pt, r_wr_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, l_sh_pt, l_el_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, l_el_pt, l_wr_pt, col, lw, cv2.LINE_AA)

    # 両脚
    cv2.line(canvas, hip_pt, rk_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, rk_pt, ra_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, hip_pt, lk_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, lk_pt, la_pt, col, lw, cv2.LINE_AA)

    # 頭（塗りつぶし円）
    cv2.circle(canvas, head_pt, 11, col, -1, cv2.LINE_AA)


def draw_whill_topdown(canvas, enriched, idx, plot_rect):
    """
    右パネルの枠内に、上から見たWHILLの動き（お手本 vs 第1段階）を描画する。
    plot_rect = (px0, py0, px1, py1)
    座標系: X ∈ [-2.0, 2.0], Y ∈ [-1.15, 1.15]
    """
    px0, py0, px1, py1 = plot_rect
    pw = px1 - px0
    ph = py1 - py0

    x_min, x_max = -2.0, 2.0
    y_min, y_max = -1.15, 1.15

    def world_to_px(wx, wy):
        u = (wx - x_min) / (x_max - x_min)
        v = (y_max - wy) / (y_max - y_min)
        return int(round(px0 + u * pw)), int(round(py0 + v * ph))

    # 薄いグリッド線
    grid_col = (232, 232, 232)
    for gx in [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]:
        x_p, _ = world_to_px(gx, 0.0)
        cv2.line(canvas, (x_p, py0), (x_p, py1), grid_col, 1, cv2.LINE_AA)
        # X軸目盛りヒゲ
        cv2.line(canvas, (x_p, py1), (x_p, py1 + 5), (60, 60, 60), 1, cv2.LINE_AA)

    for gy in [-1.0, -0.5, 0.0, 0.5, 1.0]:
        _, y_p = world_to_px(0.0, gy)
        cv2.line(canvas, (px0, y_p), (px1, y_p), grid_col, 1, cv2.LINE_AA)
        # Y軸目盛りヒゲ
        cv2.line(canvas, (px0 - 5, y_p), (px0, y_p), (60, 60, 60), 1, cv2.LINE_AA)

    # 枠線
    cv2.rectangle(canvas, (px0, py0), (px1, py1), (30, 30, 30), 1, cv2.LINE_AA)

    # 1) お手本（骨格の動き）：中心 X = -0.95
    # 2) 第1段階（物真似の決まり：骨格推定→WHILL）：中心 X = +0.75
    t_cx0 = -0.95
    s1_cx0 = 0.75

    # 過去 2.5秒間（25ステップ）の軌跡テールを描画
    tail_start = max(0, idx - 25)
    t_pts = []
    s1_pts = []
    for k in range(tail_start, idx + 1):
        ek = enriched[k]
        t_pts.append(world_to_px(t_cx0 + ek["t_x"], ek["t_y"]))
        s1_pts.append(world_to_px(s1_cx0 + ek["s1_x"], ek["s1_y"]))

    if len(t_pts) >= 2:
        cv2.polylines(canvas, [np.array(t_pts, dtype=np.int32)], False, (185, 180, 175), 2, cv2.LINE_AA)
    if len(s1_pts) >= 2:
        cv2.polylines(canvas, [np.array(s1_pts, dtype=np.int32)], False, (215, 175, 120), 2, cv2.LINE_AA)

    def draw_triangle(wx, wy, heading_deg, bgr_color):
        # heading_deg = 0° で真上 (+Y)、+で左回転（反時計回り）
        rad = math.radians(heading_deg)
        # 三角形のローカル頂点 (先端・左後・右後)
        local_verts = [
            (0.0, 0.18),     # 先端 (進行方向)
            (-0.105, -0.11), # 左後ろ
            (0.105, -0.11),  # 右後ろ
        ]
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        poly = []
        for lx, ly in local_verts:
            # 0°が+Y方向、+radが反時計回りなので:
            rx = lx * cos_a - ly * sin_a
            ry = lx * sin_a + ly * cos_a
            poly.append(world_to_px(wx + rx, wy + ry))
        pts_arr = np.array(poly, dtype=np.int32)
        cv2.fillPoly(canvas, [pts_arr], bgr_color, cv2.LINE_AA)

    cur = enriched[idx]
    # お手本（グレー三角形: BGR=(155, 148, 142)）
    draw_triangle(t_cx0 + cur["t_x"], cur["t_y"], cur["t_h"], (155, 148, 142))
    # 第1段階（ブルー三角形: BGR=(180, 130, 18) -> RGB=(18, 130, 180)）
    draw_triangle(s1_cx0 + cur["s1_x"], cur["s1_y"], cur["s1_h"], (180, 130, 18))

    return world_to_px


def render_video(enriched, start_t, end_t, out_path):
    W, H = 1280, 720
    fps = 15.0  # 参照動画 media_Tango_003_020.mp4 と同じ 15fps

    font_title = get_jp_font(21, bold=False)
    font_label = get_jp_font(18, bold=False)
    font_badge = get_jp_font(17, bold=True)
    font_tick = get_jp_font(14, bold=False)
    font_time = get_jp_font(19, bold=False)

    # 左パネル（棒人間）と右パネル（WHILL上から見た動き）の矩形（media_Tango_003_020.mp4 と完全一致の配置）
    left_box = (38, 230, 550, 520)
    right_box = (614, 182, 1254, 568)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, (W, H))

    # 静的背景テンプレートを作成（高速化）
    bg = np.full((H, W, 3), 255, dtype=np.uint8)
    bg_pil = Image.fromarray(cv2.cvtColor(bg, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(bg_pil)

    # タイトル
    draw.text((294, 200), "カメラから見た踊る人 (mirukumunari.mp4 : お手本)", font=font_title, fill=(20, 20, 20), anchor="mm")
    draw.text((934, 152), "WHILL を上から見た動き", font=font_title, fill=(20, 20, 20), anchor="mm")

    # 軸目盛りラベル
    px0, py0, px1, py1 = right_box
    pw, ph = px1 - px0, py1 - py0
    x_min, x_max = -2.0, 2.0
    y_min, y_max = -1.15, 1.15

    for gx in [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]:
        u = (gx - x_min) / (x_max - x_min)
        xp = int(round(px0 + u * pw))
        lbl = f"−{abs(gx):.1f}" if gx < 0 else f"{gx:.1f}"
        draw.text((xp, py1 + 18), lbl, font=font_tick, fill=(30, 30, 30), anchor="mm")

    for gy in [-1.0, -0.5, 0.0, 0.5, 1.0]:
        v = (y_max - gy) / (y_max - y_min)
        yp = int(round(py0 + v * ph))
        lbl = f"−{abs(gy):.1f}" if gy < 0 else f"{gy:.1f}"
        draw.text((px0 - 10, yp), lbl, font=font_tick, fill=(30, 30, 30), anchor="rm")

    # パネル内下部の凡例ラベル
    u_t = (-0.95 - x_min) / (x_max - x_min)
    u_s1 = (0.75 - x_min) / (x_max - x_min)
    y_leg = int(round(py0 + ((y_max - (-1.02)) / (y_max - y_min)) * ph))
    draw.text((int(round(px0 + u_t * pw)), y_leg), "お手本 (骨格の動き)", font=font_label, fill=(135, 140, 146), anchor="mm")
    draw.text((int(round(px0 + u_s1 * pw)), y_leg), "第1段階 (物真似の決まり)", font=font_label, fill=(18, 130, 180), anchor="mm")

    base_bgr = cv2.cvtColor(np.array(bg_pil), cv2.COLOR_RGB2BGR)

    # 15fps の各時刻に対応するインデックスを抽出
    total_dur = end_t - start_t
    n_frames = int(round(total_dur * fps))
    times = [r["t"] for r in enriched]

    import bisect
    for f_idx in range(n_frames):
        cur_t = start_t + f_idx / fps
        pos = bisect.bisect_left(times, cur_t)
        pos = min(len(enriched) - 1, max(0, pos))

        canvas = base_bgr.copy()
        draw_stick_figure(canvas, enriched[pos], left_box)
        draw_whill_topdown(canvas, enriched, pos, right_box)

        # 現在の動作名・WHILL出力(%)・経過秒数を描画
        pil_img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(pil_img)

        cur = enriched[pos]
        act = cur["action"]
        f_pct = cur["f_pct"]
        s_pct = cur["s_pct"]

        # 左パネル下に検出動作、右パネル上にWHILL指令値をスッキリ表示
        act_col = (18, 130, 180) if act != "待機中（初期位置）" else (110, 115, 120)
        d.text(
            (294, 548),
            f"検出動作：{act}",
            font=font_badge,
            fill=act_col,
            anchor="mm",
        )
        d.text(
            (int(round(px0 + u_s1 * pw)), py0 + 26),
            f"WHILL指令: 前後 {f_pct:+d}% / 旋回 {s_pct:+d}%  (向き {cur['s1_h']:+.0f}°, 前後 {cur['csv']['forward']:+.2f}m)",
            font=font_tick,
            fill=(18, 130, 180),
            anchor="mm",
        )

        # 右下の秒数表示（media_Tango_003_020.mp4 と同じ位置）
        rel_sec = cur_t - start_t
        d.text((1255, 696), f"{rel_sec:.1f} 秒 (動画 {cur_t:.1f}s)", font=font_time, fill=(20, 20, 20), anchor="rm")

        out_bgr = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
        writer.write(out_bgr)

    writer.release()
    print(f"Saved video: {out_path} ({n_frames} frames, {total_dur:.1f}s)")


def main():
    ap = argparse.ArgumentParser(description="お手本の骨格の動きと第1段階の WHILL の動きを横並びの動画にする")
    ap.add_argument("--csv", default=os.path.join(HERE, "mirukumunari_dance.csv"),
                    help="offline_dance.py で作ったお手本の CSV（既定：このスクリプトのフォルダの mirukumunari_dance.csv）")
    ap.add_argument("--out-dir", help="動画を書くフォルダ（既定：--csv と同じフォルダ）")
    ap.add_argument("--copy-to", help="CSV と動画を、このフォルダにも写す（任意）")
    a = ap.parse_args()
    CSV_PATH = a.csv
    OUT_DIR = a.out_dir or os.path.dirname(os.path.abspath(CSV_PATH))
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(CSV_PATH, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    enriched = recompute_stage1_and_teacher(rows)

    # 更新された第1段階WHILL値をCSVにも保存
    fieldnames = list(rows[0].keys())
    for cpath in [CSV_PATH] + ([os.path.join(a.copy_to, os.path.basename(CSV_PATH))] if a.copy_to else []):
        try:
            os.makedirs(os.path.dirname(cpath), exist_ok=True)
            with open(cpath, "w", newline="", encoding="utf-8-sig") as fo:
                wr = csv.DictWriter(fo, fieldnames=fieldnames)
                wr.writeheader()
                wr.writerows([e["csv"] for e in enriched])
        except Exception as e:
            print(f"CSV write skip ({cpath}): {e}")

    # 1) 最も動きが豊富で手上げ・ステップ・足上げ・ターンがすべて入る 15.0s〜45.0s (30秒間) の動画を作成
    out_30s = os.path.join(OUT_DIR, "mirukumunari_whill_stage1_30s.mp4")
    render_video(enriched, 15.0, 45.0, out_30s)

    if a.copy_to:
        os.makedirs(a.copy_to, exist_ok=True)
        shutil.copy2(out_30s, os.path.join(a.copy_to, "mirukumunari_whill_stage1_30s.mp4"))


if __name__ == "__main__":
    main()
