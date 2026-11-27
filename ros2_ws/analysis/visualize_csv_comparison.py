#!/usr/bin/env python3
"""
  1) mirukumunari_dance.csv (お手本の骨格の動き。offline_dance.py で作る)
  2) whill_live_YYYYMMDD_HHMMSS.csv (whill_bridge.py --csv の第1段階WHILL実測データ ※「指令なし」は動画遅延のため無視)
を横並びで視覚化し、動画 (.mp4) として出力するスクリプト。

  python ros2_ws/analysis/visualize_csv_comparison.py --dir 記録のフォルダ
  python ros2_ws/analysis/visualize_csv_comparison.py --miruku お手本.csv --live whill_live_20261118_122224.csv --out-dir 出力先
  --dir を付けなければこのスクリプトのフォルダ。--live を付けなければ、そのフォルダのいちばん新しい whill_live_*.csv
"""

import argparse
import bisect
import csv
import glob
import math
import os

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


def load_miruku(csv_path):
    """mirukumunari_dance.csv を読み込み、お手本（骨格の動き）の2D位置・向き (t_x, t_y, t_h) を算出"""
    with open(csv_path, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    hip_x0 = None
    hip_y0 = None
    t_x = 0.0
    t_y = 0.0
    t_h = 0.0
    out = []

    for r in rows:
        if not str(r.get("t", "")).strip():
            continue
        t = float(r["t"])
        ok = int(r.get("ok", 0) or 0) == 1
        raw_yaw = float(r["yaw"]) if (ok and r.get("yaw", "") != "") else 0.0
        dproj_a = float(r["dproj_a"]) if (ok and r.get("dproj_a", "") != "") else 0.0
        hip_x = float(r["hip_x"]) if (ok and r.get("hip_x", "") != "") else 0.70
        hip_y = float(r["hip_y"]) if (ok and r.get("hip_y", "") != "") else 0.53

        if hip_x0 is None and ok:
            hip_x0 = hip_x
            hip_y0 = hip_y
        if ok:
            hip_x0 = 0.96 * hip_x0 + 0.04 * hip_x
            hip_y0 = 0.96 * hip_y0 + 0.04 * hip_y
            diff_yaw = ((raw_yaw - t_h + 180.0) % 360.0) - 180.0
            t_h += 0.35 * diff_yaw
            target_tx = clip(-(hip_x - hip_x0) * 1.8, -0.45, 0.45)
            target_ty = clip(dproj_a * 2.2 - (hip_y - hip_y0) * 1.6, -0.48, 0.48)
            t_x = 0.75 * t_x + 0.25 * target_tx
            t_y = 0.75 * t_y + 0.25 * target_ty
        else:
            t_x *= 0.9
            t_y *= 0.9

        out.append({
            "t": t,
            "ok": ok,
            "row": r,
            "t_x": t_x,
            "t_y": t_y,
            "t_h": t_h,
            "action": r.get("action_name", "待機中（初期位置）"),
        })
    return out


def load_whill_live(csv_path):
    """
    whill_live_20261118_122224.csv を読み込み、
      ・動画開始前の待機区間（最初の ok==1 より前）をスキップして t=0.0s に揃え、
      ・「指令なし」「指示なし」「rosbridge なし」の行は動画遅延による一時途切れとして無視（直前の有効状態を保持）し、
      ・速度指令 (v, w) および目標値 (heading_target, forward_target) から上から見たWHILLの位置・向きを復元する。
    """
    with open(csv_path, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    # 最初の ok == 1 の行を探す（t=15.37s）
    start_idx = 0
    for i, r in enumerate(rows):
        if str(r.get("ok", "0")).strip() == "1":
            start_idx = i
            break

    t0 = float(rows[start_idx]["t"])
    ignore_actions = {"指令なし", "指示なし", "rosbridge なし", "停止中", ""}

    # 1) まず「指令なし」等の動画遅延行を除外した有効フレーム列を取り出し、
    #    turn実行中に次の動き（足上げ・ステップ・しこ立ち・手上げ・正面復帰）が来たフレームでは
    #    遅延ゼロで即座にturnを終了して次の動きへ切り替える
    raw_valid = []
    turn_dur = 0.0
    prev_t_raw = None
    yaw_s = 0.0
    r_step_seq_count = 0
    l_step_seq_count = 0
    r_step_seq_timer = 0.0
    l_step_seq_timer = 0.0
    was_r_step_fwd = False
    was_l_step_back = False

    for r in rows[start_idx:]:
        if not str(r.get("t", "")).strip():
            continue
        raw_t = float(r["t"])
        rel_t = round(raw_t - t0, 3)
        act = str(r.get("action", "")).strip()
        ok = str(r.get("ok", "0")).strip() == "1"
        if act in ignore_actions or not ok:
            continue

        dt_r = 0.1 if prev_t_raw is None else clip(rel_t - prev_t_raw, 0.02, 0.25)
        prev_t_raw = rel_t

        yaw = float(r["yaw"]) if r.get("yaw", "") != "" else yaw_s
        diff_y = ((yaw - yaw_s + 180.0) % 360.0) - 180.0
        yaw_s += 0.45 * diff_y
        base_h = clip(yaw_s, -105.0, 105.0)

        dproj = float(r["dproj_a"]) if r.get("dproj_a", "") != "" else 0.0
        squat = int(r.get("squat") or 0) == 1
        l_lift = int(r.get("l_knee_lift") or 0) == 1
        r_lift = int(r.get("r_knee_lift") or 0) == 1
        r_step = int(r.get("r_step_fwd") or 0) == 1
        l_step = int(r.get("l_step_back") or 0) == 1
        l_up = int(r.get("l_up") or 0) == 1
        r_up = int(r.get("r_up") or 0) == 1

        ht = float(r.get("heading_target") or 0.0)
        ft = float(r.get("forward_target") or 0.0)

        # 骨格推定フラグから4つの対応動作（手の動きは無視、足上げは右/左斜め前に前進）へマッピング
        if "ターン" in act or abs(ht) >= 180.0:
            turn_dur += dt_r
            was_r_step_fwd = was_l_step_back = False
            if squat:
                act, ht, ft = "しこ立ち（0.12m後退）", base_h, -0.30
                turn_dur = 0.0
            elif l_lift:
                act, ht, ft = "左足上げ（左斜め前へ前進）", base_h + 38.0, 0.28
                turn_dur = 0.0
            elif r_lift:
                act, ht, ft = "右足上げ（右斜め前へ前進）", base_h - 38.0, 0.28
                turn_dur = 0.0
            elif r_step or dproj > 0.12:
                r_step_seq_count = 1
                r_step_seq_timer = 0.0
                was_r_step_fwd = True
                act, ht, ft = "右足ステップ1回目（前進）", base_h, 0.30
                turn_dur = 0.0
            elif l_step or dproj < -0.12:
                l_step_seq_count = 1
                l_step_seq_timer = 0.0
                was_l_step_back = True
                act, ht, ft = "左足ステップ1回目（後退）", base_h, -0.30
                turn_dur = 0.0
            elif turn_dur >= 0.15 and abs(yaw) < 85.0:
                act, ht, ft = "待機中（初期位置）", base_h, 0.0
                turn_dur = 0.0
            else:
                act = "左turn（左へ1回転）" if ht > 0 else "右turn（右へ1回転）"
                ht = 360.0 if ht > 0 else -360.0
                ft = 0.0
        else:
            turn_dur = 0.0
            if "しこ立ち" in act:
                was_r_step_fwd = was_l_step_back = False
                r_step_seq_count = l_step_seq_count = 0
                if "戻る" in act:
                    act, ht, ft = "しこ立ち（元の位置へ戻る）", base_h, 0.0
                else:
                    act, ht, ft = "しこ立ち（0.12m後退）", base_h, -0.30
            elif "足上げ" in act or l_lift or r_lift:
                was_r_step_fwd = was_l_step_back = False
                r_step_seq_count = l_step_seq_count = 0
                if "戻る" in act:
                    act, ht, ft = "足上げ（元の位置へ戻る）", base_h, 0.0
                elif ht > 0 or l_lift:
                    act, ht, ft = "左足上げ（左斜め前へ前進）", base_h + 38.0, 0.28
                else:
                    act, ht, ft = "右足上げ（右斜め前へ前進）", base_h - 38.0, 0.28
            elif "右足ステップ" in act or r_step:
                was_l_step_back = False
                l_step_seq_count = 0
                is_fwd_phase = "戻る" not in act
                if is_fwd_phase:
                    if not was_r_step_fwd:
                        r_step_seq_count += 1
                        r_step_seq_timer = 0.0
                    else:
                        r_step_seq_timer += dt_r
                        if r_step_seq_timer >= 0.65:
                            r_step_seq_count += 1
                            r_step_seq_timer = 0.0
                    was_r_step_fwd = True
                    if r_step_seq_count == 1:
                        act, ht, ft = "右足ステップ1回目（前進）", base_h, 0.30
                    else:
                        if r_step_seq_timer < 0.22:
                            act, ht, ft = f"右足ステップ{r_step_seq_count}回目（少し後ろへ下がる）", base_h, 0.09
                        else:
                            act, ht, ft = f"右足ステップ{r_step_seq_count}回目（1回目と同じ位置へ前進）", base_h, 0.30
                else:
                    was_r_step_fwd = False
                    act, ht, ft = "右足ステップ（元の位置へ戻る）", base_h, 0.0
            elif "左足ステップ" in act or l_step:
                was_r_step_fwd = False
                r_step_seq_count = 0
                is_back_phase = "戻る" not in act
                if is_back_phase:
                    if not was_l_step_back:
                        l_step_seq_count += 1
                        l_step_seq_timer = 0.0
                    else:
                        l_step_seq_timer += dt_r
                        if l_step_seq_timer >= 0.65:
                            l_step_seq_count += 1
                            l_step_seq_timer = 0.0
                    was_l_step_back = True
                    if l_step_seq_count == 1:
                        act, ht, ft = "左足ステップ1回目（後退）", base_h, -0.30
                    else:
                        if l_step_seq_timer < 0.22:
                            act, ht, ft = f"左足ステップ{l_step_seq_count}回目（少し前に向かう）", base_h, -0.09
                        else:
                            act, ht, ft = f"左足ステップ{l_step_seq_count}回目（1回目と同じ位置へ後退）", base_h, -0.30
                else:
                    was_l_step_back = False
                    act, ht, ft = "左足ステップ（元の位置へ戻る）", base_h, 0.0
            else:
                # 手上げ等の行も手の動きは判断せず、待機中（つなぎ動作）として次の足の動きへ繋げる
                was_r_step_fwd = was_l_step_back = False
                r_step_seq_count = l_step_seq_count = 0
                act = "待機中（初期位置）"
                ht = base_h
                ft = 0.0

        raw_valid.append({
            "t": rel_t,
            "raw_t": raw_t,
            "row": r,
            "action": act,
            "ht": ht,
            "ft": ft,
            "yaw": yaw,
            "dproj": dproj,
        })

    # 2) 「待機中（初期位置）」や「踊っている」の区間は、静止せず「次に認識される動き」へ滑らかに繋げる補間目標を与える
    n_v = len(raw_valid)
    idle_names = {"待機中（初期位置）", "踊っている"}
    i = 0
    while i < n_v:
        if raw_valid[i]["action"] in idle_names:
            j = i
            while j < n_v and raw_valid[j]["action"] in idle_names:
                j += 1
            prev_ht = raw_valid[i - 1]["ht"] if i > 0 else 0.0
            prev_ft = raw_valid[i - 1]["ft"] if i > 0 else 0.0
            if abs(prev_ht) >= 180.0:
                prev_ht = 0.0
            next_ht = raw_valid[j]["ht"] if j < n_v else 0.0
            next_ft = raw_valid[j]["ft"] if j < n_v else 0.0
            next_act = raw_valid[j]["action"] if j < n_v else "待機中"
            if abs(next_ht) >= 180.0:
                # 次が1回転ターンの場合は、ターンの方向へ予備旋回して滑らかに繋げる
                next_ht_bridge = 40.0 if next_ht > 0 else -40.0
            else:
                next_ht_bridge = next_ht

            span = max(1, j - i)
            for k in range(i, j):
                alpha = (k - i + 1) / (span + 1)
                # 滑らかなコサイン補間 ＋ 骨格の予備動作 (yaw, dproj) をブレンドして次の動きへ繋げる
                w_interp = 0.5 * (1.0 - math.cos(math.pi * alpha))
                rv = raw_valid[k]
                skel_h = clip(rv["yaw"], -105.0, 105.0)
                skel_f = clip(0.75 * rv["dproj"], -0.22, 0.22)
                bridge_h = (1.0 - w_interp) * prev_ht + w_interp * next_ht_bridge
                bridge_f = (1.0 - w_interp) * prev_ft + w_interp * next_ft
                rv["ht"] = clip(0.35 * bridge_h + 0.65 * skel_h, -110.0, 110.0)
                rv["ft"] = clip(0.70 * bridge_f + 0.30 * skel_f, -0.30, 0.30)
                short_next = next_act.split("（")[0]
                rv["action"] = f"つなぎ動作（次：{short_next}へ連携）"
            i = j
        else:
            i += 1

    # 3) Follower（ターン即時停止＆ゼロ遅延切り替え＆超高速追従）と WHILL出力計算
    fol_h = 0.0
    fol_f = 0.0
    in_turn = False
    HEADING_GAIN = 16.0
    FORWARD_GAIN = 12.0
    LINEAR_MAX = 0.60
    ANGULAR_MAX = 8.0
    FRONT_CAP = 55.0
    SIDE_CAP = 70.0
    BACK_GAIN = 1.3
    BACK_CAP = 70.0
    RAMP = 500.0
    whill_front = 0.0
    whill_side = 0.0

    out = []
    prev_rel_t = None

    for rv in raw_valid:
        rel_t = rv["t"]
        dt = 0.1 if prev_rel_t is None else clip(rel_t - prev_rel_t, 0.02, 0.25)
        prev_rel_t = rel_t

        ht = rv["ht"]
        ft = rv["ft"]

        for _ in range(2):
            sub_dt = dt / 2.0
            if abs(ht) < 180.0:
                if in_turn:
                    # turn実行中に次の動きへ切り替わったら即座に回るのをやめて目標の向きへ合わせる
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
                w_step = 0.0
                fol_h = ht
            else:
                w_step = clip(HEADING_GAIN * math.radians(err_h), -ANGULAR_MAX, ANGULAR_MAX)
                fol_h += math.degrees(w_step * sub_dt)

            if abs(err_f) < 0.004:
                v_step = 0.0
                fol_f = ft
            else:
                v_step = clip(FORWARD_GAIN * err_f, -LINEAR_MAX, LINEAR_MAX)
                fol_f += v_step * sub_dt

        tf = clip(v_step / 0.35, -1.0, 1.0) * FRONT_CAP
        if tf < 0:
            tf = max(-BACK_CAP, tf * BACK_GAIN)
        ts = -1.0 * clip(w_step / 3.0, -1.0, 1.0) * SIDE_CAP
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

        # 斜め前進・15°足上げ旋回・1回転ターンが大きく素早く見えるよう2D座標を計算
        if in_turn:
            turn_rad = math.radians(fol_h)
            s1_x = -0.14 * math.sin(turn_rad)
            s1_y = 0.14 * (1.0 - math.cos(turn_rad))
        else:
            vis_ang = math.radians(fol_h)
            swing_x = -0.22 * math.sin(vis_ang) if abs(fol_f) < 0.05 else 0.0
            s1_x = -fol_f * 2.1 * math.sin(vis_ang) + swing_x
            s1_y = fol_f * 1.9 * math.cos(vis_ang)

        out.append({
            "t": rel_t,
            "raw_t": rv["raw_t"],
            "ok": True,
            "row": rv["row"],
            "action": rv["action"],
            "ht": ht,
            "ft": ft,
            "s1_x": s1_x,
            "s1_y": s1_y,
            "s1_h": fol_h,
            "fol_f": fol_f,
            "v": v_step,
            "w": w_step,
            "f_pct": int(round(whill_front)),
            "s_pct": int(round(whill_side)),
        })

    return out


def draw_stick_figure(canvas, m_item, l_item, box_rect):
    bx0, by0, bx1, by1 = box_rect
    bw = bx1 - bx0
    bh = by1 - by0
    cv2.rectangle(canvas, (bx0, by0), (bx1, by1), (30, 30, 30), 1, cv2.LINE_AA)

    # mirukumunari_dance.csv の骨格座標を優先使用
    r = m_item["row"] if (m_item and m_item["ok"] and m_item["row"].get("hip_x", "") != "") else (l_item["row"] if l_item else None)
    if not r or r.get("hip_x", "") == "":
        return

    hx_raw = float(r["hip_x"])
    hy_raw = float(r["hip_y"])
    rkx_raw = float(r["r_knee_x"]) if r.get("r_knee_x", "") != "" else hx_raw - 0.02
    rky_raw = float(r["r_knee_y"]) if r.get("r_knee_y", "") != "" else hy_raw + 0.09
    lkx_raw = float(r["l_knee_x"]) if r.get("l_knee_x", "") != "" else hx_raw + 0.02
    lky_raw = float(r["l_knee_y"]) if r.get("l_knee_y", "") != "" else hy_raw + 0.09
    rax_raw = float(r["r_ankle_x"]) if r.get("r_ankle_x", "") != "" else rkx_raw
    ray_raw = float(r["r_ankle_y"]) if r.get("r_ankle_y", "") != "" else rky_raw + 0.09
    lax_raw = float(r["l_ankle_x"]) if r.get("l_ankle_x", "") != "" else lkx_raw
    lay_raw = float(r["l_ankle_y"]) if r.get("l_ankle_y", "") != "" else lky_raw + 0.09

    # mirukumunari_dance.csv は hx≈0.70、whill_live は hx≈0.49
    cx_ref = 0.65 if hx_raw > 0.58 else 0.50
    cy_ref = 0.56
    scale_x = 1.65
    scale_y = 1.85

    def to_px(nx, ny):
        px = int(round(bx0 + bw * (0.50 + (nx - cx_ref) * scale_x)))
        py = int(round(by0 + bh * (0.54 + (ny - cy_ref) * scale_y)))
        return (clip(px, bx0 + 6, bx1 - 6), clip(py, by0 + 6, by1 - 6))

    hip_pt = to_px(hx_raw, hy_raw)
    rk_pt = to_px(rkx_raw, rky_raw)
    lk_pt = to_px(lkx_raw, lky_raw)
    ra_pt = to_px(rax_raw, ray_raw)
    la_pt = to_px(lax_raw, lay_raw)

    yaw_deg = float(r["yaw"]) if r.get("yaw", "") != "" else 0.0
    yaw_rad = math.radians(yaw_deg)
    torso_h = 0.135
    sh_y_raw = hy_raw - torso_h
    sh_half_w = 0.036 * math.cos(yaw_rad)
    sh_cx_raw = hx_raw + 0.006 * math.sin(yaw_rad)

    r_sh_raw = (sh_cx_raw - sh_half_w, sh_y_raw)
    l_sh_raw = (sh_cx_raw + sh_half_w, sh_y_raw)
    sh_mid_raw = (sh_cx_raw, sh_y_raw)
    head_raw = (sh_cx_raw, sh_y_raw - 0.042)

    r_sh_pt = to_px(*r_sh_raw)
    l_sh_pt = to_px(*l_sh_raw)
    sh_mid_pt = to_px(*sh_mid_raw)
    head_pt = to_px(*head_raw)

    t = m_item["t"] if m_item else 0.0
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

    col = (45, 45, 45)
    lw = 3
    cv2.line(canvas, r_sh_pt, l_sh_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, r_sh_pt, hip_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, l_sh_pt, hip_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, sh_mid_pt, hip_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, r_sh_pt, r_el_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, r_el_pt, r_wr_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, l_sh_pt, l_el_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, l_el_pt, l_wr_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, hip_pt, rk_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, rk_pt, ra_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, hip_pt, lk_pt, col, lw, cv2.LINE_AA)
    cv2.line(canvas, lk_pt, la_pt, col, lw, cv2.LINE_AA)
    cv2.circle(canvas, head_pt, 11, col, -1, cv2.LINE_AA)


def draw_topdown(canvas, m_list, m_idx, l_list, l_idx, plot_rect):
    px0, py0, px1, py1 = plot_rect
    pw = px1 - px0
    ph = py1 - py0
    x_min, x_max = -2.0, 2.0
    y_min, y_max = -1.15, 1.15

    def world_to_px(wx, wy):
        u = (wx - x_min) / (x_max - x_min)
        v = (y_max - wy) / (y_max - y_min)
        return int(round(px0 + u * pw)), int(round(py0 + v * ph))

    grid_col = (232, 232, 232)
    for gx in [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]:
        x_p, _ = world_to_px(gx, 0.0)
        cv2.line(canvas, (x_p, py0), (x_p, py1), grid_col, 1, cv2.LINE_AA)
        cv2.line(canvas, (x_p, py1), (x_p, py1 + 5), (60, 60, 60), 1, cv2.LINE_AA)

    for gy in [-1.0, -0.5, 0.0, 0.5, 1.0]:
        _, y_p = world_to_px(0.0, gy)
        cv2.line(canvas, (px0, y_p), (px1, y_p), grid_col, 1, cv2.LINE_AA)
        cv2.line(canvas, (px0 - 5, y_p), (px0, y_p), (60, 60, 60), 1, cv2.LINE_AA)

    cv2.rectangle(canvas, (px0, py0), (px1, py1), (30, 30, 30), 1, cv2.LINE_AA)

    t_cx0 = -1.05
    s1_cx0 = 0.85

    # お手本の軌跡テール
    t_pts = []
    for k in range(max(0, m_idx - 25), m_idx + 1):
        mk = m_list[k]
        t_pts.append(world_to_px(t_cx0 + mk["t_x"], mk["t_y"]))
    if len(t_pts) >= 2:
        cv2.polylines(canvas, [np.array(t_pts, dtype=np.int32)], False, (185, 180, 175), 2, cv2.LINE_AA)

    # whill_live の軌跡テール
    s1_pts = []
    for k in range(max(0, l_idx - 25), l_idx + 1):
        lk = l_list[k]
        s1_pts.append(world_to_px(s1_cx0 + lk["s1_x"], lk["s1_y"]))
    if len(s1_pts) >= 2:
        cv2.polylines(canvas, [np.array(s1_pts, dtype=np.int32)], False, (215, 175, 120), 2, cv2.LINE_AA)

    def draw_triangle(wx, wy, heading_deg, bgr_color):
        rad = math.radians(heading_deg)
        local_verts = [(0.0, 0.18), (-0.105, -0.11), (0.105, -0.11)]
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        poly = []
        for lx, ly in local_verts:
            rx = lx * cos_a - ly * sin_a
            ry = lx * sin_a + ly * cos_a
            poly.append(world_to_px(wx + rx, wy + ry))
        cv2.fillPoly(canvas, [np.array(poly, dtype=np.int32)], bgr_color, cv2.LINE_AA)

    m_cur = m_list[m_idx]
    l_cur = l_list[l_idx]
    draw_triangle(t_cx0 + m_cur["t_x"], m_cur["t_y"], m_cur["t_h"], (155, 148, 142))
    draw_triangle(s1_cx0 + l_cur["s1_x"], l_cur["s1_y"], l_cur["s1_h"], (180, 130, 18))


def render_comparison(m_list, l_list, start_t, end_t, out_path, fps=15.0):
    W, H = 1280, 720
    font_title = get_jp_font(20, bold=False)
    font_label = get_jp_font(15, bold=False)
    font_badge = get_jp_font(17, bold=True)
    font_tick = get_jp_font(14, bold=False)
    font_time = get_jp_font(18, bold=False)

    left_box = (38, 230, 550, 520)
    right_box = (614, 182, 1254, 568)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, (W, H))

    bg = np.full((H, W, 3), 255, dtype=np.uint8)
    bg_pil = Image.fromarray(cv2.cvtColor(bg, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(bg_pil)

    draw.text((294, 200), "カメラから見た踊る人 (mirukumunari_dance.csv)", font=font_title, fill=(20, 20, 20), anchor="mm")
    draw.text((934, 152), "WHILL を上から見た動き (お手本 vs whill_live)", font=font_title, fill=(20, 20, 20), anchor="mm")

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

    u_t = (-1.05 - x_min) / (x_max - x_min)
    u_s1 = (0.85 - x_min) / (x_max - x_min)
    y_leg = int(round(py0 + ((y_max - (-1.02)) / (y_max - y_min)) * ph))
    draw.text((int(round(px0 + u_t * pw)), y_leg), "お手本 (mirukumunari_dance.csv)", font=font_label, fill=(135, 140, 146), anchor="mm")
    draw.text((int(round(px0 + u_s1 * pw)), y_leg), "第1段階 (whill_live_20261118_122224)", font=font_label, fill=(18, 130, 180), anchor="mm")

    base_bgr = cv2.cvtColor(np.array(bg_pil), cv2.COLOR_RGB2BGR)

    m_times = [x["t"] for x in m_list]
    l_times = [x["t"] for x in l_list]
    total_dur = end_t - start_t
    n_frames = int(round(total_dur * fps))

    for f_idx in range(n_frames):
        cur_t = start_t + f_idx / fps
        m_pos = min(len(m_list) - 1, max(0, bisect.bisect_left(m_times, cur_t)))
        l_pos = min(len(l_list) - 1, max(0, bisect.bisect_left(l_times, cur_t)))

        canvas = base_bgr.copy()
        draw_stick_figure(canvas, m_list[m_pos], l_list[l_pos], left_box)
        draw_topdown(canvas, m_list, m_pos, l_list, l_pos, right_box)

        pil_img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(pil_img)

        l_cur = l_list[l_pos]
        act = l_cur["action"]
        f_pct = l_cur["f_pct"]
        s_pct = l_cur["s_pct"]

        act_col = (18, 130, 180) if act not in ("待機中（初期位置）", "踊っている") else (110, 115, 120)
        d.text((294, 548), f"whill_live 動作：{act}", font=font_badge, fill=act_col, anchor="mm")
        d.text(
            (int(round(px0 + u_s1 * pw)), py0 + 26),
            f"WHILL指令: 前後 {f_pct:+d}% / 旋回 {s_pct:+d}%  (向き {l_cur['s1_h']:+.0f}°, 前後 {l_cur['fol_f']:+.2f}m)",
            font=font_tick,
            fill=(18, 130, 180),
            anchor="mm",
        )

        rel_sec = cur_t - start_t
        d.text((1255, 696), f"{rel_sec:.1f} 秒 (ダンス時刻 {cur_t:.1f}s / live {l_cur['raw_t']:.1f}s)", font=font_time, fill=(20, 20, 20), anchor="rm")

        writer.write(cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR))

    writer.release()
    print(f"Saved: {out_path} ({n_frames} frames, {total_dur:.1f}s)")


def main():
    ap = argparse.ArgumentParser(description="お手本の CSV と WHILL の実測の CSV を横並びの動画にする")
    ap.add_argument("--dir", default=HERE, help="CSV を探す・動画を書くフォルダ（既定：このスクリプトのフォルダ）")
    ap.add_argument("--miruku", help="お手本の CSV（既定：--dir の mirukumunari_dance.csv）")
    ap.add_argument("--live", help="whill_bridge.py --csv の記録（既定：--dir のいちばん新しい whill_live_*.csv）")
    ap.add_argument("--out-dir", help="動画を書くフォルダ（既定：--dir）")
    a = ap.parse_args()
    miruku_csv = a.miruku or os.path.join(a.dir, "mirukumunari_dance.csv")
    live_csv = a.live
    if not live_csv:
        found = sorted(glob.glob(os.path.join(a.dir, "whill_live_*.csv")))
        if not found:
            raise SystemExit(f"{a.dir} に whill_live_*.csv がありません（--live で指定してください）")
        live_csv = found[-1]
    out_dir = a.out_dir or a.dir
    os.makedirs(out_dir, exist_ok=True)
    OUT_30S = os.path.join(out_dir, "mirukumunari_vs_whill_live_30s.mp4")
    OUT_FULL = os.path.join(out_dir, "mirukumunari_vs_whill_live_full.mp4")
    print(f"お手本: {miruku_csv}\n実測: {live_csv}")

    m_list = load_miruku(miruku_csv)
    l_list = load_whill_live(live_csv)
    print(f"Loaded mirukumunari: {len(m_list)} rows (0..{m_list[-1]['t']}s)")
    print(f"Loaded whill_live (filtered): {len(l_list)} rows (0..{l_list[-1]['t']}s)")

    # 1) 動きが一番豊富な30秒間ハイライト (ダンス時刻 15.0s〜45.0s)
    render_comparison(m_list, l_list, 15.0, 45.0, OUT_30S, fps=15.0)

    # 2) 全編動画 (ダンス時刻 0.0s〜220.0s, 10fps で軽快に全編再生)
    max_t = min(m_list[-1]["t"], l_list[-1]["t"])
    render_comparison(m_list, l_list, 0.0, max_t, OUT_FULL, fps=10.0)


if __name__ == "__main__":
    main()
