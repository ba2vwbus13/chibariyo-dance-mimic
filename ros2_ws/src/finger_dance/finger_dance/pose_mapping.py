import math
import time
import collections

# 点の番号（MediaPipe の決まり。左右は本人の左右）
NOSE, L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 0, 11, 12, 13, 14, 15, 16
L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANKLE, R_ANKLE = 23, 24, 25, 26, 27, 28
L_HEEL, R_HEEL, L_FOOT, R_FOOT = 29, 30, 31, 32


def _alpha(dt, tau):
    """時定数 tau [s] のなめらかさ（指数平均）の係数。"""
    return math.exp(-dt / tau) if tau > 0 else 0.0


def _in_frame(pt, min_vis=0.25, max_y=0.96):
    """関節点がカメラ画面内に映っており、かつ信頼度が十分かを判定する。"""
    return pt[2] >= min_vis and 0.01 < pt[0] < 0.99 and 0.02 < pt[1] < max_y


class DanceMapper:
    """
    骨格座標の動きからロボットの目標向き (heading [deg]) と目標前後位置 (forward [m]) を計算する。
    全身が映っていない場合（「胸から足まで」「腰から足まで」「上半身のみ」など）でも、
    その時点で画面内に映っている関節を使って正確に動きを判定する。

    優先順位（高 → 低）：
      1. しこ立ち：腰（および映っていれば肩・膝との距離）がY軸方向（下）へ沈んだとき。
                   Z軸方向に少し後ろに下がり、立ち上がった時にZ軸に前に進み元の位置にもどる
      2. 足上げ：左右どちらかの足（足首・膝・腰）がY軸方向（上）に移動したとき。
                 外側に15度回転して元の位置にもどる
      3. ステップ（手が上がっておらず、映っている足が体から前に踏み出したとき）：
         ・右足ステップ（右足を体から前に出す）：Z軸に前に行って元の位置にもどる
         ・左足ステップ（左足を体から前に出す）：Z軸に後ろに行って元の位置にもどる
      4. 手上げ（優先順位・最低）：上記1〜3のどれも行われていないときのみ反応
    """
    STEP_DIST = 0.30       # ステップ・しこ立ちでの前後移動量 [m]（大きく素早く前後移動）
    STEP_HOLD = 0.40       # 行き（前進または後退）にかける時間 [s]（お手本のテンポに素早く追従）
    STEP_PULLBACK = 0.22   # 連続ステップ（2回目以降）で一度少し引く時間 [s]
    STEP_RETURN = 0.35     # 元の位置に戻るのにかける時間 [s]
    STEP_COOLDOWN = 0.04   # 動作完了後の最小クールダウン [s]
    LIFT_ANGLE = 38.0      # 足上げ時の斜め前への角度 [deg]（左足上げ: 左斜め前 +38°, 右足上げ: 右斜め前 -38°）
    LIFT_DIST = 0.28       # 足上げ時の斜め前への前進距離 [m]
    LIFT_MIN_HOLD = 0.38   # 足上げ（斜め前進）の保持時間 [s]
    LIFT_RETURN = 0.32     # 足上げから元の位置・向きに戻る時間 [s]
    TURN_HOLD = 0.80       # ターン（1回転＝360°）にかける時間 [s]（0.8秒で素早く1回転）
    SQUAT_MAX_HOLD = 1.8   # しこ立ち状態の最大保持時間（誤ロック防止）[s]

    def __init__(self, yaw_gain=1.0, sway_gain=60.0, bounce_gain=0.4, heading_limit=110.0, forward_limit=0.40,
                 yaw_tau=0.06, sway_tau=1.3, bounce_tau=1.5, yaw_clip=110.0):
        self.yaw_gain = yaw_gain
        self.sway_gain = sway_gain
        self.bounce_gain = bounce_gain
        self.heading_limit = heading_limit
        self.forward_limit = forward_limit
        self.yaw_tau, self.sway_tau, self.bounce_tau = yaw_tau, sway_tau, bounce_tau
        self.yaw_clip = yaw_clip
        self.reset()

    def reset(self):
        self.yaw_s = None
        self.hip_x0 = None
        self.hip_y0 = None
        self.sh_y0 = None
        self.torso = None
        self.l_leg_z0 = None
        self.r_leg_z0 = None
        self.l_3dx0 = None
        self.r_3dx0 = None
        self.z_diff0 = None
        self.l_knee_y0 = None
        self.r_knee_y0 = None
        self.l_ankle_y0 = None
        self.r_ankle_y0 = None
        self.knee_hip_ratio0 = None
        self.prev_f = None
        self.r_step_confirm = 0
        self.l_step_confirm = 0
        self.r_step_count = 0
        self.l_step_count = 0
        self.turn_confirm = 0
        self.last_front_yaw = 0.0
        self.groove_phase = 0.0
        self.warmup_time = 0.0

        # アクション状態 ('idle', 'squat', 'squat_return', 'l_turn', 'r_turn', 'l_lift', 'r_lift', 'r_step', 'l_step')
        self.mode = 'idle'
        self.mode_timer = 0.0
        self.cooldown = 0.0
        self.action_name = '踊っている'

    def update(self, f, dt):
        """戻り値: (ロボットの目標向き [deg], 目標前後位置 [m])"""
        raw_yaw = f['yaw']
        yaw = max(-self.yaw_clip, min(self.yaw_clip, raw_yaw))
        sh_y = f.get('sh_y', f['hip_y'] - f['torso'])
        l_3dx = f.get('l_3dx', 0.0)
        r_3dx = f.get('r_3dx', 0.0)

        if self.yaw_s is None:
            self.yaw_s = yaw
            self.hip_x0 = f['hip_x']
            self.hip_y0 = f['hip_y']
            self.sh_y0 = sh_y
            self.torso = f['torso']
            self.l_leg_z0 = f['l_leg_z']
            self.r_leg_z0 = f['r_leg_z']
            self.l_3dx0 = l_3dx
            self.r_3dx0 = r_3dx
            self.z_diff0 = f['r_leg_z'] - f['l_leg_z']
            if f.get('has_knees'):
                self.l_knee_y0 = f['l_knee_y']
                self.r_knee_y0 = f['r_knee_y']
                if f.get('has_hips'):
                    self.knee_hip_ratio0 = ((f['l_knee_y'] - f['l_hip_y']) + (f['r_knee_y'] - f['r_hip_y'])) / (2.0 * f['torso'])
            if f.get('has_ankles'):
                self.l_ankle_y0 = f['l_ankle_y']
                self.r_ankle_y0 = f['r_ankle_y']
            self.prev_f = f.copy()
            self.warmup_time = 0.4

        # カメラに入った直後のウォームアップ中は基準値を素早く合わせる
        if self.warmup_time > 0:
            self.warmup_time = max(0.0, self.warmup_time - dt)
            self.hip_x0 = f['hip_x']
            self.hip_y0 = f['hip_y']
            self.sh_y0 = sh_y
            self.torso = f['torso']
            self.l_leg_z0 = f['l_leg_z']
            self.r_leg_z0 = f['r_leg_z']
            self.l_3dx0 = l_3dx
            self.r_3dx0 = r_3dx
            self.z_diff0 = f['r_leg_z'] - f['l_leg_z']
            if f.get('has_knees'):
                self.l_knee_y0, self.r_knee_y0 = f['l_knee_y'], f['r_knee_y']
                if f.get('has_hips'):
                    self.knee_hip_ratio0 = ((f['l_knee_y'] - f['l_hip_y']) + (f['r_knee_y'] - f['r_hip_y'])) / (2.0 * f['torso'])
            if f.get('has_ankles'):
                self.l_ankle_y0, self.r_ankle_y0 = f['l_ankle_y'], f['r_ankle_y']

        # 上半身の平滑化
        a_yaw = _alpha(dt, self.yaw_tau)
        self.yaw_s = a_yaw * self.yaw_s + (1 - a_yaw) * yaw
        a_sway = _alpha(dt, self.sway_tau)
        self.hip_x0 = a_sway * self.hip_x0 + (1 - a_sway) * f['hip_x']
        self.torso = a_sway * self.torso + (1 - a_sway) * f['torso']

        # フラグ初期化
        f['squat'] = False
        f['turn'] = False
        f['r_step_fwd'] = False
        f['l_step_back'] = False
        f['l_knee_lift'] = False
        f['r_knee_lift'] = False

        if 20.0 <= abs(raw_yaw) <= 105.0:
            self.last_front_yaw = raw_yaw

        sway = (f['hip_x'] - self.hip_x0) / self.torso
        hip_drop = (f['hip_y'] - self.hip_y0) / self.torso if f.get('has_hips') else 0.0
        sh_drop = (sh_y - self.sh_y0) / self.torso if f.get('has_shoulders') else hip_drop

        # 骨格の瞬間判定フラグ
        squat_now = False
        l_turn_now = False
        r_turn_now = False
        l_lift_now = False
        r_lift_now = False
        r_step_now = False
        l_step_now = False
        knee_hip_shrink = 0.0
        l_knee_rel = r_knee_rel = 0.0
        l_ankle_rel = r_ankle_rel = 0.0
        l_y_up = r_y_up = 0.0

        # 前フレームからの速度計算
        sh_vy = hip_vy = 0.0
        l_leg_vz = r_leg_vz = 0.0
        l_leg_move = r_leg_move = 0.0
        wrist_move = 0.0
        yaw_vel = 0.0

        if self.prev_f and dt > 0:
            prev_sh_y = self.prev_f.get('sh_y', self.prev_f['hip_y'] - self.prev_f['torso'])
            sh_vy = (sh_y - prev_sh_y) / self.torso / dt
            hip_vy = (f['hip_y'] - self.prev_f['hip_y']) / self.torso / dt
            l_leg_vz = (f['l_leg_z'] - self.prev_f['l_leg_z']) / dt
            r_leg_vz = (f['r_leg_z'] - self.prev_f['r_leg_z']) / dt
            l_leg_move = f.get('l_leg_move_step', 0.0) / dt
            r_leg_move = f.get('r_leg_move_step', 0.0) / dt
            wrist_move = f.get('wrist_move_step', 0.0) / dt
            yaw_vel = (raw_yaw - self.prev_f.get('yaw', raw_yaw)) / dt

        if self.warmup_time <= 0.0:
            # ------------------------------------------------------------------
            # 1. 足上げ（Y軸方向の上移動）の特徴量計算
            #    ※足を前に出す「ステップ」でも膝がわずかに上がるため、
            #      左右の足首・膝の明確な高さ差（Y軸の上方向移動）で足上げを判定する
            # ------------------------------------------------------------------
            l_hip_rel = (f['r_hip_y'] - f['l_hip_y']) / self.torso if f.get('has_hips') else 0.0
            r_hip_rel = (f['l_hip_y'] - f['r_hip_y']) / self.torso if f.get('has_hips') else 0.0

            l_knee_rel = r_knee_rel = 0.0
            l_knee_up0 = r_knee_up0 = 0.0
            if f.get('has_knees'):
                if self.l_knee_y0 is None:
                    self.l_knee_y0, self.r_knee_y0 = f['l_knee_y'], f['r_knee_y']
                l_knee_rel = (f['r_knee_y'] - f['l_knee_y']) / f['torso']
                r_knee_rel = (f['l_knee_y'] - f['r_knee_y']) / f['torso']
                l_knee_up0 = (self.l_knee_y0 - f['l_knee_y']) / f['torso']
                r_knee_up0 = (self.r_knee_y0 - f['r_knee_y']) / f['torso']

                if f.get('has_hips'):
                    cur_ratio = ((f['l_knee_y'] - f['l_hip_y']) + (f['r_knee_y'] - f['r_hip_y'])) / (2.0 * f['torso'])
                    if self.knee_hip_ratio0 is None:
                        self.knee_hip_ratio0 = cur_ratio
                    knee_hip_shrink = self.knee_hip_ratio0 - cur_ratio

            l_ankle_rel = r_ankle_rel = 0.0
            l_ankle_up0 = r_ankle_up0 = 0.0
            ankle_y_rise = 0.0
            if f.get('has_ankles'):
                if self.l_ankle_y0 is None:
                    self.l_ankle_y0, self.r_ankle_y0 = f['l_ankle_y'], f['r_ankle_y']
                l_ankle_rel = (f['r_ankle_y'] - f['l_ankle_y']) / f['torso']
                r_ankle_rel = (f['l_ankle_y'] - f['r_ankle_y']) / f['torso']
                l_ankle_up0 = (self.l_ankle_y0 - f['l_ankle_y']) / f['torso']
                r_ankle_up0 = (self.r_ankle_y0 - f['r_ankle_y']) / f['torso']
                ankle_y_rise = ((self.l_ankle_y0 - f['l_ankle_y']) + (self.r_ankle_y0 - f['r_ankle_y'])) / (2.0 * f['torso'])

            # 片足を地面から真上に持ち上げた量（基準位置からの上昇 と 反対足との上昇差 の両方が揃ったときのみ反応）
            # ※後ろへ歩いて両足とも画面上で上がった場合（l_ankle_up0 ≈ r_ankle_up0）や、
            #   反対足を前にステップして画面下へ移動した場合（反対足の up0 < -0.05）は除外する
            l_knee_diff0 = l_knee_up0 - r_knee_up0
            r_knee_diff0 = r_knee_up0 - l_knee_up0
            l_ank_diff0 = l_ankle_up0 - r_ankle_up0
            r_ank_diff0 = r_ankle_up0 - l_ankle_up0
            l_knee_up = min(l_knee_up0, l_knee_rel, l_knee_diff0)
            r_knee_up = min(r_knee_up0, r_knee_rel, r_knee_diff0)
            l_ank_up = min(l_ankle_up0, l_ankle_rel, l_ank_diff0)
            r_ank_up = min(r_ankle_up0, r_ankle_rel, r_ank_diff0)
            if f.get('has_knees') and f.get('has_ankles'):
                l_y_up = max(l_knee_up, l_ank_up) if (l_knee_up > 0.09 and abs(r_knee_up0) < 0.08 and r_ankle_up0 > -0.05 and abs(r_ankle_up0) < 0.09) else 0.0
                r_y_up = max(r_knee_up, r_ank_up) if (r_knee_up > 0.09 and abs(l_knee_up0) < 0.08 and l_ankle_up0 > -0.05 and abs(l_ankle_up0) < 0.09) else 0.0
            elif f.get('has_knees') or f.get('has_ankles'):
                l_y_up = max(l_knee_up, l_ank_up)
                r_y_up = max(r_knee_up, r_ank_up)
            else:
                l_y_up = l_hip_rel * 1.1
                r_y_up = r_hip_rel * 1.1
            lr_y_asymmetry = abs(max(l_ankle_rel, l_knee_rel) - max(r_ankle_rel, r_knee_rel))

            # ------------------------------------------------------------------
            # 2. 足を体から前に出す「ステップ」スコア計算（正面向き・斜め・横向きの全対応）
            #    ・2D脚軸方向の伸展差 (dproj_a, dproj_f)：カメラが斜め上から見下ろして脚が斜めに映る環境でも、
            #      直立時は 0 になり、右足を前に出すと +、左足を前に出すと - になるロバスト指標
            #    ・3D前方投影 (r_ft_fwd, l_ft_fwd) および 3D奥行き差 (dz_ank, dz_ft)
            #    ・2D歩幅比率 (r_dx2d, l_dx2d, r_side2d, l_side2d)
            # ------------------------------------------------------------------
            r_zf = self.r_leg_z0 - f['r_leg_z']
            l_zf = self.l_leg_z0 - f['l_leg_z']
            r_ft_fwd = f.get('r_ft_fwd', 0.0)
            l_ft_fwd = f.get('l_ft_fwd', 0.0)
            r_dx2d = f.get('r_dx2d', 0.0)
            l_dx2d = f.get('l_dx2d', 0.0)
            r_side2d = f.get('r_side2d', 0.0)
            l_side2d = f.get('l_side2d', 0.0)
            dproj_a = f.get('dproj_a', 0.0)
            dproj_f = f.get('dproj_f', dproj_a)
            dy_ank = l_ankle_rel  # (r_ankle_y - l_ankle_y) / torso (>0: 右足首が画面下＝手前, <0: 左足首が画面下＝手前)
            dz_ank = f.get('dz_ank', 0.0)

            r_step_score = max(r_zf, r_ft_fwd, dproj_a)
            l_step_score = max(l_zf, l_ft_fwd, -dproj_a)

            # ------------------------------------------------------------------
            # 【優先順位 1】 しこ立ち（しゃがみ）の検出
            #   ・カメラから後ろに歩いて離れたとき（ankle_y_rise > 0.08 や torso の急縮小）は誤判定しない
            #   ・「胸から足まで」など腰と膝が映っている場合：腰が下がり、かつ腰と膝の距離が縮まったとき
            #   ・「上半身〜腰」のみ映っている場合：肩と腰の両方が同時に15%以上下がったとき
            # ------------------------------------------------------------------
            walking_away = abs(ankle_y_rise) > 0.07 or (f['torso'] < self.torso * 0.90) or abs(dproj_a) > 0.09
            if not walking_away and lr_y_asymmetry < 0.15 and l_y_up < 0.14 and r_y_up < 0.14:
                if f.get('has_knees') and f.get('has_hips'):
                    if (hip_drop > 0.14 and knee_hip_shrink > 0.10) or (hip_vy > 0.80 and hip_drop > 0.10 and knee_hip_shrink > 0.07):
                        squat_now = True
                elif f.get('has_shoulders') and f.get('has_hips'):
                    if (sh_drop > 0.16 and hip_drop > 0.15) or (sh_vy > 0.80 and hip_vy > 0.80 and sh_drop > 0.10 and hip_drop > 0.10):
                        squat_now = True

            # ------------------------------------------------------------------
            # 【優先順位 2】 ターン（HALF_TURN: abs(raw_yaw) > 110°）の検出
            #   人が振り返ってターンしたとき、ターンした方向へ車いすも1回転（±360°）する
            # ------------------------------------------------------------------
            if abs(raw_yaw) > 110.0 and not squat_now:
                self.turn_confirm += 1
            else:
                self.turn_confirm = max(0, self.turn_confirm - 1)

            if self.turn_confirm >= 2:
                if self.last_front_yaw < -15.0 or (abs(self.last_front_yaw) <= 15.0 and raw_yaw < 0.0):
                    r_turn_now = True
                else:
                    l_turn_now = True

            # ------------------------------------------------------------------
            # 【優先順位 2】 足上げ（左右どちらかの足がY軸方向に上に移動）の検出
            #   しこ立ちではなく、片方の膝または足首が基準位置（up0）および軸足（rel）の両方に対して
            #   Y軸方向（真上）へ 14% 以上持ち上がったときに検出する（※ターン中であっても足上げが来たら即座に割り込めるよう常時判定）
            # ------------------------------------------------------------------
            sideways_stride = abs(raw_yaw) >= 22.0 and max(r_side2d, l_side2d) > 1.7
            if not squat_now and hip_drop < 0.12 and not sideways_stride:
                if l_y_up > 0.14 and l_y_up > r_y_up + 0.08:
                    l_lift_now = True
                elif r_y_up > 0.14 and r_y_up > l_y_up + 0.08:
                    r_lift_now = True

            # ------------------------------------------------------------------
            # 【優先順位 3】 ステップ（右足/左足を体から前に出す動作）の検出
            #   ※誤判定ガード：
            #     1) 手が肩より上に上がっている（l_up / r_up）ときは除外
            #     2) 足上げ中、または片足の膝・足首をY軸方向（上）に持ち上げ始めている途中（lifting_upward）は除外
            #     3) 後ろ向き（abs(raw_yaw) >= 85 deg）や急旋回中は除外
            #     4) 膝または足首が画面内に映っているときのみ判定する
            # ------------------------------------------------------------------
            # ※手の動きは骨格推定で判断しない（hands_active は常に False）
            hands_active = False
            # 足上げの途中（軸足が静止していて、もう片方の膝が基準位置からY軸上方向へ浮き上がり始めている状態）はステップをブロック
            if f.get('has_knees'):
                l_lifting_early = (l_knee_up > 0.085 and abs(r_knee_up0) < 0.08 and r_ankle_up0 > -0.04 and abs(r_ankle_up0) < 0.08)
                r_lifting_early = (r_knee_up > 0.085 and abs(l_knee_up0) < 0.08 and l_ankle_up0 > -0.04 and abs(l_ankle_up0) < 0.08)
            else:
                l_lifting_early = (l_ank_up > 0.12 and r_ankle_up0 > -0.04 and abs(r_ankle_up0) < 0.08)
                r_lifting_early = (r_ank_up > 0.12 and l_ankle_up0 > -0.04 and abs(l_ankle_up0) < 0.08)
            lifting_upward = (not sideways_stride) and (l_lifting_early or r_lifting_early or max(l_y_up, r_y_up) > 0.10)
            any_y_lifting = l_lift_now or r_lift_now or lifting_upward
            turning_body = abs(yaw_vel) > 260.0 or abs(raw_yaw) >= 105.0

            if f.get('has_lower_legs', False) and not squat_now and not any_y_lifting and not hands_active and not turning_body:
                if abs(raw_yaw) < 28.0 and not sideways_stride:
                    # カメラに正面向き〜やや斜め向きのとき：
                    # 1) 脚軸方向の伸展差 (dproj_a, dproj_f)：少し足を前に出すステップでも確実に反応
                    # 2) 腰幅に対する2D横踏み出し比率 (r_dx2d, l_dx2d)：大きく踏み出すステップで反応
                    # 3) 足先が見切れているときの3D奥行き変化 (r_zf, l_zf)
                    r_moved_fwd = (
                        (dproj_a > 0.065 and (dproj_f > 0.030 or dproj_a > 0.090) and (dy_ank > 0.080 or dproj_f > 0.12)) or
                        (r_dx2d > 0.35 and l_dx2d < 0.05 and r_ft_fwd > -0.025) or
                        (r_zf > 0.065 and r_zf > l_zf + 0.040 and not f.get('has_feet'))
                    )
                    l_moved_fwd = (
                        (-dproj_a > 0.080 and -dproj_f > 0.055 and (dy_ank < 0.065 or dz_ank < -0.025)) or
                        (l_dx2d > 0.18 and -0.55 < r_dx2d < -0.15 and l_ft_fwd > -0.005 and (l_ft_fwd - r_ft_fwd) > 0.020) or
                        (l_zf > 0.065 and l_zf > r_zf + 0.040 and not f.get('has_feet'))
                    )
                else:
                    # カメラに横向き・大きく斜め向きのとき：体の向いている方向へ足が前に踏み出したとき
                    r_moved_fwd = (
                        (r_ft_fwd > 0.045 and (r_ft_fwd - l_ft_fwd) > 0.05) or
                        (r_side2d > 1.8 and l_side2d < 1.2 and r_ft_fwd > -0.08)
                    )
                    l_moved_fwd = (
                        (l_ft_fwd > 0.080 and (l_ft_fwd - r_ft_fwd) > 0.08) or
                        (l_side2d > 2.2 and l_ft_fwd > 0.10)
                    )

                if r_moved_fwd and not l_moved_fwd:
                    self.r_step_confirm += 1
                    self.l_step_confirm = 0
                elif l_moved_fwd and not r_moved_fwd:
                    self.l_step_confirm += 1
                    self.r_step_confirm = 0
                else:
                    self.r_step_confirm = max(0, self.r_step_confirm - 1)
                    self.l_step_confirm = max(0, self.l_step_confirm - 1)

                # ターン中や直後は遅延ゼロで即ステップへ移れるよう 1 フレームで確定する
                step_confirm_need = 1 if self.mode in ('l_turn', 'r_turn') else 2
                if self.r_step_confirm >= step_confirm_need:
                    r_step_now = True
                elif self.l_step_confirm >= step_confirm_need:
                    l_step_now = True
            else:
                self.r_step_confirm = 0
                self.l_step_confirm = 0

        # 静止中（idle かつ足のアクションが起きていないとき）は基準位置を更新
        if self.mode == 'idle' and not (squat_now or l_lift_now or r_lift_now or r_step_now or l_step_now):
            a_bounce = _alpha(dt, self.bounce_tau)
            self.hip_y0 = a_bounce * self.hip_y0 + (1 - a_bounce) * f['hip_y']
            self.sh_y0 = a_bounce * self.sh_y0 + (1 - a_bounce) * sh_y
            # 体の向きを変えている最中は新しい向きの立ち位置へ素早く合わせ、
            # 足を踏み出し始めている途中は基準位置が足を追いかけないように固定する
            step_in_progress = (self.r_step_confirm > 0 or self.l_step_confirm > 0)
            if abs(yaw_vel) > 25.0:
                az = _alpha(dt, 0.15)
                self.l_leg_z0 = az * self.l_leg_z0 + (1 - az) * f['l_leg_z']
                self.r_leg_z0 = az * self.r_leg_z0 + (1 - az) * f['r_leg_z']
                self.l_3dx0 = az * self.l_3dx0 + (1 - az) * l_3dx
                self.r_3dx0 = az * self.r_3dx0 + (1 - az) * r_3dx
                self.z_diff0 = az * self.z_diff0 + (1 - az) * (f['r_leg_z'] - f['l_leg_z'])
            elif not step_in_progress:
                az = _alpha(dt, 1.2)
                if abs(f['r_leg_z'] - f['l_leg_z']) < 0.045:
                    self.l_leg_z0 = az * self.l_leg_z0 + (1 - az) * f['l_leg_z']
                    self.r_leg_z0 = az * self.r_leg_z0 + (1 - az) * f['r_leg_z']
                    self.l_3dx0 = az * self.l_3dx0 + (1 - az) * l_3dx
                    self.r_3dx0 = az * self.r_3dx0 + (1 - az) * r_3dx
                    self.z_diff0 = az * self.z_diff0 + (1 - az) * (f['r_leg_z'] - f['l_leg_z'])
            az_y = _alpha(dt, 0.6)
            if f.get('has_knees') and self.l_knee_y0 is not None:
                self.l_knee_y0 = az_y * self.l_knee_y0 + (1 - az_y) * f['l_knee_y']
                self.r_knee_y0 = az_y * self.r_knee_y0 + (1 - az_y) * f['r_knee_y']
                if f.get('has_hips') and self.knee_hip_ratio0 is not None:
                    cur_ratio = ((f['l_knee_y'] - f['l_hip_y']) + (f['r_knee_y'] - f['r_hip_y'])) / (2.0 * f['torso'])
                    self.knee_hip_ratio0 = az_y * self.knee_hip_ratio0 + (1 - az_y) * cur_ratio
            if f.get('has_ankles') and self.l_ankle_y0 is not None:
                self.l_ankle_y0 = az_y * self.l_ankle_y0 + (1 - az_y) * f['l_ankle_y']
                self.r_ankle_y0 = az_y * self.r_ankle_y0 + (1 - az_y) * f['r_ankle_y']

        if self.cooldown > 0:
            self.cooldown = max(0.0, self.cooldown - dt)

        # ==================================================================
        # 優先順位＆シームレス即時割り込み遷移
        #   ・turn実行中に次の動き（しこ立ち・足上げ・ステップ・手上げ・正面復帰）が検出された場合は、
        #     即座に回るのをやめて（遅延ゼロで）次の動きに移る
        #   ・その他の動作実行中も、別の動きが検出されたら待たずに滑らかに次の動作へ遷移する
        # ==================================================================
        self.mode_timer += dt

        if self.mode in ('l_turn', 'r_turn'):
            if squat_now:
                self.mode = 'squat'
                self.mode_timer = 0.0
                self.turn_confirm = 0
                self.r_step_count = self.l_step_count = 0
            elif l_lift_now:
                self.mode = 'l_lift'
                self.mode_timer = 0.0
                self.turn_confirm = 0
                self.r_step_count = self.l_step_count = 0
            elif r_lift_now:
                self.mode = 'r_lift'
                self.mode_timer = 0.0
                self.turn_confirm = 0
                self.r_step_count = self.l_step_count = 0
            elif r_step_now:
                self.mode = 'r_step'
                self.r_step_count = 1
                self.l_step_count = 0
                self.mode_timer = 0.0
                self.turn_confirm = 0
            elif l_step_now:
                self.mode = 'l_step'
                self.l_step_count = 1
                self.r_step_count = 0
                self.mode_timer = 0.0
                self.turn_confirm = 0
            elif self.mode_timer >= 0.20 and abs(raw_yaw) < 85.0:
                # 踊り手が正面付近に戻ったら即座にターンを終了し、次の動作へ繋げる
                self.mode = 'idle'
                self.mode_timer = 0.0
                self.turn_confirm = 0
                self.cooldown = 0.20
                self.r_step_count = self.l_step_count = 0
        else:
            if squat_now and self.mode != 'squat':
                self.mode = 'squat'
                self.mode_timer = 0.0
                self.r_step_count = self.l_step_count = 0
            elif (l_turn_now or r_turn_now) and self.mode != 'squat' and self.cooldown <= 0.0:
                self.mode = 'l_turn' if l_turn_now else 'r_turn'
                self.mode_timer = 0.0
                self.r_step_count = self.l_step_count = 0
            elif not squat_now:
                if l_lift_now and (self.mode != 'l_lift' or self.mode_timer >= self.LIFT_MIN_HOLD):
                    if self.mode != 'r_lift' or self.mode_timer >= 0.20:
                        self.mode = 'l_lift'
                        self.mode_timer = 0.0
                        self.r_step_count = self.l_step_count = 0
                elif r_lift_now and (self.mode != 'r_lift' or self.mode_timer >= self.LIFT_MIN_HOLD):
                    if self.mode != 'l_lift' or self.mode_timer >= 0.20:
                        self.mode = 'r_lift'
                        self.mode_timer = 0.0
                        self.r_step_count = self.l_step_count = 0
                elif r_step_now and (
                    (self.mode == 'idle' and self.cooldown <= 0.0) or
                    self.mode == 'squat_return' or
                    (self.mode in ('l_step', 'l_step_repeat') and self.mode_timer >= 0.20) or
                    (self.mode in ('l_lift', 'r_lift') and self.mode_timer >= 0.20)
                ):
                    self.mode = 'r_step'
                    self.r_step_count = 1
                    self.l_step_count = 0
                    self.mode_timer = 0.0
                elif r_step_now and self.mode == 'r_step' and self.mode_timer >= self.STEP_HOLD * 0.75:
                    # 右足ステップが連続して検出された場合（2回目へ移行）
                    self.mode = 'r_step_repeat'
                    self.r_step_count += 1
                    self.mode_timer = 0.0
                elif r_step_now and self.mode == 'r_step_repeat' and self.mode_timer >= self.STEP_PULLBACK + self.STEP_HOLD * 0.75:
                    # 右足ステップがさらに連続して検出された場合（3回目以降へ移行）
                    self.r_step_count += 1
                    self.mode_timer = 0.0
                elif l_step_now and (
                    (self.mode == 'idle' and self.cooldown <= 0.0) or
                    self.mode == 'squat_return' or
                    (self.mode in ('r_step', 'r_step_repeat') and self.mode_timer >= 0.20) or
                    (self.mode in ('l_lift', 'r_lift') and self.mode_timer >= 0.20)
                ):
                    self.mode = 'l_step'
                    self.l_step_count = 1
                    self.r_step_count = 0
                    self.mode_timer = 0.0
                elif l_step_now and self.mode == 'l_step' and self.mode_timer >= self.STEP_HOLD * 0.75:
                    # 左足ステップが連続して検出された場合（2回目へ移行）
                    self.mode = 'l_step_repeat'
                    self.l_step_count += 1
                    self.mode_timer = 0.0
                elif l_step_now and self.mode == 'l_step_repeat' and self.mode_timer >= self.STEP_PULLBACK + self.STEP_HOLD * 0.75:
                    # 左足ステップがさらに連続して検出された場合（3回目以降へ移行）
                    self.l_step_count += 1
                    self.mode_timer = 0.0

        # 人の向いている向き（平滑化した yaw_s）を車いすの基本の向きとする
        base_heading = max(-105.0, min(105.0, self.yaw_s))

        # ==================================================================
        # 各モードごとの挙動（人の向いている向き base_heading を基準に動作する）
        # ==================================================================
        if self.mode == 'squat':
            f['squat'] = True
            heading = base_heading
            forward = -self.STEP_DIST
            self.action_name = 'しこ立ち（0.12m後退）'
            stood_up = (not squat_now and (hip_drop < 0.08 or sh_drop < 0.08 or hip_vy < -0.35))
            if (self.mode_timer >= self.STEP_HOLD and stood_up) or self.mode_timer >= self.SQUAT_MAX_HOLD:
                self.mode = 'squat_return'
                self.mode_timer = 0.0

        elif self.mode == 'squat_return':
            f['squat'] = True
            heading = base_heading
            forward = 0.0
            self.action_name = 'しこ立ち（立ち上がり・元の位置へ戻る）'
            if self.mode_timer >= self.STEP_RETURN:
                self.mode = 'idle'
                self.cooldown = 0.2
                self.hip_y0 = f['hip_y']
                self.sh_y0 = sh_y
                if f.get('has_knees') and f.get('has_hips'):
                    self.knee_hip_ratio0 = ((f['l_knee_y'] - f['l_hip_y']) + (f['r_knee_y'] - f['r_hip_y'])) / (2.0 * f['torso'])

        elif self.mode == 'l_turn':
            f['turn'] = True
            forward = 0.0
            heading = 360.0
            self.action_name = '左turn（左へ1回転）'
            if self.mode_timer >= self.TURN_HOLD:
                self.mode = 'idle'
                self.cooldown = 0.35
                self.turn_confirm = 0

        elif self.mode == 'r_turn':
            f['turn'] = True
            forward = 0.0
            heading = -360.0
            self.action_name = '右turn（右へ1回転）'
            if self.mode_timer >= self.TURN_HOLD:
                self.mode = 'idle'
                self.cooldown = 0.35
                self.turn_confirm = 0

        elif self.mode == 'l_lift':
            f['l_knee_lift'] = True
            if self.mode_timer < self.LIFT_MIN_HOLD or (l_lift_now and self.mode_timer < 1.2):
                heading = base_heading + self.LIFT_ANGLE
                forward = self.LIFT_DIST
                self.action_name = '左足上げ（左斜め前へ前進）'
                if l_lift_now and self.mode_timer >= self.LIFT_MIN_HOLD:
                    self.mode_timer = self.LIFT_MIN_HOLD
            elif self.mode_timer < self.LIFT_MIN_HOLD + self.LIFT_RETURN:
                heading = base_heading
                forward = 0.0
                self.action_name = '左足上げ（元の位置へ戻る）'
            else:
                heading = base_heading
                forward = 0.0
                self.mode = 'idle'
                self.cooldown = 0.05

        elif self.mode == 'r_lift':
            f['r_knee_lift'] = True
            if self.mode_timer < self.LIFT_MIN_HOLD or (r_lift_now and self.mode_timer < 1.2):
                heading = base_heading - self.LIFT_ANGLE
                forward = self.LIFT_DIST
                self.action_name = '右足上げ（右斜め前へ前進）'
                if r_lift_now and self.mode_timer >= self.LIFT_MIN_HOLD:
                    self.mode_timer = self.LIFT_MIN_HOLD
            elif self.mode_timer < self.LIFT_MIN_HOLD + self.LIFT_RETURN:
                heading = base_heading
                forward = 0.0
                self.action_name = '右足上げ（元の位置へ戻る）'
            else:
                heading = base_heading
                forward = 0.0
                self.mode = 'idle'
                self.cooldown = 0.05

        elif self.mode == 'r_step':
            f['r_step_fwd'] = True
            heading = base_heading
            if self.mode_timer < self.STEP_HOLD:
                forward = self.STEP_DIST
                self.action_name = '右足ステップ1回目（0.12m前進）'
            elif self.mode_timer < self.STEP_HOLD + self.STEP_RETURN:
                forward = 0.0
                self.action_name = '右足ステップ（元の位置へ戻る）'
            else:
                forward = 0.0
                self.mode = 'idle'
                self.r_step_count = 0
                self.cooldown = self.STEP_COOLDOWN

        elif self.mode == 'r_step_repeat':
            f['r_step_fwd'] = True
            heading = base_heading
            if self.mode_timer < self.STEP_PULLBACK:
                # 2回目以降：少し後ろに下がる
                forward = 0.30 * self.STEP_DIST
                self.action_name = f'右足ステップ{self.r_step_count}回目（少し後ろに下がる）'
            elif self.mode_timer < self.STEP_PULLBACK + self.STEP_HOLD:
                # 1回目と同じ位置まで前に行く
                forward = self.STEP_DIST
                self.action_name = f'右足ステップ{self.r_step_count}回目（1回目と同じ位置へ前進）'
            elif self.mode_timer < self.STEP_PULLBACK + self.STEP_HOLD + self.STEP_RETURN:
                forward = 0.0
                self.action_name = '右足ステップ（元の位置へ戻る）'
            else:
                forward = 0.0
                self.mode = 'idle'
                self.r_step_count = 0
                self.cooldown = self.STEP_COOLDOWN

        elif self.mode == 'l_step':
            f['l_step_back'] = True
            heading = base_heading
            if self.mode_timer < self.STEP_HOLD:
                forward = -self.STEP_DIST
                self.action_name = '左足ステップ1回目（0.12m後退）'
            elif self.mode_timer < self.STEP_HOLD + self.STEP_RETURN:
                forward = 0.0
                self.action_name = '左足ステップ（元の位置へ戻る）'
            else:
                forward = 0.0
                self.mode = 'idle'
                self.l_step_count = 0
                self.cooldown = self.STEP_COOLDOWN

        elif self.mode == 'l_step_repeat':
            f['l_step_back'] = True
            heading = base_heading
            if self.mode_timer < self.STEP_PULLBACK:
                # 2回目以降：少し前に向かう
                forward = -0.30 * self.STEP_DIST
                self.action_name = f'左足ステップ{self.l_step_count}回目（少し前に向かう）'
            elif self.mode_timer < self.STEP_PULLBACK + self.STEP_HOLD:
                # 1回目と同じ位置まで後ろに行く
                forward = -self.STEP_DIST
                self.action_name = f'左足ステップ{self.l_step_count}回目（1回目と同じ位置へ後退）'
            elif self.mode_timer < self.STEP_PULLBACK + self.STEP_HOLD + self.STEP_RETURN:
                forward = 0.0
                self.action_name = '左足ステップ（元の位置へ戻る）'
            else:
                forward = 0.0
                self.mode = 'idle'
                self.l_step_count = 0
                self.cooldown = self.STEP_COOLDOWN

        else:
            # 通常時（idle）：手の動きは判断せず、人の向いている向き (base_heading) を維持しつつ次の動きへ滑らかに繋げる
            self.groove_phase += dt * 4.0
            prep_h = base_heading - 20.0 * sway + 4.0 * math.sin(self.groove_phase)
            prep_f = max(-0.15, min(0.15, 0.75 * f.get('dproj_a', 0.0) - 0.30 * hip_drop))
            if abs(prep_f) < 0.02:
                prep_f += 0.03 * math.cos(self.groove_phase * 0.5)
            heading = prep_h
            forward = prep_f
            self.action_name = 'つなぎ動作（次動作へ連携中）'

        limit_h = 360.0 if self.mode in ('l_turn', 'r_turn') else self.heading_limit
        heading = max(-limit_h, min(limit_h, heading))
        forward = max(-self.forward_limit, min(self.forward_limit, forward))

        f['action_name'] = self.action_name
        f['heading_target'] = heading
        f['forward_target'] = forward
        self.prev_f = f.copy()
        return heading, forward


# pose_detect.py でも同じ状態判定を共有するためのインスタンス
_DETECT_MAPPER = DanceMapper()
_DETECT_LAST_T = None
_PREV_PTS = None
_NO_POSE_TIME = 0.0


def features(pts, world, width, height, min_visibility=0.25):
    global _DETECT_LAST_T, _PREV_PTS, _NO_POSE_TIME
    now_t = time.monotonic()
    dt = 0.1 if _DETECT_LAST_T is None else min(0.3, max(0.01, now_t - _DETECT_LAST_T))
    _DETECT_LAST_T = now_t

    # どの部位がカメラ画面内に映っているかを個別に判定する
    # （「頭〜腰」「肩〜膝」「胸〜足先」「腰〜足先」のどの映り方でも骨格推定を可能にする）
    # ※下肢（膝・足首・足先）は画面下端ギリギリ（y < 0.995）まで踏み出しても見失わないようにする
    has_shoulders = _in_frame(pts[L_SH], 0.25, 0.90) and _in_frame(pts[R_SH], 0.25, 0.90)
    has_hips = _in_frame(pts[L_HIP], 0.20, 0.95) and _in_frame(pts[R_HIP], 0.20, 0.95)
    has_knees = _in_frame(pts[L_KNEE], 0.25, 0.99) and _in_frame(pts[R_KNEE], 0.25, 0.99)
    has_ankles = _in_frame(pts[L_ANKLE], 0.20, 0.995) and _in_frame(pts[R_ANKLE], 0.20, 0.995)
    has_feet = _in_frame(pts[L_FOOT], 0.15, 0.998) and _in_frame(pts[R_FOOT], 0.15, 0.998)
    has_lower_legs = (
        has_knees or has_ankles or
        (_in_frame(pts[L_KNEE], 0.15, 0.99) and _in_frame(pts[L_ANKLE], 0.15, 0.995)) or
        (_in_frame(pts[R_KNEE], 0.15, 0.99) and _in_frame(pts[R_ANKLE], 0.15, 0.995))
    )

    # 肩・腰・膝・足首のいずれも映っていない場合のみ None を返す
    if not (has_shoulders or has_hips or has_knees or has_ankles):
        _NO_POSE_TIME += dt
        if _NO_POSE_TIME > 1.5:
            _DETECT_MAPPER.reset()
        _PREV_PTS = None
        return None

    # 机・椅子・車いすなどの家具が「人」として誤認識されるのを防ぐ人体構造・信頼度フィルタ
    hip_cx_norm = (pts[L_HIP][0] + pts[R_HIP][0]) / 2.0
    hip_cy_norm = (pts[L_HIP][1] + pts[R_HIP][1]) / 2.0
    knee_cy_norm = (pts[L_KNEE][1] + pts[R_KNEE][1]) / 2.0
    ank_cy_norm = (pts[L_ANKLE][1] + pts[R_ANKLE][1]) / 2.0
    min_knee_cy = min(pts[L_KNEE][1], pts[R_KNEE][1])
    max_knee_cy = max(pts[L_KNEE][1], pts[R_KNEE][1])
    max_ank_cy = max(pts[L_ANKLE][1], pts[R_ANKLE][1])
    sh_cy_norm = (pts[L_SH][1] + pts[R_SH][1]) / 2.0
    sh_w_norm = abs(pts[L_SH][0] - pts[R_SH][0])
    hip_w_norm = abs(pts[L_HIP][0] - pts[R_HIP][0])

    # 1) 連続した人体部位（「肩＋腰」「腰＋膝」「膝＋足首」のいずれか）が十分な信頼度で存在するか
    #    ※足上げ時は上げた側の膝(min_knee_cy)が腰の高さまで上がり、
    #      しゃがみ時は腰(hip_cy_norm)が膝の高さ近くまで下がるため、
    #      軸足側(max_knee_cy, max_ank_cy)で上下関係を判定して足上げ・しゃがみが no body にならないようにする
    sh_vis_ok = min(pts[L_SH][2], pts[R_SH][2]) >= 0.45
    hip_vis_ok = min(pts[L_HIP][2], pts[R_HIP][2]) >= 0.35
    knee_vis_ok = max(pts[L_KNEE][2], pts[R_KNEE][2]) >= 0.40
    ank_vis_ok = max(pts[L_ANKLE][2], pts[R_ANKLE][2]) >= 0.35

    valid_torso = (
        has_shoulders and has_hips and sh_vis_ok and hip_vis_ok and
        (hip_cy_norm - sh_cy_norm) >= 0.08 and
        sh_w_norm <= 0.55 and hip_w_norm <= 0.42
    )
    valid_legs = (
        ((has_hips and has_knees and (hip_vis_ok or knee_vis_ok)) or
         (has_knees and has_ankles and (knee_vis_ok or ank_vis_ok)) or
         (has_hips and has_ankles and (hip_vis_ok or ank_vis_ok))) and
        hip_w_norm <= 0.42
    )

    # 2) 上下逆転（机・椅子・車いすの誤検出）や極端に小さい背景ノイズのみを除外
    #    ※足上げ中（片膝が高く上がる）や深しゃがみ中（腰が膝に近づく）は絶対に除外しない
    invalid_anatomy = (
        not (valid_torso or valid_legs) or
        (has_hips and has_knees and max_knee_cy < hip_cy_norm - 0.02) or
        (has_hips and has_ankles and (max_ank_cy < knee_cy_norm - 0.02 or (max_ank_cy - hip_cy_norm) < 0.12)) or
        (has_shoulders and has_hips and hip_cy_norm < sh_cy_norm + 0.05) or
        (not has_shoulders and hip_cx_norm > 0.62 and (ank_cy_norm - hip_cy_norm) < 0.38)
    )
    if invalid_anatomy:
        _NO_POSE_TIME += dt
        if _NO_POSE_TIME > 1.5:
            _DETECT_MAPPER.reset()
        _PREV_PTS = None
        return None

    _NO_POSE_TIME = 0.0

    # 顔（鼻）や肩が画面上に見切れている下半身のみの映像では、足を前に踏み出した瞬間に
    # MediaPipe が表裏（左右のラベル L と R）を数フレームだけ逆転させてしまうことがある。
    # 正面向き中（または肩が見切れている状態）で L_HIP と R_HIP の左右が逆転した場合は、左右ペアを元の向きに補正する。
    has_nose = _in_frame(pts[NOSE], 0.65, 0.85) and pts[NOSE][1] > 0.10
    has_clear_shoulders = has_shoulders and _in_frame(pts[L_SH], 0.50, 0.90) and _in_frame(pts[R_SH], 0.50, 0.90) and pts[L_SH][1] > 0.10 and pts[R_SH][1] > 0.10
    was_frontal = (_DETECT_MAPPER.yaw_s is None) or (abs(_DETECT_MAPPER.yaw_s) < 70.0)
    if (not has_clear_shoulders or not has_nose) and was_frontal and has_hips and (pts[L_HIP][0] < pts[R_HIP][0]):
        for li, ri in ((L_SH, R_SH), (L_EL, R_EL), (L_WR, R_WR), (L_HIP, R_HIP),
                       (L_KNEE, R_KNEE), (L_ANKLE, R_ANKLE), (L_HEEL, R_HEEL), (L_FOOT, R_FOOT)):
            pts[li], pts[ri] = pts[ri], pts[li]
            w_li, w_ri = world[li], world[ri]
            world[li] = (abs(w_ri[0]), w_ri[1], -w_ri[2])
            world[ri] = (-abs(w_li[0]), w_li[1], -w_li[2])

    # 体の向き (yaw)：肩が十分画面内に映っていれば肩の3D座標、肩が画面上端付近または見切れていれば腰の3D座標から計算
    # ※上半身が見切れているときに MediaPipe の 3D X座標だけ1フレーム符号反転して yaw が ±170° に飛ぶ現象を、
    #   2D画像上の左右位置関係 (pts_dx) と符号を一致させることで防止する
    if has_clear_shoulders:
        pts_dx = pts[L_SH][0] - pts[R_SH][0]
        w_dx = world[L_SH][0] - world[R_SH][0]
        w_dz = world[L_SH][2] - world[R_SH][2]
    else:
        pts_dx = pts[L_HIP][0] - pts[R_HIP][0]
        w_dx = world[L_HIP][0] - world[R_HIP][0]
        w_dz = world[L_HIP][2] - world[R_HIP][2]
    if (not has_clear_shoulders and was_frontal and pts_dx >= -0.02) or (abs(pts_dx) > 0.025 and (w_dx * pts_dx) <= 0):
        w_dx = math.copysign(max(abs(w_dx), 0.12), 1.0 if pts_dx >= -0.02 else pts_dx)
    yaw = math.degrees(math.atan2(w_dz, w_dx))

    # 中心座標 (hip_x, hip_y, sh_x, sh_y)
    hip_x = (pts[L_HIP][0] + pts[R_HIP][0]) / 2 * width
    hip_y = (pts[L_HIP][1] + pts[R_HIP][1]) / 2 * height
    sh_x = (pts[L_SH][0] + pts[R_SH][0]) / 2 * width
    sh_y = (pts[L_SH][1] + pts[R_SH][1]) / 2 * height

    # 体の基準サイズ (torso)：映っている部位（胴体・太もも・すね・肩幅・腰幅）から最も信頼できる長さを採用
    scales = []
    if has_shoulders and has_hips:
        scales.append(math.hypot(hip_x - sh_x, hip_y - sh_y))
    if has_hips and has_knees:
        knee_cx = (pts[L_KNEE][0] + pts[R_KNEE][0]) / 2 * width
        knee_cy = (pts[L_KNEE][1] + pts[R_KNEE][1]) / 2 * height
        scales.append(math.hypot(knee_cx - hip_x, knee_cy - hip_y) * 1.15)
    if has_knees and has_ankles:
        knee_cx = (pts[L_KNEE][0] + pts[R_KNEE][0]) / 2 * width
        knee_cy = (pts[L_KNEE][1] + pts[R_KNEE][1]) / 2 * height
        ank_cx = (pts[L_ANKLE][0] + pts[R_ANKLE][0]) / 2 * width
        ank_cy = (pts[L_ANKLE][1] + pts[R_ANKLE][1]) / 2 * height
        scales.append(math.hypot(ank_cx - knee_cx, ank_cy - knee_cy) * 1.25)
    if has_shoulders:
        scales.append(math.hypot((pts[L_SH][0] - pts[R_SH][0]) * width, (pts[L_SH][1] - pts[R_SH][1]) * height) * 1.4)
    if has_hips:
        scales.append(math.hypot((pts[L_HIP][0] - pts[R_HIP][0]) * width, (pts[L_HIP][1] - pts[R_HIP][1]) * height) * 2.2)

    torso = max(scales) if scales else math.hypot(hip_x - sh_x, hip_y - sh_y)
    if torso < 5:
        return None

    # 足の3D奥行き座標 (l_leg_z, r_leg_z) と 3D水平変位 (l_3dx, r_3dx)
    # フレームごとの条件分岐による不連続ジャンプを防ぐため、MediaPipe 3Dワールド座標の一貫した重み付けで算出
    l_leg_z = 0.45 * (world[L_FOOT][2] - world[L_HIP][2]) + 0.35 * (world[L_ANKLE][2] - world[L_HIP][2]) + 0.20 * (world[L_KNEE][2] - world[L_HIP][2])
    r_leg_z = 0.45 * (world[R_FOOT][2] - world[R_HIP][2]) + 0.35 * (world[R_ANKLE][2] - world[R_HIP][2]) + 0.20 * (world[R_KNEE][2] - world[R_HIP][2])
    l_3dx = 0.50 * (world[L_FOOT][0] - world[L_HIP][0]) + 0.50 * (world[L_ANKLE][0] - world[L_HIP][0])
    r_3dx = 0.50 * (world[R_FOOT][0] - world[R_HIP][0]) + 0.50 * (world[R_ANKLE][0] - world[R_HIP][0])

    # 体の正面方向ベクトル (fx, fz) に対する足先の3D前方投影と、腰幅に対する2D歩幅比率
    hx = (world[L_SH][0] - world[R_SH][0]) + (world[L_HIP][0] - world[R_HIP][0])
    hz = (world[L_SH][2] - world[R_SH][2]) + (world[L_HIP][2] - world[R_HIP][2])
    hn = math.hypot(hx, hz) or 1e-6
    fx, fz = hz / hn, -hx / hn
    r_ft_fwd = (world[R_FOOT][0] - world[R_HIP][0]) * fx + (world[R_FOOT][2] - world[R_HIP][2]) * fz
    l_ft_fwd = (world[L_FOOT][0] - world[L_HIP][0]) * fx + (world[L_FOOT][2] - world[L_HIP][2]) * fz

    hip_w_2d = max(0.05, abs(pts[L_HIP][0] - pts[R_HIP][0]))
    r_low_x = pts[R_FOOT][0] if (pts[R_FOOT][1] < 0.99 and pts[R_FOOT][2] >= 0.12) else (pts[R_ANKLE][0] if has_ankles else pts[R_KNEE][0])
    l_low_x = pts[L_FOOT][0] if (pts[L_FOOT][1] < 0.99 and pts[L_FOOT][2] >= 0.12) else (pts[L_ANKLE][0] if has_ankles else pts[L_KNEE][0])
    r_dx2d = (pts[R_HIP][0] - r_low_x) / hip_w_2d
    l_dx2d = (l_low_x - pts[L_HIP][0]) / hip_w_2d
    s_dir = -1.0 if yaw < 0 else 1.0
    r_side2d = (r_low_x - pts[R_HIP][0]) * s_dir / hip_w_2d
    l_side2d = (l_low_x - pts[L_HIP][0]) * s_dir / hip_w_2d

    # 2D脚軸方向（腰の中心→足首/膝の中心ベクトル）に沿った左右の脚の伸展差 (dproj_a, dproj_f)
    # カメラが斜め上から見下ろしていたり脚が画面内で斜めに映っていても、直立時は 0、
    # 右足を前に踏み出すと正 (+)、左足を前に踏み出すと負 (-) になる
    dproj_a = dproj_f = 0.0
    dz_ank = world[L_ANKLE][2] - world[R_ANKLE][2]
    if has_hips and (has_ankles or has_knees):
        low_cx = ((pts[L_ANKLE][0] + pts[R_ANKLE][0]) if has_ankles else (pts[L_KNEE][0] + pts[R_KNEE][0])) / 2.0 * width
        low_cy = ((pts[L_ANKLE][1] + pts[R_ANKLE][1]) if has_ankles else (pts[L_KNEE][1] + pts[R_KNEE][1])) / 2.0 * height
        ux, uy = (low_cx - hip_x), (low_cy - hip_y)
        un = math.hypot(ux, uy) or 1.0
        leg_scale = max(5.0, un * 0.55) if has_ankles else torso
        ux, uy = ux / un, uy / un
        r_idx_a = R_ANKLE if has_ankles else R_KNEE
        l_idx_a = L_ANKLE if has_ankles else L_KNEE
        r_proj_a = ((pts[r_idx_a][0] - pts[R_HIP][0]) * width * ux + (pts[r_idx_a][1] - pts[R_HIP][1]) * height * uy) / leg_scale
        l_proj_a = ((pts[l_idx_a][0] - pts[L_HIP][0]) * width * ux + (pts[l_idx_a][1] - pts[L_HIP][1]) * height * uy) / leg_scale
        dproj_a = r_proj_a - l_proj_a
        if has_feet:
            r_proj_f = ((pts[R_FOOT][0] - pts[R_HIP][0]) * width * ux + (pts[R_FOOT][1] - pts[R_HIP][1]) * height * uy) / leg_scale
            l_proj_f = ((pts[L_FOOT][0] - pts[L_HIP][0]) * width * ux + (pts[L_FOOT][1] - pts[L_HIP][1]) * height * uy) / leg_scale
            dproj_f = r_proj_f - l_proj_f
        else:
            dproj_f = dproj_a

    # 前フレームとの移動量比較（左右の足の移動量 vs 手首・腕の移動量）
    l_leg_move_step = r_leg_move_step = wrist_move_step = 0.0
    if _PREV_PTS is not None:
        l_joints = ([L_KNEE] if has_knees else []) + ([L_ANKLE] if has_ankles else []) + ([L_FOOT] if has_feet else [])
        r_joints = ([R_KNEE] if has_knees else []) + ([R_ANKLE] if has_ankles else []) + ([R_FOOT] if has_feet else [])
        if l_joints:
            l_leg_move_step = sum(math.hypot((pts[k][0] - _PREV_PTS[k][0]) * width,
                                             (pts[k][1] - _PREV_PTS[k][1]) * height) for k in l_joints) / len(l_joints) / torso
        if r_joints:
            r_leg_move_step = sum(math.hypot((pts[k][0] - _PREV_PTS[k][0]) * width,
                                             (pts[k][1] - _PREV_PTS[k][1]) * height) for k in r_joints) / len(r_joints) / torso
        wr_joints = [k for k in (L_WR, R_WR, L_EL, R_EL) if _in_frame(pts[k], 0.20)]
        if wr_joints:
            wrist_move_step = max(math.hypot((pts[k][0] - _PREV_PTS[k][0]) * width,
                                             (pts[k][1] - _PREV_PTS[k][1]) * height) for k in wr_joints) / torso
    _PREV_PTS = list(pts)

    # 手の動きは骨格推定で判断しない（l_up, r_up, hands_high は常に False）
    l_up = False
    r_up = False
    hands_high = False

    res_dict = {
        'yaw': yaw, 'hip_x': hip_x, 'hip_y': hip_y, 'sh_x': sh_x, 'sh_y': sh_y, 'torso': torso,
        'img_w': width, 'img_h': height,
        'l_up': bool(l_up),
        'r_up': bool(r_up),
        'hands_high': bool(hands_high),
        'has_legs': bool(has_hips or has_knees or has_ankles),
        'has_lower_legs': bool(has_lower_legs),
        'has_shoulders': bool(has_shoulders),
        'has_hips': bool(has_hips),
        'has_knees': bool(has_knees),
        'has_ankles': bool(has_ankles),
        'has_feet': bool(has_feet),
        'l_hip_y': pts[L_HIP][1] * height,
        'r_hip_y': pts[R_HIP][1] * height,
        'l_leg_z': l_leg_z,
        'r_leg_z': r_leg_z,
        'l_3dx': l_3dx,
        'r_3dx': r_3dx,
        'r_ft_fwd': r_ft_fwd,
        'l_ft_fwd': l_ft_fwd,
        'r_dx2d': r_dx2d,
        'l_dx2d': l_dx2d,
        'r_side2d': r_side2d,
        'l_side2d': l_side2d,
        'dproj_a': dproj_a,
        'dproj_f': dproj_f,
        'dz_ank': dz_ank,
        'l_leg_move_step': l_leg_move_step,
        'r_leg_move_step': r_leg_move_step,
        'wrist_move_step': wrist_move_step,
        'squat': False,
        'turn': False,
        'r_step_fwd': False,
        'l_step_back': False,
        'l_knee_lift': False,
        'r_knee_lift': False,
    }

    if has_knees:
        res_dict['l_knee_x'] = pts[L_KNEE][0] * width
        res_dict['r_knee_x'] = pts[R_KNEE][0] * width
        res_dict['l_knee_y'] = pts[L_KNEE][1] * height
        res_dict['r_knee_y'] = pts[R_KNEE][1] * height
    if has_ankles:
        res_dict['l_ankle_x'] = pts[L_ANKLE][0] * width
        res_dict['r_ankle_x'] = pts[R_ANKLE][0] * width
        res_dict['l_ankle_y'] = pts[L_ANKLE][1] * height
        res_dict['r_ankle_y'] = pts[R_ANKLE][1] * height

    _DETECT_MAPPER.update(res_dict, dt)
    return res_dict


class Follower:
    def __init__(self, heading_gain=16.0, forward_gain=12.0, linear_max=0.60, angular_max=8.0):
        self.heading_gain = heading_gain
        self.forward_gain = forward_gain
        self.linear_max, self.angular_max = linear_max, angular_max
        self.heading = 0.0
        self.forward = 0.0
        self.in_turn = False

    def step(self, heading_target, forward_target, dt):
        # 1回転ターン（±360°）を実行中に次の動き（|heading_target| <= 180°）へ切り替わった瞬間は、
        # 残りの回転を即座に打ち切って目標の向きへ遅延ゼロで合わせる
        if abs(heading_target) < 180.0:
            if self.in_turn:
                self.in_turn = False
                self.heading = heading_target
            else:
                self.heading = ((self.heading + 180.0) % 360.0) - 180.0
            err_h = ((heading_target - self.heading + 180.0) % 360.0) - 180.0
        else:
            if not self.in_turn:
                self.in_turn = True
                self.heading = 0.0
            err_h = heading_target - self.heading

        err_f = forward_target - self.forward
        if abs(err_h) < 0.4:
            w = 0.0
            self.heading = heading_target
        else:
            w = self.heading_gain * math.radians(err_h)
            w = max(-self.angular_max, min(self.angular_max, w))
            self.heading += math.degrees(w * dt)

        if abs(err_f) < 0.004:
            v = 0.0
            self.forward = forward_target
        else:
            v = self.forward_gain * err_f
            v = max(-self.linear_max, min(self.linear_max, v))
            self.forward += v * dt
        return v, w
