"""骨格が画面の上下で切れているか（＝人が近すぎるか）を判定する。ROS に依存しない。

人がカメラに近づくと、頭や脚が画面の上下からはみ出し、MediaPipe の肩・腰の点の位置がずれて
胴が短く写る。すると距離を実際より遠く見積もり、追従の前進が出てしまう（2026-11-05、1.2 m に立つと 1.7 m と出た）。
そこで、頭か下半身が切れているときは「近すぎ」とみなし、前進を 0 にする（旋回は残す）。

  body_cut(lm)      … 1 人分の 33 点から (切れているか, 理由) を返す。迷う値（NaN、確からしさが低い）は切れている側
                      頭の 9 点と膝の 2 点が「すべて」画面の上下の端より内側にあり、かつそれぞれ 1 点以上は確からしさが
                      min_vis 以上のときだけ「切れていない」。MediaPipe は画面の外にはみ出した点にも高い確からしさ（0.9 以上）を
                      付けるので、確からしさだけでは見分けられない（2026-11-05、頭を切り落とした動画で確かめた）
                      左右: 肩と腰の 4 点がすべて画面の左右の端より内側でなければ切れている（横にはみ出すと肩幅が短く写るため）
  NearGuard         … 切れている → 前進を止める。切れていない状態が release_s 秒続いてから前進を許す
                      （判定の境目で前進が細かく出たり止まったりしないように）
"""
import math

# MediaPipe Pose の点の番号
HEAD = (0, 1, 2, 3, 4, 5, 6, 7, 8)        # 鼻・目・耳
TORSO = (11, 12, 23, 24)                 # 肩・腰（左右の切れを見る）
KNEES = (25, 26)
ANKLES = (27, 28)
LOWER = {'knee': KNEES, 'ankle': ANKLES, 'none': ()}


def _read(p):
    """(x, y, 確からしさ)。読めない・NaN なら None"""
    try:
        x, y, v = float(p.x), float(p.y), float(p.visibility)
    except (AttributeError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(v)):
        return None
    return x, y, v


def _part_inside(lm, ids, min_vis, edge):
    """ids の点がすべて画面の上下の端より内側にあり、1 点以上は確からしさが min_vis 以上"""
    pts = [_read(lm[i]) for i in ids]
    if any(p is None for p in pts):
        return False
    return all(edge <= y <= 1.0 - edge for _, y, _ in pts) and any(v >= min_vis for _, _, v in pts)


def _side_inside(lm, ids, edge):
    """ids の点がすべて画面の左右の端より内側にある"""
    pts = [_read(lm[i]) for i in ids]
    return all(p is not None and edge <= p[0] <= 1.0 - edge for p in pts)


def body_cut(lm, min_vis=0.5, edge=0.03, lower='knee'):
    """(切れているか, 理由)。頭か下半身（既定は膝）の点が 1 つでも画面の上下の端を越えていれば切れている"""
    if lm is None or len(lm) < 33:
        return True, '点が足りない'
    if not _side_inside(lm, TORSO, edge):
        return True, '体が左右で切れている'    # 横にはみ出すと肩幅が短く写り、遠く見積もる
    if not _part_inside(lm, HEAD, min_vis, edge):
        return True, '頭が切れている'
    ids = LOWER[lower]
    if ids and not _part_inside(lm, ids, min_vis, edge):
        return True, ('膝' if lower == 'knee' else '足首') + 'が切れている'
    return False, ''


class NearGuard:
    """前進を許すかを決める。切れていたら止め、切れていない状態が release_s 秒続いたら許す。
    作った直後と、追う人がいないときも止める（見つけ直した直後に前進しない）"""

    def __init__(self, release_s=0.5, now=0.0):
        self.release_s = release_s
        self.last_bad = now

    def update(self, cut, now):
        """cut: 切れている（または追う人がいない）なら True。前進を許すなら True を返す"""
        if cut or not math.isfinite(now):
            self.last_bad = now if math.isfinite(now) else self.last_bad
            return False
        return now - self.last_bad >= self.release_s
