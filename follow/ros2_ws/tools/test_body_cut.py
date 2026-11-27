"""body_cut・NearGuard の試験（ROS・カメラなしで動く）。

  python3 -m pytest follow/ros2_ws/tools/test_body_cut.py -v
"""
import os
import sys
from types import SimpleNamespace as P

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from body_cut import NearGuard, body_cut  # noqa: E402

NAN = float('nan')


def person(head_y=0.15, knee_y=0.75, ankle_y=0.92, vis=0.9, head_vis=None, knee_vis=None):
    """立っている人の 33 点（画面の中で縦に並ぶ）。部位ごとに y と確からしさを変えられる"""
    lm = [P(x=0.5, y=0.45, visibility=vis) for _ in range(33)]
    for i in range(0, 9):
        lm[i] = P(x=0.5, y=head_y, visibility=vis if head_vis is None else head_vis)
    for i in (11, 12):
        lm[i] = P(x=0.5, y=0.3, visibility=vis)
    for i in (23, 24):
        lm[i] = P(x=0.5, y=0.55, visibility=vis)
    for i in (25, 26):
        lm[i] = P(x=0.5, y=knee_y, visibility=vis if knee_vis is None else knee_vis)
    for i in (27, 28):
        lm[i] = P(x=0.5, y=ankle_y, visibility=vis)
    return lm


def test_whole_body_not_cut():
    assert body_cut(person()) == (False, '')


@pytest.mark.parametrize('kw, reason', [
    ({'head_y': -0.05}, '頭'),              # 頭が画面の上にはみ出している
    ({'head_y': 0.01}, '頭'),               # 上の端（edge 0.03 より内）
    ({'head_vis': 0.2}, '頭'),              # 頭の確からしさが低い
    ({'knee_y': 1.05}, '膝'),               # 膝が画面の下にはみ出している
    ({'knee_y': 0.99}, '膝'),               # 下の端
    ({'knee_vis': 0.1}, '膝'),              # 膝の確からしさが低い
    ({'head_y': NAN}, '頭'),                # NaN は切れている側
    ({'knee_y': NAN}, '膝'),
    ({'head_vis': NAN}, '頭'),
])
def test_cut(kw, reason):
    cut, why = body_cut(person(**kw))
    assert cut and reason in why


def test_ankle_cut_allowed_by_default():
    """足首が切れていても、膝が写っていれば切れていない（既定 lower='knee'）"""
    assert body_cut(person(ankle_y=1.1))[0] is False
    assert body_cut(person(ankle_y=1.1), lower='ankle')[0] is True
    assert body_cut(person(knee_y=1.1), lower='none')[0] is False


def test_boundary():
    """端の線ちょうどは写っている扱い、それより外は切れている"""
    assert body_cut(person(head_y=0.03))[0] is False
    assert body_cut(person(head_y=0.0299))[0] is True
    assert body_cut(person(head_vis=0.5))[0] is False      # 確からしさ 0.5 ちょうどは写っている
    assert body_cut(person(head_vis=0.4999))[0] is True


def test_bad_input():
    assert body_cut(None)[0] is True
    assert body_cut(person()[:20])[0] is True
    lm = person()
    lm[0] = None                                          # 頭の点の 1 つでも壊れていたら切れている側
    assert body_cut(lm)[0] is True


def test_one_head_point_out_is_cut():
    """頭の点が 1 つでも画面の上を越えていれば切れている（頭のてっぺんが切れて口元だけ写る場面）"""
    lm = person(head_y=0.2)
    lm[2] = P(x=0.5, y=-0.02, visibility=0.99)            # 画面の外でも MediaPipe は確からしさを高く付ける
    assert body_cut(lm) == (True, '頭が切れている')


def test_offscreen_high_visibility_is_cut():
    assert body_cut(person(head_y=-0.07, head_vis=0.99))[0] is True


def test_one_knee_out_is_cut():
    lm = person()
    lm[26] = P(x=0.5, y=1.04, visibility=0.9)
    assert body_cut(lm) == (True, '膝が切れている')


def test_guard_waits_before_allowing():
    g = NearGuard(release_s=0.5, now=0.0)
    assert g.update(False, 0.2) is False       # 作った直後は 0.5 s 待つ
    assert g.update(False, 0.5) is True
    assert g.update(True, 1.0) is False        # 切れたらすぐ止める
    assert g.update(False, 1.1) is False       # 抜けても 0.5 s は止めたまま
    assert g.update(False, 1.4) is False
    assert g.update(True, 1.45) is False       # 途中でまた切れたら数え直し
    assert g.update(False, 1.9) is False
    assert g.update(False, 1.95) is True


def test_guard_nan_time_stops():
    g = NearGuard(release_s=0.5, now=0.0)
    assert g.update(False, NAN) is False
    assert g.update(False, 1.0) is True


@pytest.mark.parametrize('idx, x', [(11, 0.01), (12, 0.995), (23, -0.1), (24, 1.2), (11, NAN)])
def test_side_cut(idx, x):
    """肩・腰の点が 1 つでも画面の左右の端を越えたら切れている"""
    lm = person()
    lm[idx] = P(x=x, y=lm[idx].y, visibility=0.9)
    assert body_cut(lm) == (True, '体が左右で切れている')


def test_side_boundary():
    lm = person()
    lm[11] = P(x=0.03, y=0.3, visibility=0.9)
    lm[12] = P(x=0.97, y=0.3, visibility=0.9)
    assert body_cut(lm)[0] is False
