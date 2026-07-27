"""시각 오도메트리의 계산부 단언 (Tier 1 — 시뮬·카메라 없이 산수로).

시뮬로 재는 것(diag_vo·diag_fusion)은 "실제 밭에서 얼마나 맞나"이고, 여기서 재는 것은
**알고리즘이 옳게 구현됐나**다. 합성 영상을 정해진 만큼 밀어 넣고 그 값을 되찾는지 본다.
이게 통과해야 시뮬 결과의 오차를 "장면 탓"이라고 말할 수 있다 — 구현 버그일 가능성을 먼저 지운다.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "perception"))
from vo import MM_PER_PX, CAL_H, VoTracker, phase_shift, soil_mask  # noqa: E402


def texture(n=256, seed=3):
    """흙처럼 반복적이되 특징점은 없는 무늬 — 위상상관이 다루는 종류."""
    rng = np.random.default_rng(seed)
    a = rng.normal(128, 30, (n, n)).astype(np.float32)
    # 저주파 성분을 섞어 실제 흙처럼 얼룩지게
    yy, xx = np.mgrid[0:n, 0:n]
    return a + 40 * np.sin(xx / 17.0) * np.cos(yy / 23.0)


@pytest.mark.parametrize("drow,dcol", [(7, 0), (0, 5), (11, -9), (-4, 13)])
def test_phase_shift_recovers_known_shift(drow, dcol):
    """정해진 픽셀만큼 민 영상에서 그 이동을 되찾아야 한다."""
    a = texture()
    b = np.roll(np.roll(a, drow, axis=0), dcol, axis=1)
    got_row, got_col = phase_shift(a, b)
    assert got_row == pytest.approx(-drow, abs=0.2), f"row {got_row} vs {-drow}"
    assert got_col == pytest.approx(-dcol, abs=0.2), f"col {got_col} vs {-dcol}"


def test_soil_mask_keeps_calibration_plane_only():
    """캘리브 평면 근처만 남고 식물(가까움)·고랑(멂)은 빠져야 한다 — 깊이는 선택에 쓴다(041)."""
    d = np.full((10, 10), CAL_H, np.float32)
    d[0:3, :] = CAL_H - 0.20          # 식물 (카메라에 더 가까움)
    d[7:, :] = CAL_H + 0.25           # 고랑 (더 멂)
    m = soil_mask(d)
    assert m[4, 4] == 1.0
    assert m[1, 1] == 0.0 and m[8, 8] == 0.0
    assert m.mean() == pytest.approx(0.4, abs=0.01)


def test_tracker_converts_pixels_to_meters_in_robot_frame():
    """이동 증분은 로봇 기준(전방·좌)이고 단위는 미터여야 한다."""
    t = VoTracker()
    a = texture()
    assert t.update(a) is None                      # 첫 프레임은 기준만 잡는다
    shift = 10
    b = np.roll(a, shift, axis=0)                   # +row 로 민 영상
    fwd, left = t.update(b)
    # 캘리브(022): +row 는 base 전방(−x) → 부호가 뒤집혀 전방 이동은 양수
    assert fwd == pytest.approx(shift * MM_PER_PX, rel=0.05)
    assert abs(left) < 1e-3


def test_remember_keeps_frame_without_correlating():
    """remember() 는 상관 없이 프레임만 갱신한다 — 직진 구간 비용 절약 경로."""
    t = VoTracker()
    a = texture()
    t.remember(a)
    assert t.frames == 0                            # 상관은 안 돌았다
    b = np.roll(a, 6, axis=0)
    fwd, _ = t.update(b)
    assert t.frames == 1
    assert fwd == pytest.approx(6 * MM_PER_PX, rel=0.05), "remember 가 기준 프레임을 안 남겼다"
