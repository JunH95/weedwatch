"""관성 텐서 계산 — 알려진 닫힌형 값과 대칭성으로 검사한다 (Tier 1).

시뮬도 GPU도 필요 없다. 밀리초 안에 끝난다.

이 테스트가 존재하는 이유: diff-drive 안정성은 관성이 물리적으로 맞느냐에 달렸는데,
관성은 자리채움 값(0.01)으로 두면 조용히 틀린다 — 로봇은 여전히 "서 있어" 보이지만
아주 작은 토크에 팽이처럼 돈다. 산수는 검증 가능하므로 검증한다.

실행:  ./scripts/env.sh python3 -m pytest tests/test_inertia.py -v
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from inertia import Part, box_inertia, combine, cylinder_inertia  # noqa: E402


# ── 닫힌형 값 확인 ───────────────────────────────────────────────────────


def test_정육면체_관성은_세축_대칭():
    """한 변 a 정육면체: ixx=iyy=izz=m·a²/6."""
    m, a = 2.0, 0.5
    ixx, iyy, izz, ixy, ixz, iyz = box_inertia(m, a, a, a)
    expect = m * a * a / 6.0
    assert ixx == pytest.approx(expect)
    assert iyy == pytest.approx(expect)
    assert izz == pytest.approx(expect)
    assert (ixy, ixz, iyz) == (0.0, 0.0, 0.0)


def test_직육면체_긴축_관성이_가장_작다():
    """가늘고 긴 상자(x 로 긺)는 x 축 관성이 최소여야 한다."""
    ixx, iyy, izz, *_ = box_inertia(1.0, 2.0, 0.1, 0.1)
    assert ixx < iyy
    assert ixx < izz


def test_실린더_대칭축_관성():
    """굴러가는 바퀴: 대칭축(y) 관성 = ½ m r²."""
    m, r, L = 1.5, 0.11, 0.08
    ixx, iyy, izz, *_ = cylinder_inertia(m, r, L, axis="y")
    assert iyy == pytest.approx(0.5 * m * r * r)
    # 직교 두 축은 서로 같고 대칭축과 다르다
    assert ixx == pytest.approx(izz)
    assert ixx == pytest.approx(m / 12.0 * (3 * r * r + L * L))


def test_실린더_axis_잘못주면_에러():
    with pytest.raises(ValueError):
        cylinder_inertia(1.0, 0.1, 0.2, axis="w")


# ── 평행축 합성 ──────────────────────────────────────────────────────────


def test_한_조각은_그대로():
    """조각이 하나면 합성 COM = 그 조각 COM, 관성 그대로."""
    t = box_inertia(3.0, 0.2, 0.3, 0.4)
    m, com, tensor = combine([Part(3.0, (0.1, 0.2, 0.3), t)])
    assert m == pytest.approx(3.0)
    assert com == pytest.approx((0.1, 0.2, 0.3))
    assert tensor == pytest.approx(t)


def test_평행축_점질량_두개():
    """x=±d 에 놓인 두 점질량(m each): izz = iyy = 2·m·d², ixx=0.

    점질량은 자체 관성 0 이므로 순수 평행축 항만 남는다. 손으로 검산 가능.
    """
    m, d = 2.0, 0.5
    zero = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    parts = [Part(m, (+d, 0, 0), zero), Part(m, (-d, 0, 0), zero)]
    M, com, (ixx, iyy, izz, ixy, ixz, iyz) = combine(parts)
    assert M == pytest.approx(2 * m)
    assert com == pytest.approx((0.0, 0.0, 0.0))
    assert ixx == pytest.approx(0.0)  # x 축 둘레 회전엔 팔길이 0
    assert iyy == pytest.approx(2 * m * d * d)
    assert izz == pytest.approx(2 * m * d * d)
    # x축 위에만 있으니 곱관성 전부 0
    assert (ixy, ixz, iyz) == pytest.approx((0.0, 0.0, 0.0))


def test_y대칭_배치는_곱관성이_0():
    """y=±d 에 대칭으로 놓인 두 상자 → 곱관성(ixy 등) 모두 0.

    포탈 몸통의 사이드 포드가 정확히 이 배치다. 곱관성이 0 이어야
    URDF 가 대각 관성만 써도 정확하다.
    """
    t = box_inertia(5.0, 0.3, 0.2, 0.4)
    parts = [
        Part(5.0, (0.0, +0.6, 0.5), t),
        Part(5.0, (0.0, -0.6, 0.5), t),
    ]
    _, com, (_, _, _, ixy, ixz, iyz) = combine(parts)
    assert com[1] == pytest.approx(0.0)  # y 대칭 → COM 은 y=0
    assert ixy == pytest.approx(0.0)
    assert ixz == pytest.approx(0.0)
    assert iyz == pytest.approx(0.0)


def test_평행축은_항상_관성을_키운다():
    """COM 에서 떨어진 조각을 합치면 그 축 관성은 자체값보다 커야 한다."""
    t = box_inertia(4.0, 0.2, 0.2, 0.2)
    # 두 조각을 z 로 벌려 놓으면 합성 ixx/iyy 는 조각 하나 관성의 2배보다 커야
    parts = [Part(4.0, (0, 0, +0.3), t), Part(4.0, (0, 0, -0.3), t)]
    _, _, (ixx, iyy, izz, *_) = combine(parts)
    assert ixx > 2 * t[0]
    assert iyy > 2 * t[1]
    assert izz == pytest.approx(2 * t[2])  # z 로만 벌렸으니 izz 는 평행축 항 0
