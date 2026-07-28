"""밭 명세 단언 (Tier 1 — 시뮬 없이 산수로). DECISIONS 042.

여기서 지키는 건 숫자 하나가 아니라 **사다리의 규율**이다:

  · 개발 밭은 정본 밭의 **한 조각**이어야 한다 — 현실성 파라미터가 같고 크기만 작다.
    이게 깨지면 "작은 데서 통과했으니 큰 데서도 되겠지"가 자기기만이 된다.
  · 매끈 밭은 정말 매끈해야 한다(회귀 기준선이 흔들리면 비교가 무의미하다).
  · 같은 시드면 같은 밭이어야 한다 — 어제 실패한 밭을 오늘 다시 만들 수 있어야 한다.
  · 현실성이 켜지면 **실제로** 변이가 생겨야 한다. 파라미터만 있고 모양이 안 변하면 최악이다
    (측정은 통과하는데 현실은 안 담긴 상태).
"""
import math
import sys
from pathlib import Path

import pytest

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from field_spec import DEV, MAIN, SMOOTH, get, scaled  # noqa: E402
from garden_geometry import Garden  # noqa: E402

G = Garden()
REAL_KEYS = ("width_var", "height_var", "meander", "meander_wave",
             "clod_density", "clod_height", "cross_slope_deg", "weed_full_width")


def samples(spec, bed=0, n=200):
    xs = [spec.x0 + (spec.bed_length * i) / (n - 1) for i in range(n)]
    return xs, [spec.profile(bed, x) for x in xs]


# ── 사다리 규율 ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("key", REAL_KEYS)
def test_dev_field_is_as_hard_as_main(key):
    """개발 밭은 정본 밭과 **현실성이 같아야** 한다 — 크기만 다르다."""
    assert getattr(DEV, key) == getattr(MAIN, key), (
        f"개발 밭의 {key} 가 정본과 다르다 — 작게 만들면서 쉽게 만들었다(042 위반)")


def test_dev_is_smaller_than_main():
    assert DEV.area_m2 < MAIN.area_m2 / 5, "개발 밭이 충분히 작지 않다(빠른 반복이 목적)"


def test_smooth_field_is_actually_smooth():
    """회귀 기준선은 변이가 0 이어야 한다 — 기준선이 흔들리면 비교가 무의미하다."""
    assert not SMOOTH.realistic
    _, prof = samples(SMOOTH)
    ys = {round(p[0], 9) for p in prof}
    ws = {round(p[1], 9) for p in prof}
    hs = {round(p[2], 9) for p in prof}
    assert len(ys) == len(ws) == len(hs) == 1, "매끈 밭인데 단면이 변한다"
    assert SMOOTH.clods(0) == []


def test_scaled_keeps_difficulty():
    """크기를 바꾸는 헬퍼가 현실성을 건드리면 안 된다."""
    s = scaled(MAIN, n_beds=2, bed_length=2.0)
    for key in REAL_KEYS:
        assert getattr(s, key) == getattr(MAIN, key)
    assert s.n_beds == 2 and s.bed_length == 2.0


# ── 재현성 ────────────────────────────────────────────────────────────────

def test_same_seed_same_field():
    a = get("dev")
    _, p1 = samples(a)
    _, p2 = samples(a)
    assert p1 == p2, "같은 명세인데 다른 밭이 나온다"
    assert a.clods(0) == a.clods(0)


def test_different_beds_differ():
    """두둑마다 모양이 달라야 한다 — 전부 같으면 '균일'을 이름만 바꾼 것이다."""
    _, p0 = samples(DEV, bed=0)
    _, p1 = samples(DEV, bed=1)
    d = [abs(a[1] - b[1]) for a, b in zip(p0, p1)]
    assert max(d) > 0.005, "두둑 0 과 1 의 폭 변이가 사실상 같다"


# ── 현실성이 실제로 들어갔나 ───────────────────────────────────────────────

def test_realistic_field_actually_varies():
    xs, prof = samples(DEV)
    w = [p[1] for p in prof]
    h = [p[2] for p in prof]
    cy = [p[0] for p in prof]
    assert max(w) - min(w) > DEV.width_var, f"폭이 안 변한다 (진폭 {DEV.width_var})"
    assert max(h) - min(h) > DEV.height_var, "높이가 안 변한다"
    assert max(cy) - min(cy) > DEV.meander, "중심선이 안 굽는다"


def test_variation_stays_within_amplitude():
    """변이가 진폭을 넘으면 안 된다 — 두둑이 고랑을 침범하면 로봇이 못 지나간다."""
    _, prof = samples(DEV)
    for cy, w, h in prof:
        assert abs(w - G.bed_width) <= DEV.width_var + 1e-9
        assert abs(h - G.bed_height) <= DEV.height_var + 1e-9
        assert abs(cy - DEV.bed_centers[0]) <= DEV.meander + 1e-9


def test_meander_is_low_frequency():
    """사행이 저주파여야 한다 — 톱니처럼 굽는 두둑은 배토기가 만들지 않는다."""
    xs, prof = samples(DEV, n=400)
    cy = [p[0] for p in prof]
    crossings = sum(1 for a, b in zip(cy, cy[1:])
                    if (a - DEV.bed_centers[0]) * (b - DEV.bed_centers[0]) < 0)
    expected = 2 * DEV.bed_length / DEV.meander_wave
    assert crossings <= expected + 2, f"사행이 너무 잦다({crossings}회) — 파장이 너무 짧다"


def test_clods_land_on_wheel_paths():
    """흙덩이는 **바퀴가 지나는 고랑**에 있어야 흔들림이 생긴다 — 두둑 위면 의미 없다."""
    clods = DEV.clods(0)
    assert clods, "현실성 켰는데 흙덩이가 없다"
    cy = DEV.bed_centers[0]
    track_half = G.bed_width / 2 + G.furrow_width / 2
    for x, y, h in clods:
        off = abs(abs(y - cy) - track_half)
        assert off <= 0.07, f"흙덩이가 바퀴 경로에서 {off*100:.0f}cm 벗어났다"
        assert DEV.clod_height[0] <= h <= DEV.clod_height[1]
        assert DEV.x0 <= x <= DEV.x1


def test_bed_width_variation_eats_straddle_margin():
    """이 밭이 **실제로 어려운지** 산수로 확인한다.

    걸터타기 여유는 11cm/쪽(034). 두둑 폭이 +5cm 넓어지면 한쪽당 2.5cm 를 먹고, 사행 4cm 가
    더해지면 6.5cm — 여유의 60%다. 여기에 경사 3°가 2.2cm 를 더 먹는다. 즉 이 밭은
    "될 것 같지만 아슬아슬한" 영역이고, 그래서 시험할 값어치가 있다.
    """
    margin = 0.11
    eaten = DEV.width_var / 2 + DEV.meander + math.tan(math.radians(DEV.cross_slope_deg)) * 0.42
    assert 0.3 * margin < eaten < margin, (
        f"개발 밭 난이도가 어중간하다: 여유 {margin*100:.0f}cm 중 {eaten*100:.1f}cm 잠식")


# ── 정본 밭 ───────────────────────────────────────────────────────────────

def test_main_field_is_big_and_has_home_base():
    assert MAIN.n_beds >= 6 and MAIN.bed_length >= 6.0
    assert MAIN.home_base, "정본 밭에 창고가 없다 — 창고↔밭 이동·도킹이 쓸 것(039)"
    assert MAIN.area_m2 > 40, "정본 밭이 '진짜 밭' 이라 하기엔 작다"


def test_headland_fits_the_uturn():
    """헤드랜드가 U턴 반경보다 넓어야 한다 — 좁으면 돌다가 두둑에 낀다(040)."""
    swing = 0.955                      # 로봇 대각 반지름 (maneuver.SWING_RADIUS)
    for f in (SMOOTH, DEV, MAIN):
        assert f.headland > swing + 0.1, f"{f.name} 밭 헤드랜드가 좁다"
