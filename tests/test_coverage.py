"""보스트로페돈 커버리지 경로 (관통 P1, Tier 1 순수 산수 — 시뮬 불필요).

경로가 모든 두둑을 훑고, 방향을 번갈고, 밭 경계 안이고, transit 이 옆 두둑으로 옳게 옮기는지.
시뮬로 실제 주행 가능한가(재진입)는 P3 가 검증 — 여기선 "경로가 말이 되나"만.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from coverage_path import boustrophedon, covered_beds, path_length  # noqa: E402
from garden_geometry import Garden, Portal  # noqa: E402

X0, X1 = 0.2, 3.0


def test_모든_두둑을_커버한다():
    for n in (1, 2, 3, 4):
        g = Garden(n_beds=n)
        wps = boustrophedon(g, Portal(), X0, X1)
        assert covered_beds(wps) == set(range(n)), f"n={n} 에서 빠진 두둑"


def test_방향이_번갈아_바뀐다():
    """0번 +x, 1번 −x, ... — 지그재그여야 헛걸음(빈 복귀 주행)이 없다."""
    g = Garden(n_beds=4)
    wps = boustrophedon(g, Portal(), X0, X1)
    starts = [w for w in wps if w.kind == "pass_start"]
    ends = [w for w in wps if w.kind == "pass_end"]
    for i, (s, e) in enumerate(zip(starts, ends)):
        if i % 2 == 0:
            assert s.x < e.x, f"짝수 두둑 {i} 는 +x 여야"
        else:
            assert s.x > e.x, f"홀수 두둑 {i} 는 −x 여야"


def test_pass는_두둑_중심을_따라간다():
    g = Garden(n_beds=3)
    wps = boustrophedon(g, Portal(), X0, X1)
    centers = g.bed_centers
    for w in wps:
        if w.kind in ("pass_start", "pass_end"):
            assert abs(w.y - centers[w.bed]) < 1e-9, "pass 가 두둑 중심을 안 따라감"


def test_transit가_옆_두둑으로_옮긴다():
    """transit 끝 y 가 다음 두둑 중심이어야 (엉뚱한 데로 안 감)."""
    g = Garden(n_beds=3)
    centers = g.bed_centers
    wps = boustrophedon(g, Portal(), X0, X1)
    transits = [w for w in wps if w.kind == "transit"]
    # transit 은 두둑당 2개(헤드랜드로, 옆 열로). 둘째의 y 가 목표 두둑 중심.
    for w in transits[1::2]:
        assert any(abs(w.y - c) < 1e-9 for c in centers), "transit 이 두둑 열에 안 맞음"


def test_경로가_헤드랜드_안에_있다():
    """x 가 [x0-headland, x1+headland] 안, y 가 두둑 범위 안."""
    g = Garden(n_beds=3)
    hl = 0.8
    wps = boustrophedon(g, Portal(), X0, X1, headland=hl)
    centers = g.bed_centers
    for w in wps:
        assert X0 - hl - 1e-9 <= w.x <= X1 + hl + 1e-9, f"x {w.x} 가 헤드랜드 밖"
        assert min(centers) - 1e-9 <= w.y <= max(centers) + 1e-9, f"y {w.y} 가 두둑 범위 밖"


def test_한줄이면_transit가_없다():
    wps = boustrophedon(Garden(n_beds=1), Portal(), X0, X1)
    assert not [w for w in wps if w.kind == "transit"]
    assert path_length(wps) > 0
