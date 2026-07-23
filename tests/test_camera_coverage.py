"""카메라가 툴이 닿는 두둑 폭을 실제로 다 보는가 (Tier 1 — 순수 산수, 시뮬·GPU 불필요).

왜 이 테스트가 생겼나 (DECISIONS 026): 카메라 한 대(D405 87°)를 두둑 위 0.33m 에 달고 코드·문서에
"두둑 전체를 봄"이라고 적어 뒀는데, **산수를 안 해봤다.** 실제 가로 발자국은 0.585m 인데 툴은
±0.45(=0.90m)까지 닿는다 → 35% 가 사각. 완벽한 검출기라도 재현율 상한이 0.65 인 구조적 한계였고,
관측된 자율 재현율 0.25~0.5 의 상당 부분이 여기서 왔다. 시뮬을 아무리 돌려도 "안 보이는 잡초"는
안 보이니 통과처럼 보인다 — 이건 산수로만 잡힌다.

그래서 규율대로 산수는 산수로 단언한다(CLAUDE.md: 산수로 답할 수 있는 건 시뮬로 확인하지 않는다).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from garden_geometry import Garden, Portal  # noqa: E402

G, P = Garden(), Portal()


def test_한대로는_두둑을_못_덮는다():
    """왜 2대인가의 근거. 한 대 발자국 < 두둑 폭이면 단일 카메라는 구조적으로 불가능."""
    assert P.camera_footprint_w(G) < G.bed_width, (
        "한 대로 덮인다면 n_cameras=2 를 되돌려도 된다 — 이 단언이 2대의 존재 이유다")


def test_한대로_덮으려면_빔보다_높아야_한다():
    """카메라를 올려서 해결할 수 없음을 못 박는다 — 필요한 높이가 빔(clearance)을 넘는다."""
    need_h = G.bed_width / (2 * math.tan(P.camera_hfov / 2))   # 두둑 위 필요 높이
    need_z = need_h + G.bed_height                              # 월드 z
    assert need_z > P.clearance, (
        f"필요 camera_z {need_z:.3f} 가 빔 {P.clearance:.2f} 아래라면 한 대를 올려 해결해야 한다")


def test_카메라들이_툴이_닿는_폭을_다_본다():
    """핵심 게이트: 합쳐진 커버리지 반폭 ≥ 툴이 닿는 반폭. '보는 만큼만 친다'의 성립 조건."""
    cover = P.camera_coverage_half(G)
    reach = G.bed_width / 2
    assert cover >= reach, f"카메라 커버리지 반폭 {cover:.3f} < 툴 도달 반폭 {reach:.3f} — 사각 존재"


def test_가운데_사각이_없다():
    """카메라 사이가 벌어지면 두둑 한가운데가 안 보인다. 겹침이 양수여야 한다."""
    assert P.camera_overlap(G) > 0, (
        f"인접 카메라 겹침 {P.camera_overlap(G):.3f} ≤ 0 — 두둑 가운데 blind strip")


def test_카메라_배치가_대칭이고_두둑_안에_있다():
    ys = P.camera_ys(G)
    assert len(ys) == P.n_cameras
    assert math.isclose(sum(ys), 0.0, abs_tol=1e-9), f"좌우 비대칭: {ys}"
    for y in ys:
        assert abs(y) <= G.bed_width / 2, f"카메라 {y:.3f} 가 두둑 밖"


def test_캘리브_MPP_와_화각기하가_크게_안_벌어진다():
    """커버리지 판단에 캘리브 MPP 를 쓰므로, 화각 기하와 심하게 다르면 재캘리브 신호."""
    calib = P.camera_footprint_w(G)
    geom = P.camera_footprint_w_geometric(G)
    rel = abs(calib - geom) / geom
    assert rel < 0.15, f"캘리브 발자국 {calib:.3f} vs 기하 {geom:.3f} — {rel*100:.0f}% 차이, 재캘리브 필요"


def test_툴_밴드가_전부_카메라_커버리지_안에_있다():
    """밴드별로 확인 — 바깥 밴드가 반쯤 안 보이던 게 원래 문제였다."""
    half = P.tool_band_half(G)
    cover = P.camera_coverage_half(G)
    for i, c in enumerate(P.tool_band_centers(G)):
        edge = abs(c) + half
        assert edge <= cover + 1e-9, f"밴드{i} 바깥끝 {edge:.3f} > 커버리지 {cover:.3f}"
