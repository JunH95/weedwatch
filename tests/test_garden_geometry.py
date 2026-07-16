"""포탈 로봇이 두둑을 탈 수 있는 기하학인가 — 산수로 검사한다.

시뮬레이션도 GPU도 필요 없다. 밀리초 안에 끝난다.

이 파일이 존재하는 이유: 실제로 여기서 버그를 냈다.
두둑을 2개 놓고 로봇을 y=0 에 세웠는데, 두둑이 짝수 개면 y=0 이 고랑 한가운데라서
바퀴가 두둑을 밟고 몸통이 고랑 위에 떴다. 설계와 정확히 반대였다.
숫자(트랙 120cm, 여유 11cm)는 전부 "통과"했고, **렌더링한 사진을 눈으로 보고서야** 알았다.

눈으로 보는 건 확장이 안 된다. 그래서 산수로 검사한다.

실행:  ./scripts/env.sh python3 -m pytest tests/ -v
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from garden_geometry import Garden, Portal, parking_y  # noqa: E402


@pytest.fixture
def g() -> Garden:
    return Garden()


@pytest.fixture
def p() -> Portal:
    return Portal()


# ── 밭 자체가 농진청 규격 안에 있는가 ────────────────────────────────────


def test_두둑_높이가_농진청_범위(g):
    """농사로: 두둑 높이 20~30cm."""
    assert 0.20 <= g.bed_height <= 0.30


def test_고랑_폭이_농진청_범위(g):
    """농사로: 고랑 폭 30cm 내외."""
    assert 0.25 <= g.furrow_width <= 0.35


def test_두둑_폭이_평이랑_범위(g):
    """농사로: 평이랑 90~120cm."""
    assert 0.90 <= g.bed_width <= 1.20


def test_두둑들이_겹치지_않는다(g):
    spans = [g.bed_span(i) for i in range(g.n_beds)]
    for (_, hi), (lo, _) in zip(spans, spans[1:]):
        assert lo > hi, "두둑이 겹칩니다"


def test_두둑_사이_간격이_정확히_고랑_폭(g):
    spans = [g.bed_span(i) for i in range(g.n_beds)]
    for (_, hi), (lo, _) in zip(spans, spans[1:]):
        assert lo - hi == pytest.approx(g.furrow_width)


# ── 로봇이 실제로 탈 수 있는가 (여기가 핵심) ─────────────────────────────


def test_바퀴가_두둑을_밟지_않는다(g, p):
    """포탈 설계의 전부. 바퀴는 고랑에, 몸통은 두둑 위.

    이걸 어기면 포탈이 아니라 그냥 작물을 밟고 지나가는 기계다.
    """
    for bed_i in range(g.n_beds):
        y = parking_y(g, bed_i)
        for wy in p.wheel_ys(g, y):
            assert not g.is_over_bed(wy, p.wheel_width / 2), (
                f"{bed_i}번 두둑에 세웠는데 바퀴가 y={wy:.3f}에서 두둑을 밟습니다. "
                f"두둑 구간: {[g.bed_span(i) for i in range(g.n_beds)]}"
            )


def test_로봇_중심이_두둑_위에_있다(g, p):
    """도구가 잡초에 닿으려면 몸통이 두둑 위여야 한다."""
    for bed_i in range(g.n_beds):
        y = parking_y(g, bed_i)
        assert g.is_over_bed(y), f"{bed_i}번 두둑에 세웠는데 로봇 중심이 두둑 밖입니다"


def test_바퀴가_고랑_안에_들어간다(g, p):
    """바퀴가 고랑보다 넓으면 애초에 못 들어간다."""
    assert p.wheel_slack(g) > 0, (
        f"바퀴({p.wheel_width*100:.0f}cm)가 고랑({g.furrow_width*100:.0f}cm)에 안 들어갑니다"
    )


def test_바퀴_여유가_비현실적이지_않다(g, p):
    """여유가 너무 적으면 조향 오차에 취약하다. 최소 3cm/쪽."""
    assert p.wheel_slack(g) >= 0.03, f"바퀴 여유 {p.wheel_slack(g)*100:.1f}cm/쪽 — 너무 빡빡"


def test_작물_위로_지나간다(g, p):
    """빔이 작물을 치면 안 된다."""
    assert p.headroom(g) > 0, (
        f"빔 아랫면이 작물보다 {abs(p.headroom(g))*100:.0f}cm 낮습니다. "
        f"클리어런스 {p.clearance*100:.0f}cm < 두둑 {g.bed_height*100:.0f} + "
        f"작물 {g.crop_height*100:.0f}"
    )


def test_클리어런스가_필요치를_만족(g, p):
    assert p.clearance >= p.required_clearance(g), (
        f"클리어런스 {p.clearance*100:.0f}cm < 필요 {p.required_clearance(g)*100:.0f}cm"
    )


def test_트랙이_설계_문서와_일치(g, p):
    """docs/DECISIONS.md 006 이 트랙 120 / 전폭 128cm 라고 못박았다."""
    assert p.track(g) == pytest.approx(1.20)
    assert p.overall_width(g) == pytest.approx(1.28)


# ── 논지 검증: Tertill 은 산수로 이 밭에 못 들어온다 ──────────────────────


def test_Tertill은_상추_간격을_물리적으로_못_지난다():
    """docs/DECISIONS.md 005 의 핵심 논거를 산수로 고정한다.

    Tertill 지름 8.25인치 = 20.96cm. 농진청 상추 포기간격 20cm.
    작물 두께를 0으로 쳐도 못 지난다. 게다가 자기 매뉴얼은 30.5cm를 요구한다.

    이 테스트가 깨지면 README의 주장도 같이 틀린 것이다.
    """
    TERTILL_DIA = 0.2096  # 8.25 in — 소매 스펙 (매뉴얼엔 스펙표 없음)
    TERTILL_REQUIRED_SPACING = 0.305  # 12 in — 매뉴얼 원문 요구치
    LETTUCE_SPACING_RANGE = (0.20, 0.30)  # 농진청 20 / 충북농기원 잎상추 25·결구 30

    lo, hi = LETTUCE_SPACING_RANGE
    assert TERTILL_DIA > lo, "Tertill이 상추 간격 하한을 지날 수 있으면 논지가 약해진다"
    assert TERTILL_REQUIRED_SPACING > hi, (
        "Tertill 매뉴얼 요구치가 상추 간격 전 범위를 초과해야 논지가 성립한다"
    )


def test_우리_로봇은_상추를_안_밟는다(g, p):
    """Tertill 과의 차이는 형태에서 온다: 바퀴가 작물 사이가 아니라 고랑에 있다.

    그래서 작물 간격이 얼마든 상관이 없다 — 애초에 작물 사이를 지나가지 않는다.
    """
    for bed_i in range(g.n_beds):
        y = parking_y(g, bed_i)
        for wy in p.wheel_ys(g, y):
            assert not g.is_over_bed(wy, p.wheel_width / 2)


# ── 설계 한계를 명시적으로 고정한다 (조용히 넘어가지 않게) ────────────────


@pytest.mark.parametrize(
    "crop,height,fits_at_60cm",
    [
        ("상추", 0.25, True),  # v1 주력 작물
        ("배추", 0.40, False),  # 75cm 포탈이면 됨 — v1 범위 밖
        ("감자", 0.60, False),
        ("고추(제초 시점, 정식 20일)", 0.18, True),  # ← 놀랍게도 들어온다
        ("고추(수확기)", 1.50, False),  # 그런데 그때는 제초가 무의미하다
        ("토마토(수확기)", 2.00, False),
    ],
)
def test_작물_키_한계가_문서와_일치(crop, height, fits_at_60cm):
    """어떤 작물이 되고 안 되는지를 코드로 고정한다. 조용히 넘어가지 않게.

    안 되는 게 부끄러운 게 아니다 — 상용 비전 제초기는 **전부** 작물 키를 제한한다.
    ecoRobotix AVO 는 30cm, FarmBot 은 50cm. 우리는 그들과 같은 자리에 있다.

    그리고 그 캡은 농학이 아니라 **도구 높이의 캡**이다 — AVO 스펙표의 인접한 두 줄이
    "노즐 램프 15~30cm 조절"과 "작물 키 30cm까지 제초 가능"이다. 같은 숫자다.

    고추 두 줄을 같이 본 것이 이 표의 핵심이다: 수확기 150cm는 못 넘지만
    **제초하는 시점의 고추는 18cm라 들어온다**. 잡초 경합 한계기가 정식 후 2~6주고,
    6주 넘겨 제초하면 수량 이득이 0이다. 즉 "키 큰 작물을 못 한다"가 아니라
    "키가 커진 뒤에는 제초할 이유가 없다"가 맞는 문장이다. docs/PLAN.md §2b.
    """
    g = Garden(crop_height=height)
    p = Portal()
    assert (p.headroom(g) > 0) is fits_at_60cm, (
        f"{crop}({height*100:.0f}cm): 여유 {p.headroom(g)*100:+.0f}cm"
    )


def test_배추는_75cm_포탈이면_들어온다():
    """v1이 상추만 하는 건 기하학의 한계가 아니라 **의도적 선택**이다.

    포탈을 75cm로 올리면 배추도 들어온다. 그런데 안 올린다. 이유:

    CropCraft는 작물로 콩과 옥수수만 배포한다. 그래서 상추도 배추도 결국
    **같은 콩 메시를 크기만 바꾼 것**이 된다. 그러면 둘을 가르는 신호가 크기뿐이라,
    "비전이 키 규칙을 이긴다"는 주장이 순환논법이 된다 — 키로만 구분되는 세계를
    만들어놓고 키가 아닌 걸로 구분했다고 말하는 셈이다.

    형태 다양성은 **잡초 쪽**에서 온다(portulaca / polygonum / taraxacum — 서로 다른 메시).
    거기가 진짜 신호가 있는 곳이다. 배추를 넣어도 형태 다양성은 0이므로
    클리어런스를 올릴 값어치가 없다.

    한국 작물 메시를 직접 추가하게 되면 그때 다시 판단한다.
    """
    g = Garden(crop_height=0.40)  # 배추
    assert Portal(clearance=0.60).headroom(g) < 0, "60cm 포탈로는 배추가 안 들어와야 한다"
    assert Portal(clearance=0.75).headroom(g) > 0, "75cm 포탈이면 배추가 들어와야 한다"
