"""카메라 프레임률이 주행 커버리지에 충분한가 (Tier 1 — 순수 산수).

왜 생겼나: 카메라를 2대로 늘리자(DECISIONS 026) Tier-3 시뮬이 0.200x → 0.031x 로 6.5배 느려져
주행 하네스가 완주를 못 했다(8GB GPU 에서 1280×720 렌더타깃 2개 + best.pt 추론 경합). 프레임률은
정밀도 게이트가 아니라 **시뮬 비용 노브**라, 15Hz → 5Hz 로 낮춰 0.344x 를 얻었다(2대인데 예전
1대보다 1.7배 빠름).

다만 "비용이라 낮춰도 된다"가 무한정은 아니다. 너무 낮추면 주행 중 프레임 사이가 벌어져 두둑을
띄엄띄엄 보게 되고, 그 틈의 잡초는 영영 못 본다 — 카메라 폭이 모자랐던 것과 똑같은 실패가 주행
방향으로 생긴다. 그 하한을 산수로 박아 둔다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from garden_geometry import Garden, Portal  # noqa: E402

G, P = Garden(), Portal()

# 무정차 상한 속도 (DECISIONS 020 실측: 0.3 은 하강 창이 133ms 라 못 따라감).
DRIVE_V = 0.20
# 연속 프레임이 주행 방향으로 최소 이만큼은 겹쳐야 한다(발자국 대비). 2 = 2배 이상 겹침.
MIN_OVERLAP_FACTOR = 2.0


def test_프레임_간격이_주행방향_발자국보다_충분히_촘촘하다():
    spacing = DRIVE_V / P.camera_rate                  # 프레임 사이 이동 거리 [m]
    foot = P.camera_footprint_h(G)                     # 주행 방향 발자국 [m]
    assert spacing * MIN_OVERLAP_FACTOR <= foot, (
        f"프레임 간격 {spacing*100:.1f}cm × {MIN_OVERLAP_FACTOR} > 주행방향 발자국 {foot*100:.1f}cm "
        f"— 프레임률 {P.camera_rate}Hz 가 낮아 두둑을 띄엄띄엄 본다")


def test_프레임률이_양수이고_과하지_않다():
    """0 이면 안 찍히고, 너무 높으면 렌더 비용이 대수만큼 곱해져 시뮬이 못 쓰게 느려진다."""
    assert P.camera_rate > 0
    # 2대 15Hz 에서 0.031x 로 못 쓰게 됐다. 대수×프레임률을 예산으로 본다.
    assert P.camera_rate * P.n_cameras <= 20, (
        f"카메라 {P.n_cameras}대 × {P.camera_rate}Hz = {P.camera_rate*P.n_cameras} "
        f"— 렌더 예산 초과(실측: 2대×15Hz 는 sim 0.031x 로 주행 불가)")


def test_한_잡초가_여러_프레임에_잡힌다():
    """검출 안정성의 근거 — 지나가는 잡초가 최소 몇 장에는 찍혀야 노이즈에 덜 흔들린다."""
    frames_per_weed = P.camera_footprint_h(G) / (DRIVE_V / P.camera_rate)
    assert frames_per_weed >= 4, f"잡초 하나가 {frames_per_weed:.1f}장에만 찍힘 — 너무 적다"
