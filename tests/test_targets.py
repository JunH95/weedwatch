"""표적 재관측 매칭 단언 (Tier 1 — 시뮬 없이 산수로).

이 규칙이 코디네이터 루프 안에 박혀 있을 때 실제로 물렸다: 옆 잡초를 같은 것으로 오인해
표적이 계속 앞으로 도망갔고, 그 툴이 막혀 두둑 하나를 통째로 놓쳤다(검출 7 · 타격 0).
여기서 그 상황을 그대로 재현해 둔다.
"""
import sys
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "src" / "weedwatch_control"))
from weedwatch_control.targets import (  # noqa: E402
    MATCH_R, RECEDE_TOL, AIMING, PENDING, STRUCK, match)


def plan(wx, wy, strike_x, phase=PENDING):
    return {"wx": wx, "wy": wy, "strike_x": strike_x, "phase": phase}


def test_same_weed_with_detection_jitter_is_rematched():
    """같은 잡초를 다시 봤다 — 검출 잡음 몇 cm 은 같은 것으로 본다."""
    p = plan(1.00, 0.60, 0.91)
    got = match([p], 1.02, 0.61, 0.93, ox=0.50, d=+1)
    assert got is p


def test_next_weed_ahead_is_not_swallowed():
    """15cm 앞 다음 잡초는 **다른 잡초**다. 이걸 삼키면 표적이 도망가 영영 안 쳐진다."""
    p = plan(1.00, 0.60, 0.91)
    assert match([p], 1.15, 0.60, 1.06, ox=0.50, d=+1) is None


def test_receding_update_is_refused_even_when_close():
    """거리로는 같은 잡초여도 **멀어지는 갱신**은 안 받는다 — 도망 모드의 원인."""
    p = plan(1.00, 0.60, 0.91)
    far = 0.91 + RECEDE_TOL + 0.02
    assert match([p], 1.00 + RECEDE_TOL + 0.02, 0.60, far, ox=0.50, d=+1) is None
    near = 0.91 - 0.03                     # 가까워지는 쪽은 받는다 (슬립을 되돌리는 방향)
    assert match([p], 0.97, 0.60, near, ox=0.50, d=+1) is p


def test_reverse_pass_uses_the_same_rule():
    """되돌아오는 패스(d=-1)에선 **x 가 작을수록 앞**이다 — 방향 판정이 같이 뒤집혀야 한다."""
    p = plan(1.00, 1.80, 1.09)                                   # ox=1.50 에서 남은 거리 0.41
    assert match([p], 1.02, 1.80, 1.11, ox=1.50, d=-1) is p      # 남은 0.39 — 다가감
    assert match([p], 0.92, 1.80, 1.01, ox=1.50, d=-1) is None   # 남은 0.49 — 앞의 다음 잡초


def test_already_struck_plan_is_never_rematched():
    """친 표적을 다시 잡으면 도구가 두 번 내려간다."""
    p = plan(1.00, 0.60, 0.91, phase=STRUCK)
    assert match([p], 1.00, 0.60, 0.91, ox=0.95, d=+1) is None


def test_aiming_plan_is_still_updatable():
    """조준 중(캐리지 나감)에도 갱신은 받아야 한다 — 재관측의 이득이 거기서 나온다."""
    p = plan(1.00, 0.60, 0.91, phase=AIMING)
    assert match([p], 1.01, 0.60, 0.92, ox=0.80, d=+1) is p


def test_two_weeds_closer_than_match_radius_still_separate_by_direction():
    """MATCH_R 안이라도 앞에 있는 쪽은 다른 잡초로 남는다 — 거리만으론 못 가른다."""
    near_gap = MATCH_R - 0.02
    p = plan(1.00, 0.60, 0.91)
    assert match([p], 1.00 + near_gap, 0.60, 0.91 + near_gap, ox=0.50, d=+1) is None
