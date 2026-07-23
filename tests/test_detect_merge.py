"""두 카메라 겹침 구간 중복 병합 로직 (Tier 1 — 순수 산수, GPU·시뮬·torch 불필요).

카메라가 2대가 되면(DECISIONS 026) 가운데 겹침 0.135m 안의 잡초는 **두 번** 검출된다. 그대로 두면
로봇이 한 잡초를 두 번 찍는다. merge_detections 가 그걸 하나로 합치는데, 반경이 너무 크면 진짜로
가까운 별개 잡초까지 합쳐 재현율을 깎는다 — 그 두 실패를 다 여기서 막는다.

merge_detections 는 순수 함수(numpy·torch 안 씀)라 ML venv 없이 단언된다. detect_server 를 통째로
import 하면 torch 가 딸려오므로, 함수 소스만 떼어 실행한다(경계 규율).
"""
import math
import re
import sys
from pathlib import Path

WW = Path(__file__).resolve().parents[1]

# detect_server 는 torch 를 import 하므로 모듈 전체를 못 들여온다(ROS 쪽 파이썬엔 torch 없음).
# 순수 함수 정의만 소스에서 떼어 실행한다 — 테스트가 실제 구현을 검사하되 의존성은 안 끌어온다.
_src = (WW / "perception" / "detect_server.py").read_text()
_dedup_r = float(re.search(r"^DEDUP_R\s*=\s*([\d.]+)", _src, re.M).group(1))  # 실제 상수를 쓴다
_start = _src.index("def merge_detections")
_end = _src.index("def detect_fused")
_ns = {"math": math, "DEDUP_R": _dedup_r}
exec(compile(_src[_start:_end], "detect_server.merge", "exec"), _ns)
merge_detections = _ns["merge_detections"]


def test_기본_반경이_겹침폭보다_작다():
    """기본 dedup 반경이 카메라 겹침(0.135m)보다 훨씬 작아야 한다 — 아니면 겹침 밖 별개 잡초까지 먹는다."""
    sys.path.insert(0, str(WW / "tools"))
    from garden_geometry import Garden, Portal
    G, P = Garden(), Portal()
    assert 0 < _dedup_r < P.camera_overlap(G) / 2, (
        f"DEDUP_R {_dedup_r} 가 겹침 {P.camera_overlap(G):.3f} 대비 과하다")


def test_겹침_중복이_하나로_합쳐진다():
    """같은 잡초를 두 카메라가 2cm 떨어져 본 상황 → 1개가 돼야 한다."""
    out = merge_detections([(1.000, 0.500, 900), (1.020, 0.505, 700)], radius=0.05)
    assert len(out) == 1, f"중복이 안 합쳐짐: {out}"
    # 위치는 두 검출 사이 (면적 가중), 면적은 max(더하지 않는다 — 한 잡초다)
    assert 1.000 <= out[0][0] <= 1.020
    assert out[0][2] == 900


def test_떨어진_잡초는_안_합쳐진다():
    """반경 밖 별개 잡초 2개는 2개로 남아야 한다 — 과병합은 재현율 손실."""
    out = merge_detections([(1.00, 0.50, 900), (1.00, 0.62, 800)], radius=0.05)
    assert len(out) == 2, f"별개 잡초가 합쳐짐: {out}"


def test_경계_바로_밖은_유지된다():
    """반경보다 아주 조금 먼 건 별개로 — 경계 조건."""
    out = merge_detections([(0.0, 0.0, 500), (0.0, 0.0501, 500)], radius=0.05)
    assert len(out) == 2


def test_경계_바로_안은_합쳐진다():
    out = merge_detections([(0.0, 0.0, 500), (0.0, 0.0499, 500)], radius=0.05)
    assert len(out) == 1


def test_큰_blob_이_기준이_된다():
    """면적 큰 검출이 클러스터 대표 — 작은 파편이 위치를 끌고가면 안 된다."""
    out = merge_detections([(0.0, 0.0, 100), (0.0, 0.03, 2000)], radius=0.05)
    assert len(out) == 1
    assert out[0][1] > 0.025, f"작은 파편이 위치를 끌어감: {out}"
    assert out[0][2] == 2000


def test_빈_입력과_단일_입력():
    assert merge_detections([]) == []
    assert merge_detections([(1.0, 2.0, 300)]) == [(1.0, 2.0, 300)]


def test_세_개_연쇄가_하나로():
    """겹침 구간에 파편이 여럿 떨어져도 하나로 모인다."""
    out = merge_detections([(0.0, 0.0, 900), (0.0, 0.02, 800), (0.0, 0.04, 700)], radius=0.05)
    assert len(out) == 1, f"{out}"
