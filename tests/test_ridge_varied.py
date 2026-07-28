"""불균일 두둑 생성물 단언 (Tier 1 — 시뮬 없이). DECISIONS 042.

명세(field_spec)가 "폭이 ±5cm 변한다"고 말하는 것과, **생성된 월드가 실제로 그렇게 생긴 것**은
다른 주장이다. 여기서 후자를 확인한다 — 안 그러면 파라미터만 있고 밭은 그대로인 상태가 된다
(측정은 통과하는데 현실은 안 담긴, 가장 나쁜 실패).

특히 **충돌**을 본다. 시각 메시가 굽어도 충돌이 균일 상자면 로봇은 여전히 반듯한 밭을 달린다.
"""
import re
import sys
from pathlib import Path

import pytest

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from field_spec import DEV, SMOOTH  # noqa: E402
from garden_geometry import Garden  # noqa: E402
from make_ridge_varied import build_obj, collisions, sections  # noqa: E402

G = Garden()
BOX = re.compile(r'<pose>([-\d.]+) ([-\d.]+) ([-\d.]+) 0 0 0</pose>\s*'
                 r'<geometry><box><size>([\d.]+) ([\d.]+) ([\d.]+)</size></box></geometry>')


def boxes(spec, bed=0):
    return [tuple(float(v) for v in m.groups()) for m in BOX.finditer(collisions(spec, bed))]


# ── 충돌이 변이를 담는가 (이게 핵심) ───────────────────────────────────────

def test_collision_boxes_follow_the_varying_profile():
    """충돌 상자가 폭·높이·중심을 따라가야 한다 — 균일 상자면 로봇은 반듯한 밭을 달린다."""
    bs = boxes(DEV)
    assert len(bs) >= 5, "충돌 분절이 너무 적어 변이를 못 담는다"
    widths = [b[4] for b in bs]
    heights = [b[5] for b in bs]
    centers = [b[1] for b in bs]
    assert max(widths) - min(widths) > 0.03, f"충돌 폭이 사실상 균일: {min(widths):.3f}~{max(widths):.3f}"
    assert max(heights) - min(heights) > 0.02, "충돌 높이가 사실상 균일"
    assert max(centers) - min(centers) > 0.03, "충돌 중심선이 안 굽는다"


def test_collision_boxes_sit_on_the_ground():
    """상자 z 중심 = 높이/2 여야 바닥에 붙는다 — 뜨면 로봇이 그 밑으로 들어간다."""
    for x, y, z, dx, dy, dz in boxes(DEV):
        assert z == pytest.approx(dz / 2, abs=1e-3)


def test_collision_chain_is_continuous():
    """상자가 끊기면 그 틈으로 도구가 빠진다 — x 로 이어져야 한다."""
    bs = sorted(boxes(DEV), key=lambda b: b[0])
    for a, b in zip(bs, bs[1:]):
        gap = (b[0] - b[3] / 2) - (a[0] + a[3] / 2)
        assert abs(gap) < 1e-3, f"충돌 상자 사이 {gap*100:.1f}cm 틈"


def test_smooth_spec_still_uniform():
    """매끈 밭은 예전 그대로여야 한다 — 회귀 기준선."""
    bs = boxes(SMOOTH)
    assert len({round(b[4], 6) for b in bs}) == 1
    assert len({round(b[5], 6) for b in bs}) == 1
    assert len({round(b[1], 6) for b in bs}) == 1


# ── 메시가 Fortress 로더 요구를 지키는가 ──────────────────────────────────

def test_obj_starts_with_blender_marker():
    """첫 줄 '# Blender' 를 지우면 Fortress OBJ 로더가 다르게 분기한다 (CLAUDE.md 함정)."""
    assert build_obj(DEV, 0).splitlines()[0] == "# Blender"


def test_obj_is_triangulated_with_normals():
    """quad + UV 인데 법선이 없으면 Ogre 가 assert 로 죽는다(실측 core dump)."""
    obj = build_obj(DEV, 0)
    faces = [l for l in obj.splitlines() if l.startswith("f ")]
    assert faces and all(len(l.split()) == 4 for l in faces), "삼각형이 아니다"
    assert all(t.count("/") == 2 for l in faces for t in l.split()[1:]), "v/vt/vn 형식이 아니다"
    assert any(l.startswith("vn ") for l in obj.splitlines()), "면 법선이 없다"


def test_mesh_tracks_the_spec_amplitude():
    """메시 꼭짓점이 명세 진폭 안에서 실제로 변해야 한다."""
    obj = build_obj(DEV, 0)
    tops = [tuple(float(v) for v in l.split()[1:]) for l in obj.splitlines() if l.startswith("v ")]
    zs = [v[2] for v in tops if v[2] > 0]
    assert max(zs) - min(zs) > DEV.height_var, "메시 높이가 안 변한다"
    assert max(zs) <= G.bed_height + DEV.height_var + 1e-6


def test_section_spacing_resolves_the_meander():
    """단면 간격이 사행 파장보다 훨씬 촘촘해야 한다 — 성기면 굽이가 각지게 잘린다."""
    secs = sections(DEV, 0)
    step = secs[1][0] - secs[0][0]
    assert step <= DEV.meander_wave / 6, f"단면 간격 {step:.2f}m 이 파장 {DEV.meander_wave}m 에 비해 성기다"
