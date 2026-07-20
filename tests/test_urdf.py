"""생성된 URDF 가 diff-drive 로 주행 가능한 형태인가 — 산수로 검사한다 (Tier 1).

시뮬도 GPU도 필요 없다. build_urdf() 출력을 파싱해 확인한다.

이 테스트가 존재하는 이유: diff-drive 를 붙이며 세 버그를 냈고, 전부 여기서 잡는다.
  1. tool 의 izz 가 3.6e-6 인데 %.5f 로 "0.00000" 이 됐다 → DART 가 무효 관성으로
     관절체 전체를 망가뜨려 바퀴가 떴다. (관성 대각성분이 0 이면 실패)
  2. 바퀴 메시 이름이 ROS 규약과 반대라(fl 이 y<0) 좌/우를 이름으로 배정하면 회전이
     뒤집힌다. (left_joint 는 +Y 바퀴여야)
  3. 빔이 clearance 가 아니라 pod 아래 낮게 놓여 캐리지·도구가 바퀴 밑으로 매달려
     로봇이 캐리지로 섰다. (모든 비-바퀴 충돌은 바퀴 접지면 z=0 위에 있어야)

실행:  ./scripts/env.sh python3 -m pytest tests/test_urdf.py -v
"""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import make_urdf  # noqa: E402
from garden_geometry import Garden, Portal  # noqa: E402

G = Garden()
P = Portal()

# links.json 은 Blender export 산출물(models/ 는 gitignore). 없으면(export 전 신선한 클론)
# 이 파일 전체를 skip 한다 — 순수 산수인 test_inertia/test_garden_geometry 와 달리
# 이 테스트는 export 된 링크 원점에 의존한다. 있으면 회귀 가드로 동작.
_LINKS = ROOT / "models" / "weedwatch_robot" / "links.json"
pytestmark = pytest.mark.skipif(
    not _LINKS.exists(),
    reason="links.json 없음 — 먼저 'blender --background --python tools/robot_body.py -- export'",
)


@pytest.fixture(scope="module")
def urdf() -> ET.Element:
    return ET.fromstring(make_urdf.build_urdf())


@pytest.fixture(scope="module")
def origins() -> dict:
    return json.loads((ROOT / "models" / "weedwatch_robot" / "links.json").read_text())


# ── 버그 1: 관성 대각성분이 0 이면 DART 가 물리를 망가뜨린다 ────────────────


def test_모든_링크_관성_대각성분_양수(urdf):
    """izz=0 같은 무효 관성이 하나라도 있으면 DART 가 관절체 전체를 망가뜨린다.
    tool(가는 막대)의 izz=3.6e-6 이 %.5f 반올림으로 0 이 됐던 게 실제 버그였다."""
    inertias = urdf.findall(".//inertial/inertia")
    assert len(inertias) == 7, f"링크 7개인데 관성 {len(inertias)}개"
    for i in inertias:
        for ax in ("ixx", "iyy", "izz"):
            v = float(i.get(ax))
            assert v > 0, f"{ax}={v} — 0 이하 관성은 DART 를 깨뜨린다"


# ── 버그 2: 좌/우는 이름이 아니라 실제 Y 부호로 배정해야 회전이 맞다 ─────────


def _diff_drive(urdf):
    for plugin in urdf.findall(".//gazebo/plugin"):
        if "DiffDrive" in (plugin.get("name") or ""):
            return plugin
    raise AssertionError("DiffDrive 플러그인이 없다")


def test_diff_drive_좌우가_실제Y부호대로(urdf, origins):
    """left_joint 는 +Y(REP-103 왼쪽), right_joint 는 -Y 여야 한다.
    메시 이름(fl 이 y<0)으로 배정하면 +z 명령에 오른쪽으로 도는 버그가 난다."""
    dd = _diff_drive(urdf)
    left = [e.text for e in dd.findall("left_joint")]
    right = [e.text for e in dd.findall("right_joint")]
    assert len(left) == 2 and len(right) == 2
    for j in left:
        link = j.replace("_joint", "")
        assert origins[link][1] > 0, f"left_joint {j} 가 +Y 가 아니다 (y={origins[link][1]})"
    for j in right:
        link = j.replace("_joint", "")
        assert origins[link][1] < 0, f"right_joint {j} 가 -Y 가 아니다 (y={origins[link][1]})"


def test_wheel_separation_이_실제_바퀴간격(urdf, origins):
    """track()=1.20 이 아니라 실제 바퀴 y 간격(≈1.2249)을 써야 오도메트리가 맞다."""
    dd = _diff_drive(urdf)
    sep = float(dd.find("wheel_separation").text)
    ys = [origins[w][1] for w in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")]
    assert sep == pytest.approx(max(ys) - min(ys), abs=1e-4)


def test_wheel_radius_가_바퀴_반지름(urdf):
    dd = _diff_drive(urdf)
    r = float(dd.find("wheel_radius").text)
    assert r == pytest.approx(P.wheel_dia / 2)


# ── 버그 3: 비-바퀴 충돌이 바퀴 접지면(z=0) 아래로 내려오면 안 된다 ──────────


def test_캐리지가_바퀴_접지면_위(origins):
    """캐리지 충돌 박스(높이 carriage_size)가 바퀴 바닥(z=0)보다 위에 있어야 한다.
    안 그러면 로봇이 캐리지로 서고 바퀴가 떠서 접지력이 0 이 된다 (실제 버그)."""
    carriage_bottom = origins["carriage"][2] - P.carriage_size / 2
    assert carriage_bottom > 0.05, f"캐리지 바닥 {carriage_bottom:.3f} 이 바퀴 접지면에 너무 가깝다"


def test_도구_기본자세가_바퀴_접지면_위(origins):
    """도구(Z 프리즘, 기본=접힘)의 충돌이 바퀴 바닥 위에 있어야 한다.
    (내려찍기 자세에서는 당연히 내려가지만, 이동 기본자세에서는 접혀 있어야 한다.)"""
    tool_bottom = origins["tool"][2] - P.tool_rod_len / 2
    assert tool_bottom > 0, f"도구 기본자세 바닥 {tool_bottom:.3f} 이 지면 아래다"


# ── Y/Z 관절 위치 컨트롤러 (mm 정밀의 수단) ──────────────────────────────────


def test_관절_위치컨트롤러가_두_프리즘관절에_붙었다(urdf):
    """캐리지(Y)·도구(Z) 에 JointPositionController + 명령 토픽이 있어야 한다."""
    ctrl = {}
    for plugin in urdf.findall(".//gazebo/plugin"):
        if "JointPositionController" in (plugin.get("name") or ""):
            ctrl[plugin.find("joint_name").text] = plugin.find("topic").text
    assert ctrl.get("carriage_joint") == "carriage_cmd"
    assert ctrl.get("tool_joint") == "tool_cmd"


def test_프리즘_관절_한계가_설계대로(urdf):
    """캐리지 ±carriage_travel, 도구 -z_travel~0 (0=접힘). 성공 기준의 도달 범위."""
    joints = {j.get("name"): j for j in urdf.findall(".//joint")}
    c = joints["carriage_joint"].find("limit")
    assert float(c.get("lower")) == pytest.approx(-P.carriage_travel)
    assert float(c.get("upper")) == pytest.approx(P.carriage_travel)
    t = joints["tool_joint"].find("limit")
    assert float(t.get("lower")) == pytest.approx(-P.z_travel)
    assert float(t.get("upper")) == pytest.approx(0.0)
