#!/usr/bin/env python3
"""로봇 URDF 를 생성한다. 시각 메시(예쁜 것) + 충돌 프리미티브(단순한 것) + 조인트.

── 왜 스크립트로 생성하나 ──────────────────────────────────────────────
치수 단일 출처가 tools/garden_geometry.py 다. URDF 를 손으로 쓰면 숫자가 두 곳에
적혀서 어긋난다. 그래서 여기서 garden_geometry.py 와 robot_body.py 가 만든
models/weedwatch_robot/links.json(링크별 조인트 원점)을 읽어 URDF 를 찍는다.

── 시각 vs 충돌 (중요) ─────────────────────────────────────────────────
<visual>  = robot_body.py 가 만든 예쁜 OBJ (러그 바퀴, 쐐기 몸통...). 카메라용.
<collision> = 단순 프리미티브(상자·실린더). 물리 엔진용. 시각 메시로 충돌 계산하면
             느리고 불안정하다. 표준 관행: 보는 건 정교하게, 부딪히는 건 단순하게.

── 조인트 구조 ─────────────────────────────────────────────────────────
base_link (몸통, 고정 기준)
 ├─ wheel_fl/fr/rl/rr  : continuous (바퀴 회전, diff-drive)
 ├─ carriage           : prismatic Y (두둑 폭을 좌우로 훑음, ±0.45)
 │   └─ tool           : prismatic Z (점 타격 막대, 아래로)
 └─ (카메라·LED 는 캐리지 메시에 포함, 별도 링크 아님 — 캐리지 따라 움직임)

사용법:
    tools/make_urdf.py > models/weedwatch_robot/weedwatch.urdf
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from garden_geometry import Garden, Portal  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "weedwatch_robot"
MESH_URI = "package://weedwatch_robot"  # Gazebo 가 model:// 로도 찾게 아래에서 조정

G = Garden()
P = Portal()


def load_origins() -> dict:
    f = MODEL / "links.json"
    if not f.exists():
        sys.exit("links.json 이 없습니다. 먼저: blender --background --python tools/robot_body.py -- export")
    return json.loads(f.read_text())


def xyz(v) -> str:
    return f"{v[0]:.5f} {v[1]:.5f} {v[2]:.5f}"


def link_with_mesh(name: str, mesh: str, collision: str) -> str:
    """시각은 OBJ 메시, 충돌은 프리미티브 XML 문자열."""
    return f"""  <link name="{name}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="model://weedwatch_robot/{mesh}"/></geometry>
    </visual>
{collision}
    <inertial>
      <mass value="{_mass(name)}"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
"""


def _mass(name: str) -> float:
    if name == "base_link":
        return 20.0
    if name.startswith("wheel"):
        return 1.5
    if name == "carriage":
        return 1.0
    if name == "tool":
        return 0.2
    return 0.5


def collision_box(size, origin=(0, 0, 0)) -> str:
    sx, sy, sz = size
    return f"""    <collision>
      <origin xyz="{origin[0]:.4f} {origin[1]:.4f} {origin[2]:.4f}" rpy="0 0 0"/>
      <geometry><box size="{sx:.4f} {sy:.4f} {sz:.4f}"/></geometry>
    </collision>"""


def collision_cyl(radius, length) -> str:
    # 바퀴: 축이 y 라 rpy 로 눕힘
    return f"""    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{radius:.4f}" length="{length:.4f}"/></geometry>
    </collision>"""


def build_urdf() -> str:
    o = load_origins()
    deck_w = P.deck_width(G)

    out = ['<?xml version="1.0"?>', '<robot name="weedwatch">', ""]

    # ── base_link: 몸통. 충돌은 두 사이드 포드 상자로 근사 (가운데는 터널이라 비움) ──
    pod_h = P.pod_drop
    pod_cz = (P.deck_top_z() - P.deck_thickness) - pod_h / 2
    base_col = "\n".join([
        collision_box((P.deck_length, P.pod_width, pod_h), (0, +P.track(G) / 2, pod_cz)),
        collision_box((P.deck_length, P.pod_width, pod_h), (0, -P.track(G) / 2, pod_cz)),
        collision_box((P.deck_length, deck_w, P.deck_thickness),
                      (0, 0, P.deck_top_z() - P.deck_thickness / 2)),  # 데크 상판
    ])
    out.append(link_with_mesh("base_link", "base.obj", base_col))

    # ── 바퀴 4개: continuous, 축 y ──
    wheel_col = collision_cyl(P.wheel_dia / 2, P.wheel_width)
    for link in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        out.append(link_with_mesh(link, f"{link}.obj", wheel_col))
        out.append(f"""  <joint name="{link}_joint" type="continuous">
    <parent link="base_link"/>
    <child link="{link}"/>
    <origin xyz="{xyz(o[link])}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
""")

    # ── 캐리지: prismatic Y (두둑 폭 훑기) ──
    cs = P.carriage_size
    car_col = collision_box((cs * 1.4, cs, cs))
    out.append(link_with_mesh("carriage", "carriage.obj", car_col))
    out.append(f"""  <joint name="carriage_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="carriage"/>
    <origin xyz="{xyz(o['carriage'])}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{-P.carriage_travel:.3f}" upper="{P.carriage_travel:.3f}" effort="50" velocity="0.5"/>
  </joint>
""")

    # ── tool: prismatic Z (점 타격 막대, 캐리지 자식) ──
    tool_col = collision_cyl(P.tool_rod_dia / 2, P.z_travel)
    out.append(link_with_mesh("tool", "tool.obj", tool_col))
    # tool 원점은 캐리지 원점 기준 상대좌표로
    tool_rel = [o["tool"][i] - o["carriage"][i] for i in range(3)]
    out.append(f"""  <joint name="tool_joint" type="prismatic">
    <parent link="carriage"/>
    <child link="tool"/>
    <origin xyz="{xyz(tool_rel)}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{-P.z_travel:.3f}" upper="0" effort="50" velocity="0.3"/>
  </joint>
""")

    out.append("</robot>")
    return "\n".join(out)


if __name__ == "__main__":
    sys.stdout.write(build_urdf())
