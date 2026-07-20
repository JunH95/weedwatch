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
from inertia import Part, box_inertia, combine, cylinder_inertia  # noqa: E402

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


def link_with_mesh(name: str, mesh: str, collision: str, inertial: str) -> str:
    """시각은 OBJ 메시, 충돌은 프리미티브 XML, 관성은 산수로 계산한 XML."""
    return f"""  <link name="{name}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="model://weedwatch_robot/{mesh}"/></geometry>
    </visual>
{collision}
{inertial}
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


def inertial_xml(mass: float, com, tensor) -> str:
    """URDF <inertial>. origin=COM(링크 원점 기준), inertia 는 COM 프레임 기준.

    관성이 0.01 자리채움이면 diff-drive 가 불안정하다 (yaw 관성이 실제의 750배 작음).
    tools/inertia.py 가 프리미티브 치수에서 물리적으로 계산한 값을 넣는다.
    """
    ixx, iyy, izz, ixy, ixz, iyz = tensor
    cx, cy, cz = com
    # ⚠️ 유효숫자를 보존해야 한다. tool(가는 막대)의 izz 는 3.6e-6 이라 %.5f 로 찍으면
    # "0.00000" 이 되고, izz=0 인 무효 관성은 DART 의 관절체 알고리즘을 통째로 망가뜨려
    # 바퀴가 지면에서 뜨고 접지력이 사라진다 (실제로 이 버그로 몇 시간 헤맴). %g 로 찍는다.
    if not (ixx > 0 and iyy > 0 and izz > 0):
        raise ValueError(f"관성 대각성분이 0 이하: ixx={ixx} iyy={iyy} izz={izz} (질량 {mass})")
    return f"""    <inertial>
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="0 0 0"/>
      <mass value="{mass:.4f}"/>
      <inertia ixx="{ixx:.8g}" iyy="{iyy:.8g}" izz="{izz:.8g}" ixy="{ixy:.8g}" ixz="{ixz:.8g}" iyz="{iyz:.8g}"/>
    </inertial>"""


def base_inertial() -> str:
    """base_link 관성 = 충돌 프리미티브(사이드 포드 2 + 데크)의 합성.

    질량(20kg)은 부피 비례로 세 상자에 나눠 담고 평행축 정리로 합친다. COM 이 위쪽
    (z≈0.5, 데크·포드가 위에 있음)으로 나오는 게 정직하다 — 예전 origin=(0,0,0)은
    질량이 지면에 있다는 뜻이라 물리적으로 틀렸다. build_urdf 의 충돌 상자와 동일 치수.
    """
    pod_h = P.pod_drop
    pod_cz = (P.deck_top_z() - P.deck_thickness) - pod_h / 2
    deck_cz = P.deck_top_z() - P.deck_thickness / 2
    deck_w = P.deck_width(G)
    half_track = P.track(G) / 2
    boxes = [  # (size, center) — build_urdf 의 base 충돌 상자와 정확히 일치해야 한다
        ((P.deck_length, P.pod_width, pod_h), (0.0, +half_track, pod_cz)),
        ((P.deck_length, P.pod_width, pod_h), (0.0, -half_track, pod_cz)),
        ((P.deck_length, deck_w, P.deck_thickness), (0.0, 0.0, deck_cz)),
    ]
    vols = [sx * sy * sz for (sx, sy, sz), _ in boxes]
    total_v = sum(vols)
    mass = _mass("base_link")
    parts = [
        Part(mass * v / total_v, c, box_inertia(mass * v / total_v, *size))
        for (size, c), v in zip(boxes, vols)
    ]
    m, com, tensor = combine(parts)
    return inertial_xml(m, com, tensor)


def collision_box(size, origin=(0, 0, 0)) -> str:
    sx, sy, sz = size
    return f"""    <collision>
      <origin xyz="{origin[0]:.4f} {origin[1]:.4f} {origin[2]:.4f}" rpy="0 0 0"/>
      <geometry><box size="{sx:.4f} {sy:.4f} {sz:.4f}"/></geometry>
    </collision>"""


def collision_cyl(radius, length, axis="y") -> str:
    """실린더 충돌. axis='y'(바퀴, rpy 로 눕힘) 또는 'z'(도구 막대, 기본 세움)."""
    rpy = "1.5708 0 0" if axis == "y" else "0 0 0"
    return f"""    <collision>
      <origin xyz="0 0 0" rpy="{rpy}"/>
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
    out.append(link_with_mesh("base_link", "base.obj", base_col, base_inertial()))

    # ── 바퀴 4개: continuous, 축 y ──
    wheel_col = collision_cyl(P.wheel_dia / 2, P.wheel_width)
    wm = _mass("wheel_fl")
    wheel_in = inertial_xml(wm, (0, 0, 0),
                            cylinder_inertia(wm, P.wheel_dia / 2, P.wheel_width, axis="y"))
    for link in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        out.append(link_with_mesh(link, f"{link}.obj", wheel_col, wheel_in))
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
    cm = _mass("carriage")
    car_in = inertial_xml(cm, (0, 0, 0), box_inertia(cm, cs * 1.4, cs, cs))
    out.append(link_with_mesh("carriage", "carriage.obj", car_col, car_in))
    out.append(f"""  <joint name="carriage_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="carriage"/>
    <origin xyz="{xyz(o['carriage'])}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="{-P.carriage_travel:.3f}" upper="{P.carriage_travel:.3f}" effort="50" velocity="0.5"/>
  </joint>
""")

    # ── tool: prismatic Z (점 타격 막대, 캐리지 자식) ──
    # 세로 막대라 충돌도 z 축. 길이는 z_travel(행정)이 아니라 막대 물리 길이(tool_rod_len).
    # 행정을 쓰면 충돌·관성이 시각 막대보다 길어져 접힘 자세에서 두둑을 파고든다.
    tool_col = collision_cyl(P.tool_rod_dia / 2, P.tool_rod_len, axis="z")
    tm = _mass("tool")
    tool_in = inertial_xml(tm, (0, 0, 0),
                           cylinder_inertia(tm, P.tool_rod_dia / 2, P.tool_rod_len, axis="z"))
    out.append(link_with_mesh("tool", "tool.obj", tool_col, tool_in))
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

    out.append(wheel_friction_gazebo())
    out.append(diff_drive_gazebo(o))
    out.append(joint_controllers_gazebo())
    out.append(camera_sensor_gazebo())
    out.append("</robot>")
    return "\n".join(out)


def camera_sensor_gazebo() -> str:
    """캐리지에 강체 고정된 하방 카메라 (DECISIONS 006).

    카메라가 캐리지에 붙어 같이 움직이므로 툴 팁이 항상 같은 픽셀에 온다 → 헤드리스
    픽셀 단언의 기반. 캐리지가 Y 로 훑으며 좁은 시야로 두둑을 커버하는 근접 스캔형이다.

    ⚠️ 높이: 밭 위 ~18cm(근접). px/cm 은 높지만 시야가 좁다. 더 높여 넓게 보려면 카메라를
    카리지·툴에 안 가리게 앞으로 빼는 설계가 필요 — 디자인 패스로 남김. 지금은 "잡초를
    일찍(작을 때) 잡는다"는 타깃 체제(작은 식물)엔 근접 스캔이 맞다.

    pose 는 캐리지 링크 원점 기준. rpy=(0,π/2,0) → 광축 +X 를 -Z(정하방)로.
    ⚠️ z 는 LED 디스크(월드 z≈0.40) **아래**(월드 z≈0.395)에 둔다. 안 그러면 하방 카메라가
    자기 LED 를 들여다본다(균일 화면). 카메라+LED 를 세로로 쌓은 게 원인 — LED 를 렌즈
    둘레 링으로 바꾸는 조립 재설계는 디자인 패스로 남김. 지금은 센서만 LED 밑으로 내린다.
    """
    return """  <gazebo reference="carriage">
    <sensor name="down_cam" type="camera">
      <pose>0.0225 0 -0.0698 0 1.5708 0</pose>
      <topic>robot/camera</topic>
      <update_rate>15</update_rate>
      <always_on>1</always_on>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.02</near><far>10</far></clip>
        <save enabled="true"><path>artifacts/camera</path></save>
      </camera>
    </sensor>
  </gazebo>
"""


def joint_controllers_gazebo() -> str:
    """캐리지(Y)·도구(Z) 프리즘 관절의 위치 컨트롤러 (Fortress JointPositionController).

    각 관절이 명령한 위치로 PID 힘 제어된다. 명령 토픽:
      /carriage_cmd  (ignition.msgs.Double, m)  — 두둑 폭을 좌우로 (±0.45)
      /tool_cmd      (ignition.msgs.Double, m)  — 점 타격 막대 상하 (-0.35~0, 0=접힘)

    ── 게인은 왜 이 값인가 ──────────────────────────────────────────────────
    위치 제어는 스프링-댐퍼처럼 동작한다: ω=√(p/m), 임계감쇠 d=2√(pm).
    캐리지(~1.2kg, 수평이라 중력 무관): p=150 → ω≈11, d≈27.
    도구(0.2kg, 수직이라 중력 1.96N 이 정상상태 오차를 만든다): p 만으론 오차 mg/p 가
    남으므로 i 게인으로 없앤다. 값은 assert_joints.py 로 실측 튜닝했다 (오버슈트·정착).
    """
    specs = [
        ("carriage_joint", "carriage_cmd", 700.0, 6.0, 60.0),
        ("tool_joint", "tool_cmd", 1000.0, 25.0, 35.0),
    ]
    blocks = []
    for joint, topic, p, i, d in specs:
        blocks.append(f"""  <gazebo>
    <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>{topic}</topic>
      <p_gain>{p}</p_gain>
      <i_gain>{i}</i_gain>
      <d_gain>{d}</d_gain>
    </plugin>
  </gazebo>""")
    return "\n".join(blocks)


def wheel_friction_gazebo() -> str:
    """바퀴-지면 접촉의 마찰과 강성 (URDF <gazebo reference> → SDF surface).

    이게 없으면 바퀴가 명령대로 회전만 하고 **접지력이 없어 헛돈다** (실측: odom 은
    1.69m 전진을 보고하는데 몸통은 0.001m). 원인 두 가지를 같이 잡는다:
      mu1/mu2 = 마찰 (헛돎 방지)
      kp/kd   = 접촉 강성 (약하면 바퀴가 지면에 살짝 떠 접지력이 0 이 된다)
    sdformat 의 URDF 파서가 이 태그들을 surface/friction/ode 로 번역한다.
    """
    blocks = []
    for link in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        blocks.append(f"""  <gazebo reference="{link}">
    <mu1>1.2</mu1>
    <mu2>1.2</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>1.0</maxVel>
  </gazebo>""")
    return "\n".join(blocks)


def diff_drive_gazebo(o: dict) -> str:
    """Fortress 내장 DiffDrive 시스템 플러그인 (URDF <gazebo> 확장).

    ── 좌/우는 이름이 아니라 실제 Y 부호로 배정한다 (중요) ──────────────────
    러그 바퀴 메시의 bbox 중심이 track/2(0.60)이 아니라 ±0.6124 로 나오고, 게다가
    robot_body 의 'fl'(front-left) 이름이 실제로는 y<0 (ROS REP-103 에서 +Y=왼쪽이므로
    오른쪽)에 있다. 이름으로 배정하면 회전이 반대로 돈다. links.json 의 실제 y 로 가른다.

    wheel_separation 도 track()=1.20 이 아니라 실제 바퀴 간격(≈1.2249)을 써야
    오도메트리와 회전 반경이 맞는다.
    """
    wheels = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
    left = sorted(w for w in wheels if o[w][1] > 0)   # +Y = 왼쪽 (REP-103)
    right = sorted(w for w in wheels if o[w][1] < 0)   # -Y = 오른쪽
    assert len(left) == 2 and len(right) == 2, f"좌우 바퀴 배정 실패: {left=} {right=}"
    ys = [o[w][1] for w in wheels]
    wheel_sep = max(ys) - min(ys)
    wheel_radius = P.wheel_dia / 2

    left_tags = "\n".join(f"      <left_joint>{w}_joint</left_joint>" for w in left)
    right_tags = "\n".join(f"      <right_joint>{w}_joint</right_joint>" for w in right)

    # max_*_acceleration: 부드럽게 출발/정지시켜 무게중심 높은 몸통이 피치로 넘어가지
    # 않게 한다 (휠베이스 0.45 < 2·COM높이라 급가속엔 앞으로 고꾸라질 수 있다).
    return f"""  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive">
{left_tags}
{right_tags}
      <wheel_separation>{wheel_sep:.5f}</wheel_separation>
      <wheel_radius>{wheel_radius:.5f}</wheel_radius>
      <max_linear_acceleration>0.5</max_linear_acceleration>
      <max_angular_acceleration>1.0</max_angular_acceleration>
      <topic>cmd_vel</topic>
      <odom_topic>odometry</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>
    <plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher"/>
  </gazebo>
"""


if __name__ == "__main__":
    sys.stdout.write(build_urdf())
