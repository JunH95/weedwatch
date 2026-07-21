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
    # 실부품 질량 (DESIGN.md). 총 ~34.5kg — sim 초기 임의값 27kg 을 실물로 승격.
    if name == "base_link":
        return 22.0   # 프레임·데크 12.4 + 배터리 4.8 + 전장 4.8 (포드 안 낮게, base_inertial)
    if name.startswith("wheel"):
        return 2.5    # ZLTECH 8″ 허브 BLDC 서보 (모터 내장)
    if name == "carriage":
        return 2.0    # Y 게이트리 + NEMA23 + 카메라 팔·D405
    if name == "tool":
        return 0.5    # Z 스텝 + 리드스크류 너트 + STS304 막대
    return 0.5


def inertial_xml(mass: float, com, tensor) -> str:
    """URDF <inertial>. origin=COM(링크 원점 기준), inertia 는 COM 프레임 기준.

    관성이 0.01 자리채움이면 diff-drive 가 불안정하다 (yaw 관성이 실제의 750배 작음).
    tools/inertia.py 가 프리미티브 치수에서 물리적으로 계산한 값을 넣는다.
    """
    ixx, iyy, izz, ixy, ixz, iyz = tensor
    cx, cy, cz = com
    # 주의: 유효숫자를 보존해야 한다. tool(가는 막대)의 izz 는 3.6e-6 이라 %.5f 로 찍으면
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
    m_lowmass = 9.6                        # 배터리 4.8 + 전장함 4.8 (포드 안 낮게)
    m_struct = _mass("base_link") - m_lowmass   # 12.4 = 프레임·데크·포드 구조
    parts = [
        Part(m_struct * v / total_v, c, box_inertia(m_struct * v / total_v, *size))
        for (size, c), v in zip(boxes, vols)
    ]
    # 배터리·전장함을 포드 안 낮게(좌우 대칭 → COM 중앙·낮게). robot_body 배치와 일치.
    batt_h = P.battery_size * 1.2
    batt_z = (pod_cz - pod_h / 2) + batt_h / 2 + 0.02
    batt_x = (P.deck_length - 2 * P.body_inset) / 2 - (P.battery_size * 1.6) / 2 - 0.05
    for sy in (-1, +1):
        parts.append(Part(4.8, (batt_x, sy * half_track, batt_z),
                          box_inertia(4.8, P.battery_size * 1.6, P.battery_size, batt_h)))
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
    out.append(camera_sensor_gazebo(o))
    out.append(imu_sensor_gazebo())
    out.append("</robot>")
    return "\n".join(out)


def imu_sensor_gazebo() -> str:
    """base_link 에 IMU (기울기·각속도·가속도). 넘어짐 감지 + 위치추정 보정용.

    Stage 4(주행/EKF)에서 쓴다. 월드에 ignition-gazebo-imu-system 플러그인이 있어야
    실제로 발행된다 — 주행 월드에 붙일 때 같이 넣는다.
    """
    # MEMS(BNO085 급) 노이즈 — 각속도·가속도 가우시안 + 바이어스. 노이즈 0 이면 실제보다
    # 낙관적이라 EKF·heading 이 실물보다 쉬워진다. (지자기 교란은 시뮬 밖 — 원장 §7)
    # IMU 의 <noise> 는 type 을 속성으로 받는다 (카메라 노이즈는 <type> 자식 — 스키마가 다름).
    n_gyro = '<noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev><bias_mean>7.5e-6</bias_mean><bias_stddev>8e-7</bias_stddev></noise>'
    n_acc = '<noise type="gaussian"><mean>0</mean><stddev>0.017</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev></noise>'
    return f"""  <gazebo reference="base_link">
    <sensor name="imu" type="imu">
      <topic>robot/imu</topic>
      <update_rate>100</update_rate>
      <always_on>1</always_on>
      <imu>
        <angular_velocity>
          <x>{n_gyro}</x><y>{n_gyro}</y><z>{n_gyro}</z>
        </angular_velocity>
        <linear_acceleration>
          <x>{n_acc}</x><y>{n_acc}</y><z>{n_acc}</z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
"""


def camera_sensor_gazebo(o: dict) -> str:
    """캐리지에 강체 고정된 하방 RGB + 깊이 카메라 (Intel RealSense D405, DESIGN.md).

    카메라가 캐리지에 붙어 같이 움직이므로 툴 팁이 항상 같은 픽셀에 온다 → 헤드리스
    픽셀 단언의 기반(DECISIONS 006). 전방 팔에 올려 두둑 위 ~0.33m 에서 내려다본다:
    카리지·툴에 안 가리고, LED 는 렌즈 둘레 링(자기 LED 를 안 봄).

    센서 위치는 robot_body 가 export 한 카메라 시각 박스의 월드좌표(camera_world)에서
    카리지 원점을 빼서 정확히 맞춘다 — 센서와 시각 카메라가 어긋나지 않는다.
    rpy=(0,π/2,0) → 광축 +X 를 -Z(정하방)로. 인트린식은 D405(1280×720, HFOV 87°).
    """
    cam, car = o.get("camera_world", o["carriage"]), o["carriage"]
    off = f"{cam[0]-car[0]:.4f} {cam[1]-car[1]:.4f} {cam[2]-car[2]:.4f}"
    HFOV, W, H = 1.5184, 1280, 720   # D405: 87° / 1280×720
    return f"""  <gazebo reference="carriage">
    <sensor name="down_cam" type="camera">
      <pose>{off} 0 1.5708 0</pose>
      <topic>robot/camera</topic>
      <update_rate>15</update_rate>
      <always_on>1</always_on>
      <camera>
        <horizontal_fov>{HFOV}</horizontal_fov>
        <image><width>{W}</width><height>{H}</height><format>R8G8B8</format></image>
        <clip><near>0.02</near><far>10</far></clip>
        <save enabled="true"><path>artifacts/camera</path></save>
        <!-- 센서 노이즈. 0 이면 실제보다 낙관적이라(심-리얼 원장) 가우시안 픽셀 노이즈를 넣는다.
             실 옥외 조명·젖은 잎 반사는 여기 넘어 학습때 도메인 랜덤화로 보완. -->
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
      </camera>
    </sensor>
    <!-- 깊이 카메라 (RGB 와 같은 자리·방향). 높이를 비전으로 → "작물이 클리어런스 넘었나"
         를 오라클 없이(Aigen gen2 방식). 주의: 시뮬 깊이 노이즈는 근사(실 D405 는 거리의존·IR). -->
    <sensor name="down_depth" type="depth_camera">
      <pose>{off} 0 1.5708 0</pose>
      <topic>robot/depth</topic>
      <update_rate>15</update_rate>
      <always_on>1</always_on>
      <camera>
        <horizontal_fov>{HFOV}</horizontal_fov>
        <image><width>{W}</width><height>{H}</height></image>
        <clip><near>0.02</near><far>10</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.003</stddev></noise>
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
        # 도구 0.5kg(허브 스텝+막대) 중력 4.9N 이겨내려 P 상향 — 정상상태 오차 < 3mm
        ("tool_joint", "tool_cmd", 2000.0, 40.0, 55.0),
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
