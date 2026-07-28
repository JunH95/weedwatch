#!/usr/bin/env python3
"""밭 월드 생성 — 명세(field_spec)대로 밭 하나를 통째로 짓는다 (DECISIONS 042).

기존 `make_field_world.py` 는 균일 두둑을 pitch 간격으로 늘어놓기만 했다. 여기서는 명세가 말하는
**현실성**까지 짓는다: 굽은 두둑(make_ridge_varied), 고랑 흙덩이, 가로경사, 그리고 창고.

  ./scripts/env.sh python3 tools/make_field.py dev   > worlds/field_dev.sdf
  ./scripts/env.sh python3 tools/make_field.py main  > worlds/field_main.sdf

기존 월드(robot_field_multi)는 **건드리지 않는다** — 지금 통과 중인 게이트들의 기준선이라,
새 밭에서 무엇이 깨지는지 비교하려면 옛 밭이 그대로 남아 있어야 한다.

── 왜 흙덩이가 상자인가 ────────────────────────────────────────────────────
연속 지형(하이트필드)이 이상적이지만 DART 안정성이 확인 안 됐고(030 부채), 상자는 shake 월드에서
"안 터지고 흔들린다"가 실측됐다. 형상을 단순화한 대가는 마찰 파라미터로 갚는다(032, mu 0.4).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from field_spec import FieldSpec, get  # noqa: E402
from garden_geometry import Garden  # noqa: E402
from make_ridge_varied import generate as gen_ridge, model_name  # noqa: E402

G = Garden()
GARDEN_OFF = (0.0, 0.17, 0.25)      # CropCraft 정원 include 오프셋 (robot_field 와 동일)


def clod_models(spec: FieldSpec) -> str:
    """고랑 흙덩이 — 바퀴 경로에 놓인 낮은 상자. 로봇을 흔들어 타격 정밀도를 시험한다."""
    out = []
    for bed in range(spec.n_beds):
        for i, (x, y, h) in enumerate(spec.clods(bed)):
            out.append(f"""    <model name="clod_{bed}_{i}"><static>true</static><link name="l">
      <collision name="c"><geometry><box><size>0.12 0.12 {h:.3f}</size></box></geometry>
        <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><box><size>0.12 0.12 {h:.3f}</size></box></geometry>
        <material><ambient>0.22 0.16 0.11 1</ambient><diffuse>0.30 0.22 0.15 1</diffuse></material></visual>
      <pose>{x:.3f} {y:.3f} {h/2:.3f} 0 0 0</pose></link></model>""")
    return "\n".join(out)


def home_base(spec: FieldSpec) -> str:
    """창고(홈 베이스) — 보관 + 도킹 + 에너지 저장 (DECISIONS 039).

    지금은 **형상만**이다: 로봇이 들어갈 크기의 빈 상자(뒤·옆 벽 + 지붕)와 정면 바닥의 도킹 마커.
    충전·에너지는 배터리 단계에서 붙인다. 밭 끝 헤드랜드 너머에 둬서 "창고에서 나와 밭으로 간다"가
    실제 이동이 되게 한다.
    """
    if not spec.home_base:
        return ""
    w, d, h, t = 1.6, 2.0, 1.0, 0.06          # 폭·깊이·높이·벽 두께 [m]
    x = spec.x1 + spec.headland + d / 2       # 헤드랜드 너머
    y = spec.bed_centers[0]
    parts = [
        (x + d / 2, y, h / 2, t, w, h),                       # 뒷벽
        (x, y - w / 2, h / 2, d, t, h),                       # 왼벽
        (x, y + w / 2, h / 2, d, t, h),                       # 오른벽
        (x, y, h, d, w, t),                                   # 지붕
    ]
    body = "\n".join(
        f"""      <collision name="c{i}"><pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
      <visual name="v{i}"><pose>{px:.3f} {py:.3f} {pz:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>0.35 0.33 0.30 1</ambient><diffuse>0.55 0.52 0.48 1</diffuse></material></visual>"""
        for i, (px, py, pz, sx, sy, sz) in enumerate(parts))
    # 도킹 마커: 창고 입구 바닥의 밝은 사각 — 나중에 카메라가 볼 표적(039 도킹)
    marker = f"""      <visual name="dock_marker">
        <pose>{x - d / 2 + 0.15:.3f} {y:.3f} 0.005 0 0 0</pose>
        <geometry><box><size>0.30 0.30 0.01</size></box></geometry>
        <material><ambient>0.9 0.9 0.85 1</ambient><diffuse>1 1 0.95 1</diffuse></material></visual>"""
    return f"""    <model name="home_base"><static>true</static><link name="link">
{body}
{marker}
    </link></model>"""


def sdf(spec: FieldSpec) -> str:
    for bed in range(spec.n_beds):
        gen_ridge(spec, bed)                   # 두둑 메시·충돌을 먼저 굽는다

    includes = []
    for bed, cy in enumerate(spec.bed_centers):
        includes.append(f"""    <include>
      <uri>model://{model_name(spec, bed)}</uri><name>ridge_{bed}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://garden_field</uri><name>garden_{bed}</name>
      <pose>{GARDEN_OFF[0]:.3f} {GARDEN_OFF[1] + cy - 0.60:.3f} {GARDEN_OFF[2]:.3f} 0 0 0</pose>
    </include>""")

    roll = math.radians(spec.cross_slope_deg)   # 밭 전체 가로경사 — 지면을 기울인다
    spawn_x = spec.x0 + 0.30
    return f'''<?xml version="1.0" ?>
<!-- 생성물: tools/make_field.py {spec.name} — 두둑 {spec.n_beds}줄 × {spec.bed_length}m
     현실성: 폭±{spec.width_var*100:.0f} 높이±{spec.height_var*100:.0f} 사행±{spec.meander*100:.0f}cm ·
     흙덩이 {spec.clod_density}/m · 경사 {spec.cross_slope_deg}° · 창고 {"있음" if spec.home_base else "없음"}
     손대지 말 것 — 명세는 tools/field_spec.py 에 있다. -->
<sdf version="1.9">
  <world name="field_{spec.name}">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.8 0.9 1</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.4 0.4 -0.85</direction>
    </light>

    <!-- 고랑 바닥. 가로경사 {spec.cross_slope_deg}° 는 지면을 roll 로 기울여 만든다(tilt 월드와 같은 방식). -->
    <model name="furrow_floor"><static>true</static><link name="link">
      <pose>0 0 0 {roll:.5f} 0 0</pose>
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        <material><ambient>0.20 0.15 0.10 1</ambient><diffuse>0.30 0.22 0.15 1</diffuse></material></visual></link></model>

{chr(10).join(includes)}

{clod_models(spec)}

{home_base(spec)}

    <include><uri>model://weedwatch_robot</uri><name>weedwatch</name>
      <pose>{spawn_x:.3f} {spec.bed_centers[0]:.3f} 0.05 0 0 0</pose></include>
  </world>
</sdf>
'''


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "dev"
    spec = get(name)
    sys.stdout.write(sdf(spec))
    print(f"# {spec.name}: 두둑 {spec.n_beds}×{spec.bed_length}m ({spec.area_m2:.1f}m²) · "
          f"흙덩이 {sum(len(spec.clods(b)) for b in range(spec.n_beds))}개 · "
          f"창고 {'있음' if spec.home_base else '없음'}", file=sys.stderr)


if __name__ == "__main__":
    main()
