#!/usr/bin/env python3
"""관통 P2 — 두둑 여러 줄 밭 월드 생성 (DECISIONS 036).

지금 robot_field 는 두둑 1줄이다. 커버리지(여러 줄 자율 주행)를 보려면 여러 줄이 필요하다.
기존 model://ridge(사면 두둑)·model://garden_field(CropCraft 식물)를 **두둑 중심마다 배치**해
재사용한다 — 메시 재생성 없이 pose 만 옮긴다. 카메라 렌더 필요라 sensors-system 포함.

── 좌표 (단일 두둑 robot_field 와 정합) ────────────────────────────────────────
  두둑 i 중심 y = FIRST_BED_Y + i·pitch  (pitch = 두둑폭+고랑폭 = 1.2m). bed0 = 0.6 (기존과 동일).
  ridge 모델은 내부 y중심 0.6 → pose y=(center−0.6) 로 옮긴다.
  garden 모델은 기존 오프셋(0, 0.17, 0.25) 에 같은 (center−0.6) 를 더한다.
  robot 은 bed0(y=0.6) 걸터타고 시작. 커버리지 하네스가 두둑 사이를 옮긴다.

스켈레톤은 속도를 위해 기본 2줄(카메라 렌더+best.pt 가 GPU 경합해 느림 — 036). n 은 인자로.
생성: tools/make_field_world.py [n_beds] > worlds/robot_field_multi.sdf
"""
from __future__ import annotations

import sys

FIRST_BED_Y = 0.60
PITCH = 1.20              # 두둑폭 0.9 + 고랑폭 0.3 (garden_geometry.Portal.pitch)
GARDEN_OFF = (0.0, 0.17, 0.25)   # robot_field.sdf 의 garden include 오프셋


def bed_centers(n: int):
    return [FIRST_BED_Y + i * PITCH for i in range(n)]


def sdf(n_beds: int = 2) -> str:
    includes = []
    for i, cy in enumerate(bed_centers(n_beds)):
        dy = cy - 0.60                          # ridge 내부중심 0.6 을 cy 로
        includes.append(f'''    <include>
      <uri>model://ridge</uri><name>ridge_{i}</name>
      <pose>0 {dy:.3f} 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://garden_field</uri><name>garden_{i}</name>
      <pose>{GARDEN_OFF[0]:.3f} {GARDEN_OFF[1]+dy:.3f} {GARDEN_OFF[2]:.3f} 0 0 0</pose>
    </include>''')
    body = "\n".join(includes)
    return f'''<?xml version="1.0" ?>
<!-- 생성물: tools/make_field_world.py (관통 P2, 두둑 {n_beds}줄). 손대지 말 것. -->
<sdf version="1.9">
  <world name="robot_field_multi">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.8 0.9 1</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.4 0.4 -0.85</direction>
    </light>

    <!-- 고랑 바닥 (평지 z=0). 스켈레톤은 흙덩이 없이 매끈하게 — 끼임(034)은 관통 뒤 근사 수리 (036). -->
    <model name="furrow_floor"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.20 0.15 0.10 1</ambient><diffuse>0.30 0.22 0.15 1</diffuse></material></visual></link></model>
{body}

    <include><uri>model://weedwatch_robot</uri><name>weedwatch</name><pose>0 0.600 0.05 0 0 0</pose></include>
  </world>
</sdf>
'''


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    sys.stdout.write(sdf(n))
