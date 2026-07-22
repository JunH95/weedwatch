#!/usr/bin/env python3
"""Stage 5 Tier 1 준정적 기울기 월드 생성 — 로봇이 크로스슬로프에서 실제로 기울게 한다.

DECISIONS 025: 지금 로봇은 평지(furrow_floor z=0)를 달려 절대 안 기운다 → IMU 는 유휴, 게다가
어떤 월드에도 imu-system 플러그인이 없어 **발행조차 안 됐다**(실측). 이 월드는 지면을 roll 방향으로
TILT_DEG 만큼 기울여(크로스슬로프) 로봇을 그만큼 기울이고, imu-system 을 얹어 IMU 를 살린다.

목적(선검증, make tilt): (1) DART 가 기운 접촉에서 안 터지는가, (2) IMU 가 그 기울기를 읽는가.
robot_drive.sdf 를 본떴다(카메라 없음 → 렌더 없음 → GPU 불필요, Tier 2 물리 시뮬).

기울기 각(TILT_DEG)의 단일 출처. assert_tilt.py 가 이 상수를 import 해서 목표로 삼는다 → 안 어긋남.
생성: tools/make_tilt_world.py > worlds/robot_tilt.sdf   (Makefile 이 자동 생성)
"""
from __future__ import annotations

import math
import sys

# ── 크로스슬로프 각 (roll). 단일 출처 ─────────────────────────────────────────
# 5° 는 현실적 밭 요철 범위이고, 보정이 필요할 만큼 크다: 두둑 위 0.33m 카메라 직하점이
# 0.33·tan5° ≈ 2.9cm 밀린다 → 점타격 허용오차 2cm 초과. (Stage 5 A/B 의 기반 각.)
TILT_DEG = 5.0
ROLL = math.radians(TILT_DEG)

# 스탬핑 A/B(make tilt-stamp) 전용 각. 무보정 오차가 허용오차 2cm 를 확실히 넘도록 크게:
# 도구가 기운 축으로 하강해 tanφ·(두둑깊이 ~0.24m) 만큼 옆으로 밀린다. 8° → ~3.4cm 미스(>2cm),
# 보정하면 <2cm 히트로 뚜렷이 갈린다(문턱 근처 flaky 방지).
STAMP_TILT_DEG = 8.0
STAMP_ROLL = math.radians(STAMP_TILT_DEG)


def sdf() -> str:
    return f'''<?xml version="1.0" ?>
<!-- 생성물: tools/make_tilt_world.py (Stage 5 Tier 1 준정적 기울기). 손으로 고치지 말 것. -->
<sdf version="1.9">
  <world name="robot_tilt">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <!-- IMU 를 살리는 플러그인. 이게 없으면 URDF 에 imu 센서가 있어도 /robot/imu 가 안 뜬다(실측). -->
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.4 -0.85</direction>
    </light>

    <!-- 크로스슬로프 지면: roll {TILT_DEG}° 로 기운 평면. 좌우 바퀴(y=±0.60)가 서로 다른 높이를
         딛어 로봇이 그만큼 roll 한다. 마찰 mu=1.0 >> tan{TILT_DEG}°({math.tan(ROLL):.3f}) → 안 미끄러짐. -->
    <model name="ground">
      <static>true</static>
      <pose>0 0 0 {ROLL:.6f} 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.3 0.22 0.14 1</ambient><diffuse>0.5 0.37 0.24 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 로봇: 기운 지면 위. 프레임 z=0 이 바퀴 바닥이라, 높은 쪽 바퀴 초기 관통을 피해 살짝 띄워 스폰. -->
    <include>
      <uri>model://weedwatch_robot</uri>
      <name>weedwatch</name>
      <pose>0 0 0.12 0 0 0</pose>
    </include>
  </world>
</sdf>
'''


def stamp_sdf() -> str:
    """스탬핑 A/B 월드: 로봇이 STAMP_TILT_DEG 기운 채 두둑 위 잡초를 찍는다(make tilt-stamp).

    핵심 모델: 고랑 바닥을 y=0.6(로봇 중심) 축으로 roll 시켜 로봇을 굴린다. 두둑 윗면은 **수평**으로
    둬(box top z=0.25) 잡초 높이가 균일 → 스코어링이 깨끗. (현실 크로스슬로프에서 두둑 윗면을 고른
    상태 근사. 두둑까지 함께 기울이는 건 나중 사실성 강화.) 도구는 기운 몸통 -z 로 하강해 수평
    두둑 윗면에 닿고, 무보정이면 옆으로 밀린다. imu-system 으로 IMU 자세를 제어에 쓴다.
    """
    return f'''<?xml version="1.0" ?>
<!-- 생성물: tools/make_tilt_world.py stamp (Stage 5 기울기 스탬핑 A/B). 손으로 고치지 말 것. -->
<sdf version="1.9">
  <world name="robot_tilt_stamp">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.4 -0.85</direction>
    </light>

    <!-- 크로스슬로프 고랑 바닥: y=0.6 축으로 roll {STAMP_TILT_DEG}°. 바퀴(y=0.0 / 1.2)가 ∓0.6·tan
         만큼 높이차 → 로봇이 그만큼 roll. mu=1.0 >> tan{STAMP_TILT_DEG}°({math.tan(STAMP_ROLL):.3f}). -->
    <model name="furrow_floor">
      <static>true</static>
      <pose>0 0.6 0 {STAMP_ROLL:.6f} 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.3 0.22 0.14 1</ambient><diffuse>0.5 0.37 0.24 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 두둑: 윗면 수평 z=0.25 (robot_stamp.sdf 와 동일). 도구가 여기서 멈춘다. -->
    <model name="bed">
      <static>true</static>
      <pose>0 0.600 0.125 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>4.00 0.90 0.25</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>4.00 0.90 0.25</size></box></geometry>
          <material><ambient>0.25 0.17 0.10 1</ambient><diffuse>0.42 0.30 0.18 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 잡초 마커 3개: 밴드 중심(월드 y=0.30/0.60/0.90), 담당 툴 X 선. 시각 전용. assert_tilt_stamp.WEEDS 와 일치. -->
    <model name="weed_0"><static>true</static><pose>-0.09 0.30 0.28 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><cylinder><radius>0.015</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.1 0.5 0.1 1</ambient><diffuse>0.15 0.7 0.15 1</diffuse></material></visual></link></model>
    <model name="weed_1"><static>true</static><pose>-0.27 0.60 0.28 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><cylinder><radius>0.015</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.1 0.5 0.1 1</ambient><diffuse>0.15 0.7 0.15 1</diffuse></material></visual></link></model>
    <model name="weed_2"><static>true</static><pose>-0.45 0.90 0.28 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><cylinder><radius>0.015</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.1 0.5 0.1 1</ambient><diffuse>0.15 0.7 0.15 1</diffuse></material></visual></link></model>

    <!-- 로봇: 두둑 걸터타고 기운 바닥 위. 높은 쪽 바퀴 초기 관통 피해 살짝 띄워 스폰. -->
    <include>
      <uri>model://weedwatch_robot</uri>
      <name>weedwatch</name>
      <pose>0 0.600 0.12 0 0 0</pose>
    </include>
  </world>
</sdf>
'''


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "plain"
    sys.stdout.write(stamp_sdf() if which == "stamp" else sdf())
