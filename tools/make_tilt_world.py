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


if __name__ == "__main__":
    sys.stdout.write(sdf())
