#!/usr/bin/env python3
"""Stage 5 Tier 2 — 동적 요철 월드 생성. 로봇이 주행 중 실제로 흔들리게 한다.

Tier 1(정적 크로스슬로프)과 달리, 고랑 바퀴 경로에 절차적 흙덩이(시드 고정 범프)를 흩뿌려
주행하며 바퀴가 오르내려 pitch·roll·bounce 가 동적으로 생긴다. 하이트필드 아님(결정적 → 단언 가능,
DECISIONS 025 보정). imu-system 포함(주행 중 자세 추적). 카메라 없음 → GPU 불필요(Tier 2).

목적(make shake, Step A): (1) DART 가 범프에서 안 터지고 로봇이 안 넘어지고 완주하는가,
(2) 자세가 실제로 시변(shake)하는가, (3) IMU 가 그 시변 자세를 GT 대로 추적하는가.
셋 다 통과해야 주행 중 타격 보정 A/B(Step B)로 넘어간다.

각도가 아니라 요철 형상(높이·간격)이 파라미터. 시드 단일 출처.
생성: tools/make_shake_world.py > worlds/robot_shake.sdf   (Makefile 이 자동 생성)
"""
from __future__ import annotations

import random
import sys

# ── 요철 파라미터 (시드 단일 출처) ────────────────────────────────────────────
SEED = 7
Y_TRACKS = (-0.6, 0.6)      # 로봇 y=0 주행 시 좌우 바퀴 중심 (track=1.2)
X0, X1 = 0.5, 2.8           # 흙덩이 분포 x 범위 (로봇은 x=-0.3 서 출발 → 평지서 시작)
CLOD_DX = 0.18              # x 방향 평균 간격 (+ 지터)
CLOD_H = (0.03, 0.06)       # 흙덩이 높이 [m]. 바퀴 반경 0.11 대비 climbable(안 넘어지되 흔들림)
CLOD_FOOT = (0.08, 0.13)    # 발자국 (x,y) 크기 [m]
JIT_X, JIT_Y = 0.05, 0.07   # 위치 지터 (양 바퀴가 다른 위상으로 밟게 → roll 도 유발)


def clods():
    """[(cx, cy, h, fx, fy)] — 시드 고정 흙덩이. 양 바퀴 경로에 위상 어긋나게 흩뿌린다."""
    rng = random.Random(SEED)
    out = []
    for yt in Y_TRACKS:
        x = X0 + rng.uniform(0, CLOD_DX)   # 트랙마다 시작 위상 다르게 → 좌우 비동기 → roll
        while x < X1:
            out.append((x + rng.uniform(-JIT_X, JIT_X), yt + rng.uniform(-JIT_Y, JIT_Y),
                        rng.uniform(*CLOD_H), rng.uniform(*CLOD_FOOT), rng.uniform(*CLOD_FOOT)))
            x += CLOD_DX + rng.uniform(0, CLOD_DX)
    return out


def sdf() -> str:
    models = []
    for k, (cx, cy, h, fx, fy) in enumerate(clods()):
        models.append(
            f'''    <model name="clod_{k}"><static>true</static><pose>{cx:.3f} {cy:.3f} {h/2:.3f} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>{fx:.3f} {fy:.3f} {h:.3f}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><box><size>{fx:.3f} {fy:.3f} {h:.3f}</size></box></geometry>
          <material><ambient>0.28 0.2 0.12 1</ambient><diffuse>0.45 0.32 0.2 1</diffuse></material></visual></link></model>''')
    body = "\n".join(models)
    return f'''<?xml version="1.0" ?>
<!-- 생성물: tools/make_shake_world.py (Stage 5 Tier 2 동적 요철). 손으로 고치지 말 것. -->
<sdf version="1.9">
  <world name="robot_shake">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.4 0.4 -0.85</direction></light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.3 0.22 0.14 1</ambient><diffuse>0.5 0.37 0.24 1</diffuse></material></visual></link></model>
{body}
    <include><uri>model://weedwatch_robot</uri><name>weedwatch</name><pose>-0.3 0 0.03 0 0 0</pose></include>
  </world>
</sdf>
'''


if __name__ == "__main__":
    sys.stdout.write(sdf())
