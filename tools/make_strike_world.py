#!/usr/bin/env python3
"""Stage 5 Step B — 경사 + 흙덩이 위에서 타격하는 월드 생성.

왜 둘을 합치나 (측정 근거): 흙덩이만 있으면 자세가 ±2.9° 라 무보정 타격 오차가 ~1.2cm 로 허용오차
2cm 안에 들어와 **A/B 가 안 갈린다**(make shake 실측). 크로스슬로프(정적 기울기)를 얹어야 무보정이
확실히 빗나가고 보정의 값어치가 드러난다. 현실적이기도 하다 — 경사진 밭에 흙덩이가 있는 게 보통이다.

잡초·작물은 **개별 모델**이다(CropCraft 통메시와 달리). 그래야 명중 시 잡초를 지우고 표적 링을
남겨 사람이 눈으로 오차를 볼 수 있다(사용자 요청).

색 규약 (중요 — 헷갈리면 눈으로 검증이 안 된다):
  잡초 = 빨강 · 작물 = 초록  (robot_row 관례, 인식 오버레이와도 같음)
  표적 링 = 노랑(잡초가 원래 있던 자리) · 타격 자국 = 파랑(명중) / 주황(빗나감)
  자국 색을 초록/빨강으로 하면 작물·잡초와 섞여 아무것도 못 읽는다 — 실제로 그 실수를 했다.

경사 축 (DECISIONS 033): SLOPE_AXIS 로 축을 고른다.
  pitch(종단, 두둑 따라 오르내림) = **실제 밭 시나리오**. 슬립 1.2%, odom 멀쩡.
  roll(크로스슬로프, 두둑 가로지름) = **스트레스/끼임 테스트**. 걸터타면 두둑에 끼여 56% — 실제 밭은
    이렇게 안 달린다(두둑이 경사 방향으로 남). 인위적임을 알고 쓰는 것.
기본은 roll(끼임 테스트 유지). 현실 주행 성능은 SLOPE_AXIS=pitch.

생성: tools/make_strike_world.py > worlds/robot_strike.sdf   (Makefile 이 자동 생성)
"""
from __future__ import annotations

import math
import os
import random
import sys

TILT_DEG = float(os.environ.get('STRIKE_TILT_DEG', '6.0'))  # 크로스슬로프 각. 데모는 0(평지+흙덩이=현실 동적 흔들림)
ROLL = math.radians(TILT_DEG)
# 경사 축: roll=크로스슬로프(옆으로 기욺, 두둑 가로지름) / pitch=종단(오르내림, 두둑 따라감).
# 실제 밭은 두둑이 경사 방향으로 나 로봇이 따라 달리므로 pitch 가 현실적. roll 은 끼임 시나리오.
SLOPE_AXIS = os.environ.get("SLOPE_AXIS", "roll")
_FLOOR_RPY = (f"0 0.6 0 {ROLL:.6f} 0 0" if SLOPE_AXIS == "roll"
              else f"0 0 0 0 {-ROLL:.6f} 0")   # pitch: x 앞으로 갈수록 올라가게(-pitch)
ROBOT_Y = 0.600                 # 로봇(=두둑) 중심 y
SEED = 11
Y_TRACKS = (ROBOT_Y - 0.6, ROBOT_Y + 0.6)   # 좌우 바퀴 경로
X0, X1 = 0.35, 2.6
CLOD_DX = 0.18
CLOD_H = (0.03, 0.06)
CLOD_FOOT = (0.08, 0.13)
JIT_X, JIT_Y = 0.05, 0.07

# 잡초·작물 (두둑 폭 0.9 = y 0.15~1.05 안). 잡초는 툴 3밴드에 흩어지게.
WEEDS = [(0.70, 0.45), (1.00, 0.75), (1.35, 0.55), (1.70, 0.40), (2.05, 0.72)]
CROPS = [(0.88, 0.60), (1.50, 0.62), (1.90, 0.55)]


def clods():
    rng = random.Random(SEED)
    out = []
    for yt in Y_TRACKS:
        x = X0 + rng.uniform(0, CLOD_DX)
        while x < X1:
            out.append((x + rng.uniform(-JIT_X, JIT_X), yt + rng.uniform(-JIT_Y, JIT_Y),
                        rng.uniform(*CLOD_H), rng.uniform(*CLOD_FOOT), rng.uniform(*CLOD_FOOT)))
            x += CLOD_DX + rng.uniform(0, CLOD_DX)
    return out


def _cyl(name, x, y, z, r, ln, rgb, static=True):
    a = " ".join(f"{v:.2f}" for v in rgb)
    b = " ".join(f"{min(1.0, v * 1.4):.2f}" for v in rgb)
    return (f'    <model name="{name}"><static>{"true" if static else "false"}</static>'
            f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
            f'      <link name="l"><visual name="v"><geometry>'
            f'<cylinder><radius>{r}</radius><length>{ln}</length></cylinder></geometry>\n'
            f'        <material><ambient>{a} 1</ambient><diffuse>{b} 1</diffuse></material>'
            f'</visual></link></model>')


def _box(name, x, y, z, sx, sy, sz, rgb):
    a = " ".join(f"{v:.2f}" for v in rgb)
    b = " ".join(f"{min(1.0, v * 1.5):.2f}" for v in rgb)
    return (f'    <model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
            f'      <link name="l">'
            f'<collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
            f'<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>\n'
            f'        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
            f'<material><ambient>{a} 1</ambient><diffuse>{b} 1</diffuse></material></visual>'
            f'</link></model>')


_BED = "" if (os.environ.get("STRIKE_NO_BED") or os.environ.get("SLOPE_AXIS")=="pitch") else """    <!-- 두둑: 윗면 수평 z=0.25. 도구가 여기서 멈추고 잡초가 여기 선다. -->
    <model name="bed"><static>true</static><pose>0 %.3f 0.125 0 0 0</pose>
      <link name="link">
      <collision name="c"><geometry><box><size>4.00 0.90 0.25</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4.00 0.90 0.25</size></box></geometry>
        <material><ambient>0.25 0.17 0.10 1</ambient><diffuse>0.42 0.30 0.18 1</diffuse></material></visual></link></model>""" % ROBOT_Y


def sdf() -> str:
    parts = [_box(f"clod_{k}", cx, cy, h / 2, fx, fy, h, (0.28, 0.20, 0.12))
             for k, (cx, cy, h, fx, fy) in enumerate(clods())]
    parts += [_cyl(f"weed_{i}", x, y, 0.28, 0.015, 0.06, (0.55, 0.10, 0.10))
              for i, (x, y) in enumerate(WEEDS)]
    parts += [_cyl(f"crop_{i}", x, y, 0.30, 0.020, 0.10, (0.10, 0.50, 0.10))
              for i, (x, y) in enumerate(CROPS)]
    body = "\n".join(parts)
    BED = _BED
    SPAWN_Z = 0.6 * math.tan(ROLL) + 0.04   # 낮은쪽 바퀴가 기운 바닥에 닿게 (평지=0.04)
    return f'''<?xml version="1.0" ?>
<!-- 생성물: tools/make_strike_world.py (Stage 5 Step B: 경사 {TILT_DEG}° + 흙덩이 위 타격). 손대지 말 것. -->
<sdf version="1.9">
  <world name="robot_strike">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.4 0.4 -0.85</direction></light>

    <!-- 크로스슬로프 고랑 바닥: y={ROBOT_Y} 축으로 roll {TILT_DEG}°. 그 위에 흙덩이가 얹힌다. -->
    <model name="furrow_floor"><static>true</static><pose>{_FLOOR_RPY}</pose>
      <link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.30 0.22 0.14 1</ambient><diffuse>0.50 0.37 0.24 1</diffuse></material></visual></link></model>

{BED}
{body}

    <include><uri>model://weedwatch_robot</uri><name>weedwatch</name><pose>-0.3 {ROBOT_Y} {SPAWN_Z:.3f} 0 0 0</pose></include>
  </world>
</sdf>
'''


if __name__ == "__main__":
    sys.stdout.write(sdf())
