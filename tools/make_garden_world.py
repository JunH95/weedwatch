#!/usr/bin/env python3
"""주말농장 지형(두둑/고랑)을 SDF 월드로 생성한다.

── 왜 이걸 직접 만들어야 하는가 ──────────────────────────────────────────
CropCraft(정원 생성 도구)에는 **지형이라는 개념이 아예 없다.** 바닥이 완전한 평면이고,
CropCraft가 말하는 "bed"는 융기된 두둑이 아니라 그냥 "식물을 줄 세우는 논리적 묶음"이다.

그런데 우리 로봇은 포탈형이다 — 두둑 하나를 걸터타고 양쪽 고랑을 달린다 (두둑 위엔 안 올라간다).
두둑이 없으면 포탈형이라는 설계 자체가 성립하지 않는다.
그래서 지형은 우리가 만들고, CropCraft에서는 식물만 가져온다.

── 왜 상자(box)로 만드는가 ───────────────────────────────────────────────
진짜 두둑은 옆면이 비스듬하다. 하지만 지금 확인하려는 건 "예쁜가"가 아니라
"로봇이 탈 수 있는 기하학인가"다. 상자면 충분하고, Blender 없이 30분이면 답이 나온다.
예쁘게 만드는 건 그 답이 '예'인 다음이다.

── 치수 근거 (농촌진흥청 농사로) ─────────────────────────────────────────
  두둑 높이 20~30cm, 고랑 폭 30cm 내외, 평이랑 90~120cm
  상추 줄간격×포기간격 20×20cm  →  90cm 두둑에 4줄

사용법:
    tools/make_garden_world.py > worlds/garden_ridge.sdf
    tools/make_garden_world.py --beds 3 --bed-width 0.9 > worlds/garden_ridge.sdf
"""

from __future__ import annotations

import argparse
import sys

# ── 기본 치수 (미터) ──────────────────────────────────────────────────────
BED_WIDTH = 0.90  # 두둑 폭
BED_HEIGHT = 0.25  # 두둑 높이 (고랑 바닥 기준)
FURROW_WIDTH = 0.30  # 고랑 폭 — 로봇 바퀴가 여기 들어간다
BED_LENGTH = 4.00  # 두둑 길이
N_BEDS = 2

# 로봇이 실제로 탈 수 있는지 눈으로 보려고 같이 그리는 마네킹.
# 진짜 로봇(URDF)은 Stage 2에서 만든다. 지금은 "칫수가 맞나"만 본다.
WHEEL_DIA = 0.22
WHEEL_WIDTH = 0.08
CLEARANCE = 0.60  # 고랑 바닥 ~ 빔 아랫면
BEAM_HEIGHT = 0.08


def bed_pitch(bed_w: float, furrow_w: float) -> float:
    """두둑 하나 + 고랑 하나의 간격. 다음 두둑 중심까지의 거리."""
    return bed_w + furrow_w


def robot_track(bed_w: float, furrow_w: float) -> float:
    """좌우 바퀴 중심 사이 거리.

    바퀴는 각 고랑의 한가운데에 있어야 한다.
    두둑 가장자리가 ±(bed_w/2) 이고 고랑이 거기서 furrow_w 만큼 뻗으므로,
    고랑 중심은 bed_w/2 + furrow_w/2 이고 트랙은 그 두 배다.
    """
    return bed_w + furrow_w


def emit(beds: int, bed_w: float, bed_h: float, furrow_w: float, length: float,
         with_robot: bool) -> str:
    pitch = bed_pitch(bed_w, furrow_w)
    track = robot_track(bed_w, furrow_w)
    # 두둑들을 y=0 기준으로 좌우 대칭 배치
    centers = [(i - (beds - 1) / 2) * pitch for i in range(beds)]

    out: list[str] = []
    w = out.append

    w('<?xml version="1.0" ?>')
    w("<!--")
    w("  주말농장 지형 — tools/make_garden_world.py 가 생성함. 직접 고치지 말 것.")
    w("")
    w(f"  두둑 {beds}개 · 폭 {bed_w*100:.0f}cm · 높이 {bed_h*100:.0f}cm · 길이 {length:.1f}m")
    w(f"  고랑 폭 {furrow_w*100:.0f}cm · 두둑 간격(pitch) {pitch*100:.0f}cm")
    w(f"  → 포탈 로봇 트랙 {track*100:.0f}cm (바퀴가 고랑 한가운데)")
    w(f"  → 필요 클리어런스 {CLEARANCE*100:.0f}cm = 두둑 {bed_h*100:.0f} + 상추 25 + 여유 10")
    w("")
    w("  근거: 농촌진흥청 농사로 — 두둑 높이 20~30cm, 고랑 폭 30cm 내외, 평이랑 90~120cm")
    w("-->")
    w('<sdf version="1.9">')
    w('  <world name="garden">')
    w('    <physics name="1ms" type="ignored">')
    w("      <max_step_size>0.001</max_step_size>")
    w("      <real_time_factor>1.0</real_time_factor>")
    w("    </physics>")
    w('    <plugin filename="ignition-gazebo-physics-system"')
    w('            name="ignition::gazebo::systems::Physics"/>')
    w('    <plugin filename="ignition-gazebo-sensors-system"')
    w('            name="ignition::gazebo::systems::Sensors">')
    w("      <render_engine>ogre2</render_engine>")
    w("    </plugin>")
    w('    <plugin filename="ignition-gazebo-scene-broadcaster-system"')
    w('            name="ignition::gazebo::systems::SceneBroadcaster"/>')
    w("")
    w('    <light type="directional" name="sun">')
    w("      <cast_shadows>true</cast_shadows>")
    w("      <pose>0 0 10 0 0 0</pose>")
    w("      <diffuse>1 1 1 1</diffuse>")
    w("      <specular>0.3 0.3 0.3 1</specular>")
    w("      <direction>-0.4 0.5 -0.85</direction>")
    w("    </light>")
    w("")
    w("    <!-- 고랑 바닥 = 기준면 z=0. 로봇 바퀴가 굴러가는 곳. -->")
    w('    <model name="furrow_floor">')
    w("      <static>true</static>")
    w('      <link name="link">')
    w('        <collision name="collision">')
    w("          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>")
    w("        </collision>")
    w('        <visual name="visual">')
    w("          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>")
    w("          <material>")
    w("            <ambient>0.28 0.20 0.13 1</ambient>")
    w("            <diffuse>0.45 0.33 0.21 1</diffuse>")
    w("          </material>")
    w("        </visual>")
    w("      </link>")
    w("    </model>")
    w("")

    for i, cy in enumerate(centers):
        w(f"    <!-- 두둑 {i+1} · 중심 y={cy:+.2f}m · 윗면 z={bed_h:.2f}m -->")
        w(f'    <model name="bed_{i+1}">')
        w("      <static>true</static>")
        w(f"      <pose>0 {cy:.3f} {bed_h/2:.3f} 0 0 0</pose>")
        w('      <link name="link">')
        w('        <collision name="collision">')
        w(f"          <geometry><box><size>{length:.2f} {bed_w:.2f} {bed_h:.2f}</size></box></geometry>")
        w("        </collision>")
        w('        <visual name="visual">')
        w(f"          <geometry><box><size>{length:.2f} {bed_w:.2f} {bed_h:.2f}</size></box></geometry>")
        w("          <material>")
        w("            <ambient>0.32 0.23 0.15 1</ambient>")
        w("            <diffuse>0.52 0.38 0.25 1</diffuse>")
        w("          </material>")
        w("        </visual>")
        w("      </link>")
        w("    </model>")
        w("")

        # 상추 4줄 × 20cm 간격. 두둑 위에 실제로 얹히는지 확인용.
        rows = 4
        row_gap = 0.20
        y0 = cy - (rows - 1) * row_gap / 2
        n_along = int(length / row_gap) - 1
        for r in range(rows):
            for c in range(n_along):
                px = -length / 2 + row_gap * (c + 1)
                py = y0 + r * row_gap
                w(f'    <model name="crop_{i+1}_{r}_{c}">')
                w("      <static>true</static>")
                w(f"      <pose>{px:.3f} {py:.3f} {bed_h + 0.10:.3f} 0 0 0</pose>")
                w('      <link name="link"><visual name="visual">')
                w("        <geometry><sphere><radius>0.09</radius></sphere></geometry>")
                w("        <material><ambient>0.10 0.30 0.08 1</ambient>")
                w("                  <diffuse>0.25 0.68 0.20 1</diffuse></material>")
                w("      </visual></link>")
                w("    </model>")
        w("")

    if with_robot:
        beam_z = CLEARANCE + BEAM_HEIGHT / 2
        wheel_y = track / 2
        wheel_z = WHEEL_DIA / 2
        # 로봇은 **두둑 중심** 위에 선다. y=0 이 아니다 —
        # 두둑을 짝수 개 놓으면 y=0 은 고랑 한가운데라서, 거기 세우면
        # 바퀴가 두둑을 밟고 몸통이 고랑 위에 뜬다. 정확히 반대가 된다.
        # (실제로 이 버그를 냈고, 렌더링 사진을 보고서야 알았다.
        #  그래서 tests/test_garden_geometry.py 가 이걸 자동으로 검사한다.)
        robot_y = centers[0]
        w("    <!-- 포탈 로봇 마네킹. 진짜 로봇은 Stage 2(URDF)에서 만든다.")
        w("         지금 확인하려는 건 딱 하나: 바퀴가 고랑에 들어가고 몸체가 두둑 위를 지나가는가. -->")
        w('    <model name="portal_mockup">')
        w("      <static>true</static>")
        w(f"      <pose>0 {robot_y:.3f} 0 0 0 0</pose>")
        w('      <link name="link">')
        w("        <!-- 가로 빔 -->")
        w('        <visual name="beam">')
        w(f"          <pose>0 0 {beam_z:.3f} 0 0 0</pose>")
        w(f"          <geometry><box><size>0.14 {track + WHEEL_WIDTH:.2f} {BEAM_HEIGHT:.2f}</size></box></geometry>")
        w("          <material><ambient>0.05 0.25 0.30 1</ambient>")
        w("                    <diffuse>0.09 0.45 0.53 1</diffuse></material>")
        w("        </visual>")
        for sign, name in ((-1, "left"), (1, "right")):
            w(f"        <!-- {name} 다리 -->")
            w(f'        <visual name="leg_{name}">')
            w(f"          <pose>0 {sign*wheel_y:.3f} {(CLEARANCE + wheel_z)/2:.3f} 0 0 0</pose>")
            w(f"          <geometry><box><size>0.10 0.06 {CLEARANCE - wheel_z:.3f}</size></box></geometry>")
            w("          <material><ambient>0.05 0.25 0.30 1</ambient>")
            w("                    <diffuse>0.09 0.45 0.53 1</diffuse></material>")
            w("        </visual>")
            w(f'        <visual name="wheel_{name}">')
            w(f"          <pose>0 {sign*wheel_y:.3f} {wheel_z:.3f} {1.5708:.4f} 0 0</pose>")
            w(f"          <geometry><cylinder><radius>{WHEEL_DIA/2:.3f}</radius>"
              f"<length>{WHEEL_WIDTH:.2f}</length></cylinder></geometry>")
            w("          <material><ambient>0.06 0.06 0.07 1</ambient>")
            w("                    <diffuse>0.13 0.13 0.15 1</diffuse></material>")
            w("        </visual>")
        w("      </link>")
        w("    </model>")
        w("")

    # 밭 전체가 보이도록 비스듬히 내려다보는 카메라 (검증용, 로봇 카메라 아님)
    w("    <!-- 검증용 관찰 카메라. 로봇에 달린 카메라가 아니라, 사람(과 에이전트)이")
    w("         '이 기하학이 말이 되나'를 눈으로 보려고 두는 것이다. -->")
    w('    <model name="inspector">')
    w("      <static>true</static>")
    w("      <pose>-3.2 -2.6 1.9 0 0.42 0.68</pose>")
    w('      <link name="link">')
    w('        <sensor name="cam" type="camera">')
    w("          <topic>garden/inspect</topic>")
    w("          <update_rate>10</update_rate>")
    w("          <always_on>1</always_on>")
    w("          <camera>")
    w("            <horizontal_fov>1.047</horizontal_fov>")
    w("            <image><width>960</width><height>720</height><format>R8G8B8</format></image>")
    w("            <clip><near>0.05</near><far>60</far></clip>")
    w("            <save enabled=\"true\"><path>artifacts/garden</path></save>")
    w("          </camera>")
    w("        </sensor>")
    w("      </link>")
    w("    </model>")
    w("")
    w("  </world>")
    w("</sdf>")
    return "\n".join(out) + "\n"


def main() -> None:
    p = argparse.ArgumentParser(description="주말농장 두둑/고랑 지형 SDF 생성기")
    p.add_argument("--beds", type=int, default=N_BEDS, help="두둑 개수")
    p.add_argument("--bed-width", type=float, default=BED_WIDTH, help="두둑 폭 (m)")
    p.add_argument("--bed-height", type=float, default=BED_HEIGHT, help="두둑 높이 (m)")
    p.add_argument("--furrow-width", type=float, default=FURROW_WIDTH, help="고랑 폭 (m)")
    p.add_argument("--length", type=float, default=BED_LENGTH, help="두둑 길이 (m)")
    p.add_argument("--no-robot", action="store_true", help="로봇 마네킹 빼기")
    a = p.parse_args()

    # 설계가 자기모순이 아닌지 먼저 확인한다. 월드를 만든 뒤에 알면 늦다.
    track = robot_track(a.bed_width, a.furrow_width)
    slack = (a.furrow_width - WHEEL_WIDTH) / 2
    if slack <= 0:
        sys.exit(f"고랑({a.furrow_width*100:.0f}cm)이 바퀴({WHEEL_WIDTH*100:.0f}cm)보다 좁습니다.")
    print(
        f"두둑 {a.beds}개 · 폭 {a.bed_width*100:.0f} · 높이 {a.bed_height*100:.0f} · "
        f"고랑 {a.furrow_width*100:.0f}cm\n"
        f"→ 로봇 트랙 {track*100:.0f}cm · 전폭 {(track+WHEEL_WIDTH)*100:.0f}cm · "
        f"바퀴 좌우 여유 {slack*100:.1f}cm/쪽",
        file=sys.stderr,
    )
    sys.stdout.write(
        emit(a.beds, a.bed_width, a.bed_height, a.furrow_width, a.length, not a.no_robot)
    )


if __name__ == "__main__":
    main()
