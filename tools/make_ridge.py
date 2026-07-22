#!/usr/bin/env python3
"""사실적 두둑(사면 있는 사다리꼴) 메시를 절차적으로 생성한다 (Stage 4-3 Phase 4b).

지금까지 두둑은 옆면 수직 상자였다(robot_row/stamp). 진짜 두둑은 흙을 파 올려 만든 사다리꼴이라
옆이 비스듬하고, 그 양옆이 고랑(로봇 바퀴가 달리는 낮은 골)이다. 카메라는 위에서 보므로 두둑 옆면을
안 보지만, GUI 로 보면 "진짜 밭" 느낌이 산다.

단면(y-z): 아랫변 넓고 윗변(두둑 폭 bed_width) 좁은 사다리꼴. 높이 bed_height. x 방향으로 길게 압출.
윗면은 CropCraft 흙+식물이 덮으므로 살짝 아래(z=height-eps)에 둬 z-fighting 회피. 소일 갈색 재질.

치수는 garden_geometry(Garden) 단일 출처. 출력: models/ridge/{ridge.obj,ridge.mtl,model.{sdf,config}}.

실행:  ./scripts/env.sh python3 tools/make_ridge.py
"""
from __future__ import annotations

import sys
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from garden_geometry import Garden  # noqa: E402

G = Garden()
OUT = WW / "models" / "ridge"

LENGTH = 3.6          # x 방향 두둑 길이 (주행 거리 커버)
SIDE_RUN = 0.10       # 사면의 수평 뻗음 (아랫변이 윗변보다 이만큼 양옆 넓음). 고랑 폭(0.30) 안.
TOP_EPS = 0.002       # 윗면을 이만큼 낮춰 CropCraft 흙과 z-fighting 회피


def build_obj() -> str:
    cy = 0.60                     # 두둑 중심 y (로봇 straddle 중심)
    top_h = G.bed_width / 2       # 윗변 반폭 (0.45)
    base_h = top_h + SIDE_RUN     # 아랫변 반폭
    h = G.bed_height              # 두둑 높이 (0.25)
    x0, x1 = -0.30, LENGTH - 0.30
    zt = h - TOP_EPS

    # 단면 4점 (y,z): A 아랫左 · B 아랫右 · C 윗右 · D 윗左
    A = (cy - base_h, 0.0)
    B = (cy + base_h, 0.0)
    C = (cy + top_h, zt)
    D = (cy - top_h, zt)
    # x0 면(v1..4), x1 면(v5..8) — OBJ 는 1-index
    verts = []
    for (y, z) in (A, B, C, D):
        verts.append((x0, y, z))
    for (y, z) in (A, B, C, D):
        verts.append((x1, y, z))

    lines = ["# Blender", "mtllib ridge.mtl", "o ridge", "usemtl soil"]
    for x, y, z in verts:
        lines.append(f"v {x:.4f} {y:.4f} {z:.4f}")
    # 법선 대략 (윗면 +z, 사면 바깥). double_sided 로 안전.
    # 면(사각형, 바깥에서 CCW): 윗면 D C G H(4 3 7 8), 오른사면 B C G F(2 3 7 6),
    # 왼사면 A D H E(1 4 8 5), 아랫면 A B F E(1 2 6 5), 끝단 x0 A B C D, x1 E F G H
    faces = [
        (4, 3, 7, 8),   # 윗면 (식물 얹힘)
        (2, 3, 7, 6),   # 오른 사면
        (1, 4, 8, 5),   # 왼 사면
        (1, 2, 6, 5),   # 아랫면
        (1, 2, 3, 4),   # 끝단 x0
        (5, 6, 7, 8),   # 끝단 x1
    ]
    for f in faces:
        lines.append("f " + " ".join(str(i) for i in f))
    return "\n".join(lines) + "\n"


MTL = """# 두둑 흙 재질 (갈색 무광). 카메라는 윗면(CropCraft 흙)만 보고, 이 재질은 GUI 사면용.
newmtl soil
Ka 0.20 0.14 0.09
Kd 0.34 0.24 0.15
Ks 0.02 0.02 0.02
Ns 4.0
d 1.0
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>ridge</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>절차적 사다리꼴 두둑 (사면+고랑). tools/make_ridge.py 생성.</description>
</model>
"""

# 시각=사면 메시(예쁨), 충돌=상자(DART 안정 + 기존 straddle 물리와 동일 — 도구가 윗면 0.25 에서 멈춤).
# 표준 관행: 보는 건 정교하게, 부딪히는 건 단순하게. 상자 = (길이, bed_width, bed_height).
def model_sdf() -> str:
    cx = (-0.30 + (LENGTH - 0.30)) / 2   # 상자 중심 x
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="ridge">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><mesh><uri>model://ridge/ridge.obj</uri></mesh></geometry>
        <material><double_sided>true</double_sided></material>
      </visual>
      <collision name="c">
        <pose>{cx:.4f} 0.60 {G.bed_height/2:.4f} 0 0 0</pose>
        <geometry><box><size>{LENGTH:.3f} {G.bed_width:.3f} {G.bed_height:.3f}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "ridge.obj").write_text(build_obj())
    (OUT / "ridge.mtl").write_text(MTL)
    (OUT / "model.config").write_text(MODEL_CONFIG)
    (OUT / "model.sdf").write_text(model_sdf())
    print(f"생성: {OUT}/ (ridge.obj/mtl + model.sdf/config)")
    print(f"  두둑: 윗변 {G.bed_width*100:.0f}cm · 아랫변 {(G.bed_width+2*SIDE_RUN)*100:.0f}cm · "
          f"높이 {G.bed_height*100:.0f}cm · 길이 {LENGTH:.1f}m · 사면 {SIDE_RUN*100:.0f}cm")


if __name__ == "__main__":
    main()
