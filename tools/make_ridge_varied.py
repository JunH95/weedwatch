#!/usr/bin/env python3
"""불균일 두둑 생성 — 명세(field_spec)대로 굽고 두꺼워졌다 얇아지는 두둑 (DECISIONS 042).

기존 `make_ridge.py` 는 사다리꼴 단면을 x 방향으로 **그대로 밀어낸** 완전 균일 두둑을 만든다.
실제 배토기로 만든 두둑은 폭도 높이도 중심선도 흔들린다. 여기서는 x 를 잘게 나눠 각 위치의
단면을 `FieldSpec.profile()` 로 얻어 **띠(loft)** 로 잇는다.

── 시각과 충돌 ──────────────────────────────────────────────────────────────
  시각: 변이가 그대로 보이는 loft 메시 (사람이 GUI 로 "진짜 밭"인지 판단할 수 있어야 한다)
  충돌: **분절 상자 체인** — 단면마다 상자 하나. 메시 충돌은 DART 가 불안정하고(002·030),
        상자는 안정적이면서 폭·높이·중심 변이를 그대로 담는다. 도구가 멈추는 윗면 높이도
        위치마다 달라진다 — 그게 이 작업의 요점이다(균일 밭에선 타격 깊이가 늘 같았다).

실행:  ./scripts/env.sh python3 tools/make_ridge_varied.py <밭이름> <두둑번호>
       (월드 생성기 make_field_world 가 호출한다)
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from field_spec import FieldSpec, get  # noqa: E402
from garden_geometry import Garden  # noqa: E402

G = Garden()
SIDE_RUN = 0.10        # 사면의 수평 뻗음 (아랫변이 윗변보다 이만큼 양옆 넓음)
TOP_EPS = 0.002        # 윗면을 이만큼 낮춰 CropCraft 흙과 z-fighting 회피
TILE = 1.0             # 소일 텍스처 반복 간격 [m]
SEG = 0.25             # 단면 간격 [m] — 25cm 면 사행 파장(2.5m)을 10점으로 표현한다
SOIL_SRC = WW / "models" / "oracle_test" / "materials" / "dry_mud_field_001_diff_2k.jpg"


def sections(spec: FieldSpec, bed: int):
    """[(x, 중심 y, 윗변 반폭, 아랫변 반폭, 높이)] — 단면 목록."""
    n = max(2, int(spec.bed_length / SEG) + 1)
    out = []
    for i in range(n):
        x = spec.x0 + spec.bed_length * i / (n - 1)
        cy, w, h = spec.profile(bed, x)
        out.append((x, cy, w / 2, w / 2 + SIDE_RUN, h))
    return out


def build_obj(spec: FieldSpec, bed: int) -> str:
    """단면들을 띠로 이어 OBJ. 첫 줄 '# Blender' 필수(Fortress 로더 분기, CLAUDE.md)."""
    secs = sections(spec, bed)
    verts, rings = [], []
    for x, cy, top_h, base_h, h in secs:
        zt = h - TOP_EPS
        ring = [(x, cy - base_h, 0.0), (x, cy + base_h, 0.0),
                (x, cy + top_h, zt), (x, cy - top_h, zt)]
        rings.append([len(verts) + k + 1 for k in range(4)])   # OBJ 1-index
        verts += ring

    def sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    quads = []
    for r0, r1 in zip(rings, rings[1:]):                       # 옆면·윗면 띠
        for k in range(4):
            a, b = r0[k], r0[(k + 1) % 4]
            c, d = r1[(k + 1) % 4], r1[k]
            quads.append((a, b, c, d))
    quads.append(tuple(reversed(rings[0])))                    # 양 끝 마개
    quads.append(tuple(rings[-1]))

    cxm = sum(v[0] for v in verts) / len(verts)
    cym = sum(v[1] for v in verts) / len(verts)
    ctr = (cxm, cym, G.bed_height / 2)

    lines = ["# Blender", "mtllib ridge.mtl", f"o ridge_{bed}", "usemtl soil"]
    for x, y, z in verts:
        lines.append(f"v {x:.4f} {y:.4f} {z:.4f}")
    for x, y, z in verts:
        lines.append(f"vt {x / TILE:.4f} {y / TILE:.4f}")
    faces = []
    for fi, q in enumerate(quads, start=1):
        p = [verts[i - 1] for i in q]
        n = cross(sub(p[1], p[0]), sub(p[2], p[0]))
        fc = tuple(sum(v[k] for v in p) / 4 for k in range(3))
        if sum(n[k] * (fc[k] - ctr[k]) for k in range(3)) < 0:      # 법선을 바깥으로
            n = (-n[0], -n[1], -n[2])
        m = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1.0
        lines.append(f"vn {n[0] / m:.4f} {n[1] / m:.4f} {n[2] / m:.4f}")
        a, b, c, d = q
        for tri in ((a, b, c), (a, c, d)):                          # quad → 삼각형 2개 (Ogre 요구)
            faces.append("f " + " ".join(f"{i}/{i}/{fi}" for i in tri))
    return "\n".join(lines + faces) + "\n"


def collisions(spec: FieldSpec, bed: int) -> str:
    """분절 상자 체인 — 단면 사이를 상자 하나로. 폭·높이·중심 변이가 물리에 그대로 들어간다."""
    secs = sections(spec, bed)
    out = []
    for i, (s0, s1) in enumerate(zip(secs, secs[1:])):
        x = (s0[0] + s1[0]) / 2
        cy = (s0[1] + s1[1]) / 2
        w = (s0[2] + s1[2])                    # 윗변 반폭 평균 × 2
        h = (s0[4] + s1[4]) / 2
        dx = s1[0] - s0[0]
        out.append(f"""      <collision name="c{i}">
        <pose>{x:.4f} {cy:.4f} {h/2:.4f} 0 0 0</pose>
        <geometry><box><size>{dx:.4f} {w:.4f} {h:.4f}</size></box></geometry>
      </collision>""")
    return "\n".join(out)


def model_sdf(spec: FieldSpec, bed: int) -> str:
    return f"""<?xml version="1.0"?>
<!-- 생성물: tools/make_ridge_varied.py ({spec.name} 밭, 두둑 {bed}). 손대지 말 것. -->
<sdf version="1.9">
  <model name="ridge_{bed}">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><mesh><uri>model://{model_name(spec, bed)}/ridge.obj</uri></mesh></geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>
          <pbr><metal>
            <albedo_map>model://{model_name(spec, bed)}/soil.jpg</albedo_map>
            <metalness>0.0</metalness><roughness>0.9</roughness>
          </metal></pbr>
          <double_sided>true</double_sided>
        </material>
      </visual>
{collisions(spec, bed)}
    </link>
  </model>
</sdf>
"""


def model_name(spec: FieldSpec, bed: int) -> str:
    return f"ridge_{spec.name}_{bed}"


CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>불균일 두둑 ({field} 밭 {bed}번). tools/make_ridge_varied.py 생성.</description>
</model>
"""


def generate(spec: FieldSpec, bed: int) -> Path:
    out = WW / "models" / model_name(spec, bed)
    out.mkdir(parents=True, exist_ok=True)
    (out / "ridge.obj").write_text(build_obj(spec, bed))
    (out / "ridge.mtl").write_text("newmtl soil\nmap_Kd soil.jpg\nKd 0.34 0.24 0.15\n")
    (out / "model.config").write_text(CONFIG.format(name=model_name(spec, bed),
                                                    field=spec.name, bed=bed))
    (out / "model.sdf").write_text(model_sdf(spec, bed))
    if SOIL_SRC.exists():
        shutil.copy(SOIL_SRC, out / "soil.jpg")
    return out


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "dev"
    spec = get(name)
    for bed in range(spec.n_beds):
        out = generate(spec, bed)
        secs = sections(spec, bed)
        ws = [s[2] * 2 for s in secs]
        hs = [s[4] for s in secs]
        cys = [s[1] for s in secs]
        print(f"{out.name}: 단면 {len(secs)}개 · 폭 {min(ws)*100:.0f}~{max(ws)*100:.0f}cm · "
              f"높이 {min(hs)*100:.0f}~{max(hs)*100:.0f}cm · "
              f"중심 {min(cys):.3f}~{max(cys):.3f}m")


if __name__ == "__main__":
    main()
