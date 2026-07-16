#!/usr/bin/env python3
"""CropCraft가 내보낸 Gazebo 모델의 머티리얼을 Fortress가 알아듣는 형식으로 바꾼다.

── 안 하면 정원 전체가 새까맣게 렌더링된다 (실측함) ──────────────────────
CropCraft는 이런 걸 내보낸다:

    <material>
      <script>
        <uri>model://test1/materials/bed1.material</uri>
        <name>bed1</name>
      </script>
    </material>

이건 **Gazebo Classic** 시절의 Ogre1 머티리얼 스크립트 방식이다. 그런데 gz-sim(Fortress)은
`<script>` 를 통째로 무시한다. 무시만 하면 그나마 나은데, 문제는 그 다음이다:

  1. gz-sim 은 `<material>` 태그가 있으니 머티리얼을 읽으려 한다
  2. `<script>` 를 못 알아들으므로 아무것도 못 읽고 기본값(검정)을 만든다
  3. 그 검정을 **메시 자체의 머티리얼 위에 덮어쓴다**
  4. → 땅도, 두둑도, 식물도 전부 새까맣게 렌더링된다

실측 (2026-07-16): 하늘만 회색이고 나머지 전부 검정. 평균 밝기 33.4.
에러 메시지는 한 줄도 없다.

역설적이지만 `<material>` 태그를 **아예 안 쓰는 게** 이것보단 낫다. 그러면 메시 자체
머티리얼이 살아남으니까. 하지만 CropCraft는 `export_materials=False` 로 OBJ를 내보내서
.mtl 파일 자체가 없다. 그래서 우리가 SDF 쪽에 제대로 된 머티리얼을 넣어줘야 한다.

── 진짜 원인은 <script> 가 아니라 색의 기본값이었다 (직접 돌려서 알아냄) ──
sdformat 스펙을 보면:

    <element name="ambient" type="color" default="0 0 0 1">   ← 검정
    <element name="diffuse" type="color" default="0 0 0 1">   ← 검정

**SDF 에서 색의 기본값이 검정이다.** CropCraft 는 <material> 안에 <script> 만 넣고
색은 안 준다. gz-sim 은 script 를 못 알아듣지만 <material> 태그는 있으니 머티리얼을
만드는데, 이때 색이 기본값 검정이 되고 그게 메시 위에 덮인다.

그래서 <pbr> 로 바꾸는 것만으로는 **부족하다**. 텍스처를 줘도 검정 diffuse 가 곱해져서
여전히 검게 나온다 (실제로 이 실수를 했고, 평균 밝기 33.4 → 33.3 으로 안 바뀌었다).
색을 명시적으로 줘야 한다.

── 무엇으로 바꾸는가 ────────────────────────────────────────────────────
    <material>
      <ambient>0.5 0.5 0.5 1</ambient>   <!-- 없으면 검정이 기본값 -->
      <diffuse>1 1 1 1</diffuse>         <!-- 흰색 = 텍스처 색을 그대로 통과시킴 -->
      <specular>0.1 0.1 0.1 1</specular> <!-- 잎은 거의 반사 안 함 -->
      <pbr><metal>
        <albedo_map>model://test1/materials/maize_leaf.jpg</albedo_map>
        <metalness>0.0</metalness>      <!-- 잎은 금속이 아니다 -->
        <roughness>0.85</roughness>     <!-- 잎은 거칠다 (반들거리면 안 됨) -->
      </metal></pbr>
      <double_sided>true</double_sided>
    </material>

Ogre1 스크립트에 있던 두 줄이 왜 중요한지:
  cull_hardware none          잎을 뒤에서도 보이게 한다. 없으면 잎이 한쪽에서만 보이고
                              반대편에서는 사라진다 → 카메라 각도에 따라 식물이 없어진다.
  alpha_rejection greater 128 텍스처의 투명한 부분을 뚫는다. 없으면 잎이 네모난 판때기가 된다.

gz-sim 에서는 albedo_map 을 주면 알파 테스트가 자동으로 켜지고, double_sided 로 양면을 켠다.

사용법:
    tools/fix_materials.py models/test1/model.sdf
    tools/fix_materials.py models/test1/model.sdf --check   # 고칠 게 있는지만 확인
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# Ogre1 .material 파일에서 텍스처 파일명을 뽑는다.
#   texture_unit { texture maize_leaf.jpg }
TEXTURE_RE = re.compile(r"^\s*texture\s+(\S+)\s*$", re.MULTILINE)

# 잎사귀에 맞는 값. 금속성 0, 거칠기 높게 — 반들거리는 잎은 가짜처럼 보인다.
METALNESS = "0.0"
ROUGHNESS = "0.85"

# 색을 명시하지 않으면 SDF 기본값이 검정(0 0 0 1)이라 텍스처가 통째로 죽는다.
# diffuse 를 흰색으로 둬야 텍스처 색이 그대로 통과한다.
AMBIENT = "0.5 0.5 0.5 1"
DIFFUSE = "1 1 1 1"
SPECULAR = "0.1 0.1 0.1 1"


def texture_of(material_file: Path) -> str | None:
    """Ogre1 머티리얼 스크립트에서 텍스처 파일명을 읽는다."""
    if not material_file.exists():
        return None
    m = TEXTURE_RE.search(material_file.read_text(errors="replace"))
    return m.group(1) if m else None


def resolve(uri: str, model_dir: Path) -> Path:
    """model://이름/materials/x.material → 실제 파일 경로."""
    # model://test1/materials/bed1.material 에서 'materials/bed1.material' 만 뽑는다
    rest = uri.split("//", 1)[-1].split("/", 1)[-1] if "//" in uri else uri
    return model_dir / rest


def convert(sdf_path: Path, check_only: bool = False) -> int:
    model_dir = sdf_path.parent
    model_name = model_dir.name
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    scripts = [m for m in root.iter("material") if m.find("script") is not None]
    if not scripts:
        print(f"  고칠 것 없음 ({sdf_path})")
        return 0

    if check_only:
        print(
            f"실패: {sdf_path} 에 <material><script> 가 {len(scripts)}개 남아 있습니다.\n"
            "  이대로 Fortress 에 띄우면 전부 새까맣게 렌더링됩니다.\n"
            "  고치려면: make garden-fix",
            file=sys.stderr,
        )
        return 1

    fixed = 0
    for mat in scripts:
        script = mat.find("script")
        uri_el = script.find("uri")
        if uri_el is None or not uri_el.text:
            continue

        mat_file = resolve(uri_el.text, model_dir)
        tex = texture_of(mat_file)
        if tex is None:
            print(f"  경고: {mat_file.name} 에서 텍스처를 못 찾음 — 건너뜀", file=sys.stderr)
            continue

        # <script> 를 걷어내고 제대로 된 머티리얼로 갈아끼운다.
        mat.remove(script)

        # 색 먼저. 이걸 빠뜨리면 기본값이 검정이라 텍스처를 줘도 검게 나온다.
        ET.SubElement(mat, "ambient").text = AMBIENT
        ET.SubElement(mat, "diffuse").text = DIFFUSE
        ET.SubElement(mat, "specular").text = SPECULAR

        pbr = ET.SubElement(mat, "pbr")
        metal = ET.SubElement(pbr, "metal")
        ET.SubElement(metal, "albedo_map").text = f"model://{model_name}/materials/{tex}"
        ET.SubElement(metal, "metalness").text = METALNESS
        ET.SubElement(metal, "roughness").text = ROUGHNESS
        # 잎은 양면이어야 한다. 안 그러면 뒤에서 볼 때 사라진다.
        ET.SubElement(mat, "double_sided").text = "true"
        fixed += 1
        print(f"  {mat_file.name} → pbr/albedo_map={tex}")

    ET.indent(tree, space="  ")
    tree.write(sdf_path, encoding="utf-8", xml_declaration=True)
    print(f"{fixed}개 머티리얼 변환됨 → {sdf_path}")
    return 0


def main() -> None:
    p = argparse.ArgumentParser(description="CropCraft Gazebo 모델 머티리얼 변환")
    p.add_argument("sdf", type=Path, help="model.sdf 경로")
    p.add_argument("--check", action="store_true", help="고칠 게 남았는지만 확인 (고치지 않음)")
    a = p.parse_args()
    sys.exit(convert(a.sdf, a.check))


if __name__ == "__main__":
    main()
