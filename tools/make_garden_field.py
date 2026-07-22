#!/usr/bin/env python3
"""충돌 없는 '주행용 정원' 모델을 만든다 (Stage 4-3 Phase 4b).

oracle_test(CropCraft 산출)는 식물마다 충돌 메시가 있다. 정지 인식(4a)엔 무해했지만, 주행(4b)엔
로봇이 두둑 위 식물 캐노피(콩 ~0.47m)에 부딪힌다. 실제 잎은 휘어지지 도구를 실물 차단하지 않고,
우리는 잡초 죽음도 시뮬 안 하므로(DECISIONS 002) **식물은 시각 전용이어야** 맞다. 두둑 윗면(도구가
멈추는 면)은 model://ridge 의 상자 충돌이 제공한다.

이 스크립트는 oracle_test/model.sdf 를 읽어 <collision> 블록만 제거하고 model://garden_field 로
낸다(메시·재질·좌표는 model://oracle_test/... 를 그대로 참조 — 복제 안 함). 잡초 정답 좌표는 여전히
models/oracle_test.json + 월드 include 오프셋.

실행:  ./scripts/env.sh python3 tools/make_garden_field.py
"""
from __future__ import annotations

import re
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
SRC = WW / "models" / "oracle_test" / "model.sdf"
OUT = WW / "models" / "garden_field"

CONFIG = """<?xml version="1.0"?>
<model>
  <name>garden_field</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>충돌 제거한 주행용 CropCraft 정원 (oracle_test 메시 재사용). tools/make_garden_field.py 생성.</description>
</model>
"""


def main():
    if not SRC.exists():
        raise SystemExit(f"{SRC} 없음 — 먼저 make cropcraft 로 oracle_test 생성")
    src = SRC.read_text()
    # <collision ...>...</collision> 블록 제거 (여러 줄, 비탐욕)
    stripped = re.sub(r"[ \t]*<collision\b.*?</collision>\s*", "", src, flags=re.DOTALL)
    n_col = src.count("<collision") - stripped.count("<collision")
    # ground 링크 통째 제거 — CropCraft 평지 흙(4.8×2.9m)이 z=0.25 에 깔려 사면 두둑을 가린다.
    # 흙은 이제 model://ridge 가 제공(사면 있는 텍스처 두둑). 식물만 남겨 두둑 위에 얹는다.
    stripped = re.sub(r'[ \t]*<link name="ground">.*?</link>\s*', "", stripped, flags=re.DOTALL)
    stripped = stripped.replace('name="oracle_test"', 'name="garden_field"')
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "model.sdf").write_text(stripped)
    (OUT / "model.config").write_text(CONFIG)
    print(f"생성: {OUT}/model.sdf — 충돌 {n_col}개 + ground 링크 제거(식물만, 시각 전용). 메시는 oracle_test 참조")
    assert "<collision" not in stripped and 'name="ground"' not in stripped, "충돌/ground 가 남았다"


if __name__ == "__main__":
    main()
