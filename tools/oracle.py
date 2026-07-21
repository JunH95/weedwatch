#!/usr/bin/env python3
"""오라클 — 정원에 뭐가 어디 있는지의 **정답**을 읽는다.

── 이게 왜 프로젝트의 심장인가 ──────────────────────────────────────────
우리 성공 기준은 "잡초 위에 정확히 선다"이다. 채점하려면 잡초가 **진짜로** 어디 있는지를
알아야 한다. 카메라가 본 것 말고, 정답 말이다. 안 그러면 "로봇이 잡초를 찾았다"를
로봇 자신에게 물어보는 셈이고, 그건 검증이 아니다.

그리고 이 프로젝트의 하드 제약이 "Claude 가 스스로 검증한다"이므로, 사람 눈이 없는
상태에서 숫자로 채점할 수 있어야 한다. 오라클이 그걸 가능하게 한다.

── 문제와 해법 ─────────────────────────────────────────────────────────
CropCraft 는 잡초 좌표를 **안 내보낸다** (직접 확인함):
    FieldState = { beds, leaf_area }   ← weeds 필드 없음
    Weed       = { density, ... }      ← 좌표 없음
잡초는 Blender 지오메트리 노드가 뿌리는 인스턴스라 좌표가 파이썬에 존재한 적이 없다.

해법: **작물/잡초는 에셋 속성이 아니라 설정이 정한다.** 두 경로가 같은 함수를 부른다
(beds.py:60,144 와 ground.py:58 이 모두 get_model_list_by_height 호출).
그래서 채점할 잡초를 `beds:` 블록에 선언하면 좌표가 나온다. 코드 수정 0줄.

── 어느 bed 가 잡초인가 ────────────────────────────────────────────────
종으로 판별하지 **않는다**. "portulaca 는 잡초"라고 박아버리면 위의 발견 자체를 버리는
셈이다 — 같은 종이 herb garden 에서는 작물일 수도 있다. 잡초는 "내가 안 심은 식물"이지
특정 종이 아니다.

대신 **설정이 이름으로 선언한다**: bed 이름이 `target_` 로 시작하면 채점 대상 잡초다.
description.json 의 config.beds[i].name 이 field.beds[i] 와 같은 순서로 남아 있어서
그대로 읽으면 된다.

사용법:
    tools/oracle.py models/oracle_test.json          # 요약
    tools/oracle.py models/oracle_test.json --json   # 기계용
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

# 이 접두사로 시작하는 bed 는 "우리가 채점할 잡초"다.
# weeds: 블록에 선언된 잡초는 좌표가 없고, 채점도 안 한다 (밭을 현실적으로 만드는 방해물).
TARGET_PREFIX = "target_"


@dataclass(frozen=True)
class Plant:
    """정원에 실제로 심긴 식물 하나의 정답."""

    kind: str  # "crop" | "weed"
    species: str  # bean, polygonum, portulaca ...
    bed: str  # 설정에서의 bed 이름
    x: float
    y: float
    z: float
    height: float
    width: float

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)


@dataclass
class Garden:
    """한 정원의 정답 전체."""

    seed: int
    crops: list[Plant]
    weeds: list[Plant]  # 채점 대상만. 방해물은 여기 없다(좌표가 없으니까).

    @property
    def plants(self) -> list[Plant]:
        return self.crops + self.weeds

    def nearest_crop_distance(self, p: Plant) -> float:
        """이 식물에서 가장 가까운 작물까지의 거리.

        잡초가 작물에 붙어 있을수록 어렵다 — 잘못 건드리면 작물이 죽는다.
        거리별 recall 곡선을 그리면 키 규칙이 어디서 죽는지가 보인다.
        """
        if not self.crops:
            return float("inf")
        return min(((p.x - c.x) ** 2 + (p.y - c.y) ** 2) ** 0.5 for c in self.crops)


def load(path: str | Path) -> Garden:
    """description.json 에서 정답을 읽는다."""
    d = json.loads(Path(path).read_text())
    cfg, state = d["config"], d["field"]

    cfg_beds, state_beds = cfg["beds"], state["beds"]
    if len(cfg_beds) != len(state_beds):
        raise ValueError(
            f"config.beds({len(cfg_beds)}) 와 field.beds({len(state_beds)}) 개수가 다릅니다. "
            "CropCraft 출력 형식이 바뀌었을 수 있습니다."
        )

    crops: list[Plant] = []
    weeds: list[Plant] = []

    for meta, bed in zip(cfg_beds, state_beds):
        name = meta.get("name") or "?"
        # 순서가 맞는지 확인한다. 종이 어긋나면 zip 이 틀린 것이다.
        want = meta.get("plant_type")
        got = next(
            (c["type"] for r in bed.get("rows", []) for c in r.get("crops", [])), None
        )
        if got is not None and want != got:
            raise ValueError(
                f"bed '{name}': 설정은 {want} 인데 좌표는 {got} 입니다. "
                "config.beds 와 field.beds 의 순서가 안 맞습니다."
            )

        is_target = name.startswith(TARGET_PREFIX)
        bucket = weeds if is_target else crops
        for row in bed.get("rows", []):
            for c in row.get("crops", []):
                bucket.append(
                    Plant(
                        kind="weed" if is_target else "crop",
                        species=c["type"],
                        bed=name,
                        x=c["x"],
                        y=c["y"],
                        z=c["z"],
                        height=c["height"],
                        width=c.get("width", 0.0),
                    )
                )

    return Garden(seed=cfg.get("seed"), crops=crops, weeds=weeds)


def main() -> None:
    ap = argparse.ArgumentParser(description="정원 정답(오라클) 읽기")
    ap.add_argument("description", type=Path, help="CropCraft 의 field_description json")
    ap.add_argument("--json", action="store_true", help="기계용 출력")
    a = ap.parse_args()

    g = load(a.description)

    if a.json:
        json.dump(
            {
                "seed": g.seed,
                "crops": [vars(p) for p in g.crops],
                "weeds": [vars(p) for p in g.weeds],
            },
            sys.stdout,
            ensure_ascii=False,
        )
        return

    print(f"시드: {g.seed}")
    print(f"작물 {len(g.crops)}개 · 채점 대상 잡초 {len(g.weeds)}개")
    print()

    if not g.weeds:
        print("주의:  채점 대상 잡초가 0개입니다.")
        print(f"    설정의 beds: 블록에 '{TARGET_PREFIX}...' 이름으로 잡초를 선언해야 합니다.")
        print("    weeds: 블록에 선언하면 좌표가 안 나옵니다 (지오메트리 노드가 뿌려서).")
        sys.exit(1)

    print("채점 대상 잡초 (로봇이 이 위에 서야 한다):")
    print(f"  {'종':<12} {'x':>7} {'y':>7} {'키cm':>6} {'작물까지cm':>10}")
    for p in sorted(g.weeds, key=lambda p: (p.x, p.y)):
        print(
            f"  {p.species:<12} {p.x:>7.3f} {p.y:>7.3f} {p.height*100:>6.1f} "
            f"{g.nearest_crop_distance(p)*100:>10.1f}"
        )

    print()
    # Tertill 특허의 키 규칙을 이 정원에 실제로 적용해보면?
    # 특허 US10888045: 잡초 센서 1인치(2.54cm) / 작물 센서 1.5인치(3.81cm)
    #                  "tall objects are considered crops, short objects are weeds"
    T = 0.0254
    survived = [p for p in g.weeds if p.height > T]
    killed_crops = [p for p in g.crops if p.height <= T]
    print(f"Tertill 키 규칙(2.54cm)을 이 정원에 적용하면:")
    print(f"  잡초 {len(survived)}/{len(g.weeds)}개가 '작물'로 오인돼 살아남음")
    print(f"  작물 {len(killed_crops)}/{len(g.crops)}개가 '잡초'로 오인돼 잘림")


if __name__ == "__main__":
    main()
