#!/usr/bin/env python3
"""밭 기하 · 오라클(채점 정답) — sim 쪽 헬퍼.

coordinator 가 예전엔 tools/field_run 에서 이걸 가져왔는데, field_run 이 WwCmd(직결)를 딸려와서
직결 삭제를 막았다. 여기로 옮겨 그 끈을 끊는다. oracle 은 순수 config(description.json 읽기)라 안전.

순간이동 치트(set_pose)는 **삭제됐다** — 두둑 사이를 실제 헤드랜드 U턴으로 돌게 되면서(DECISIONS 040,
weedwatch_control.maneuver) 쓸 데가 없어졌다. oracle 채점은 제어와 분리된 지상진실(GT)이다.
"""
import os
import sys
from pathlib import Path

WW = Path(os.environ.get("WW_ROOT", str(Path(__file__).resolve().parents[3])))
sys.path.insert(0, str(WW / "tools"))
from oracle import load as oracle_load  # noqa: E402  (순수 — description.json 읽기)

FIRST_BED_Y, PITCH = 0.60, 1.20         # 두둑 i 중심 = 0.60 + i·1.20 (두둑폭0.9+고랑0.3)
INCLUDE_OFF = (0.0, 0.17)               # oracle → world (make_field_world garden 오프셋)
_ORACLE = str(WW / "models" / "oracle_test.json")


def bed_centers(n: int):
    return [FIRST_BED_Y + i * PITCH for i in range(n)]


def oracle_weeds_for_bed(cy: float):
    """두둑 중심 cy 정원의 잡초 world 좌표 [(x,y)] (채점용 GT). bed0(cy=0.6)이 기존 robot_field 동일."""
    og = oracle_load(_ORACLE); dy = cy - 0.60
    return [(w.x + INCLUDE_OFF[0], w.y + INCLUDE_OFF[1] + dy) for w in og.weeds]


def crops_for_bed(cy: float):
    og = oracle_load(_ORACLE); dy = cy - 0.60
    return [(c.x + INCLUDE_OFF[0], c.y + INCLUDE_OFF[1] + dy) for c in og.crops]

