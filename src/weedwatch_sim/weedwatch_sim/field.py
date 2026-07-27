#!/usr/bin/env python3
"""밭 기하 · 오라클(채점 정답) · 텔레포트 — sim 쪽 헬퍼.

coordinator 가 예전엔 tools/field_run 에서 이걸 가져왔는데, field_run 이 WwCmd(직결)를 딸려와서
직결 삭제를 막았다. 여기로 옮겨 그 끈을 끊는다. oracle 은 순수 config(description.json 읽기)라 안전.

set_pose 텔레포트는 스켈레톤 치트(걸터타기 재진입 물리 난제 우회, DECISIONS 036). 실물엔 없음 —
진짜 자율주행(두둑 끝 회전) 구현 시 대체. oracle 채점은 제어와 분리된 지상진실(GT)이다.
"""
import math
import os
import subprocess
import sys
from pathlib import Path

WW = Path(os.environ.get("WW_ROOT", str(Path(__file__).resolve().parents[3])))
ENV = str(WW / "scripts" / "env.sh")
sys.path.insert(0, str(WW / "tools"))
from oracle import load as oracle_load  # noqa: E402  (순수 — description.json 읽기)

FIRST_BED_Y, PITCH = 0.60, 1.20         # 두둑 i 중심 = 0.60 + i·1.20 (두둑폭0.9+고랑0.3)
INCLUDE_OFF = (0.0, 0.17)               # oracle → world (make_field_world garden 오프셋)
WORLD_NAME, MODEL = "robot_field_multi", "weedwatch"
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


def set_pose(x, y, z, yaw=0.0):
    """로봇을 (x,y) 두둑 시작점에 순간이동(스켈레톤 치트, 036). ign set_pose 서비스."""
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "{MODEL}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{z: {qz:.4f}, w: {qw:.4f}}}')
    subprocess.run([ENV, "ign", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
                    "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "3000", "--req", req], capture_output=True, text=True)
