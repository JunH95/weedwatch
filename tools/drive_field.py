#!/usr/bin/env python3
"""이미 떠 있는 robot_field 시뮬에 붙어, CropCraft 사실적 밭에서 오라클 잡초 좌표로 무정차 주행+
스탬핑을 재생한다 (사람 눈 관람용, Stage 4-3 Phase 4b-1).

watch-row 와 같은 관람용이지만, 마커가 아니라 **사실적 두둑·식물 위**를 달린다. 표적은 아직 카메라
라이브가 아니라 오라클 정답 좌표(4b-3 에서 카메라 검출로 교체). scripts/watch_field.sh 가 부른다.
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from assert_row_stamp import (  # noqa: E402
    WwCmd, build_plans, drive_loop, stop, weed_tool, ENV, WW_CMD, MODEL, N, BASE_Y,
)
from oracle import load as oracle_load  # noqa: E402

INCLUDE_OFF = (0.0, 0.17)   # robot_field.sdf 의 garden include (dx,dy). 잡초 world = 오라클+이것.


def field_weeds():
    """오라클 target 잡초를 필드 world 좌표로. 툴 도달 밴드(±0.45) 안만."""
    og = oracle_load(str(WW / "models" / "oracle_test.json"))
    ws = []
    for w in og.weeds:
        wx, wy = w.x + INCLUDE_OFF[0], w.y + INCLUDE_OFF[1]
        if abs(wy - BASE_Y) <= 0.45 and wx > 0.30:   # 밴드 안 + 출발 뒤
            ws.append((wx, wy))
    ws.sort()
    return ws


def main():
    weeds = field_weeds()
    print(f"오라클 잡초 {len(weeds)}개를 필드에서 타격 (사실적 두둑·식물 위 주행)", flush=True)
    wwp = subprocess.Popen(
        [ENV, WW_CMD, "--world", "robot_field", "--model", MODEL, "--n-tools", str(N)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        start_new_session=True,
    )
    ww = WwCmd(wwp)
    if not ww.ready.wait(timeout=15):
        sys.exit("ww_cmd 준비(R) 신호가 안 왔습니다 — robot_field 가 -r 로 떠 있나 확인")
    print("연결됨. 2초 뒤 무정차 주행 시작...", flush=True)
    time.sleep(2.0)

    plans, drive_dist = build_plans(weeds)
    completed = drive_loop(ww, plans, drive_dist)
    struck = sum(1 for p in plans if p["phase"] == 3)
    print(f"주행 {'완주' if completed else '종료'} — 잡초 {struck}/{len(plans)} 타격 동작 재생.", flush=True)

    ww.send("q")
    try:
        ww.proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        stop(ww.proc)
    print("재생 끝. GUI 창을 돌려보며 살펴봐도 되고, 닫으면 종료.", flush=True)


if __name__ == "__main__":
    main()
