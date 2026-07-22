#!/usr/bin/env python3
"""이미 떠 있는 robot_row 시뮬에 붙어 정답좌표로 무정차 주행+스탬핑을 재생한다 (사람 눈 관람용).

make row(=assert_row_stamp)는 헤드리스로 sim 을 직접 띄워 단언만 찍는다. 이건 그 **제어 로직만**
재사용해서, scripts/watch_row.sh 가 GUI 로 띄운 sim 에 붙어 로봇을 앞으로 몰며 툴을 잡초에 내려찍는다.
채점(GT)은 안 한다 — 그건 make row 가 헤드리스로 이미 한다. 여기는 "눈으로 본다"가 목적.

scripts/watch_row.sh 가 이 스크립트를 부른다(직접 실행하려면 robot_row 가 이미 -r 로 떠 있어야).
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from assert_row_stamp import (  # noqa: E402
    WwCmd, build_plans, drive_loop, stop, ENV, WW_CMD, MODEL, N,
)


def main():
    print("ww_cmd 로 실행 중인 robot_row 에 붙는다...", flush=True)
    wwp = subprocess.Popen(
        [ENV, WW_CMD, "--world", "robot_row", "--model", MODEL, "--n-tools", str(N)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        start_new_session=True,
    )
    ww = WwCmd(wwp)
    if not ww.ready.wait(timeout=15):
        sys.exit("ww_cmd 준비(R) 신호가 안 왔습니다 — robot_row 가 -r 로 떠 있나 확인")
    print("연결됨. 2초 뒤 무정차 주행 시작 (앞으로 가며 잡초 위 툴 하강)...", flush=True)
    time.sleep(2.0)

    plans, drive_dist = build_plans()
    completed = drive_loop(ww, plans, drive_dist)
    print(f"주행 {'완주' if completed else '시간초과'} — 잡초 {sum(1 for p in plans if p['phase']==3)}/{len(plans)} 처리 동작 재생됨.",
          flush=True)

    ww.send("q")
    try:
        ww.proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        stop(ww.proc)
    print("재생 끝. GUI 창은 열어둔 채 살펴봐도 되고, 닫으면 종료.", flush=True)


if __name__ == "__main__":
    main()
