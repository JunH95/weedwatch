#!/usr/bin/env python3
"""타격 자국 시각화 — 찍은 자리에 자국을 남기고, 정확히 맞은 잡초는 사라지게 한다.

사람이 눈으로 확인하려고 만든 도구다(사용자 요청). 지금까지 검증은 전부 수치 단언이라 "정말 그
자리에 찍었나"를 사람이 볼 방법이 없었다. 여기서는 로봇이 주행·타격하면서:

  · 도구 끝이 실제로 내려앉은 자리에 **자국 원판**을 남긴다 (초록=명중, 빨강=빗나감)
  · 명중한 잡초는 월드에서 **지워진다** (제거를 눈으로 확인)

가능한 이유(실측): 월드에 `/world/<w>/create`·`/remove` 서비스가 있고, `robot_row.sdf` 의 잡초는
**개별 모델**(weed_0..5)이라 하나씩 지울 수 있다. 반면 CropCraft 사실적 밭은 종별 **통메시**라
개별 제거가 불가능하다 — 거기선 자국만 남길 수 있다.

자국·제거는 **꾸밈**이라 제어 루프를 막으면 안 된다(서비스 호출은 ~1초). 워커 스레드로 비동기 처리.

  make strike-marks   헤드리스. 자국 N개·제거 M개를 월드 상태로 단언한다(에이전트가 돌린다)
  make watch-strikes  GUI. 사람이 눈으로 본다 (데스크톱 전용 — 에이전트는 GUI 금지)
"""
from __future__ import annotations

import argparse
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
WORLD_FILE = str(WW / "worlds" / "robot_row.sdf")
WORLD = "robot_row"
WW_CMD = str(WW / "build" / "ww_cmd")

sys.path.insert(0, str(WW / "tools"))
from assert_row_stamp import (  # noqa: E402
    WwCmd, stop, weed_tool, tool_tip, nearest_joints, parse_gt_series,
    TOOL_XS, BAND_CENTERS, BASE_Y, V, STRIKE, RAISE, Z_SETTLE, N,
)
GT_TOPIC = "/world/robot_row/dynamic_pose/info"

# robot_row.sdf 의 weed_* 마커와 일치해야 한다 (월드가 정본).
WEEDS = [(0.70, 0.30), (0.95, 0.85), (1.25, 0.55), (1.60, 0.35), (1.90, 0.75), (2.20, 0.50)]
HIT_TOL = 0.02          # 명중 판정 반경 = 성공 기준 2cm (DECISIONS 002)
BED_TOP = 0.25
TIP_DZ = 0.3075
DRIVE_X = 2.6


def svc(service: str, reqtype: str, req: str, timeout_ms: int = 3000) -> bool:
    r = subprocess.run([ENV, "ign", "service", "-s", f"/world/{WORLD}/{service}",
                        "--reqtype", reqtype, "--reptype", "ignition.msgs.Boolean",
                        "--timeout", str(timeout_ms), "--req", req],
                       capture_output=True, text=True)
    return "data: true" in r.stdout


def mark_sdf(name: str, hit: bool) -> str:
    """자국 원판. 얇은 실린더 — 흙 위에 스탬프 자국처럼 보이게."""
    rgb = "0.15 0.85 0.25" if hit else "0.9 0.15 0.15"
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<link name="l"><visual name="v">'
            f'<geometry><cylinder><radius>0.018</radius><length>0.004</length></cylinder></geometry>'
            f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
            f'<emissive>{rgb} 1</emissive></material>'
            f'</visual></link></model></sdf>')


class Painter(threading.Thread):
    """자국 생성·잡초 제거를 비동기로. 서비스 호출이 ~1초라 제어 루프에서 하면 주행이 끊긴다."""

    def __init__(self):
        super().__init__(daemon=True)
        self.q: queue.Queue = queue.Queue()
        self.marks = 0
        self.removed = 0

    def mark(self, i, x, y, hit):
        self.q.put(("mark", i, x, y, hit))

    def remove_weed(self, wi):
        self.q.put(("remove", wi, 0, 0, False))

    def run(self):
        while True:
            kind, a, x, y, hit = self.q.get()
            if kind == "stop":
                return
            if kind == "mark":
                sdf = mark_sdf(f"mark_{a}", hit).replace('"', '\\"')
                req = (f'sdf: "{sdf}", name: "mark_{a}", allow_renaming: true, '
                       f'pose: {{position: {{x: {x:.4f}, y: {y:.4f}, z: {BED_TOP + 0.003:.4f}}}}}')
                if svc("create", "ignition.msgs.EntityFactory", req):
                    self.marks += 1
            else:
                if svc("remove", "ignition.msgs.Entity", f'name: "weed_{a}", type: MODEL'):
                    self.removed += 1


def run(gui: bool):
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    args = [ENV, "ign", "gazebo", "-r", "--iterations", str(int((10 + DRIVE_X / V + 8) * 1000)),
            WORLD_FILE]
    if not gui:
        args.insert(3, "-s")          # 헤드리스 (에이전트용)
    log = open("/tmp/ww_marks.log", "w")
    sim = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    painter = Painter()
    painter.start()
    ww = None
    struck = []
    try:
        deadline = time.time() + 30
        while time.time() < deadline:
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if "/odometry" in t:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("토픽이 안 떴습니다")

        gtf = open("/tmp/ww_row_gt.log", "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gtf, stderr=subprocess.DEVNULL, start_new_session=True)
        wwp = subprocess.Popen([ENV, WW_CMD, "--world", WORLD, "--model", "weedwatch",
                                "--n-tools", str(N)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                               bufsize=1, start_new_session=True)
        ww = WwCmd(wwp)
        if not ww.ready.wait(timeout=15):
            raise RuntimeError("ww_cmd 준비 안 됨")
        time.sleep(2.0)

        # 잡초를 툴별로 나눠 x 순서대로 처리 (make row 와 같은 스케줄)
        plans = []
        for wi, (wx, wy) in enumerate(WEEDS):
            i = weed_tool(wy)
            plans.append({"wi": wi, "wx": wx, "wy": wy, "i": i,
                          "strike_x": wx - TOOL_XS[i], "phase": 0})
        active = [None] * N
        ww.send(f"v {V:.3f} 0")
        t_end = time.time() + DRIVE_X / V + 40
        while time.time() < t_end:
            O = ww.odom
            if O is None:
                time.sleep(0.01); continue
            ox = O[1]
            for i in range(N):
                if active[i] is None:
                    cand = [p for p in plans if p["i"] == i and p["phase"] == 0
                            and p["strike_x"] > ox + 0.02]
                    if cand:
                        p = min(cand, key=lambda z: z["strike_x"])
                        active[i] = p
                        p["phase"] = 1
                        ww.send(f"carriage {i} {(p['wy'] - BASE_Y) - BAND_CENTERS[i]:.4f}")
                else:
                    p = active[i]
                    if p["phase"] == 1 and ox >= p["strike_x"] - V * Z_SETTLE:
                        ww.send(f"tool {i} {STRIKE:.3f}")
                        p["phase"] = 2
                    elif p["phase"] == 2 and ox >= p["strike_x"]:
                        # **채점 순간** = 도구가 내려가 있고 base 가 잡초의 strike_x 를 지나는 그때.
                        # 도구를 올리는 순간(+5cm 뒤)을 쓰면 로봇이 그만큼 더 가 있어서 오차가
                        # 통째로 5cm 로 나온다 — 실제로 그 버그를 밟았다.
                        struck.append({"wi": p["wi"], "i": i, "simt": O[0],
                                       "wx": p["wx"], "wy": p["wy"]})
                        p["phase"] = 25
                    elif p["phase"] == 25 and ox >= p["strike_x"] + 0.05:
                        # **자국 위치는 여기서 정하지 않는다.** 명령 좌표에 찍으면 오차가 정의상 0 이
                        # 되어 "명령대로 갔다고 가정"하는 거짓 그림이 된다(이 프로젝트 금지사항).
                        # 타격 시각과 그때 achieved 관절만 적어두고, 자국은 주행이 끝난 뒤
                        # **지상진실 base pose + achieved 관절**로 FK 를 풀어 실제 내려앉은 자리에 찍는다.
                        ww.send(f"tool {i} {RAISE:.3f}")
                        p["phase"] = 3
                        active[i] = None
            if ox >= DRIVE_X:
                break
            time.sleep(0.01)
        ww.send("v 0 0")
        time.sleep(0.8)
        joints = list(ww.joints)
        ww.send("q")
        time.sleep(0.5)
        stop(gtsub)
        gtf.close()

        # ── 사후: 지상진실로 실제 도구끝을 구해 그 자리에 자국을 찍는다 ──────────
        gt = parse_gt_series()
        for st in struck:
            base = min(gt, key=lambda g: abs(g[0] - st["simt"])) if gt else None
            cpos, tpos = nearest_joints(joints, st["simt"])
            if base is None or cpos is None:
                st["d"], st["hit"] = float("nan"), False
                continue
            tx, ty, _tz = tool_tip((base[1], base[2], base[3], base[4]),
                                   st["i"], cpos[st["i"]], tpos[st["i"]])
            st["d"] = math.hypot(tx - st["wx"], ty - st["wy"])
            st["hit"] = st["d"] <= HIT_TOL
            painter.mark(st["wi"], tx, ty, st["hit"])       # 실측 자리에 자국
            if st["hit"]:
                painter.remove_weed(st["wi"])

        # 워커가 남은 작업을 끝낼 시간 (서비스 호출 ~1초씩)
        deadline = time.time() + 30
        while not painter.q.empty() and time.time() < deadline:
            time.sleep(0.2)
        time.sleep(2.0)
    finally:
        painter.q.put(("stop", 0, 0, 0, False))
        if ww is not None:
            stop(ww.proc)
        if not gui:
            stop(sim)
            try:
                sim.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        else:
            print("\nGUI 창을 닫으면 끝납니다. (창에서 자국과 사라진 잡초를 확인하세요)")
            sim.wait()
        log.close()
    return struck, painter.marks, painter.removed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="GUI 로 띄운다 (데스크톱 전용, 사람이 봄)")
    a = ap.parse_args()
    print("=== 타격 자국 시각화 — 찍은 자리에 자국, 명중한 잡초는 사라짐 ===\n")
    struck, marks, removed = run(a.gui)
    for st in struck:
        print(f"  잡초{st['wi']} {WEEDS[st['wi']]}: 실측오차 {st['d']*100:5.2f}cm  "
              f"{'명중(제거)' if st['hit'] else '빗나감'}")
    print(f"\n자국 {marks}개 생성 · 잡초 {removed}개 제거 (타격 {len(struck)}회)")
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if not a.gui:
        errs = []
        if marks != len(struck):
            errs.append(f"자국이 타격 수와 다름: {marks} vs {len(struck)}")
        nhit = sum(1 for st in struck if st["hit"])
        if removed != nhit:
            errs.append(f"제거 수가 명중 수와 다름: {removed} vs {nhit}")
        if not struck:
            errs.append("타격이 한 번도 안 일어남")
        if errs:
            print("\nFAIL:\n    - " + "\n    - ".join(errs), file=sys.stderr)
            sys.exit(1)
        print("\n=== OK 자국·제거가 월드에 실제로 반영됐다 (GUI 로 보려면 make watch-strikes) ===")


if __name__ == "__main__":
    main()
