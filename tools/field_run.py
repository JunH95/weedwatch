#!/usr/bin/env python3
"""관통 P3 — 로봇이 여러 두둑 밭을 자율로 훑으며 잡초를 찍고, 한 일을 로그로 남긴다 (DECISIONS 036).

walking skeleton 의 심장: 부분들(카메라 자율 타격=row-live, 커버리지 경로=P1, 여러 줄 밭=P2)을
하나로 꿰어 **끝에서 끝까지** 돌린다. 세부 완벽 아님 — "돌아가고 데이터가 나온다"가 목표.

── 흐름 ────────────────────────────────────────────────────────────────────────
  sim(robot_field_multi, 두둑 N줄) + 카메라 2대 구독(렌더 깨움) + GT 구독(채점) + ww_cmd(제어=odom)
  + detect_server(best.pt, 카메라만으로 검출). 커버리지 순서(coverage_path)대로 두둑을 하나씩:
    · set_pose 로 두둑 시작점에 정렬(순간이동 — 걸터타기 재진입은 P1 의 물리 난제라 스켈레톤은 치트, 036)
    · +x 주행하며 detect_server 가 카메라로 본 잡초를 담당 툴이 예측 하강으로 타격
    · 오라클(정답 좌표)로 사후 채점: 처리/사람몫(작물 근접)/놓침
  전부 artifacts/field_run.json 에 기록 → P4 대시보드가 읽는다.

제어=카메라(센서)·채점=GT 규율(036 재확인): 오라클 좌표는 **제어에 안 쓰고** 채점에만.

실행:  make field-run   (GPU. 카메라 렌더+best.pt 라 느림 — 스켈레톤은 2줄·짧게)
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
PENV = str(WW / "perception" / "env.sh")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
WW_CMD = str(WW / "build" / "ww_cmd")
MODEL = "weedwatch"
WORLD_NAME = "robot_field_multi"
GT_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
GT_FILE = "/tmp/ww_fr_gt.log"
ODOM_FILE = "/tmp/ww_fr_odom.txt"
DETS_FILE = "/tmp/ww_fr_dets.txt"
GUI = "--gui" in sys.argv    # 사람이 데스크톱에서 직접 볼 때만. 기본은 헤드리스(에이전트 단언).

sys.path.insert(0, str(WW / "tools"))
from assert_row_stamp import (  # noqa: E402
    WwCmd, stop, tool_tip, nearest_joints, weed_tool,
    TOOL_XS, BAND_CENTERS, V, STRIKE, RAISE, Z_SETTLE, N,
)
from assert_drive import parse_messages, g, quat_to_rpy  # noqa: E402
from oracle import load as oracle_load  # noqa: E402
from coverage_path import boustrophedon  # noqa: E402
from garden_geometry import Garden, Portal  # noqa: E402
from make_field_world import bed_centers  # noqa: E402

_G, _P = Garden(), Portal()
N_BEDS = 2
X_DRIVE0, X_DRIVE1 = 0.2, 1.6      # 두둑 주행 구간 (짧게 — 카메라+best.pt GPU 경합 느림, 036)
INCLUDE_OFF = (0.0, 0.17)          # oracle → world (make_field_world garden 오프셋)
TOL_XY = 0.08                      # "그 잡초를 맞게 타격" 반경 (4a 절대오차 규모)
SAFE_DIST = 0.025                  # 작물 근접 잡초는 사람 몫(007)
DESCEND_Z = 0.30
CROP_CLEAR = 0.012
CAMDIRS = [WW / "artifacts" / "camera", WW / "artifacts" / "camera1"]
CAM_TOPICS = ["/robot/camera", "/robot/camera1"]


def oracle_weeds_for_bed(cy: float):
    """두둑 중심 cy 에 놓인 정원의 잡초 world 좌표 [(x,y)]. bed0(cy=0.6)이 기존 robot_field 와 동일."""
    og = oracle_load(str(WW / "models" / "oracle_test.json"))
    dy = cy - 0.60
    return [(w.x + INCLUDE_OFF[0], w.y + INCLUDE_OFF[1] + dy) for w in og.weeds]


def crops_for_bed(cy: float):
    og = oracle_load(str(WW / "models" / "oracle_test.json"))
    dy = cy - 0.60
    return [(c.x + INCLUDE_OFF[0], c.y + INCLUDE_OFF[1] + dy) for c in og.crops]


def read_dets():
    try:
        out = []
        for ln in Path(DETS_FILE).read_text().splitlines():
            if ln.startswith("#") or not ln.strip():
                continue
            p = ln.split()
            out.append((float(p[0]), float(p[1]), int(p[2])))
        return out
    except (FileNotFoundError, IndexError, ValueError):
        return []


def parse_gt_series():
    try:
        text = Path(GT_FILE).read_text(errors="ignore")
    except FileNotFoundError:
        return []
    out = []
    for m in parse_messages(text):
        t = g(m, "header", "stamp", "sec") + g(m, "header", "stamp", "nsec") * 1e-9
        poses = m.get("pose")
        if poses is None:
            continue
        if isinstance(poses, dict):
            poses = [poses]
        for p in poses:
            if isinstance(p, dict) and p.get("name") == MODEL:
                q = p.get("orientation", {})
                yaw = quat_to_rpy(g(q, "x"), g(q, "y"), g(q, "z"), g(q, "w") or 1.0)[2]
                out.append((t, g(p, "position", "x"), g(p, "position", "y"),
                            g(p, "position", "z"), yaw))
                break
    out.sort(key=lambda s: s[0])
    return out


def _stop(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def set_pose(x, y, z, yaw=0.0):
    """로봇을 (x,y) 두둑 시작점에 정렬(순간이동). 걸터타기 재진입 물리 난제는 스켈레톤에선 치트(036)."""
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "{MODEL}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{z: {qz:.4f}, w: {qw:.4f}}}')
    subprocess.run([ENV, "ign", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
                    "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "3000", "--req", req], capture_output=True, text=True)


def write_odom(x, y):
    tmp = ODOM_FILE + ".tmp"
    Path(tmp).write_text(f"{x:.4f} {y:.4f}")     # x y — detect_server 가 두둑별 base_y 를 앎
    os.replace(tmp, ODOM_FILE)


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    for d in CAMDIRS:
        d.mkdir(parents=True, exist_ok=True)
        for f in d.glob("*.png"):
            f.unlink()
    for fp in (ODOM_FILE, DETS_FILE):
        Path(fp).write_text("")

    centers = bed_centers(N_BEDS)
    wps = boustrophedon(_G, _P, X_DRIVE0, X_DRIVE1)   # 커버리지 순서(P1)
    bed_order = []                                    # pass 순서대로 (bed, forward)
    for w in wps:
        if w.kind == "pass_start":
            bed_order.append(w.bed)
    # 총 sim 시간 여유: 두둑당 (주행 + 정렬)
    # 시뮬은 개방형(--iterations 없음)으로 돌리고 루프가 끝나면 finally 가 죽인다.
    # iterations 로 수명을 못박으면 sim-time(헤드리스 렌더는 실시간배속<1)과 벽시계 루프가 어긋나
    # 두둑1 주행 전에 시계가 멈춘다 — 그러면 프레임이 안 나와 두둑1 검출 0 (2026-07-24 실측 원인).
    log = open("/tmp/ww_fr.log", "w")
    # GUI(--gui): 사람이 데스크톱에서 프로토타입을 눈으로 본다. 서버+GUI 를 함께 띄운다(-s/헤드리스 뺌).
    # 에이전트는 GUI 금지(CLAUDE.md)라 헤드리스가 기본 — 이 옵션은 오직 사람이 직접 실행할 때.
    sim_cmd = ["ign", "gazebo", "-r", WORLD] if GUI else \
              ["ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD]
    sim = subprocess.Popen([ENV, *sim_cmd],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    subs = []
    ww = det = None
    result = {"field": {"n_beds": N_BEDS, "bed_centers": [round(c, 3) for c in centers],
                        "drive_x": [X_DRIVE0, X_DRIVE1]},
              "beds": [], "started": True}
    try:
        deadline = time.time() + 30
        while time.time() < deadline:
            topics = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if GT_TOPIC in topics and "/odometry" in topics and "/robot/camera" in topics:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("토픽 안 뜸 (GT/odom/camera)")

        for t in CAM_TOPICS:                          # 카메라 구독(렌더 깨움)
            subs.append(subprocess.Popen([ENV, "ign", "topic", "-e", "-t", t],
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                         start_new_session=True))
        gf = open(GT_FILE, "w")
        subs.append(subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                     stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True))
        wwp = subprocess.Popen([ENV, WW_CMD, "--world", WORLD_NAME, "--model", MODEL, "--n-tools", str(N)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
                               start_new_session=True)
        ww = WwCmd(wwp)
        if not ww.ready.wait(timeout=15):
            raise RuntimeError("ww_cmd 준비 안 됨")
        detlog = open("/tmp/ww_fr_det.log", "w")
        det = subprocess.Popen([PENV, "python", str(WW / "perception" / "detect_server.py"),
                                "--watch", ",".join(str(d) for d in CAMDIRS),
                                "--out", DETS_FILE, "--odom-file", ODOM_FILE,
                                "--base", "0", "0.6", "0", "0", "--safe-dist", str(SAFE_DIST)],
                               stdout=detlog, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(6.0)      # best.pt 로드

        t0 = time.time()
        # 스켈레톤: 두둑마다 시작점으로 순간이동 후 항상 +x 주행(치트라 역주행 이득 없음 — 036).
        # 진짜 보스트로페돈 회전은 P1 물리 난제라 관통 뒤 세부.
        for bed in bed_order:
            cy = centers[bed]
            set_pose(X_DRIVE0 - 0.05, cy, 0.05, 0.0)
            time.sleep(2.5)
            bed_log = {"bed": bed, "y": round(cy, 3), "reached": False,
                       "detected": [], "struck": [], "oracle_weeds": len(oracle_weeds_for_bed(cy))}
            seen = set()
            active = [None] * N
            pool = [[] for _ in range(N)]
            ox = X_DRIVE0 - 0.05
            ox_ref = None                             # 이 두둑 odom 기준(아래)
            ww.send(f"v {V:.3f} 0")
            # 안전 데드라인(벽시계) — 정상 종료는 odom 이 X_DRIVE1 도달(아래 break). 개방형 시뮬이라
            # 실시간배속만큼 넉넉히: 주행 sim-time / 최저 RTF 가정 0.15 + 여유.
            drive_deadline = time.time() + (X_DRIVE1 - X_DRIVE0) / V / 0.15 + 30
            while time.time() < drive_deadline:
                O = ww.odom
                if O is None:
                    time.sleep(0.01); continue
                # odom(O[1])은 두둑 간 누적이라 텔레포트로 안 리셋된다 → 두둑 시작을 기준0 으로 상대화.
                # 이걸 안 하면 두둑1 은 첫 줄에서 ox=이전두둑끝 ≥ X_DRIVE1 로 즉시 종료(주행 0). 2026-07-24.
                if ox_ref is None:
                    ox_ref = O[1]
                ox = (X_DRIVE0 - 0.05) + (O[1] - ox_ref)   # 참 world x
                write_odom(ox, cy)                    # detect_server 앵커링(두둑별 y)
                for wx, wy, _a in read_dets():
                    key = (round(wx / 0.06), round(wy / 0.06))
                    if key in seen or abs(wy - cy) > 0.45:
                        continue
                    i = weed_tool(wy - (cy - 0.6))     # 밴드는 두둑 중심 기준
                    strike_x = wx - TOOL_XS[i]
                    if ox >= strike_x - V * Z_SETTLE:  # 이미 하강 시점 지남
                        continue
                    seen.add(key)
                    pool[i].append({"wx": wx, "wy": wy, "i": i, "strike_x": strike_x, "phase": 0})
                    bed_log["detected"].append([round(wx, 3), round(wy, 3)])
                for i in range(N):
                    if active[i] is None:
                        cand = [p for p in pool[i] if p["phase"] == 0 and p["strike_x"] > ox + 0.01]
                        if cand:
                            p = min(cand, key=lambda z: z["strike_x"])
                            active[i] = p; p["phase"] = 1
                            ww.send(f"carriage {i} {(p['wy'] - cy) - BAND_CENTERS[i]:.4f}")
                    else:
                        p = active[i]
                        if p["phase"] == 1 and ox >= p["strike_x"] - V * Z_SETTLE:
                            ww.send(f"tool {i} {STRIKE:.3f}"); p["phase"] = 2; p["simt"] = O[0]
                            bed_log["struck"].append([round(p["wx"], 3), round(p["wy"], 3)])
                        elif p["phase"] == 2 and ox >= p["strike_x"] + 0.06:
                            ww.send(f"tool {i} {RAISE:.3f}"); p["phase"] = 3; active[i] = None
                if ox >= X_DRIVE1:
                    break
                time.sleep(0.01)
            bed_log["reached"] = ox >= X_DRIVE1 - 0.05    # 실제 완주 여부(게이트 정직성)
            ww.send("v 0 0"); time.sleep(0.4)
            result["beds"].append(bed_log)

        ww.send("v 0 0"); time.sleep(0.3)
        result["duration_s"] = round(time.time() - t0, 1)
        joints = list(ww.joints)
        ww.send("q")
        time.sleep(0.4)
        for s in subs:
            _stop(s)
        try:
            gf.close()
        except Exception:
            pass

        # 사후 채점: 오라클 잡초별 처리/사람몫/놓침 (제어와 분리, GT)
        gt = parse_gt_series()
        summ = {"struck": 0, "handed_to_human": 0, "missed": 0, "detected": 0}
        for bl in result["beds"]:
            cy = bl["y"]
            crops = crops_for_bed(cy)
            summ["detected"] += len(bl["detected"])
            bl["weeds"] = []                          # 오라클 잡초별 위치+판정 (대시보드 지도용)
            for wx, wy in oracle_weeds_for_bed(cy):
                near_crop = crops and min(math.hypot(wx - cx, wy - cyp) for cx, cyp in crops) < SAFE_DIST
                hit = any(math.hypot(wx - sx, wy - sy) <= TOL_XY for sx, sy in bl["struck"])
                outcome = "struck" if hit else ("handed_to_human" if near_crop else "missed")
                summ[outcome] += 1
                bl["weeds"].append({"x": round(wx, 3), "y": round(wy, 3), "outcome": outcome})
            bl["crops"] = [[round(cx, 3), round(cyp, 3)] for cx, cyp in crops]
        result["summary"] = summ
        result["coverage"] = {"beds_done": sum(1 for b in result["beds"] if b.get("reached")),
                               "beds_total": N_BEDS}
    finally:
        for s in (det, ww.proc if ww else None):
            _stop(s)
        _stop(sim)
        try:
            sim.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
            except Exception:
                pass
        log.close()
    return result


def main():
    print("=== 관통 P3 — 여러 두둑 자율 주행+타격+로깅 (GPU) ===\n")
    r = run()
    out = WW / "artifacts" / "field_run.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(r, ensure_ascii=False, indent=2))
    s = r.get("summary", {})
    cov = r.get("coverage", {})
    print(f"커버리지: 두둑 {cov.get('beds_done')}/{cov.get('beds_total')} 완주 · {r.get('duration_s')}s")
    print(f"검출: {s.get('detected')}개 · 처리 {s.get('struck')} · 사람몫 {s.get('handed_to_human')} · 놓침 {s.get('missed')}")
    print(f"로그: {out}")
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    # 스켈레톤 게이트: 관통이 "돌아가고 데이터가 나온다" (재현율로 안 막음 — 036)
    if cov.get("beds_done") != N_BEDS:
        print("\nFAIL 두둑을 다 못 훑음 (커버리지 미완)", file=sys.stderr); sys.exit(1)
    if s.get("detected", 0) == 0:
        print("\nFAIL 카메라가 아무것도 검출 못 함 (렌더/인식 실패)", file=sys.stderr); sys.exit(1)
    print("\n=== OK 관통 — 밭을 자율로 훑고 잡초 찍고 데이터를 남겼다 (P4 대시보드로 볼 수 있음) ===")


if __name__ == "__main__":
    main()
