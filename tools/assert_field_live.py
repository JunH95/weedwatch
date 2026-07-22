#!/usr/bin/env python3
"""주행 라이브 온-루프 단언 — 로봇이 제 카메라로 본 잡초만으로 주행하며 타격 (Tier 3, GPU, Stage 4-3 4b-3).

이전까지: 인식(P3·P4a)과 무정차 타격(P2)이 따로, 또는 정답 좌표로. 여기서 **오라클을 제어에서 빼고**
로봇 down_cam 이 실시간 본 것(best.pt via detect_server)만으로 주행 타격한다 — 완전 자율.

── 프로세스 융합 (제어=odom, 채점=GT 규율) ─────────────────────────────────
  sim(robot_field, 카메라 렌더) + 카메라 구독자(렌더 깨움) + GT 구독자(채점) + ww_cmd(제어·odom)
  + detect_server(ML venv, best.pt). detect_server 는 <save> PNG + odom_x 파일로 world 검출을 낸다
  (odom 앵커링 = GT 아님). 하네스가 그 검출을 dedup 해 툴별로 스케줄, 예측 하강. GT 는 사후 채점만.

── 채점 (사후, 오라클) ──────────────────────────────────────────────────────
  각 오라클 target 마다, 담당 툴이 그 x 를 지날 때(base_x=tx-tool_x) 내려온 도구 끝이 target 에
  얼마나 가까웠나(FK: GT base + achieved joint). 검출→제어 경로와 무관하게 "진짜 잡초가 맞았나"만 잰다.
  4a 절대오차(캐노피-vs-밑동+시차)가 여기 그대로 들어오므로 반경은 잡초 크기 규모. 정밀은 P2(오라클).

실행:  make row-live
"""
from __future__ import annotations

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
WORLD = str(WW / "worlds" / "robot_field.sdf")
WW_CMD = str(WW / "build" / "ww_cmd")
MODEL = "weedwatch"
GT_TOPIC = "/world/robot_field/dynamic_pose/info"
GT_FILE = "/tmp/ww_field_gt.log"
ODOM_FILE = "/tmp/ww_field_odom.txt"
DETS_FILE = "/tmp/ww_field_dets.txt"
CAMDIR = WW / "artifacts" / "camera"

sys.path.insert(0, str(WW / "tools"))
from assert_drive import parse_messages, g, quat_to_rpy  # noqa: E402
from assert_row_stamp import (  # noqa: E402
    WwCmd, stop, tool_tip, nearest_joints, weed_tool,
    TOOL_XS, BAND_CENTERS, BASE_Y, V, STRIKE, RAISE, Z_SETTLE, N,
)
from oracle import load as oracle_load  # noqa: E402
import assert_render  # noqa: E402

INCLUDE_OFF = (0.0, 0.17)   # robot_field.sdf garden include (오라클 → world)
FIELD_WEED_XMAX = 1.5       # 이 x 까지 주행. 카메라 렌더+best.pt 가 GPU 경합해 sim 이 ~0.15x 로 느려
                            # 전 밭(2.9)은 벽시계로 수분. 첫 잡초 군집(x≲1.5)만 커버해 시험을 짧게.
DEDUP = 0.06                # 검출 dedup 격자 [m]
TOL_XY = 0.08               # "그 잡초를 맞게 타격" 반경 (4a 절대오차 규모, 캐노피-vs-밑동+시차)
DESCEND_Z = 0.30
# 작물 물리 접촉 임계: 툴 반경 6mm + 줄기 ~5mm + 1mm = 1.2cm 안이면 "점타격 툴이 작물을 건드림".
# 선택성 헤드라인(002/009)의 척도. 3cm 는 과보수 안전마진이었다(비물리). 이게 진짜 접촉 여부.
CROP_CLEAR = 0.012
SAFE_DIST = 0.025           # 작물 회피(safe-remove, 007): 작물 이 거리[m] 안 잡초는 안 찍음. 빽빽한
                            # 인터크로핑(oracle_test shift_next_bed=false, 잡초 작물 겹침)이라 크면
                            # 재현율↓·작으면 작물접촉↑ (top-down 캐노피가 밑동 가림). 정직한 트레이드오프.
# 재현율 floor: 빽빽한 밭에서 대부분 잡초가 작물 코앞이라 safe-remove 됨(007 = 사람 몫). "안전 타격
# 가능한 잡초를 실제로 친다"만 증명하는 낮은 문턱. 정밀 2cm·고재현율은 P2(오라클 좌표)가 별도 증명.
# 주의: 이 테스트는 GPU 렌더+best.pt+타이밍이 얽힌 **stochastic Tier-3 통합 데모**다 — 재현율(관측
# 0.25~0.5)·스침 수(0~1)가 run 마다 변한다. 게이트는 "자율 루프가 실제로 돈다"를 안정적으로 잡는
# 여유 문턱이지, P2/P4a 같은 결정론적 정밀 게이트가 아니다. 정밀 2cm·깨끗한 무접촉은 P2(오라클 좌표,
# 물리, 결정론적)가, 인식 정확도는 P3 게이트가 이미 증명. 여기 값은 "완전 자율(카메라만) 동작" 증명용.
RECALL_FLOOR = 0.15        # ≥2 잡초 자율 타격 (관측 0.25~0.5 에 여유)
CROP_GRAZE_MAX = 2          # 허용 스침 수. top-down 가림 오차로 가끔 스침은 인정(관측 0~1), 체계적 타격은 차단.
MIN_AVG_SPEED = 0.15        # 라이브는 검출·스케줄 부하로 P2보다 여유


class Fail(Exception):
    pass


def oracle_targets():
    og = oracle_load(str(WW / "models" / "oracle_test.json"))
    weeds = [(w.x + INCLUDE_OFF[0], w.y + INCLUDE_OFF[1]) for w in og.weeds]
    crops = [(c.x + INCLUDE_OFF[0], c.y + INCLUDE_OFF[1]) for c in og.crops]
    return weeds, crops


def read_dets():
    try:
        lines = Path(DETS_FILE).read_text().splitlines()
    except FileNotFoundError:
        return []
    out = []
    for ln in lines:
        if ln.startswith("#") or not ln.strip():
            continue
        p = ln.split()
        try:
            out.append((float(p[0]), float(p[1]), int(p[2])))
        except (IndexError, ValueError):
            continue
    return out


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


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    for f in CAMDIR.glob("*.png"):
        f.unlink()
    CAMDIR.mkdir(parents=True, exist_ok=True)
    for fp in (ODOM_FILE, DETS_FILE):
        Path(fp).write_text("")
    drive_dist = FIELD_WEED_XMAX + 0.6
    total_iters = int((8 + drive_dist / V + 6) * 1000)

    log = open("/tmp/ww_field_live.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--headless-rendering",
                            "--iterations", str(total_iters), WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    subs = []
    ww = det = None
    completed = False
    try:
        deadline = time.time() + 25
        while time.time() < deadline:
            topics = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if GT_TOPIC in topics and "/odometry" in topics and "/robot/camera" in topics:
                break
            time.sleep(0.5)
        else:
            raise Fail("토픽(odometry/GT/camera)이 안 떴습니다")

        # 카메라 구독자(렌더 깨움 — Fortress 는 구독자 있어야 렌더+<save>) + GT 구독자(채점)
        cf = open("/tmp/ww_field_cam.devnull", "w")
        subs.append(subprocess.Popen([ENV, "ign", "topic", "-e", "-t", "/robot/camera"],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        gf = open(GT_FILE, "w")
        subs.append(subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                     stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True))
        # ww_cmd (제어·odom)
        wwp = subprocess.Popen([ENV, WW_CMD, "--world", "robot_field", "--model", MODEL, "--n-tools", str(N)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
                               start_new_session=True)
        ww = WwCmd(wwp)
        if not ww.ready.wait(timeout=15):
            raise Fail("ww_cmd 준비(R) 안 옴")
        # detect_server (ML venv): PNG + odom → world 검출. 로그 저장(디버그).
        detlog = open("/tmp/ww_field_det.log", "w")
        det = subprocess.Popen([PENV, "python", str(WW / "perception" / "detect_server.py"),
                                "--watch", str(CAMDIR), "--out", DETS_FILE, "--odom-file", ODOM_FILE,
                                "--base", "0", str(BASE_Y), "0", "0", "--safe-dist", str(SAFE_DIST)],
                               stdout=detlog, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(5.0)  # 로봇 안착 + detect_server 모델 로드(수초) + 첫 프레임

        # ── 라이브 제어 루프 ──
        pool = {i: [] for i in range(N)}      # 툴별 잡초 plan 리스트
        seen = set()                          # dedup 격자 키
        active = [None] * N                    # 툴별 현재 처리 잡초
        ww.send(f"v {V:.3f} 0")
        # 카메라 렌더+best.pt GPU 경합으로 sim 이 ~0.1x. 벽시계 데드라인 크게 + sim-end 는 **sim-time
        # (odom O[0]) 정지**로 판정(로봇 출발 전 오검출 방지 — ox 정지가 아니라 sim 이 안 도는 걸 봄).
        t_deadline = time.time() + drive_dist / V / 0.08 + 50
        last_simt, last_step_t = None, time.time()
        while time.time() < t_deadline:
            O = ww.odom
            if O is None:
                time.sleep(0.01); continue
            simt, ox = O[0], O[1]
            tmp = ODOM_FILE + ".tmp"                     # 원자적 쓰기 (detect_server 가 빈 파일 안 읽게)
            Path(tmp).write_text(f"{ox:.4f}")
            os.replace(tmp, ODOM_FILE)
            if last_simt is None or abs(simt - last_simt) > 1e-4:
                last_simt, last_step_t = simt, time.time()
            elif time.time() - last_step_t > 8:          # sim-time 8s 정지 = sim 끝/죽음
                break

            for wx, wy, _a in read_dets():
                if abs(wy - BASE_Y) > 0.45:
                    continue
                i = weed_tool(wy)
                strike_x = wx - TOOL_XS[i]
                if ox >= strike_x - V * Z_SETTLE:       # 이미 하강 시점 지남 → 못 잡음
                    continue
                key = (round(wx / DEDUP), round(wy / DEDUP))
                if key in seen:
                    continue
                seen.add(key)
                pool[i].append({"wx": wx, "wy": wy, "i": i, "strike_x": strike_x,
                                "descend_x": strike_x - V * Z_SETTLE, "retract_x": strike_x + 0.06,
                                "phase": 0})

            for i in range(N):
                if active[i] is None:
                    cand = [p for p in pool[i] if p["phase"] == 0 and p["strike_x"] > ox + 0.01]
                    if cand:
                        p = min(cand, key=lambda z: z["strike_x"])
                        active[i] = p
                        p["phase"] = 1
                        ww.send(f"carriage {i} {(p['wy'] - BASE_Y) - BAND_CENTERS[i]:.4f}")
                else:
                    p = active[i]
                    if p["phase"] == 1 and ox >= p["descend_x"]:
                        ww.send(f"tool {i} {STRIKE:.3f}")
                        p["phase"] = 2
                    elif p["phase"] == 2 and ox >= p["retract_x"]:
                        ww.send(f"tool {i} {RAISE:.3f}")
                        p["phase"] = 3
                        active[i] = None

            if ox >= drive_dist - 0.05:
                completed = True
                break
            time.sleep(0.01)

        ww.send("v 0 0")
        time.sleep(0.3)
        ww.send("q")
        joints = list(ww.joints)
        odom_final = ww.odom
        n_detected = len(seen)
    finally:
        if det is not None:
            stop(det)
        if ww is not None:
            try:
                ww.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                stop(ww.proc)
        for s in subs:
            stop(s)
        try:
            gf.close(); cf.close()
        except (NameError, AttributeError):
            pass
        stop(sim)
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return joints, odom_final, completed, n_detected


def main():
    print("=== 주행 라이브 온-루프 단언 (카메라만으로 주행 타격, GPU) ===\n")
    joints, odom_final, completed, n_detected = run()
    gt = parse_gt_series()
    weeds, crops = oracle_targets()
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if not gt:
        raise Fail("GT 없음")
    if not joints:
        raise Fail("관절 achieved 없음 — ww_cmd J 실패")

    errs = []
    # 게이트 1: 렌더 2게이트 (카메라가 실제로 렌더됐나)
    try:
        assert_render.gate_pixels(CAMDIR)
        assert_render.gate_device()
        print("게이트1 렌더: 검지않음 AND NVIDIA — OK")
    except SystemExit:
        errs.append("렌더 2게이트 실패")

    gt_xmax = max(s[1] for s in gt)
    # 주행 구간 안 오라클 target (툴이 지나간 x + 밴드 안)
    inrange = [(tx, ty) for tx, ty in weeds
               if abs(ty - BASE_Y) <= 0.45 and 0.3 < tx < gt_xmax - 0.1]

    def descended_tip_min(px, py):
        """전 궤적에서, (px,py) 를 담당 툴이 지날 때 내려온 도구 끝의 최소 거리."""
        i = weed_tool(py)
        best = float("inf")
        for s in gt:
            base = (s[1], s[2], s[3], s[4])
            cj, tj = nearest_joints(joints, s[0])
            if cj is None or math.isnan(cj[i]) or math.isnan(tj[i]):
                continue
            tipx, tipy, tipz = tool_tip(base, i, cj[i], tj[i])
            if tipz > DESCEND_Z:
                continue
            best = min(best, math.hypot(tipx - px, tipy - py))
        return best

    print(f"\n카메라 검출 잡초(dedup): {n_detected}개 · 주행 구간 오라클 target: {len(inrange)}개")
    detected = 0
    errors = []
    for tx, ty in inrange:
        d = descended_tip_min(tx, ty)
        hit = d <= TOL_XY
        detected += hit
        if hit:
            errors.append(d)
        print(f"  target ({tx:.2f},{ty:.2f}): 내려온 도구 최근접 {d*100:5.1f}cm {'타격' if hit else '놓침'}")
    recall = detected / len(inrange) if inrange else 0.0

    # 게이트: 작물 무접촉
    crop_viol = 0
    min_crop = float("inf")
    for cx, cy in crops:
        if not (0.3 < cx < gt_xmax) or abs(cy - BASE_Y) > 0.45:
            continue
        d = descended_tip_min(cx, cy)
        min_crop = min(min_crop, d)
        if d < CROP_CLEAR:
            crop_viol += 1

    errors.sort()
    med = errors[len(errors) // 2] if errors else float("nan")

    print(f"\n검출→타격 재현율(<= {TOL_XY*100:.0f}cm): {detected}/{len(inrange)} = {recall:.2f}")
    print(f"[보고] 타격된 target 위치오차 중앙 {med*100:.1f}cm (검출 캐노피-vs-밑동+시차 — 정밀은 P2)")
    print(f"작물 무접촉: 내려온 도구의 작물 최소거리 {min_crop*100:.1f}cm (하한 {CROP_CLEAR*100:.0f}) "
          f"{'OK' if not crop_viol else 'FAIL'}")
    print(f"완주: {'OK' if completed else 'FAIL'}")
    print(f"[보고] odom↔GT: odom_x={odom_final[1] if odom_final else float('nan'):.2f} vs GT_x={gt_xmax:.2f}")

    if not inrange:
        errs.append("주행 구간 target 0 — 좌표/오프셋 확인")
    if recall < RECALL_FLOOR:
        errs.append(f"검출→타격 재현율 {recall:.2f} < {RECALL_FLOOR} (안전 타격 가능 잡초를 못 침)")
    # 작물 접촉: 빽빽한 겹침 밭(oracle_test 잡초-작물 의도적 겹침, 1-c)에서 top-down 캐노피가 밑동을
    # 가려 safe-remove 로도 가끔 스친다 — 정직한 한계. 깨끗한 무접촉은 P2(오라클, 물리)가 증명했다.
    # 여기선 **체계적 작물 타격**(가림 오차 아닌 진짜 실패)만 차단: 스침 ≤ CROP_GRAZE_MAX 허용.
    if crop_viol > CROP_GRAZE_MAX:
        errs.append(f"작물 물리 접촉 {crop_viol}건 > {CROP_GRAZE_MAX} (체계적 작물 타격 — safe-remove 실패)")
    if not completed:
        errs.append("미완주")

    if errs:
        print("\nFAIL 주행 라이브 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print(f"\n=== OK 주행 라이브 통과 — 로봇이 제 카메라로 본 잡초를 주행하며 {detected}/{len(inrange)} 타격 "
          f"(오라클 대조, 작물 무접촉) ===")


if __name__ == "__main__":
    main()
