#!/usr/bin/env python3
"""무정차 행 스윕 단언 — 주행하며 임의 (x,y) 잡초를 타격하는가 (Tier 2, 렌더 없음, Stage 4-3).

4-2(assert_stamp)는 로봇이 서 있는 채로 두둑 폭(Y)만 검증했다. 여기선 로봇이 두둑을 걸터탄 채
+x 로 **무정차 주행**하며, 카메라(base x=0.22)가 먼저 본 잡초를 담당 툴(엇갈린 X)이 지나갈 때
예측 하강으로 타격한다. DECISIONS 020(무정차 + 멀티툴 점타격) 실증.

── 제어와 채점의 분리 (핵심 규율) ──────────────────────────────────────────
제어는 **오도메트리**로 한다(ww_cmd 의 O 라인). ww_cmd 는 지상진실을 의도적으로 안 본다.
채점은 **지상진실**(dynamic_pose/info)로 한다 — 별도 구독자 파일에서 사후에. 둘을 프로세스로
분리해 "명령대로 갔다 가정"이 아니라 "실제로 그 위치를 지났나"를 잰다(4-2 정신 계승, 주행판).

── 인과 공개 (알려진 좌표지만 온-루프처럼) ─────────────────────────────────
표적 좌표는 안다(Phase 2). 그러나 컨트롤러엔 **카메라가 지나간 순간(odom_x ≥ wx − camera_x)에만**
드러낸다 — 실제 카메라가 그 잡초를 본 시점이다. 그 뒤 리드타임 안에 캐리지를 정렬하고, 툴 끝이
잡초에 닿기 180ms 전(Z 정착시간, 020 실측)에 하강을 건다. Phase 4 에서 이 공개를 best.pt 라이브
추론으로 교체한다(제어 구조는 그대로).

── FK (툴 i) ───────────────────────────────────────────────────────────────
도구끝 = base_GT + R(yaw)·(tool_xs[i], band_center[i] + carriage_i, 0) + (0,0, 0.3075 + tool_i)
carriage_i·tool_i 는 achieved(sim 보고). base 는 지상진실. 주행 중 yaw 를 회전으로 반영.

실행:  ./scripts/env.sh python3 tools/assert_row_stamp.py   (make row)
"""
from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_row.sdf")
WW_CMD = str(WW / "build" / "ww_cmd")
MODEL = "weedwatch"
GT_TOPIC = "/world/robot_row/dynamic_pose/info"
GT_FILE = "/tmp/ww_row_gt.log"

sys.path.insert(0, str(WW / "tools"))
from assert_drive import g, parse_messages, quat_to_rpy  # noqa: E402
from garden_geometry import Garden, Portal  # noqa: E402

_G, _P = Garden(), Portal()
N = _P.n_tools
TOOL_XS = _P.tool_xs()                    # [-0.09, -0.27, -0.45]
BAND_CENTERS = _P.tool_band_centers(_G)   # [-0.30, 0.0, +0.30]

# 잡초·작물 좌표 — worlds/robot_row.sdf 의 weed_*/crop_* 마커와 일치해야 한다.
WEEDS = [(0.70, 0.30), (0.95, 0.85), (1.25, 0.55), (1.60, 0.35), (1.95, 0.95), (2.30, 0.62)]
# crop_4 (1.25,0.51) 는 잡초 w2(1.25,0.55) 4cm 옆 — 점타격 선택성(002/009) 직접 시험.
CROPS = [(0.85, 0.55), (1.45, 0.80), (1.75, 0.60), (2.10, 0.40), (1.25, 0.51)]

BASE_Y = 0.60          # 로봇 spawn y (두둑 중심). carriage_i 명령 = (wy-BASE_Y) - band_center[i]
CAMERA_X = _P.camera_x  # 0.22 — 이 거리만큼 카메라가 툴보다 앞서 본다
V = 0.20               # 무정차 주행 속도 (020: ≤0.2 라야 ±2cm 창 200ms > Z 180ms)
STRIKE = -0.15         # 도구 하강 명령 (두둑이 충돌로 멈춤)
RAISE = 0.0            # 도구 접힘 (주행 중 두둑 안 긁게)
TIP_DZ = 0.3075        # base 기준 도구 끝 Z (tool_pos=0)
Z_SETTLE = 0.180       # Z 하강 정착 시간 [s] (020 실측) → 이만큼 앞서 하강 건다
TOL_XY = 0.02          # 성공 기준 2cm (DECISIONS 002)
DESCEND_Z = 0.30       # 타격 판정: 도구 끝 z 가 이 아래면 "내려온 것"
CROP_CLEAR = 0.03      # 무접촉 게이트: 내려온 툴 끝이 이 안에 들면 물리 접촉 [m]
                       # = 툴 반경 0.006 + 작물 반경 0.02 ≈ 0.026 에 여유. 점타격 12mm 의 선택성 척도.
MIN_AVG_SPEED = 0.18   # 안티크리프: 주행 평균속도 하한 (저속 기어가기로 통과하는 부정 차단)


class Fail(Exception):
    pass


def weed_tool(wy: float) -> int:
    return _P.band_of(_G, wy - BASE_Y)


# ── ww_cmd 상주 프로세스 래퍼 (제어 = odom) ──────────────────────────────────

class WwCmd:
    """ww_cmd 를 Popen 으로 몰며 O(odom)/J(joints) 스트림을 실시간 파싱. 제어는 이 odom 으로만."""

    def __init__(self, proc):
        self.proc = proc
        self.lock = threading.Lock()
        self.odom = None            # (simt, x, y, yaw, vx, wz) — 최신
        self.joints = []            # [(simt, [c0..], [t0..])] — 채점용 시계열
        self.ready = threading.Event()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for raw in self.proc.stdout:
            line = raw.rstrip("\n")
            if not line:
                continue
            tag = line[0]
            if tag == "R":
                self.ready.set()
            elif tag == "O":
                p = line.split()
                try:
                    s = (float(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6]))
                except (IndexError, ValueError):
                    continue
                with self.lock:
                    self.odom = s
            elif tag == "J":
                p = line.split()
                try:
                    simt = float(p[1])
                    vals = [float(x) for x in p[2:2 + 2 * N]]
                except (IndexError, ValueError):
                    continue
                if len(vals) < 2 * N:
                    continue
                with self.lock:
                    self.joints.append((simt, vals[:N], vals[N:2 * N]))
            elif tag == "E":
                print(f"  [ww_cmd] {line}", file=sys.stderr)

    def odom_x(self):
        with self.lock:
            return self.odom[1] if self.odom else None

    def send(self, line: str):
        try:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass


def stop(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, NameError, AttributeError):
        pass


# ── 지상진실 파싱 (채점 전용, 사후) ─────────────────────────────────────────

def parse_gt_series():
    """GT_FILE 전체를 (simt, x, y, z, yaw) 시계열로. weedwatch 모델 pose 만."""
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


def nearest_joints(joints, simt):
    """simt 에 가장 가까운 J 샘플의 (carriage[], tool[]). nan 은 마지막 유효값으로 안 채우고 그대로."""
    if not joints:
        return None, None
    best = min(joints, key=lambda j: abs(j[0] - simt))
    return best[1], best[2]


def tool_tip(base, i, carriage_i, tool_i):
    """도구끝 월드좌표. base=(x,y,z,yaw). yaw 회전을 (x,y) 오프셋에 반영."""
    bx, by, bz, yaw = base
    ox = TOOL_XS[i]
    oy = BAND_CENTERS[i] + carriage_i
    c, s = math.cos(yaw), math.sin(yaw)
    tip_x = bx + c * ox - s * oy
    tip_y = by + s * ox + c * oy
    tip_z = bz + TIP_DZ + tool_i
    return tip_x, tip_y, tip_z


# ── 주행 + 폐루프 제어 (헤드리스 채점·GUI watch 공용) ────────────────────────

def build_plans():
    """각 잡초의 이벤트 x(odom 기준) 스케줄 + 완주거리. 정답 좌표 + 인과공개(카메라 통과 시 드러남)."""
    plans = []
    for wx, wy in WEEDS:
        i = weed_tool(wy)
        strike_x = wx - TOOL_XS[i]                     # base_x 가 이 값이면 tip_x = wx
        plans.append({
            "wx": wx, "wy": wy, "i": i,
            "reveal_x": wx - CAMERA_X,                 # 카메라가 잡초를 지나는 순간
            "carriage_cmd": (wy - BASE_Y) - BAND_CENTERS[i],
            "descend_x": strike_x - V * Z_SETTLE,      # 180ms(거리 V·0.18) 앞서 하강
            "retract_x": strike_x + 0.06,              # 지나간 뒤 접기
            "phase": 0,                                # 0 대기 1 정렬 2 하강 3 접힘
        })
    drive_dist = max(wx - TOOL_XS[weed_tool(wy)] for wx, wy in WEEDS) + 0.30
    return plans, drive_dist


def drive_loop(ww, plans, drive_dist, timeout_extra=12):
    """무정차 출발 + odom 기반 스케줄 실행. 완주하면 True. 끝에 정지. GUI watch·헤드리스 공용."""
    ww.send(f"v {V:.3f} 0")
    deadline = time.time() + (drive_dist / V) + timeout_extra
    completed = False
    while time.time() < deadline:
        ox = ww.odom_x()
        if ox is None:
            time.sleep(0.005)
            continue
        for pl in plans:
            if pl["phase"] == 0 and ox >= pl["reveal_x"]:
                ww.send(f"carriage {pl['i']} {pl['carriage_cmd']:.4f}")   # 카메라가 봄 → 정렬
                pl["phase"] = 1
            elif pl["phase"] == 1 and ox >= pl["descend_x"]:
                ww.send(f"tool {pl['i']} {STRIKE:.3f}")                    # 예측 하강
                pl["phase"] = 2
            elif pl["phase"] == 2 and ox >= pl["retract_x"]:
                ww.send(f"tool {pl['i']} {RAISE:.3f}")                     # 접기
                pl["phase"] = 3
        if all(pl["phase"] == 3 for pl in plans) and ox >= drive_dist - 0.05:
            completed = True
            break
        time.sleep(0.005)
    ww.send("v 0 0")
    return completed


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    # 필요한 sim 시간: (마지막 strike_x + 여유)/V + 안착. 넉넉히 iterations 준다(realtime).
    last_strike_x = max(wx - TOOL_XS[weed_tool(wy)] for wx, wy in WEEDS)
    drive_dist = last_strike_x + 0.30
    total_iters = int((6 + drive_dist / V + 4) * 1000)

    log = open("/tmp/ww_row.log", "w")
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(total_iters), WORLD],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    gtsub = ww = None
    gf = None
    completed = False
    try:
        # 토픽 뜰 때까지
        deadline = time.time() + 20
        while time.time() < deadline:
            topics = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if GT_TOPIC in topics and "/odometry" in topics:
                break
            time.sleep(0.5)
        else:
            raise Fail("odometry/dynamic_pose 토픽이 안 떴습니다 — 시뮬 초기화 실패")

        # 채점용 GT 구독자 (파일). 제어와 별개 프로세스 → GT 를 제어가 못 본다.
        gf = open(GT_FILE, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)

        # ww_cmd 상주 프로세스 (제어 = odom). --n-tools 로 멀티툴.
        wwp = subprocess.Popen(
            [ENV, WW_CMD, "--world", "robot_row", "--model", MODEL, "--n-tools", str(N)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
            start_new_session=True,
        )
        ww = WwCmd(wwp)
        if not ww.ready.wait(timeout=15):
            raise Fail("ww_cmd 준비(R) 신호가 안 왔습니다 — 명령 경로 실패")
        time.sleep(2.0)  # 로봇 안착 + odom 안정

        plans, _dd = build_plans()
        completed = drive_loop(ww, plans, drive_dist)   # 무정차 주행 + 스케줄 (공용)
        time.sleep(0.3)
        ww.send("q")
        joints_snapshot = list(ww.joints)
        odom_final = ww.odom
    finally:
        if ww is not None:
            try:
                ww.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                stop(ww.proc)
        stop(gtsub)
        try:
            gf.close()
        except (NameError, AttributeError):
            pass
        stop(sim)
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return joints_snapshot, odom_final, completed


def main():
    print("=== 무정차 행 스윕 단언 (헤드리스, GPU 불필요) ===\n")
    print(f"── 두둑 위 잡초 {len(WEEDS)}개를 {V} m/s 무정차 주행하며 타격. 성공기준 |도구-잡초| < {TOL_XY*100:.0f}cm ──")
    print(f"   툴 X 엇갈림 {['%.2f'%x for x in TOOL_XS]} · 리드 {['%.2f'%_P.tool_lead(i) for i in range(N)]}m · 예측하강 {Z_SETTLE*1000:.0f}ms 앞서\n")
    joints, odom_final, completed = run()
    gt = parse_gt_series()
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)

    if not gt:
        raise Fail("지상진실 샘플이 없습니다 — GT 구독/파싱 실패")
    if not joints:
        raise Fail("관절 achieved 샘플이 없습니다 — ww_cmd J 스트림 실패")

    errs = []

    # ── 게이트 3: iterations 예산 — 완주했는가 ──
    if not completed:
        errs.append("주행이 예산(iterations) 안에 안 끝남 — 마지막 잡초 전 소진")

    # ── 게이트 2: 안티크리프 — 주행 평균속도 (움직인 구간, base_x>0.05 뒤부터) ──
    moving = [(t, x) for t, x, *_ in gt if x > 0.05]
    if len(moving) >= 2:
        avg_speed = (moving[-1][1] - moving[0][1]) / (moving[-1][0] - moving[0][0])
    else:
        avg_speed = 0.0
    creep_ok = avg_speed >= MIN_AVG_SPEED

    # ── 게이트 1: 각 잡초에서 도구 끝 X/Y 정렬 < 2cm (타격 순간 = tip_x 가 wx 지날 때) ──
    print("잡초별 타격 (지상진실 base + achieved joint → FK):")
    for wx, wy in WEEDS:
        i = weed_tool(wy)
        strike_x = wx - TOOL_XS[i]
        # tip_x 가 wx 에 가장 가까운 GT 샘플 = base_x 가 strike_x 에 가장 가까운 순간(yaw≈0)
        s = min(gt, key=lambda z: abs(z[1] - strike_x))
        base = (s[1], s[2], s[3], s[4])
        cj, tj = nearest_joints(joints, s[0])
        if cj is None or math.isnan(cj[i]) or math.isnan(tj[i]):
            errs.append(f"잡초 ({wx:.2f},{wy:.2f}): 타격 순간 관절값 없음")
            print(f"  FAIL 잡초 ({wx:.2f},{wy:.2f}) t{i}: 관절값 없음")
            continue
        tip_x, tip_y, tip_z = tool_tip(base, i, cj[i], tj[i])
        dxy = math.hypot(tip_x - wx, tip_y - wy)
        descended = tip_z <= DESCEND_Z
        ok = dxy <= TOL_XY and descended
        mark = "OK" if ok else "FAIL"
        print(f"  {mark} 잡초 ({wx:.2f},{wy:.2f}) t{i}: 도구끝=({tip_x:+.3f},{tip_y:+.3f},{tip_z:.3f}) "
              f"오차={dxy*100:5.2f}cm  {'하강' if descended else '미하강'}  (carriage={cj[i]:+.3f} tool={tj[i]:+.3f})")
        if dxy > TOL_XY:
            errs.append(f"잡초 ({wx:.2f},{wy:.2f}): X/Y 정렬 {dxy*100:.2f}cm > {TOL_XY*100:.0f}cm")
        if not descended:
            errs.append(f"잡초 ({wx:.2f},{wy:.2f}): 도구가 안 내려옴 (끝 z={tip_z:.3f})")

    # ── 게이트 4: 작물 무접촉 — 전 궤적에서 내려온 툴 끝이 작물 반경 침범 0 ──
    violations = []
    min_crop_dist = float("inf")
    for s in gt:
        base = (s[1], s[2], s[3], s[4])
        cj, tj = nearest_joints(joints, s[0])
        if cj is None:
            continue
        for i in range(N):
            if math.isnan(cj[i]) or math.isnan(tj[i]):
                continue
            tip_x, tip_y, tip_z = tool_tip(base, i, cj[i], tj[i])
            if tip_z > DESCEND_Z:
                continue  # 접혀 있으면 무해
            for cx, cy in CROPS:
                d = math.hypot(tip_x - cx, tip_y - cy)
                min_crop_dist = min(min_crop_dist, d)
                if d < CROP_CLEAR:
                    violations.append((cx, cy, i, d))
    if violations:
        for cx, cy, i, d in violations[:3]:
            errs.append(f"작물 ({cx:.2f},{cy:.2f}) 침범: 툴{i} 끝이 {d*100:.1f}cm (< {CROP_CLEAR*100:.0f}cm)")

    # ── 게이트 5(보고): odom ↔ GT 표류 ──
    odom_x = odom_final[1] if odom_final else float("nan")
    gt_x = gt[-1][1] if gt else float("nan")

    print(f"\n안티크리프: 주행 평균속도 {avg_speed:.3f} m/s (하한 {MIN_AVG_SPEED}) {'OK' if creep_ok else 'FAIL'}")
    print(f"작물 무접촉: 내려온 툴 끝의 작물 최소거리 {min_crop_dist*100:.1f}cm (하한 {CROP_CLEAR*100:.0f}cm) "
          f"{'OK' if not violations else 'FAIL'}")
    print(f"완주(iterations 예산): {'OK' if completed else 'FAIL'}")
    print(f"[보고] odom↔GT 표류: odom_x={odom_x:.3f} vs GT_x={gt_x:.3f} (Δ={abs(odom_x-gt_x)*100:.1f}cm)")

    if not creep_ok:
        errs.append(f"안티크리프: 평균속도 {avg_speed:.3f} < {MIN_AVG_SPEED} (저속 기어가기)")

    if errs:
        print("\nFAIL 무정차 행 스윕 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print(f"\n=== OK 무정차 행 스윕 통과 — 주행하며 잡초 {len(WEEDS)}개를 {TOL_XY*100:.0f}cm 안으로 타격, 작물 무접촉 ===")


if __name__ == "__main__":
    main()
