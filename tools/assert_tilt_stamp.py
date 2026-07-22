#!/usr/bin/env python3
"""Stage 5 — 기울기 보정 스탬핑 A/B (Tier 2, 렌더 없음). 흔들려도 잡초 위에 서는가.

로봇이 크로스슬로프에서 STAMP_TILT_DEG 기운 채 두둑 위 잡초를 찍는다. 도구는 기운 몸통 -z 로
하강해 수평 두둑 윗면에 닿으므로, 기울기를 무시하면 tanφ·(두둑깊이)만큼 옆으로 밀린다.

A/B (같은 잡초·같은 시드):
  무보정: 로봇이 수평이라 가정하고 캐리지를 푼다(지금 시스템). → 기울기 때문에 빗나가야 한다(>2cm).
  보정  : IMU 자세(roll)로 옆밀림을 미리 상쇄한다. → 잡초 위 2cm 안(히트).
채점은 제어와 분리한다(프로젝트 규율): 제어는 **IMU**(센서) roll, 채점은 **지상진실** base pose +
achieved joint_state 를 전체 회전 FK 에 넣은 **실측 도구끝**. "명령대로 갔다 가정"이 아니라 실측.

센서 정직성 (사용자 Q, DECISIONS 025 보정): gz IMU orientation 은 참자세에서 계산돼 오차 0이다.
실물 BNO085 는 칩이 융합해 쿼터니언을 직출하되 동적 잔차 ~1° 가 남는다. 그래서 IMU roll 에 실물
잔차(bias+시드 노이즈)를 얹고 **전처리 없이 그대로** 보정에 쓴다 — 우리가 준 노이즈를 되빼지 않는다
(실현값은 랜덤이라 모른다). 보정이 이 잔차를 견디고도 2cm 안에 드는지가 진짜 시험이다.

실행:  ./scripts/env.sh python3 tools/assert_tilt_stamp.py   (make tilt-stamp)
"""
from __future__ import annotations

import math
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_tilt_stamp.sdf")
MODEL = "weedwatch"
GT_TOPIC = "/world/robot_tilt_stamp/dynamic_pose/info"
JOINT_TOPIC = "/world/robot_tilt_stamp/model/weedwatch/joint_state"

sys.path.insert(0, str(WW / "tools"))
from assert_drive import g, parse_messages, quat_to_rpy  # noqa: E402
from assert_joints import read_joint, JSTATE_FILE  # noqa: E402 (joint_state 파서 + 파일경로 재사용)
from garden_geometry import Garden, Portal  # noqa: E402
from make_tilt_world import STAMP_TILT_DEG  # noqa: E402

_G, _P = Garden(), Portal()
TOOL_XS = _P.tool_xs()                    # [-0.09, -0.27, -0.45]
BAND_CENTERS = _P.tool_band_centers(_G)   # [-0.30, 0.0, +0.30] (로봇 중심 기준)

BASE_Y = 0.60
BED_TOP = 0.25          # 수평 두둑 윗면 z (make_tilt_world stamp)
TIP_DZ = 0.3075         # base→도구끝 Z (tool_pos=0). robot_stamp 와 동일 상수
STRIKE = -0.22          # 도구 하강 명령 (두둑이 충돌로 멈춤 → achieved 는 두둑 윗면서 정지)
RAISE = 0.0
TOL_XY = 0.02           # 성공 기준 2cm (DECISIONS 002)
RAISE_WAIT = 1.2
SETTLE = 2.2

# 잡초: 밴드 중심(월드 y=0.30/0.60/0.90) × 담당 툴 X. world 의 weed_* 마커와 일치.
WEEDS = [(TOOL_XS[0], 0.30), (TOOL_XS[1], 0.60), (TOOL_XS[2], 0.90)]

# IMU 자세 실물 잔차 (BNO085 동적 ~1°). 제어에만 쓰고 전처리 안 함. 시드 고정 → flaky 아님.
IMU_BIAS_DEG = 0.8
IMU_NOISE_DEG = 0.5
RNG = random.Random(42)

GT_FILE = "/tmp/ww_tstamp_gt.log"
IMU_FILE = "/tmp/ww_tstamp_imu.log"


class Fail(Exception):
    pass


def publish(topic: str, value: float):
    subprocess.run([ENV, "ign", "topic", "-t", topic, "-m", "ignition.msgs.Double",
                    "-p", f"data: {value}"], capture_output=True, text=True)


def _tail_msgs(path: str, nbytes: int = 16384):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - nbytes))
            return parse_messages(f.read().decode("utf-8", "ignore"))
    except FileNotFoundError:
        return []


def base_pose():
    """지상진실 base (x,y,z,roll,pitch,yaw). 채점·솔버 입력."""
    for m in reversed(_tail_msgs(GT_FILE)):
        poses = m.get("pose")
        if poses is None:
            continue
        if isinstance(poses, dict):
            poses = [poses]
        b = next((p for p in poses if isinstance(p, dict) and p.get("name") == MODEL), None)
        if b is None:
            continue
        r, p, y = quat_to_rpy(g(b, "orientation", "x"), g(b, "orientation", "y"),
                              g(b, "orientation", "z"), g(b, "orientation", "w", default=1.0))
        return (g(b, "position", "x"), g(b, "position", "y"), g(b, "position", "z"), r, p, y)
    return None


def imu_roll():
    """IMU 가 보고하는 roll [rad] 중앙값 (정적 기울기라 안정)."""
    rolls = []
    for m in _tail_msgs(IMU_FILE, 32768):
        if "orientation" not in m:
            continue
        r, _, _ = quat_to_rpy(g(m, "orientation", "x"), g(m, "orientation", "y"),
                              g(m, "orientation", "z"), g(m, "orientation", "w", default=1.0))
        rolls.append(r)
    if not rolls:
        return None
    rolls.sort()
    return rolls[len(rolls) // 2]


def rot(off, r, p, y):
    """R = Rz(y)·Ry(p)·Rx(r) 를 body 오프셋에 적용 → world 오프셋."""
    ox, oy, oz = off
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    x1, y1, z1 = ox, oy * cr - oz * sr, oy * sr + oz * cr           # Rx
    x2, y2, z2 = x1 * cp + z1 * sp, y1, -x1 * sp + z1 * cp          # Ry
    return (x2 * cy - y2 * sy, x2 * sy + y2 * cy, z2)               # Rz


def tip_world(base, i, cpos, tpos):
    """실측 도구끝 world 좌표 = base + R(rpy)·(tool_xs, band_center+carriage, TIP_DZ+tool)."""
    bx, by, bz, r, p, y = base
    ox, oy, oz = rot((TOOL_XS[i], BAND_CENTERS[i] + cpos, TIP_DZ + tpos), r, p, y)
    return (bx + ox, by + oy, bz + oz)


def cpos_uncorrected(i, wy, by):
    """무보정: 로봇이 수평이라 가정 → 캐리지 = (잡초y − base_y) − 밴드중심."""
    return (wy - by) - BAND_CENTERS[i]


def cpos_corrected(i, wy, by, bz, roll):
    """보정: roll 로 옆밀림을 상쇄. 도구가 기운 축으로 두둑윗면(BED_TOP)에 닿는 조건을 역산.
    tip_y=wy 를 풀면 oy = cos·(wy − by + tan·(BED_TOP − bz)).  (pitch·yaw ≈ 0, 크로스슬로프.)"""
    oy = math.cos(roll) * (wy - by + math.tan(roll) * (BED_TOP - bz))
    return oy - BAND_CENTERS[i]


def wait_settled(target_deg, deadline_s=12.0, stable_deg=0.2, hold_s=1.0):
    """base roll 이 목표(±1.5°) 근처에서 hold_s 동안 안정될 때까지 대기 → 첫 타격 전 안착 보장.
    첫 잡초가 세틀 과도상태에 걸려 flaky 하던 것을 결정적으로 없앤다(운이 아니라 조건으로)."""
    t_end = time.time() + deadline_s
    prev, stable_since = None, None
    while time.time() < t_end:
        bp = base_pose()
        if bp is not None:
            roll = math.degrees(bp[3])
            if prev is not None and abs(roll - prev) < stable_deg and abs(abs(roll) - target_deg) < 1.5:
                stable_since = stable_since or time.time()
                if time.time() - stable_since > hold_s:
                    return bp
            else:
                stable_since = None
            prev = roll
        time.sleep(0.3)
    return base_pose()  # 타임아웃 — 마지막 값으로 진행(뒤 단언이 안착 실패를 잡음)


def _stop(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, NameError):
        pass


def strike(i, cpos):
    """담당 툴을 올리고 → 캐리지 cpos 로 → 타격 하강. achieved (cpos,tpos) + base 반환."""
    publish(f"/tool{i}_cmd", RAISE)
    time.sleep(RAISE_WAIT)
    publish(f"/carriage{i}_cmd", cpos)
    time.sleep(SETTLE)
    publish(f"/tool{i}_cmd", STRIKE)
    time.sleep(SETTLE)
    return read_joint(f"carriage{i}_joint"), read_joint(f"tool{i}_joint"), base_pose()


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    total_iters = int((15 + len(WEEDS) * 2 * 11) * 1000)
    log = open("/tmp/ww_tstamp.log", "w")
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(total_iters), WORLD],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    jsub = gtsub = imusub = None
    out = {}
    try:
        deadline = time.time() + 20
        imu_topic = None
        while time.time() < deadline:
            topics = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            imu_topic = next((l.strip() for l in topics.splitlines() if "imu" in l.lower()), None)
            if JOINT_TOPIC in topics and GT_TOPIC in topics and imu_topic:
                break
            time.sleep(0.5)
        else:
            raise Fail(f"토픽 안 뜸 (joint={JOINT_TOPIC in topics}, gt={GT_TOPIC in topics}, imu={imu_topic})")

        jf = open(JSTATE_FILE, "w"); gf = open(GT_FILE, "w"); imf = open(IMU_FILE, "w")
        jsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", JOINT_TOPIC],
                                stdout=jf, stderr=subprocess.DEVNULL, start_new_session=True)
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        imusub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", imu_topic],
                                  stdout=imf, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(1.0)                       # 구독자 연결 + 로그 채움
        b0 = wait_settled(STAMP_TILT_DEG)     # 안착 완료까지 대기 (flaky 방지)
        roll_gt = b0[3] if b0 else 0.0
        roll_imu = imu_roll()
        out["imu_topic"] = imu_topic
        out["roll_gt"] = roll_gt
        out["roll_imu"] = roll_imu
        if b0 is None or roll_imu is None:
            raise Fail("초기 base/IMU 를 못 읽음")

        by, bz = b0[1], b0[2]
        recs = []
        for wx, wy in WEEDS:
            i = _P.band_of(_G, wy - BASE_Y)
            # 무보정
            cu = cpos_uncorrected(i, wy, by)
            cpos_a, tpos_a, base_a = strike(i, cu)
            tip_u = tip_world(base_a, i, cpos_a, tpos_a) if base_a else None
            # 보정: IMU roll + 실물 잔차(전처리 없이 그대로)
            roll_ctrl = roll_imu + math.radians(IMU_BIAS_DEG + RNG.gauss(0, IMU_NOISE_DEG))
            cc = cpos_corrected(i, wy, by, bz, roll_ctrl)
            cpos_b, tpos_b, base_b = strike(i, cc)
            tip_c = tip_world(base_b, i, cpos_b, tpos_b) if base_b else None
            recs.append((wx, wy, i, tip_u, tip_c, roll_ctrl))
        out["recs"] = recs
    finally:
        for s in (jsub, gtsub, imusub):
            _stop(s)
        for fobj in ("jf", "gf", "imf"):
            try:
                locals()[fobj].close()
            except (KeyError, NameError):
                pass
        _stop(sim)
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return out


def main():
    print(f"=== Stage 5 기울기 보정 스탬핑 A/B — 크로스슬로프 {STAMP_TILT_DEG:.0f}° (헤드리스) ===\n")
    r = run()
    recs = r.get("recs")
    if not recs or any(x[3] is None or x[4] is None for x in recs):
        subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
        print("\nFAIL base/joint 실측 실패 — 시뮬/토픽 확인. /tmp/ww_tstamp.log", file=sys.stderr)
        sys.exit(1)

    print(f"  IMU 토픽 : {r['imu_topic']}")
    print(f"  기울기   : GT roll={math.degrees(r['roll_gt']):+.2f}°  IMU roll={math.degrees(r['roll_imu']):+.2f}° "
          f"(제어엔 실물 잔차 bias {IMU_BIAS_DEG}°+노이즈 {IMU_NOISE_DEG}° 얹음)\n")

    u_errs, c_errs, fails = [], [], []
    for wx, wy, i, tip_u, tip_c, roll_ctrl in recs:
        du = math.hypot(tip_u[0] - wx, tip_u[1] - wy)
        dc = math.hypot(tip_c[0] - wx, tip_c[1] - wy)
        u_errs.append(du); c_errs.append(dc)
        um = "MISS" if du > TOL_XY else "hit"
        cm = "HIT " if dc <= TOL_XY else "MISS"
        print(f"  잡초 t{i} ({wx:+.2f},{wy:.2f}):  무보정 {du*100:5.2f}cm [{um}]   →   보정 {dc*100:5.2f}cm [{cm}]")
        print(f"       무보정 dx={ (tip_u[0]-wx)*100:+.2f} dy={(tip_u[1]-wy)*100:+.2f} tip=({tip_u[0]:+.3f},{tip_u[1]:+.3f})  "
              f"보정 dx={(tip_c[0]-wx)*100:+.2f} dy={(tip_c[1]-wy)*100:+.2f}")
        if dc > TOL_XY:
            fails.append(f"보정했는데 빗나감: 잡초 ({wx:+.2f},{wy:.2f}) {dc*100:.2f}cm > {TOL_XY*100:.0f}cm")
        if tip_c[2] > 0.30:
            fails.append(f"도구가 안 내려옴: 잡초 ({wx:+.2f},{wy:.2f}) 끝 z={tip_c[2]:.3f}")

    u_max = max(u_errs) * 100
    c_max = max(c_errs) * 100
    print(f"\n  무보정 최대오차 {u_max:.2f}cm  vs  보정 최대오차 {c_max:.2f}cm")

    # A/B 논지: 기울기가 무보정을 깬다(적어도 하나 >2cm) AND 보정이 전부 2cm 안에 넣는다.
    if u_max <= TOL_XY * 100:
        fails.append(f"무보정이 안 빗나감(무보정 최대 {u_max:.2f}cm ≤ {TOL_XY*100:.0f}cm) — 기울기가 너무 작아 A/B 성립 안 함")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if fails:
        print("\nFAIL:\n    - " + "\n    - ".join(fails), file=sys.stderr)
        sys.exit(1)
    print(f"\n=== OK 흔들려도(={STAMP_TILT_DEG:.0f}° 기울기) IMU 보정으로 잡초 위 {TOL_XY*100:.0f}cm 안에 선다 ===")
    print(f"    무보정이면 최대 {u_max:.1f}cm 빗나가는 걸 보정이 {c_max:.1f}cm 로 잡는다. 실물 IMU 잔차 견딤.")


if __name__ == "__main__":
    main()
