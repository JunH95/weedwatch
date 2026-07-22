#!/usr/bin/env python3
"""Stage 5 Tier 1 기울기 선검증 — 로봇이 크로스슬로프에서 기울고, 물리가 안 터지고, IMU 가 그 기울기를 읽는가.

두 게이트 (프로젝트 철학, assert_drive 와 동형):
  게이트 A (센서): /robot/imu 가 roll ≈ TILT_DEG 를 보고하는가. IMU 가 실제로 발행되고 정확한가.
  게이트 B (물리): 지상진실 pose 가 몸이 실제로 그만큼 기울었다 확인하는가 + 안 넘어짐 + 발산 없음.
IMU 만 보면 거짓 통과(센서가 맞아도 물리가 발산할 수 있음), GT 만 보면 IMU 배선·정확도를 못 잡는다.

DECISIONS 025: 지금 sim 은 평지라 안 기울고, 어떤 월드에도 imu-system 이 없어 IMU 가 발행조차 안 됐다.
이 검증이 Stage 5 의 물리 기반(기운 접촉 DART 안정 + IMU 신뢰)을 세운다. 통과해야 흔들림 보정으로 넘어간다.

주의(정직): gz IMU 의 orientation 은 노이즈 없는 지상진실 자세에서 계산돼 GT 와 거의 같다(적분 안 함).
실물 IMU 자세추정은 드리프트가 있다 — 이 갭은 Stage 5 후반에 orientation 노이즈로 메운다(원장).

실행: ./scripts/env.sh python3 tools/assert_tilt.py   (make tilt)
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
sys.path.insert(0, str(WW / "tools"))
from assert_drive import parse_messages, quat_to_rpy, gt_samples, g, stamp_s  # noqa: E402
from make_tilt_world import TILT_DEG  # noqa: E402

ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_tilt.sdf")
GT_TOPIC = "/world/robot_tilt/dynamic_pose/info"

SETTLE_S = 4.0     # 안착 + 정상상태 수집 벽시계
TILT_TOL = 1.5     # 기울기 목표 허용오차 [도]
IMU_TOL = 1.0      # IMU↔GT 자세 허용오차 [도]
ROLL_STD_MAX = 0.5  # 정상상태 roll 표준편차 상한 [도] — 진동/발산 감지


class Fail(Exception):
    pass


def imu_samples(text: str):
    """[(t, roll, pitch, yaw)] — IMU 가 보고하는 자세(orientation)."""
    out = []
    for m in parse_messages(text):
        if "orientation" not in m:
            continue
        r, p, y = quat_to_rpy(
            g(m, "orientation", "x"), g(m, "orientation", "y"),
            g(m, "orientation", "z"), g(m, "orientation", "w", default=1.0))
        out.append((stamp_s(m), r, p, y))
    return out


def _stop(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass


def run():
    """robot_tilt 월드를 띄우고 안착시킨 뒤 GT pose + IMU 자세 스트림을 수집."""
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    iters = int((6 + SETTLE_S) * 1000)  # 초기화 + 안착. RTF=1 → 1ms/스텝
    log = open("/tmp/ww_tilt.log", "w")
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(iters), WORLD],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.time() + 20
        imu_topic = None
        out = ""
        while time.time() < deadline:
            out = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            imu_topic = next((l.strip() for l in out.splitlines() if "imu" in l.lower()), None)
            if GT_TOPIC in out and imu_topic:
                break
            time.sleep(0.5)
        else:
            raise Fail(f"토픽 안 뜸 (GT={GT_TOPIC in out}, imu={imu_topic}). /tmp/ww_tilt.log 확인")

        gt_f = open("/tmp/ww_tilt_gt.log", "w")
        imu_f = open("/tmp/ww_tilt_imu.log", "w")
        gt_sub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                  stdout=gt_f, stderr=subprocess.DEVNULL, start_new_session=True)
        imu_sub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", imu_topic],
                                   stdout=imu_f, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(SETTLE_S)
        _stop(gt_sub); _stop(imu_sub)
        gt_f.close(); imu_f.close()
        gt_sub.wait(timeout=5); imu_sub.wait(timeout=5)
    finally:
        _stop(sim)
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()

    return (gt_samples(open("/tmp/ww_tilt_gt.log").read()),
            imu_samples(open("/tmp/ww_tilt_imu.log").read()), imu_topic)


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if not n:
        raise Fail("정상상태 샘플 없음")
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def main():
    print(f"=== Stage 5 Tier 1 기울기 선검증 — 목표 크로스슬로프 roll = {TILT_DEG:.1f}° ===\n")
    gt, imu, imu_topic = run()
    print(f"  IMU 토픽 : {imu_topic}")
    print(f"  수집     : GT {len(gt)} 샘플, IMU {len(imu)} 샘플")
    if len(gt) < 10 or len(imu) < 10:
        subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
        print("\nFAIL 샘플 부족 — 시뮬/토픽 실패. /tmp/ww_tilt.log 확인", file=sys.stderr)
        sys.exit(1)

    # 정상상태 = 뒤 절반 (안착 뒤)
    gt_ss = gt[len(gt) // 2:]
    imu_ss = imu[len(imu) // 2:]
    gt_roll = math.degrees(median([s[4] for s in gt_ss]))
    gt_pitch = math.degrees(median([s[5] for s in gt_ss]))
    gt_z = median([s[3] for s in gt_ss])
    imu_roll = math.degrees(median([s[1] for s in imu_ss]))
    imu_pitch = math.degrees(median([s[2] for s in imu_ss]))
    roll_std = (sum((math.degrees(s[4]) - gt_roll) ** 2 for s in gt_ss) / len(gt_ss)) ** 0.5
    nan = any(v != v for s in gt_ss for v in s)

    print(f"  게이트B 물리(GT) : roll={gt_roll:+.2f}° pitch={gt_pitch:+.2f}° z={gt_z:+.3f}m  (roll std={roll_std:.2f}°)")
    print(f"  게이트A 센서(IMU): roll={imu_roll:+.2f}° pitch={imu_pitch:+.2f}°")
    print(f"  IMU↔GT 일치      : Δroll={abs(imu_roll - gt_roll):.2f}°")

    errs = []
    if abs(abs(gt_roll) - TILT_DEG) > TILT_TOL:
        errs.append(f"몸이 목표만큼 안 기욺: GT roll {gt_roll:+.2f}° vs 목표 ±{TILT_DEG}° (허용 ±{TILT_TOL})")
    if abs(gt_pitch) > TILT_TOL:
        errs.append(f"의도 없는 pitch: {gt_pitch:+.2f}° (크로스슬로프인데 pitch 가 큼)")
    if roll_std > ROLL_STD_MAX:
        errs.append(f"정상상태 아님(진동/발산): roll std {roll_std:.2f}° > {ROLL_STD_MAX}°")
    if abs(gt_roll) > 30:
        errs.append(f"넘어짐: roll {gt_roll:+.2f}°")
    if nan:
        errs.append("NaN pose — DART 폭발")
    if abs(imu_roll - gt_roll) > IMU_TOL:
        errs.append(f"IMU↔GT 불일치: Δroll {abs(imu_roll - gt_roll):.2f}° > {IMU_TOL}° (IMU 배선/정확도)")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if errs:
        print("\nFAIL 기울기 선검증 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print("\n=== OK 로봇이 크로스슬로프에서 안정적으로 기울고, IMU 가 그 기울기를 정확히 읽는다 ===")
    print("    DART 가 기운 접촉에서 안 터짐 + IMU 발행·정확 확인 → Stage 5 흔들림 보정 하네스로 진행 가능.")


if __name__ == "__main__":
    main()
