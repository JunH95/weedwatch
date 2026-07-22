#!/usr/bin/env python3
"""Stage 5 Tier 2 Step A — 동적 요철 주행 소양 검증 (Tier 2, 렌더 없음).

로봇이 흙덩이 밭을 주행한다. Step B(주행 중 타격 보정)로 가기 전에 기반 세 가지를 단언한다:
  (1) DART 안정 + 완주: 물리가 안 터지고(NaN 없음), 안 넘어지고(|roll|,|pitch| 한계), 앞으로 나아간다.
  (2) 실제로 흔들린다: 자세가 시변(roll/pitch peak-to-peak 가 의미 있는 각도)해야 보정할 게 생긴다.
  (3) IMU 가 그 시변 자세를 GT 대로 추적: 시각 정합해 |IMU−GT| 가 작아야 주행 중 보정의 입력이 된다.

(2)+(3) 이 정적 선검증(make tilt)의 동적 확장이다. 정적은 한 번 읽으면 됐지만, 동적은 IMU 가
매 순간 GT 를 따라와야 타격 순간에 쓸 수 있다. 셋 다 통과해야 Step B 로 넘어간다.

실행:  ./scripts/env.sh python3 tools/assert_shake.py   (make shake)
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
WORLD = str(WW / "worlds" / "robot_shake.sdf")
GT_TOPIC = "/world/robot_shake/dynamic_pose/info"
CMD_TOPIC = "/cmd_vel"

sys.path.insert(0, str(WW / "tools"))
from assert_drive import gt_samples  # noqa: E402
from assert_tilt import imu_samples  # noqa: E402

V_FWD = 0.20        # 운영 속도 [m/s]
DRIVE_S = 13.0      # 주행 벽시계 (≈2.6m → 흙덩이 밭 통과)
WARMUP_S = 1.0      # 평지 안착 + 구독 연결

# 게이트 임계
UPRIGHT_DEG = 25.0     # 이보다 기울면 넘어짐 (DART 발산/전복)
Z_LO, Z_HI = -0.06, 0.30
FWD_MIN = 1.2          # 최소 전진 [m] (완주)
SHAKE_MIN_DEG = 2.0    # roll 또는 pitch peak-to-peak 최소 (실제로 흔들렸다는 증거)
IMU_TRACK_MAX = 1.5    # 시변 |IMU−GT| 중앙값 상한 [도]


class Fail(Exception):
    pass


def _stop(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, NameError):
        pass


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    iters = int((7 + DRIVE_S) * 1000)
    log = open("/tmp/ww_shake.log", "w")
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(iters), WORLD],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    gtsub = imusub = None
    try:
        deadline = time.time() + 20
        imu_topic = None
        while time.time() < deadline:
            topics = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            imu_topic = next((l.strip() for l in topics.splitlines() if "imu" in l.lower()), None)
            if GT_TOPIC in topics and imu_topic:
                break
            time.sleep(0.5)
        else:
            raise Fail(f"토픽 안 뜸 (gt={GT_TOPIC in topics}, imu={imu_topic}). /tmp/ww_shake.log")

        gf = open("/tmp/ww_shake_gt.log", "w")
        imf = open("/tmp/ww_shake_imu.log", "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        imusub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", imu_topic],
                                  stdout=imf, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(WARMUP_S)
        subprocess.run([ENV, "ign", "topic", "-t", CMD_TOPIC, "-m", "ignition.msgs.Twist",
                        "-p", f"linear: {{x: {V_FWD}}}, angular: {{z: 0.0}}"], capture_output=True, text=True)
        time.sleep(DRIVE_S)
        _stop(gtsub); _stop(imusub)
        gf.close(); imf.close()
        gtsub.wait(timeout=5); imusub.wait(timeout=5)
    finally:
        _stop(sim)
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()

    return (gt_samples(open("/tmp/ww_shake_gt.log").read()),
            imu_samples(open("/tmp/ww_shake_imu.log").read()), imu_topic)


def median(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2] if xs else 0.0


def main():
    print("=== Stage 5 Tier 2 Step A — 동적 요철 주행 소양 (헤드리스) ===\n")
    gt, imu, imu_topic = run()
    print(f"  IMU 토픽 : {imu_topic}")
    print(f"  수집     : GT {len(gt)} 샘플, IMU {len(imu)} 샘플")
    if len(gt) < 20 or len(imu) < 20:
        subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
        print("\nFAIL 샘플 부족 — 시뮬/토픽 실패", file=sys.stderr)
        sys.exit(1)

    # (1) 완주 + 안 넘어짐 + NaN 없음
    xs = [s[1] for s in gt]
    rolls = [math.degrees(s[4]) for s in gt]
    pitches = [math.degrees(s[5]) for s in gt]
    zs = [s[3] for s in gt]
    fwd = max(xs) - min(xs)
    max_tilt = max(max(abs(r) for r in rolls), max(abs(p) for p in pitches))
    z_min, z_max = min(zs), max(zs)
    nan = any(v != v for s in gt for v in s)

    # (2) 실제로 흔들림: 정상상태(전진 시작 후) roll/pitch peak-to-peak
    x0 = min(xs)
    moving = [s for s in gt if s[1] > x0 + 0.2]      # 출발 뒤
    r_mv = [math.degrees(s[4]) for s in moving]
    p_mv = [math.degrees(s[5]) for s in moving]
    roll_pp = (max(r_mv) - min(r_mv)) if r_mv else 0.0
    pitch_pp = (max(p_mv) - min(p_mv)) if p_mv else 0.0
    shake = max(roll_pp, pitch_pp)

    # (3) IMU 가 시변 자세를 GT 추적: 시각 최근접 정합 후 |Δ| 중앙값
    imu_sorted = sorted(imu, key=lambda s: s[0])
    dr, dp = [], []
    for s in moving:
        t = s[0]
        best = min(imu_sorted, key=lambda q: abs(q[0] - t))
        dr.append(abs(math.degrees(best[1]) - math.degrees(s[4])))
        dp.append(abs(math.degrees(best[2]) - math.degrees(s[5])))
    imu_dr, imu_dp = median(dr), median(dp)

    print(f"  (1) 완주/안정 : 전진 {fwd:.2f}m · 최대기울기 {max_tilt:.1f}° · z[{z_min:+.3f},{z_max:+.3f}] · NaN={nan}")
    print(f"  (2) 흔들림    : roll p2p {roll_pp:.1f}° · pitch p2p {pitch_pp:.1f}°  → shake {shake:.1f}°")
    print(f"  (3) IMU 추적  : |IMU−GT| 중앙값 roll {imu_dr:.2f}° · pitch {imu_dp:.2f}°")

    errs = []
    if fwd < FWD_MIN:
        errs.append(f"완주 못 함: 전진 {fwd:.2f}m < {FWD_MIN}m (흙덩이에 막힘?)")
    if max_tilt > UPRIGHT_DEG:
        errs.append(f"넘어짐/발산: 최대기울기 {max_tilt:.1f}° > {UPRIGHT_DEG}°")
    if not (Z_LO <= z_min and z_max <= Z_HI):
        errs.append(f"떴거나 파묻힘/발산: z[{z_min:+.3f},{z_max:+.3f}] ∉ [{Z_LO},{Z_HI}]")
    if nan:
        errs.append("NaN pose — DART 폭발")
    if shake < SHAKE_MIN_DEG:
        errs.append(f"안 흔들림: shake {shake:.1f}° < {SHAKE_MIN_DEG}° (요철이 너무 작음 → 보정할 게 없음)")
    if imu_dr > IMU_TRACK_MAX or imu_dp > IMU_TRACK_MAX:
        errs.append(f"IMU 가 GT 를 못 따라감: roll {imu_dr:.2f}° pitch {imu_dp:.2f}° > {IMU_TRACK_MAX}°")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if errs:
        print("\nFAIL 동적 요철 주행 소양 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print(f"\n=== OK 요철 밭을 완주하며 {shake:.1f}° 흔들리고, IMU 가 그 시변 자세를 {max(imu_dr,imu_dp):.2f}° 안으로 추적한다 ===")
    print("    DART 범프 안정 + 시변 자세 IMU 추적 확인 → Step B(주행 중 타격 보정)로 진행 가능.")


if __name__ == "__main__":
    main()
