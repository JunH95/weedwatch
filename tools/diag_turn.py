#!/usr/bin/env python3
"""걸터타기 재진입 허용오차 진단 — 자율주행 크럭스(두둑 끝 회전 후 재진입).

두둑 사이 이동은 지금 순간이동 치트(036). 진짜로 하려면: 로봇이 두둑을 벗어나 → 헤드랜드서 회전 →
다음 두둑에 다시 걸터탄다. 회전 자체는 됨(make drive 에서 +z→+yaw 검증). 미지수는 **재진입 정렬** —
yaw 가 조금 틀어진 채 두둑에 들어가면 바퀴가 두둑 옆면(25cm 릿지)에 끼는가(034: 여유 11cm/쪽).

각 yaw 오차로 두둑1(y=1.8)을 걸터탄 채 +x 주행시키고 GT(지상진실)로 잰다:
  · x 진행 — 끼임 없이 끝까지 갔나
  · y 표류 — 옆으로 얼마나 밀렸나 (yaw 오차 × 거리)
  · 자세 스파이크 — 바퀴가 릿지를 타면 roll/pitch 튄다
끼임 시작 yaw 를 찾으면 → 회전+정렬이 그보다 정밀해야 한다는 요구사항이 나온다.

실행:  ./scripts/env.sh python3 tools/diag_turn.py
"""
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
sys.path.insert(0, str(WW / "tools"))
from assert_drive import gt_samples  # noqa: E402

WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
WORLD_NAME, MODEL = "robot_field_multi", "weedwatch"
BED1_Y = 1.8            # 두둑1 중심 (걸터탈 대상)
X_START = -1.5          # 두둑(4m, x -2..2)의 -x 끝 근처에서 출발
V = 0.15
DRIVE_S = 12.0
YAW_ERRS = [0, 4, 8]   # 기준·임계근처·초과 (respawn 비용 줄임)


def _stop(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def set_pose(x, y, z, yaw):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "{MODEL}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{z: {qz:.4f}, w: {qw:.4f}}}')
    subprocess.run([ENV, "ign", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
                    "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "3000", "--req", req], capture_output=True, text=True)


def run(yaw_deg):
    gt_topic = f"/world/{WORLD_NAME}/dynamic_pose/info"
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    iters = int((8 + DRIVE_S) * 1000)
    log = open("/tmp/ww_turn.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(iters), WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    gsub = None
    try:
        deadline = time.time() + 20
        while time.time() < deadline:
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if gt_topic in t:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("토픽 안 뜸")
        # 두둑1 에 yaw 오차로 걸터타게 순간이동 → 정착
        set_pose(X_START, BED1_Y, 0.05, math.radians(yaw_deg))
        time.sleep(2.5)
        gf = open("/tmp/ww_turn_gt.log", "w")
        gsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", gt_topic],
                                stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(1.0)
        # +x 로 주행(로봇 heading = yaw_deg, 그 각도로 직진 → y 표류)
        subprocess.run([ENV, "ign", "topic", "-t", "/cmd_vel", "-m", "ignition.msgs.Twist",
                        "-p", f"linear: {{x: {V}}}, angular: {{z: 0.0}}"], capture_output=True)
        time.sleep(DRIVE_S)
        _stop(gsub); gf.close(); gsub.wait(timeout=5)
    finally:
        _stop(sim)
        try:
            sim.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return gt_samples(open("/tmp/ww_turn_gt.log").read())


def main():
    print("=== 걸터타기 재진입 허용오차 — yaw 오차별 (두둑1 위 +x 주행, GT) ===")
    print(f"    기대 이동 ≈ {V*DRIVE_S:.1f}m · 두둑 릿지 여유 ≈ 11cm/쪽 (034)\n")
    print(f"    {'yaw°':>5} {'x진행':>8} {'y표류':>8} {'roll_pp':>8} {'pitch_pp':>9}  판정")
    for yaw in YAW_ERRS:
        gt = run(yaw)
        if len(gt) < 10:
            print(f"    {yaw:>5} 샘플 부족({len(gt)})"); continue
        xs = [g[1] for g in gt]; ys = [g[2] for g in gt]
        rolls = [math.degrees(g[4]) for g in gt]; pitches = [math.degrees(g[5]) for g in gt]
        x_prog = max(xs) - X_START               # 얼마나 전진했나
        y_drift = max(abs(y - BED1_Y) for y in ys)
        roll_pp = max(rolls) - min(rolls); pitch_pp = max(pitches) - min(pitches)
        jammed = x_prog < 0.6 * V * DRIVE_S or roll_pp > 8 or pitch_pp > 8
        verdict = "끼임/실패" if jammed else "통과"
        print(f"    {yaw:>5} {x_prog:>7.2f}m {y_drift*100:>6.1f}cm {roll_pp:>7.1f}° {pitch_pp:>8.1f}°  {verdict}")


if __name__ == "__main__":
    main()
