#!/usr/bin/env python3
"""바퀴 드리프트 진단 — odom 이 GT 대비 얼마나 과보고하나, 그리고 그 원인이 무엇인가.

어제(029) 경사+요철에서 odom 이 GT 의 3배를 보고했다(1.8m 미끄러짐). 그런데 그게 두 가지 중
무엇인지 안 갈랐다:
  (A) 견인 슬립 — 바퀴가 땅을 딛고 있는데 마찰이 부족해 미끄러진다 (경사에서 심함)
  (B) 공중 헛돎 — 강체 바퀴가 흙덩이를 넘으며 지면을 떠서, DiffDrive 가 계속 돌리니 odom 만 는다

원인이 다르면 처방이 다르다: (A)면 마찰·wheel-slip 모델, (B)면 서스펜션/컴플라이언스 문제(또는
심-리얼 갭). 추측 말고 조건을 분리해 잰다 — 경사만 / 흙덩이만 / 둘 다.

각 조건에서 로봇을 +x 로 일정 시간 몰고, GT 이동거리 vs odom 이동거리 vs 슬립률을 보고한다.
슬립률 = (odom_dist − gt_dist) / odom_dist  (바퀴는 odom 만큼 돌았다고 믿는데 실제론 gt 만 갔다)

실행:  ./scripts/env.sh python3 tools/diag_slip.py <world_file> <world_name>
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
sys.path.insert(0, str(WW / "tools"))
from assert_drive import odom_samples, gt_samples  # noqa: E402

V = 0.20
DRIVE_S = 10.0


def _stop(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def run(world_file, world_name, robot_y):
    gt_topic = f"/world/{world_name}/dynamic_pose/info"
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    iters = int((8 + DRIVE_S) * 1000)
    log = open("/tmp/ww_slip.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(iters), world_file],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    osub = gsub = None
    try:
        deadline = time.time() + 20
        while time.time() < deadline:
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if "/odometry" in t and gt_topic in t:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("토픽 안 뜸")
        of = open("/tmp/ww_slip_odom.log", "w")
        gf = open("/tmp/ww_slip_gt.log", "w")
        osub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", "/odometry"],
                                stdout=of, stderr=subprocess.DEVNULL, start_new_session=True)
        gsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", gt_topic],
                                stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(2.0)
        subprocess.run([ENV, "ign", "topic", "-t", "/cmd_vel", "-m", "ignition.msgs.Twist",
                        "-p", f"linear: {{x: {V}}}, angular: {{z: 0.0}}"], capture_output=True)
        time.sleep(DRIVE_S)
        _stop(osub); _stop(gsub); of.close(); gf.close()
        osub.wait(timeout=5); gsub.wait(timeout=5)
    finally:
        _stop(sim)
        try:
            sim.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()

    odom = odom_samples(open("/tmp/ww_slip_odom.log").read())
    gt = gt_samples(open("/tmp/ww_slip_gt.log").read())
    return odom, gt


def main():
    world_file, world_name = sys.argv[1], sys.argv[2]
    robot_y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    odom, gt = run(world_file, world_name, robot_y)
    if len(odom) < 10 or len(gt) < 10:
        print(f"FAIL 샘플 부족 (odom {len(odom)} gt {len(gt)})", file=sys.stderr)
        sys.exit(1)

    # 이동 시작 후 구간만
    ox0 = odom[0][1]
    odom_dx = odom[-1][1] - odom[0][1]
    gx = [g[1] for g in gt]
    gt_dx = max(gx) - min(gx)
    # 공중 헛돎의 지표: GT z 진동폭 (바퀴가 뜨면 몸이 오르내린다)
    gz = [g[3] for g in gt]
    z_p2p = max(gz) - min(gz)
    rolls = [math.degrees(g[4]) for g in gt]
    pitches = [math.degrees(g[5]) for g in gt]
    roll_pp = max(rolls) - min(rolls)
    pitch_pp = max(pitches) - min(pitches)
    slip = (odom_dx - gt_dx) / odom_dx if odom_dx > 0.01 else float("nan")

    print(f"  {world_name}:")
    print(f"    odom 이동 {odom_dx:+.3f}m · GT 이동 {gt_dx:+.3f}m · 슬립률 {slip*100:5.1f}%")
    print(f"    자세 진동 — z p2p {z_p2p*100:.1f}cm · roll {roll_pp:.1f}° · pitch {pitch_pp:.1f}°")


if __name__ == "__main__":
    main()
