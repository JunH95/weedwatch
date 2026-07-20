#!/usr/bin/env python3
"""두둑 걸터타고 주행 단언 — 포탈 설계가 물리로 성립하는가 (Tier 2, 렌더 없음).

── 무엇을 증명하나 (DECISIONS 006) ────────────────────────────────────────
포탈형의 핵심 주장: "바퀴 두 개가 서로 다른 두 고랑에 있고, 몸통이 그 사이 두둑(90cm)을
건너지른다. 두둑 위로는 절대 안 올라간다." 이게 산수로는 맞지만(트랙 120 = 두둑90+고랑30),
실제로 그 자세로 세워 주행하면 바퀴가 고랑에 머무는지, 몸통이 두둑에 안 걸리는지,
안 넘어지는지는 물리로 확인해야 한다. Stage 1 의 "두둑을 만들 수 있는가" 위험의 완결편.

로봇을 두둑 중심(y=+0.6)에 세우고 두둑 길이 방향(+x)으로 몰아서:
  1. 앞으로 갔나 (명령 속도 달성)
  2. 걸터탄 채였나 — y 가 +0.6 근처 유지 (두둑 쪽으로 안 밀리고 고랑 밖으로 안 벗어남)
  3. 바퀴가 두둑에 안 올라탔나 — z 가 고랑 바닥 높이 유지 (올라탔으면 z 상승 + 기울어짐)
  4. 안 넘어졌나 — roll/pitch 작음

실행:  ./scripts/env.sh python3 tools/assert_straddle.py   (또는 make straddle)
"""
from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from assert_drive import Fail, _steady_window, median, run_maneuver  # noqa: E402

STRADDLE_WORLD = str(WW / "worlds" / "robot_straddle.sdf")
STRADDLE_GT = "/world/robot_straddle/dynamic_pose/info"
V = 0.20          # 전진 속도 [m/s] (두둑 위라 조금 천천히)
SPAWN_Y = 0.60    # 두둑 중심 = 걸터탄 자세의 로봇 y
DRIVE_S = 8


def main():
    print("=== 두둑 걸터타고 주행 단언 (헤드리스, GPU 불필요) ===\n")
    print(f"── 두둑 중심 y={SPAWN_Y} 에 걸터타고 +x 로 {V} m/s 주행 ──")
    odom, gt = run_maneuver(f"linear: {{x: {V}}}, angular: {{z: 0.0}}",
                            drive_seconds=DRIVE_S, world=STRADDLE_WORLD, gt_topic=STRADDLE_GT)
    print(f"  수집: odom {len(odom)} 샘플, 지상진실 {len(gt)} 샘플")
    if len(odom) < 20 or len(gt) < 20:
        raise Fail("샘플이 너무 적습니다 — 시뮬이 제대로 안 돌았을 수 있음")

    _, steady_start = _steady_window(odom, 4, 0.5 * V)          # idx4 = vx
    v_gt_v = median([s[4] for s in odom if s[0] >= steady_start])

    gt_steady = [s for s in gt if s[0] >= steady_start]
    if len(gt_steady) < 5:
        raise Fail("정상상태 지상진실 샘플 부족")
    a, b = gt_steady[0], gt_steady[-1]
    dt, dx = b[0] - a[0], b[1] - a[1]
    if dt < 1.0:
        raise Fail(f"정상상태 구간이 너무 짧습니다 ({dt:.2f}s)")
    v_gt = dx / dt

    # 걸터탄 상태 유지: 전 구간에서 y 가 SPAWN_Y 근처, z 가 고랑 바닥 근처인가
    y_dev_max = max(abs(s[2] - SPAWN_Y) for s in gt_steady)     # idx2 = y
    z_max = max(s[3] for s in gt_steady)                        # idx3 = z
    roll_max = max(abs(s[4]) for s in gt_steady)                # idx4 = roll
    pitch_max = max(abs(s[5]) for s in gt_steady)               # idx5 = pitch

    print(f"  전진        : Δx={dx:+.3f}m / {dt:.2f}s = {v_gt:+.3f} m/s (명령 {V}, odom {v_gt_v:+.3f})")
    print(f"  걸터탐 유지  : y 편차 최대 {y_dev_max*100:.1f}cm (두둑중심 {SPAWN_Y} 기준)")
    print(f"  두둑 안 올라탐: z 최대 {z_max*100:+.1f}cm (두둑 윗면은 +25cm — 여기 오르면 실패)")
    print(f"  안 넘어짐    : roll≤{math.degrees(roll_max):.1f}° pitch≤{math.degrees(pitch_max):.1f}°")

    errs = []
    if not (0.80 * V <= v_gt <= 1.15 * V):
        errs.append(f"전진 실패: 지상진실 {v_gt:.3f} m/s (명령 {V})")
    if y_dev_max > 0.10:
        errs.append(f"걸터탐 이탈: y 편차 {y_dev_max*100:.1f}cm > 10cm (두둑으로 밀렸거나 고랑 벗어남)")
    if z_max > 0.10:
        errs.append(f"두둑에 올라탐: z {z_max*100:.1f}cm > 10cm (바퀴가 두둑 위로)")
    if roll_max > 0.14 or pitch_max > 0.14:
        errs.append(f"넘어짐: roll={math.degrees(roll_max):.1f}° pitch={math.degrees(pitch_max):.1f}°")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if errs:
        print("\n❌ 걸터타기 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print("\n=== ✅ 두둑 걸터타고 주행 통과 — 바퀴는 고랑, 몸통은 두둑 위. 포탈 성립. ===")


if __name__ == "__main__":
    main()
