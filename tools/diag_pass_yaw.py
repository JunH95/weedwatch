#!/usr/bin/env python3
"""두둑 패스 중 로봇이 왜 돌아가나 — yaw 표류를 타격과 나란히 놓고 본다.

사용자 실행에서 나온 증상: 두둑 0 을 직진으로 훑고 헤드랜드에 도착했는데 **yaw 가 +9.1°**,
y 가 1.207(두둑 중심 0.6). 직진 명령만 줬는데 로봇이 돌아간 것이다. 헤드리스 실행에서는
같은 지점에서 yaw 0.2° 였으므로, 매번 나는 게 아니라 **무언가에 걸릴 때** 난다.

가설: 도구가 내려간 동안(타격) 강체 막대가 두둑을 긁으며 비트는 토크를 준다. 실물이라면 흙이
부서지지만 시뮬의 두둑은 강체 상자다. 그렇다면 yaw 는 **타격 순간마다 계단식으로** 튈 것이고,
아무 일 없는 구간에서는 평평할 것이다. 아니라면 원인이 다른 데 있다.

그래서 지상진실 yaw 시계열과 도구 명령(내려감/올라감)을 같은 시간축에 찍는다.
제어는 평소대로 코디네이터가 하고, 여기서는 **보기만** 한다(GT 는 채점·진단 전용).

밭·옵션을 받는다:  FIELD=dev VO=true ./scripts/env.sh python3 tools/diag_pass_yaw.py

**지금 쓰는 이유**(2026-07-27): 현실 밭 관통이 U턴 진입에서 시간 초과로 멈췄다. 로봇이 자기
추정상 목표에 못 닿았는데, 두 가지가 구분이 안 된다 —
  ① 물리적으로 **끼였다**(흙덩이·두둑 모서리) → 지상진실도 안 움직인다
  ② 추정이 **과하게 깎였다**(VO 보정) → 지상진실은 가는데 추정만 안 는다
지상진실을 같이 녹화하면 한 줄로 갈린다.
"""
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
sys.path.insert(0, str(WW / "tools"))
from assert_drive import gt_samples  # noqa: E402

FIELD = os.environ.get("FIELD", "")
WORLD_NAME = f"field_{FIELD}" if FIELD else "robot_field_multi"
GT_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
GT_FILE = "/tmp/ww_passyaw_gt.log"
LAUNCH_LOG = "/tmp/ww_passyaw_launch.log"


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def main():
    print("=== 패스 중 yaw 표류 진단 (관통을 헤드리스로 돌리며 지상진실 관찰) ===")
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    subprocess.run(["pkill", "-f", "[r]os2 launch"], capture_output=True)
    time.sleep(1.0)

    args = (f"field:={FIELD} " if FIELD else "") + os.environ.get("LAUNCH_ARGS", "")
    launch = subprocess.Popen(
        [ENV, "bash", "-c",
         f"source {WW}/install/setup.bash && ros2 launch weedwatch_bringup skeleton.launch.py {args}"],
        stdout=open(LAUNCH_LOG, "w"), stderr=subprocess.STDOUT, start_new_session=True)
    gtsub = None
    try:
        # 시뮬이 뜨면 GT 구독 시작
        deadline = time.time() + 40
        while time.time() < deadline:
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if GT_TOPIC in t:
                break
            time.sleep(1.0)
        else:
            raise RuntimeError("시뮬 토픽이 안 떴습니다")
        gf = open(GT_FILE, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        print("    관통 실행 중 — 두둑 0 패스가 끝날 때까지 관찰(최대 5분)")
        deadline = time.time() + 300
        while time.time() < deadline:
            if launch.poll() is not None:
                break
            log = Path(LAUNCH_LOG).read_text(errors="ignore")
            if "시간 초과" in log or "관통 완료" in log or "U턴 E" in log:
                time.sleep(2)
                break
            time.sleep(2)
    finally:
        kill(gtsub)
        time.sleep(0.5)
        kill(launch)
        try:
            launch.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
        subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)

    gt = gt_samples(Path(GT_FILE).read_text(errors="ignore"))
    if len(gt) < 50:
        print(f"[실패] GT 샘플 부족({len(gt)})")
        return 2

    t0 = gt[0][0]
    print(f"\n    GT 샘플 {len(gt)}개 · {gt[-1][0]-t0:.1f}초\n")
    print(f"    {'t[s]':>6} {'x':>7} {'y':>7} {'yaw°':>7} {'Δyaw°/s':>8}")
    step = max(1, len(gt) // 30)
    prev = None
    worst = (0.0, None)
    for s in gt[::step]:
        t, x, y, yaw = s[0] - t0, s[1], s[2], math.degrees(s[6])
        rate = ""
        if prev:
            dt = t - prev[0]
            if dt > 0:
                r = (yaw - prev[1]) / dt
                rate = f"{r:+8.2f}"
                if abs(r) > abs(worst[0]):
                    worst = (r, t)
        print(f"    {t:6.1f} {x:7.3f} {y:7.3f} {yaw:7.2f} {rate:>8}")
        prev = (t, yaw)

    yaws = [math.degrees(s[6]) for s in gt]
    ys = [s[2] for s in gt]
    # U턴 진입 구간(=마지막 60초)에서 로봇이 **물리적으로** 움직였나
    tail = [g for g in gt if g[0] >= gt[-1][0] - 60]
    if len(tail) > 10:
        dx = max(t[1] for t in tail) - min(t[1] for t in tail)
        dy = max(t[2] for t in tail) - min(t[2] for t in tail)
        moved = math.hypot(dx, dy)
        print(f"\n    마지막 60초 지상진실 이동: {moved*100:.1f}cm "
              f"(x {min(t[1] for t in tail):.2f}~{max(t[1] for t in tail):.2f})")
        print("    ⟹ " + ("**로봇이 물리적으로 안 움직였다 = 끼임**" if moved < 0.05 else
                          "로봇은 움직였다 — 추정이 안 따라온 것(융합 과보정 의심)"))
    print(f"\n    yaw 총 변화 {yaws[-1]-yaws[0]:+.2f}° · 최대 회전율 {worst[0]:+.2f}°/s (t={worst[1]:.1f}s)")
    print(f"    y 이동 {ys[0]:.3f} → {ys[-1]:.3f} ({(ys[-1]-ys[0])*100:+.1f}cm)")

    # 타격(도구 하강) 시각을 런치 로그에서 못 얻으므로, 회전율이 큰 구간이 몇 군데인지로 성격을 본다
    rates = []
    for a, b in zip(gt, gt[1:]):
        dt = b[0] - a[0]
        if dt > 0:
            rates.append(abs(math.degrees(b[6] - a[6])) / dt)
    big = sum(1 for r in rates if r > 1.0)
    print(f"    회전율 >1°/s 인 샘플 {big}/{len(rates)}개 "
          f"({'계단식 — 특정 순간에 튄다' if big < len(rates) * 0.2 else '지속적 — 계속 돌고 있다'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
