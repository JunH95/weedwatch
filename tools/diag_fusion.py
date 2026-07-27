#!/usr/bin/env python3
"""융합 A/B — 시각 오도메트리를 넣으면 U턴 뒤 절대 위치 표류가 줄어드나.

DECISIONS 041 은 "회전 중 미끄러짐은 카메라만 본다"까지 재고 끝났다. 여기서는 그걸 실제 스택에
붙인 결과를 잰다: 같은 관통을 **vo:=false / vo:=true** 로 한 번씩 돌리고, 두둑1 재진입 시점의
|추정 − 지상진실| 을 비교한다.

  · 제어는 그대로 온보드 센서만 쓴다(지상진실은 채점 전용, 별도 프로세스로 받는다).
  · 비교 시점은 코디네이터가 "U턴 E 재진입" 을 찍는 순간 — U턴이 만든 오차가 다 쌓인 지점.

실행:  ./scripts/env.sh python3 tools/diag_fusion.py
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

WORLD_NAME = "robot_field_multi"
GT_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
EST_RE = re.compile(r"U턴 E 재진입: 추정 \(([-+0-9.]+), ([-+0-9.]+)\) yaw=([-+0-9.]+)")


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def run(vo: bool, timeout=600):
    tag = "vo_on" if vo else "vo_off"
    gt_file, log_file = f"/tmp/ww_fusion_{tag}_gt.log", f"/tmp/ww_fusion_{tag}.log"
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    subprocess.run(["pkill", "-f", "[r]os2 launch"], capture_output=True)
    time.sleep(1.5)
    launch = subprocess.Popen(
        [ENV, "bash", "-c",
         f"source {WW}/install/setup.bash && ros2 launch weedwatch_bringup skeleton.launch.py "
         f"vo:={'true' if vo else 'false'}"],
        stdout=open(log_file, "w"), stderr=subprocess.STDOUT, start_new_session=True)
    gtsub = None
    try:
        deadline = time.time() + 60
        while time.time() < deadline:
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if GT_TOPIC in t:
                break
            time.sleep(1.0)
        gf = open(gt_file, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if launch.poll() is not None:
                break
            if "U턴 E 재진입" in Path(log_file).read_text(errors="ignore"):
                time.sleep(1.5)
                break
            time.sleep(2)
    finally:
        kill(gtsub); time.sleep(0.5); kill(launch)
        try:
            launch.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
        subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)

    log = Path(log_file).read_text(errors="ignore")
    m = EST_RE.search(log)
    gt = gt_samples(Path(gt_file).read_text(errors="ignore"))
    if not m or len(gt) < 20:
        return None
    est = (float(m.group(1)), float(m.group(2)), math.radians(float(m.group(3))))
    truth = (gt[-1][1], gt[-1][2], gt[-1][6])
    vo_lines = log.count("ww_vo")
    return {"est": est, "truth": truth, "vo_node": vo_lines > 0,
            "d": math.hypot(est[0] - truth[0], est[1] - truth[1])}


def main():
    print("=== 융합 A/B — VO 를 넣으면 U턴 뒤 절대 위치 표류가 줄어드나 ===")
    print("    제어=온보드 센서 · 채점=지상진실(별도 프로세스) · 비교 시점=두둑1 재진입\n")
    only = sys.argv[1] if len(sys.argv) > 1 else None     # "on" / "off" 만 돌리기 (재측정 절약)
    out = {}
    for vo in (False, True):
        if only == "on" and not vo or only == "off" and vo:
            continue
        tag = "VO 켬 " if vo else "VO 끔 "
        print(f"    [{tag}] 관통 실행 중...", flush=True)
        r = run(vo)
        out[vo] = r
        if r is None:
            print(f"    [{tag}] 재진입 지점을 못 잡음 (로그 확인)")
            continue
        e, t = r["est"], r["truth"]
        print(f"    [{tag}] 추정 ({e[0]:+.3f},{e[1]:+.3f}) vs 지상진실 ({t[0]:+.3f},{t[1]:+.3f}) "
              f"→ 표류 {r['d']*100:.1f}cm")

    a, b = out.get(False), out.get(True)
    if a and b:
        print(f"\n    표류: VO 끔 {a['d']*100:.1f}cm → VO 켬 {b['d']*100:.1f}cm "
              f"({(1 - b['d']/a['d'])*100:+.0f}%)")
        print("    ⟹ " + ("VO 가 표류를 줄인다 — 융합 채택." if b["d"] < a["d"] * 0.85 else
                          "차이가 뚜렷하지 않다 — 게이팅 조건·VO 품질을 다시 봐야 한다."))
    return 0


if __name__ == "__main__":
    sys.exit(main())
