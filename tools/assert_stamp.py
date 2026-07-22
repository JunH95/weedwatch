#!/usr/bin/env python3
"""스탬핑 단언 - 도구 끝이 잡초 위에 정밀하게 서는가 (Tier 2, 렌더 없음, Stage 4-2).

인식(Stage 3)과 정밀 위치(Stage 2)를 합친 검증. 두둑 위 알려진 위치의 잡초마다 캐리지(Y)와
도구(Z)를 명령하고, 도구 끝의 물리적 위치가 잡초 좌표 위 2cm 안에 오는지 본다 (DECISIONS 002:
"|도구 위치 - 잡초 좌표| < 2cm").

도구 끝 위치는 명령이 아니라 실측으로 구한다: 지상진실 base pose + 실제 joint_state(achieved)를
정방향기구학에 넣는다. joint_state 는 명령이 아니라 sim 이 보고하는 achieved 라, PID 가 실제로
도달한 위치다(make joints 가 mm 정밀 확인). base pose 는 물리 지상진실. 둘 다 실측이므로
"명령대로 갔다고 가정"이 아니라 "실제로 그 위치에 갔나"를 잰다.

멀티툴(DECISIONS 020): 툴 N개가 X 로 엇갈려(tool_xs) 각자 두둑 폭 1/N 밴드를 맡는다. 잡초마다
담당 툴(band_of)로 라우팅해 그 캐리지·도구만 명령한다. 정적이라 각 잡초는 담당 툴의 X 선에 있다.

기구학 (툴 i, URDF weedwatch.urdf, links.json):
  carriage{i}_joint : Y 축, 원점 y=band_center[i], 한계 ±tool_band_half(0.15)
  tool{i}_joint     : Z 축, [-0.35, 0], tool 충돌 실린더 length 0.1925 → 끝은 링크서 Z -0.09625
  ⟹ 도구끝 = base + (tool_xs[i], band_center[i] + carriage_pos, 0.3075 + tool_pos)  (정자세, 회전 무시)

실행:  ./scripts/env.sh python3 tools/assert_stamp.py   (make stamp)
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
WORLD = str(WW / "worlds" / "robot_stamp.sdf")
MODEL = "weedwatch"
GT_TOPIC = "/world/robot_stamp/dynamic_pose/info"
JOINT_TOPIC = "/world/robot_stamp/model/weedwatch/joint_state"

sys.path.insert(0, str(WW / "tools"))
from assert_drive import g, parse_messages  # noqa: E402
from assert_joints import read_joint  # noqa: E402  (joint_state 파일 tail 파서 재사용)
from garden_geometry import Garden, Portal  # noqa: E402

_G, _P = Garden(), Portal()
TOOL_XS = _P.tool_xs()                       # [-0.09, -0.19, -0.29]
BAND_CENTERS = _P.tool_band_centers(_G)      # [-0.30, 0.0, +0.30] (로봇 중심 기준)

# 잡초 좌표 - worlds/robot_stamp.sdf 의 weed_* 마커와 일치. 각 잡초는 담당 툴의 X 선에 있다.
# X 는 실제 tool_xs (stagger 0.18 → -0.09 / -0.27 / -0.45). Y 는 각 툴 밴드 안 2개.
WEEDS = [(TOOL_XS[0], 0.25), (TOOL_XS[0], 0.40),
         (TOOL_XS[1], 0.55), (TOOL_XS[1], 0.68),
         (TOOL_XS[2], 0.82), (TOOL_XS[2], 0.98)]
BASE_Y = 0.60          # 로봇 spawn y (두둑 중심). 담당 툴 carriage 명령 = (y-BASE_Y) - band_center[i]
STRIKE = -0.15         # 도구 하강 명령 (두둑이 충돌로 멈춤 → achieved 는 두둑 윗면에서 정지)
RAISE = 0.0            # 잡초 간 이동 시 도구 올림 (두둑을 긁지 않게)
TIP_DZ = 0.3075        # base 기준 도구 끝 Z (tool_pos=0 일 때). tip_z = base_z + TIP_DZ + tool_pos
TOL_XY = 0.02          # 성공 기준 2cm (DECISIONS 002)
SETTLE = 2.5


def weed_tool(wy: float) -> int:
    """잡초의 월드 y → 담당 툴 인덱스 (로봇 중심 기준 밴드)."""
    return _P.band_of(_G, wy - BASE_Y)

JSTATE_FILE = "/tmp/ww_jstate.log"   # assert_joints.read_joint 이 이 경로를 읽는다(재사용)
GT_FILE = "/tmp/ww_stamp_gt.log"


class Fail(Exception):
    pass


def publish(topic: str, value: float):
    subprocess.run([ENV, "ign", "topic", "-t", topic, "-m", "ignition.msgs.Double",
                    "-p", f"data: {value}"], capture_output=True, text=True)


def read_gt_poses():
    """dynamic_pose/info 파일 끝에서 마지막 완결 메시지의 {이름: pose} 를 읽는다."""
    try:
        with open(GT_FILE, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 16384))
            tail = f.read().decode("utf-8", "ignore")
    except FileNotFoundError:
        return {}
    for m in reversed(parse_messages(tail)):
        poses = m.get("pose")
        if poses is None:
            continue
        if isinstance(poses, dict):
            poses = [poses]
        named = {p.get("name"): p for p in poses if isinstance(p, dict) and "name" in p}
        if named:
            return named
    return {}


def base_xyz():
    poses = read_gt_poses()
    b = poses.get(MODEL)
    if b is None:
        return None
    return (g(b, "position", "x"), g(b, "position", "y"), g(b, "position", "z"))


def stop(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, NameError):
        pass


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    # 넉넉히. 퍼블리시(ign topic)마다 subprocess 오버헤드가 벽시계에 붙어 sim 시간을 넘기면
    # 마지막 잡초 전에 iterations 가 소진된다 → 여유를 크게 준다.
    total_iters = int((20 + len(WEEDS) * 10) * 1000)
    log = open("/tmp/ww_stamp.log", "w")
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(total_iters), WORLD],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    results = []
    jsub = gtsub = None
    try:
        deadline = time.time() + 15
        while time.time() < deadline:
            topics = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if JOINT_TOPIC in topics and GT_TOPIC in topics:
                break
            time.sleep(0.5)
        else:
            raise Fail("joint_state/dynamic_pose 토픽이 안 떴습니다 - 시뮬 초기화 실패")

        jf = open(JSTATE_FILE, "w")
        gf = open(GT_FILE, "w")
        jsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", JOINT_TOPIC],
                                stdout=jf, stderr=subprocess.DEVNULL, start_new_session=True)
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(2.5)  # 로봇 안착 + 구독자 연결

        rest = base_xyz()
        for wx, wy in WEEDS:
            i = weed_tool(wy)                            # 담당 툴 인덱스
            cmd_y = (wy - BASE_Y) - BAND_CENTERS[i]      # 밴드 중심 기준 캐리지 명령
            publish(f"/tool{i}_cmd", RAISE)              # 담당 툴 올림
            time.sleep(1.2)
            publish(f"/carriage{i}_cmd", cmd_y)          # 담당 캐리지를 잡초 Y 로
            time.sleep(SETTLE)
            publish(f"/tool{i}_cmd", STRIKE)             # 타격 하강 (두둑이 멈춤)
            time.sleep(SETTLE)
            cpos = read_joint(f"carriage{i}_joint")
            tpos = read_joint(f"tool{i}_joint")
            base = base_xyz()
            results.append((wx, wy, i, cpos, tpos, base, rest))
    finally:
        stop(jsub); stop(gtsub)
        try:
            jf.close(); gf.close()
        except NameError:
            pass
        stop(sim)
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return results


def main():
    print("=== 스탬핑 단언 (헤드리스, GPU 불필요) ===\n")
    print(f"── 두둑 위 잡초 {len(WEEDS)}개에 도구 끝을 얹는다. 성공기준 |도구-잡초| < {TOL_XY*100:.0f}cm ──")
    results = run()
    if any(base is None for *_, base, _ in results) or \
       any(c is None or t is None for _, _, _, c, t, *_ in results):
        subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
        raise Fail("base pose 또는 joint_state 를 못 읽음 - 시뮬/토픽 확인")

    errs = []
    for wx, wy, i, cpos, tpos, base, rest in results:
        bx, by, bz = base
        tip_x = bx + TOOL_XS[i]                       # 담당 툴의 X (엇갈림)
        tip_y = by + BAND_CENTERS[i] + cpos           # 밴드 중심 + 캐리지 achieved
        tip_z = bz + TIP_DZ + tpos
        dxy = math.hypot(tip_x - wx, tip_y - wy)
        descended = (rest[2] + TIP_DZ) - tip_z if rest else 0.0   # 안정: 정지 대비 하강량
        ok = dxy <= TOL_XY and tip_z <= 0.30
        mark = "OK" if ok else "FAIL"
        print(f"  {mark} 잡초 t{i} ({wx:+.2f},{wy:.2f}): 도구끝=({tip_x:+.3f},{tip_y:+.3f},{tip_z:.3f}) "
              f"오차={dxy*100:5.2f}cm  하강={descended*100:4.1f}cm  (carriage={cpos:+.3f} tool={tpos:+.3f})")
        if dxy > TOL_XY:
            errs.append(f"잡초 ({wx:+.2f},{wy:.2f}): 수평오차 {dxy*100:.2f}cm > {TOL_XY*100:.0f}cm")
        if tip_z > 0.30:
            errs.append(f"잡초 ({wx:+.2f},{wy:.2f}): 도구가 안 내려옴 (끝 z={tip_z:.3f} > 0.30)")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if errs:
        print("\nFAIL 스탬핑 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print(f"\n=== OK 스탬핑 통과 - 도구 끝이 잡초 {len(WEEDS)}개 위에 {TOL_XY*100:.0f}cm 안으로 섰다 ===")


if __name__ == "__main__":
    main()
