#!/usr/bin/env python3
"""Y/Z 관절 위치 제어 단언 — 캐리지·도구가 명령 위치에 정밀 도달하는가 (Tier 2).

── 왜 이게 프로젝트의 핵심인가 ─────────────────────────────────────────────
성공 기준이 "잡초 위에 ±2cm 로 정밀하게 선다" 인데(docs/DECISIONS.md 006), 비홀로노믹
차체를 ±2cm 로 세우는 건 어려운 문제다. 대신 **프리즘 관절이 mm 를 공짜로 준다** — 차체는
대충 고랑에 세우고, 캐리지(Y)와 도구(Z)가 정밀 위치를 담당한다. 그 mm 정밀도가 실제로
나오는지가 여기서 증명된다. 없으면 성공 기준을 달성할 물리적 수단이 없다.

멀티툴(DECISIONS 020): N개 툴이 각자 독립 캐리지(Y)+도구(Z). 각 툴이 자기 밴드(90/N cm)를
짧게 훑는다. 여기선 툴마다 캐리지를 밴드 안 임의값으로, 도구를 하강/복귀시켜 독립 도달을 단언.

  carriage{i}_joint : prismatic Y (±tool_band_half=0.15) — 툴 i 의 밴드 좌우 정렬
  tool{i}_joint     : prismatic Z (-0.35~0) — 툴 i 점 타격 막대 하강(음수)/접힘(0)

명령 토픽(ignition.msgs.Double, m):  /carriage{i}_cmd  /tool{i}_cmd
관절 상태 읽기: /world/robot_drive/model/weedwatch/joint_state

주의: "잡초가 죽었나" 는 시뮬 못 한다(연성체 물리 없음). 여기서 재는 건 "막대가 정확히
그 위치·깊이에 갔나" 이지 "죽었나" 가 아니다. (docs/DECISIONS.md 002)

실행:  ./scripts/env.sh python3 tools/assert_joints.py   (또는 make joints)
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_drive.sdf")
JOINT_TOPIC = "/world/robot_drive/model/weedwatch/joint_state"

sys.path.insert(0, str(WW / "tools"))
from assert_drive import g, parse_messages  # noqa: E402  (텍스트 protobuf 파서 재사용)
from garden_geometry import Portal  # noqa: E402

N_TOOLS = Portal().n_tools

SETTLE = 3.0        # 명령 후 PID 정착 대기 [s]
TOL_Y = 0.003       # 캐리지 허용오차 3mm (수평, 중력 무관)
TOL_Z = 0.005       # 도구 허용오차 5mm (중력을 i 게인으로 이겨야 함)
STABLE = 0.0015     # 정착 판정: 0.4s 간격 두 샘플 차이 [m]


JSTATE_FILE = "/tmp/ww_jstate.log"


def read_joint(joint: str):
    """백그라운드 캡처 파일의 마지막 부분에서 해당 관절 position 을 읽는다.

    joint_state 는 초당 수천 건이라 파일이 거대하다 → 끝 8KB 만 읽어 마지막 완결
    메시지를 파싱한다. (`ign topic -e -n 1` 로 매번 새 구독하면 discovery 지연으로
    타임아웃한다 — 그래서 assert_drive 처럼 상시 구독자를 하나 띄워 파일로 받는다.)
    """
    try:
        with open(JSTATE_FILE, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 8192))
            tail = f.read().decode("utf-8", "ignore")
    except FileNotFoundError:
        return None
    for m in reversed(parse_messages(tail)):
        joints = m.get("joint")
        if joints is None:
            continue
        if isinstance(joints, dict):
            joints = [joints]
        for j in joints:
            if j.get("name") == joint:
                return g(j, "axis1", "position")
    return None


def publish(topic: str, value: float):
    subprocess.run([ENV, "ign", "topic", "-t", topic, "-m", "ignition.msgs.Double",
                    "-p", f"data: {value}"], capture_output=True, text=True)


# 시험 순서: (명령토픽, 관절이름, 목표[m], 허용오차, 설명)
# 멀티툴: N개 툴 각각을 독립 시험. 캐리지 목표는 밴드 반폭(±0.15) 안 임의값(방향 번갈아 → 양방향
# 다 커버), 도구는 하강(중력 이겨내기) 후 복귀. 조인트 값은 밴드 중심 기준 상대(0=중심, 0=접힘).
_CARR_TARGETS = [+0.123, -0.123, +0.100]  # 툴별 임의값 (mm 정밀). ±0.15 밴드 안.
STEPS = []
for _i in range(N_TOOLS):
    _ct = _CARR_TARGETS[_i % len(_CARR_TARGETS)]
    STEPS += [
        (f"/carriage{_i}_cmd", f"carriage{_i}_joint", _ct, TOL_Y, f"툴{_i} 캐리지 {_ct:+.3f} (밴드 훑기)"),
        (f"/tool{_i}_cmd", f"tool{_i}_joint", -0.20, TOL_Z, f"툴{_i} 도구 하강 -0.20 (중력 이겨내기)"),
        (f"/tool{_i}_cmd", f"tool{_i}_joint", 0.0, TOL_Z, f"툴{_i} 도구 접힘 0.0 (복귀)"),
    ]


class Fail(Exception):
    pass


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    total_iters = int((6 + len(STEPS) * (SETTLE + 1.5)) * 1000)
    log = open("/tmp/ww_joints.log", "w")
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(total_iters), WORLD],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )

    def stop(p):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    results = []
    try:
        # joint_state 토픽이 뜰 때까지
        deadline = time.time() + 15
        while time.time() < deadline:
            if JOINT_TOPIC in subprocess.run([ENV, "ign", "topic", "-l"],
                                             capture_output=True, text=True).stdout:
                break
            time.sleep(0.5)
        else:
            raise Fail("joint_state 토픽이 안 떴습니다 — 시뮬 초기화 실패")

        # 상시 joint_state 구독자 (파일로 캡처). read_joint 이 이 파일 끝을 읽는다.
        jf = open(JSTATE_FILE, "w")
        jsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", JOINT_TOPIC],
                                stdout=jf, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(2.0)  # 로봇 안착 (관절 0) + 구독자 연결

        for topic, joint, target, tol, desc in STEPS:
            publish(topic, target)
            time.sleep(SETTLE)
            p1 = read_joint(joint)
            time.sleep(0.4)
            p2 = read_joint(joint)
            results.append((desc, joint, target, tol, p1, p2))
    finally:
        try:
            stop(jsub)
            jf.close()
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
    print("=== Y/Z 관절 위치 제어 단언 (헤드리스, GPU 불필요) ===\n")
    results = run()
    errs = []
    for desc, joint, target, tol, p1, p2 in results:
        if p1 is None or p2 is None:
            print(f"  FAIL {desc}: 관절 상태를 못 읽음")
            errs.append(f"{desc}: joint_state 읽기 실패")
            continue
        err = abs(p1 - target)
        moving = abs(p2 - p1)
        ok = err <= tol and moving <= STABLE
        mark = "OK" if ok else "FAIL"
        print(f"  {mark} {desc:28s} 목표={target:+.3f}  도달={p1:+.4f}  "
              f"오차={err*1000:5.2f}mm  잔진동={moving*1000:4.2f}mm")
        if err > tol:
            errs.append(f"{desc}: 오차 {err*1000:.2f}mm > {tol*1000:.1f}mm")
        if moving > STABLE:
            errs.append(f"{desc}: 안 정착(잔진동 {moving*1000:.2f}mm) — 게인 튜닝 필요")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if errs:
        print("\nFAIL 관절 단언 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print("\n=== OK 관절 단언 통과 — 캐리지·도구가 명령 위치에 mm 정밀 도달 ===")


if __name__ == "__main__":
    main()
