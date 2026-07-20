#!/usr/bin/env python3
"""diff-drive 주행 단언 — cmd_vel 로 로봇이 실제로 움직이는가 (Tier 2, 렌더 없음).

── 왜 게이트가 둘인가 (프로젝트 철학) ──────────────────────────────────────
smoke 테스트가 "사진 나왔나 AND NVIDIA가 그렸나" 두 게이트를 갖듯, 주행도 둘이다:
  게이트 A: DiffDrive 플러그인이 명령한 속도를 **보고**하는가 (/odometry)
  게이트 B: 몸통이 **물리적으로** 그만큼 움직였는가 (/world/.../dynamic_pose/info 지상진실)
A만 보면 거짓 통과한다 — 바퀴가 접지 없이 헛돌면 오도메트리는 전진을 보고하지만
로봇은 제자리다. 지상진실이 그걸 잡는다.

── 검증 항목 ───────────────────────────────────────────────────────────
1. 전진: linear.x 명령 → 명령 속도 달성 + 몸통이 그만큼 전진 + 직진 + 안 넘어짐
   + 오도메트리와 지상진실이 일치 (바퀴 반지름·간격이 맞다는 증거)
2. 회전: angular.z>0 (CCW) 명령 → 실제로 +yaw 로 돈다
   (좌/우 바퀴 배정이 맞다는 증거. 메시 이름이 ROS 규약과 반대라 틀리기 쉬운 곳)

실행:  ./scripts/env.sh python3 tools/assert_drive.py   (또는 make drive)
"""
from __future__ import annotations

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
WORLD = str(WW / "worlds" / "robot_drive.sdf")

CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odometry"
GT_TOPIC = "/world/robot_drive/dynamic_pose/info"
MODEL = "weedwatch"

V_FWD = 0.30       # 전진 명령 속도 [m/s]
W_TURN = 0.40      # 회전 명령 각속도 [rad/s], +z = CCW
DRIVE_SECONDS = 8  # 명령 유지 벽시계 시간


# ── 텍스트 protobuf 파서 (ign topic -e 출력) ────────────────────────────────
# ign 은 메시지를 빈 줄로 구분하고, 0인 필드는 생략한다. 중첩을 스택으로 따라간다.
# 반복 키(Pose_V 의 여러 pose{})는 리스트가 된다.

_OPEN = re.compile(r"^(\w+)\s*\{$")
_LEAF = re.compile(r"^(\w+):\s*(.+)$")


def _coerce(s: str):
    s = s.strip()
    if len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        return s[1:-1]
    try:
        return float(s)
    except ValueError:
        return s


def _add(d: dict, k: str, v):
    if k in d:
        if not isinstance(d[k], list):
            d[k] = [d[k]]
        d[k].append(v)
    else:
        d[k] = v


def parse_messages(text: str) -> list[dict]:
    """빈 줄로 구분된 텍스트 protobuf 스트림을 메시지별 중첩 dict 리스트로."""
    msgs: list[dict] = []
    root: dict = {}
    stack: list[dict] = [root]
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            if len(stack) == 1 and root:
                msgs.append(root)
                root = {}
                stack = [root]
            continue
        m = _OPEN.match(line)
        if m:
            child: dict = {}
            _add(stack[-1], m.group(1), child)
            stack.append(child)
            continue
        if line == "}":
            if len(stack) > 1:
                stack.pop()
            continue
        m = _LEAF.match(line)
        if m:
            _add(stack[-1], m.group(1), _coerce(m.group(2)))
    if len(stack) == 1 and root:
        msgs.append(root)
    return msgs


def g(d: dict, *path, default=0.0):
    """중첩 dict 안전 조회. 생략된(0인) 필드는 default."""
    cur = d
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur if not isinstance(cur, dict) else default


def stamp_s(msg: dict) -> float:
    return g(msg, "header", "stamp", "sec") + g(msg, "header", "stamp", "nsec") * 1e-9


def quat_to_rpy(x, y, z, w):
    """쿼터니언 → (roll, pitch, yaw) [rad]."""
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sp)))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# ── 오도메트리 / 지상진실 샘플 추출 ─────────────────────────────────────────


def odom_samples(text: str):
    """[(t, x, y, yaw, vx, wz)] — DiffDrive 가 보고하는 상태."""
    out = []
    for m in parse_messages(text):
        if "pose" not in m or "twist" not in m:
            continue
        t = stamp_s(m)
        x = g(m, "pose", "position", "x")
        y = g(m, "pose", "position", "y")
        _, _, yaw = quat_to_rpy(
            g(m, "pose", "orientation", "x"), g(m, "pose", "orientation", "y"),
            g(m, "pose", "orientation", "z"), g(m, "pose", "orientation", "w", default=1.0),
        )
        vx = g(m, "twist", "linear", "x")
        wz = g(m, "twist", "angular", "z")
        out.append((t, x, y, yaw, vx, wz))
    return out


def gt_samples(text: str):
    """[(t, x, y, z, roll, pitch, yaw)] — 모델의 물리적 지상진실 pose."""
    out = []
    for m in parse_messages(text):
        poses = m.get("pose")
        if poses is None:
            continue
        if isinstance(poses, dict):
            poses = [poses]
        me = next((p for p in poses if p.get("name") == MODEL), None)
        if me is None:
            continue
        t = stamp_s(m)
        x = g(me, "position", "x")
        y = g(me, "position", "y")
        z = g(me, "position", "z")
        r, p, yw = quat_to_rpy(
            g(me, "orientation", "x"), g(me, "orientation", "y"),
            g(me, "orientation", "z"), g(me, "orientation", "w", default=1.0),
        )
        out.append((t, x, y, z, r, p, yw))
    return out


# ── 시뮬 실행 (Python 이 수명 관리 — 셸 sleep 은 하네스가 차단) ──────────────


def run_maneuver(twist_pb: str, drive_seconds: float = DRIVE_SECONDS,
                 world: str = WORLD, gt_topic: str = GT_TOPIC):
    """시뮬을 띄우고 cmd_vel 을 한 번 발행한 뒤 odom/지상진실 스트림을 수집.

    world/gt_topic 을 바꾸면 다른 월드(예: 두둑 걸터타기)에도 그대로 재사용된다.
    반환: (odom_samples, gt_samples). cmd_vel 은 한 번만 발행한다 —
    Fortress 6 DiffDrive 는 cmd_timeout 이 없어 마지막 명령이 지속된다(실측).
    """
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)

    total_iters = int((6 + drive_seconds) * 1000)  # 초기화+주행 여유. RTF=1 → 1ms/스텝
    log = open("/tmp/ww_drive.log", "w")
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--iterations", str(total_iters), world],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )

    def stop(proc):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    try:
        # odom 토픽이 뜰 때까지 대기 (최대 15s)
        deadline = time.time() + 15
        while time.time() < deadline:
            out = subprocess.run([ENV, "ign", "topic", "-l"],
                                 capture_output=True, text=True).stdout
            if ODOM_TOPIC in out and gt_topic in out:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("토픽이 안 떴습니다 — 시뮬 초기화 실패. /tmp/ww_drive.log 확인")

        odom_f = open("/tmp/ww_odom.log", "w")
        gt_f = open("/tmp/ww_gt.log", "w")
        odom_sub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", ODOM_TOPIC],
                                    stdout=odom_f, stderr=subprocess.DEVNULL, start_new_session=True)
        gt_sub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", gt_topic],
                                  stdout=gt_f, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(1.5)  # 구독자 연결 + 정지 상태 베이스라인 수집

        subprocess.run([ENV, "ign", "topic", "-t", CMD_TOPIC, "-m", "ignition.msgs.Twist",
                        "-p", twist_pb], capture_output=True, text=True)

        time.sleep(drive_seconds)

        stop(odom_sub); stop(gt_sub)
        odom_f.close(); gt_f.close()
        odom_sub.wait(timeout=5); gt_sub.wait(timeout=5)
    finally:
        stop(sim)
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()

    return (odom_samples(open("/tmp/ww_odom.log").read()),
            gt_samples(open("/tmp/ww_gt.log").read()))


# ── 단언 ────────────────────────────────────────────────────────────────


class Fail(Exception):
    pass


def _steady_window(odom, v_field_idx, thresh):
    """운동이 시작된(|해당속도|>thresh) 첫 시각 t0 을 찾고, 램프가 끝난
    t0+1.0s 이후를 정상상태로 본다. (t0, steady_start) 반환."""
    t0 = None
    for s in odom:
        if abs(s[v_field_idx]) > thresh:
            t0 = s[0]
            break
    if t0 is None:
        raise Fail("운동이 시작되지 않았습니다 (오도메트리 속도가 계속 0)")
    return t0, t0 + 1.0


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        raise Fail("정상상태 샘플이 없습니다 — 주행 시간을 늘리세요")
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def assert_forward():
    print("── 전진 시험: linear.x =", V_FWD, "m/s ──")
    odom, gt = run_maneuver(f"linear: {{x: {V_FWD}}}, angular: {{z: 0.0}}")
    print(f"  수집: odom {len(odom)} 샘플, 지상진실 {len(gt)} 샘플")
    if len(odom) < 20 or len(gt) < 20:
        raise Fail("샘플이 너무 적습니다 — 시뮬이 제대로 안 돌았을 수 있음")

    _, steady_start = _steady_window(odom, 4, 0.5 * V_FWD)  # idx4 = vx

    # 게이트 A: 플러그인이 명령 속도를 달성
    vx_steady = [s[4] for s in odom if s[0] >= steady_start]
    achieved_v = median(vx_steady)

    # 게이트 B: 지상진실로 실제 전진 속도 측정 (정상상태 구간 선형)
    gt_steady = [s for s in gt if s[0] >= steady_start]
    if len(gt_steady) < 5:
        raise Fail("정상상태 지상진실 샘플 부족")
    a, b = gt_steady[0], gt_steady[-1]
    dt = b[0] - a[0]
    if dt < 1.0:
        raise Fail(f"정상상태 구간이 너무 짧습니다 ({dt:.2f}s)")
    dx, dy = b[1] - a[1], b[2] - a[2]
    v_gt = dx / dt
    yaw_drift = b[6] - a[6]
    roll_end, pitch_end, z_end = b[4], b[5], b[3]

    # odom 변위(같은 구간) — 지상진실과 비교해 바퀴 반지름/간격 교정 확인
    oa = min((s for s in odom if s[0] >= a[0]), key=lambda s: s[0])
    ob = max((s for s in odom if s[0] <= b[0]), key=lambda s: s[0])
    odom_dx = ob[1] - oa[1]

    print(f"  게이트A 명령속도 달성 : {achieved_v:+.3f} m/s (목표 {V_FWD})")
    print(f"  게이트B 지상진실 전진 : Δx={dx:+.3f}m / {dt:.2f}s = {v_gt:+.3f} m/s")
    print(f"  직진성               : Δy={dy:+.4f}m, yaw drift={yaw_drift:+.4f} rad")
    print(f"  안 넘어짐             : roll={math.degrees(roll_end):+.1f}°, "
          f"pitch={math.degrees(pitch_end):+.1f}°, z={z_end:+.3f}m")
    print(f"  odom↔지상진실 일치    : odomΔx={odom_dx:+.3f}m vs gtΔx={dx:+.3f}m")

    errs = []
    if not (0.85 * V_FWD <= achieved_v <= 1.10 * V_FWD):
        errs.append(f"명령 속도 미달성: {achieved_v:.3f} ∉ [{0.85*V_FWD:.3f},{1.10*V_FWD:.3f}]")
    if not (0.80 * V_FWD <= v_gt <= 1.10 * V_FWD):
        errs.append(f"몸통이 명령대로 안 감: 지상진실 {v_gt:.3f} m/s")
    if abs(dy) > 0.08:
        errs.append(f"옆으로 샘: |Δy|={abs(dy):.3f} > 0.08m")
    if abs(yaw_drift) > 0.10:
        errs.append(f"직진 중 회전: |yaw|={abs(yaw_drift):.3f} > 0.10 rad")
    if abs(roll_end) > 0.14 or abs(pitch_end) > 0.14:
        errs.append(f"넘어짐: roll={roll_end:.2f} pitch={pitch_end:.2f} rad")
    if not (-0.05 <= z_end <= 0.12):
        errs.append(f"떴거나 파묻힘: z={z_end:.3f}m")
    if abs(dx) > 0.05 and abs(odom_dx - dx) / abs(dx) > 0.15:
        errs.append(f"odom↔지상진실 불일치 15%↑: {odom_dx:.3f} vs {dx:.3f}")
    if errs:
        raise Fail("전진 실패:\n    - " + "\n    - ".join(errs))
    print("  ✅ 전진 통과\n")


def assert_turn():
    print("── 회전 시험: angular.z =", W_TURN, "rad/s (CCW, +yaw 여야) ──")
    odom, gt = run_maneuver(f"linear: {{x: 0.0}}, angular: {{z: {W_TURN}}}")
    print(f"  수집: odom {len(odom)} 샘플, 지상진실 {len(gt)} 샘플")
    if len(gt) < 20:
        raise Fail("샘플이 너무 적습니다")

    _, steady_start = _steady_window(odom, 5, 0.3 * W_TURN)  # idx5 = wz
    gt_steady = [s for s in gt if s[0] >= steady_start]
    if len(gt_steady) < 5:
        raise Fail("정상상태 지상진실 샘플 부족")
    a, b = gt_steady[0], gt_steady[-1]
    dt = b[0] - a[0]
    dyaw = b[6] - a[6]
    # 제자리 회전이라 병진은 작아야
    dx, dy = b[1] - a[1], b[2] - a[2]
    yaw_rate_gt = dyaw / dt if dt > 0 else 0.0

    print(f"  지상진실 yaw 변화 : Δyaw={dyaw:+.3f} rad / {dt:.2f}s = {yaw_rate_gt:+.3f} rad/s")
    print(f"  제자리성          : Δx={dx:+.3f}m, Δy={dy:+.3f}m")

    errs = []
    # 4륜 스키드스티어는 미끄러져 명령보다 훨씬 덜 돌지만, **부호와 최소 회전량**은
    # 좌/우 배정이 맞다는 증거다. 명령이 +z(CCW)면 yaw 는 증가해야 한다.
    if dyaw <= 0:
        errs.append(f"회전 방향이 반대 (좌/우 바퀴 배정 오류 의심): Δyaw={dyaw:+.3f}")
    elif dyaw < 0.15:
        errs.append(f"거의 안 돎: Δyaw={dyaw:+.3f} < 0.15 rad")
    if math.hypot(dx, dy) > 0.6:
        errs.append(f"제자리 회전이 아니라 너무 이동: {math.hypot(dx,dy):.2f}m")
    if errs:
        raise Fail("회전 실패:\n    - " + "\n    - ".join(errs))
    print("  ✅ 회전 통과 (방향 맞음 = 좌/우 배정 정상)\n")


def main():
    print("=== diff-drive 주행 단언 (헤드리스, GPU 불필요) ===\n")
    try:
        assert_forward()
        assert_turn()
    except Fail as e:
        print(f"\n❌ {e}", file=sys.stderr)
        subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
        sys.exit(1)
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    print("=== ✅ 주행 단언 통과 — cmd_vel 로 로봇이 움직인다 ===")


if __name__ == "__main__":
    main()
