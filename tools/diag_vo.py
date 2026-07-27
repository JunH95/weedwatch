#!/usr/bin/env python3
"""시각 오도메트리 타당성 측정 — 하방 카메라가 바퀴 대신 이동을 잴 수 있나.

U턴이 남긴 문제(DECISIONS 040): 회전 중 몸통이 계통적으로 미끄러지는데 바퀴도 IMU 도 못 본다
→ U턴당 절대 x 가 ~52cm 밀린다. 바퀴 오도메트리는 **바퀴가 헛돌면 원리적으로 못 고친다**.
지면을 직접 보는 센서가 필요하고, 로봇엔 이미 하방 카메라 2대(0.457mm/px)가 달려 있다.

여기서 재는 것은 하나다: **연속 프레임의 지면 흐름으로 이동거리를 얼마나 정확히 재나.**
방법은 위상상관(phase correlation) — FFT 로 두 프레임의 평행이동을 픽셀 단위로 찾는다.
회전은 IMU 가 이미 0.0° 로 주므로(040) 여기서는 평행이동만 본다.

  · 직진 구간에서 VO 적산 vs 지상진실 vs 휠 오도메트리를 나란히
  · 제자리 회전에서도 같은 비교 (여기서 바퀴가 무너진다 — VO 는?)

실행:  ./scripts/env.sh perception/condaenv/bin/python tools/diag_vo.py
       (rclpy + numpy 가 한 파이썬에 있어야 한다 — 3.10 통일의 이유, DECISIONS 038)
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
sys.path.insert(0, str(WW / "tools"))

WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
WORLD_NAME, MODEL = "robot_field_multi", "weedwatch"
GT_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
GT_FILE = "/tmp/ww_vo_gt.log"

MM_PER_PX = 0.457e-3     # 캘리브 실측 (DECISIONS 022, 색 마커로 복원오차 0)
V = 0.20


def phase_shift(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """두 그레이 이미지 사이 평행이동 (drow, dcol) — 위상상관.

    FFT 로 상호 파워 스펙트럼을 만들고 역변환의 최대점을 찾는다. 조명 변화에 강하고
    특징점 검출이 필요 없어서, 흙처럼 반복적인 질감에서도 동작한다.
    """
    A, B = np.fft.rfft2(a), np.fft.rfft2(b)
    R = A * np.conj(B)
    mag = np.abs(R)
    mag[mag == 0] = 1e-9
    r = np.fft.irfft2(R / mag, s=a.shape)
    i0, j0 = np.unravel_index(np.argmax(r), r.shape)
    # 서브픽셀: 최대점 주변 3점 포물선 피팅. 정수 픽셀로 끊으면 프레임마다 최대 반 픽셀씩
    # 계통적으로 모자라고, 그게 수십 프레임 적산되면 수 cm 가 된다(실측으로 확인).
    def refine(prev, peak, nxt):
        d = prev - 2 * peak + nxt
        return 0.0 if d == 0 else 0.5 * (prev - nxt) / d
    h, w = r.shape
    di = refine(r[(i0 - 1) % h, j0], r[i0, j0], r[(i0 + 1) % h, j0])
    dj = refine(r[i0, (j0 - 1) % w], r[i0, j0], r[i0, (j0 + 1) % w])
    drow = (i0 if i0 < h // 2 else i0 - h) + di
    dcol = (j0 if j0 < w // 2 else j0 - w) + dj
    return float(drow), float(dcol)


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    from assert_drive import gt_samples

    print("=== 시각 오도메트리 타당성 — 하방 카메라가 이동을 잴 수 있나 ===")
    print(f"    {MM_PER_PX*1000:.3f} mm/px · 5Hz · {V} m/s → 프레임 간 ~{V/5*100:.0f}cm 이동\n")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    log = open("/tmp/ww_vo_sim.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--headless-rendering",
                            "--iterations", "120000", WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    procs, gtsub = [sim], None
    try:
        time.sleep(8)
        procs.append(subprocess.Popen(
            [ENV, "ros2", "run", "ros_gz_bridge", "parameter_bridge",
             "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
             "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
             "/robot/camera@sensor_msgs/msg/Image[ignition.msgs.Image"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        gf = open(GT_FILE, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        procs.append(gtsub)
        time.sleep(5)

        rclpy.init()
        node = Node("ww_vo_diag")
        state = {"frames": [], "odom": None, "vo_x": 0.0, "vo_y": 0.0, "n": 0, "prev": None}

        def on_img(m):
            ch = (m.step // m.width) if m.width else 3
            a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, ch)
            g = a[:, :, :3].mean(axis=2).astype(np.float32)
            # 전해상도 그대로 쓴다 — 절반으로 줄이면 픽셀당 0.9mm 라 적산 오차가 커진다
            prev = state["prev"]
            state["prev"] = g
            if prev is None:
                return
            drow, dcol = phase_shift(prev, g)
            # 캘리브(022): world_x = cam_x − 0.457mm·(row−360) → 로봇이 +x 로 가면 지면은 row 증가 방향
            state["vo_x"] += -drow * MM_PER_PX
            state["vo_y"] += -dcol * MM_PER_PX
            state["n"] += 1

        def on_odom(m):
            p = m.pose.pose.position
            state["odom"] = (p.x, p.y)

        node.create_subscription(Image, "/robot/camera", on_img, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/odometry", on_odom, 10)
        pub = node.create_publisher(Twist, "/cmd_vel", 10)

        def spin(sec):
            t0 = time.time()
            while time.time() - t0 < sec:
                rclpy.spin_once(node, timeout_sec=0.02)

        spin(6.0)                                    # 프레임·odom 붙을 때까지
        if state["n"] == 0:
            raise RuntimeError("카메라 프레임이 안 옵니다 — 브리지/구독자 확인")
        base_odom, base_vo = state["odom"], (state["vo_x"], state["vo_y"])
        gt0 = gt_samples(Path(GT_FILE).read_text(errors="ignore"))
        t_cmd = time.time()

        tw = Twist(); tw.linear.x = V
        pub.publish(tw)
        spin(12.0)                                   # 직진
        tw.linear.x = 0.0
        pub.publish(tw)
        spin(2.0)

        gt = gt_samples(Path(GT_FILE).read_text(errors="ignore"))
        d_gt = math.hypot(gt[-1][1] - gt0[-1][1], gt[-1][2] - gt0[-1][2]) if gt and gt0 else float("nan")
        d_odom = math.hypot(state["odom"][0] - base_odom[0], state["odom"][1] - base_odom[1])
        d_vo = math.hypot(state["vo_x"] - base_vo[0], state["vo_y"] - base_vo[1])
        print(f"    직진 구간 ({time.time()-t_cmd:.0f}s, 프레임 {state['n']}장)")
        print(f"      지상진실   {d_gt*100:7.1f} cm")
        print(f"      휠 오도메트리 {d_odom*100:7.1f} cm  (오차 {abs(d_odom-d_gt)*100:+.1f})")
        print(f"      시각 오도메트리 {d_vo*100:7.1f} cm  (오차 {abs(d_vo-d_gt)*100:+.1f})")

        # 제자리 회전 — 바퀴가 무너지는 구간에서 VO 는 어떤가
        base_odom, base_vo = state["odom"], (state["vo_x"], state["vo_y"])
        gt0 = gt_samples(Path(GT_FILE).read_text(errors="ignore"))
        tw = Twist(); tw.angular.z = 0.5
        pub.publish(tw)
        spin(10.0)
        tw.angular.z = 0.0
        pub.publish(tw)
        spin(2.0)
        gt = gt_samples(Path(GT_FILE).read_text(errors="ignore"))
        d_gt = math.hypot(gt[-1][1] - gt0[-1][1], gt[-1][2] - gt0[-1][2])
        d_odom = math.hypot(state["odom"][0] - base_odom[0], state["odom"][1] - base_odom[1])
        d_vo = math.hypot(state["vo_x"] - base_vo[0], state["vo_y"] - base_vo[1])
        print(f"\n    제자리 회전 구간 (바퀴가 긁히는 곳)")
        print(f"      지상진실 이동 {d_gt*100:7.1f} cm   ← 회전 중에도 몸통은 이만큼 미끄러진다")
        print(f"      휠 오도메트리 {d_odom*100:7.1f} cm  (오차 {abs(d_odom-d_gt)*100:+.1f})")
        print(f"      시각 오도메트리 {d_vo*100:7.1f} cm  (오차 {abs(d_vo-d_gt)*100:+.1f})")
        rclpy.shutdown()
    finally:
        for p in reversed(procs):
            kill(p)
        time.sleep(0.5)
        log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
