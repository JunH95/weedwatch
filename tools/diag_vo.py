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

1차 측정 결과(고정 스케일·평행이동만): 직진 7% 부족, 회전 미끄러짐은 27%만 봄. 원인 둘을
따로 고치고 **각각의 기여를 따로 잰다** — 한 번에 둘 다 바꾸면 뭐가 들었는지 모른다.

  --depth-scale   픽셀당 거리를 **깊이 중앙값으로** 매 프레임 보정. → **실패했다**(직진 56.5cm 를
                  18.5cm 로 봄). 장면이 평면이 아니라 하나의 스케일이 애초에 없다.
  --soil-mask     그래서 깊이를 **스케일이 아니라 선택**에 쓴다: 두둑 윗면(캘리브 평면) 근처
                  픽셀만 남기고 식물·고랑을 지운 뒤 상관. 남은 픽셀에는 고정 0.457mm/px 가 맞다.
  --derotate      직전 프레임을 **IMU yaw 변화만큼 되돌린 뒤** 상관. 위상상관은 평행이동만
                  찾으므로 영상이 돌면 못 따라간다(회전 방위 자체는 040 에서 이미 0.0°).

실행:  ./scripts/env.sh perception/condaenv/bin/python tools/diag_vo.py [--depth-scale] [--derotate]
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
CAL_H = 0.33             # 그 값을 잰 거리 [m] (카메라 → 두둑 윗면). 깊이 보정의 기준.
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


def derotate(img: np.ndarray, dyaw: float) -> np.ndarray:
    """영상을 중심 기준으로 -dyaw 만큼 돌린다 (IMU 가 준 회전을 상쇄).

    scipy.ndimage.rotate 는 되지만 프레임마다 보간이라 비싸다 — 여기서는 진단이므로 그대로 쓴다.
    실시간 노드로 옮길 땐 회전을 뺀 관심영역만 상관하는 쪽이 싸다.
    """
    if abs(dyaw) < math.radians(0.05):
        return img
    from scipy import ndimage
    return ndimage.rotate(img, math.degrees(dyaw), reshape=False, order=1, mode="nearest")


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--depth-scale", action="store_true", help="깊이로 픽셀당 거리 보정")
    ap.add_argument("--derotate", action="store_true", help="IMU yaw 로 프레임 되돌린 뒤 상관")
    ap.add_argument("--soil-mask", action="store_true", help="깊이로 두둑 윗면(흙) 픽셀만 남기고 상관")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    from assert_drive import gt_samples

    print("=== 시각 오도메트리 — 하방 카메라가 이동을 잴 수 있나 ===")
    print(f"    {MM_PER_PX*1000:.3f} mm/px(고정, {CAL_H}m 기준) · 5Hz · {V} m/s "
          f"→ 프레임 간 ~{V/5*100:.0f}cm")
    print(f"    깊이 스케일 {'ON' if args.depth_scale else 'off'} · "
          f"흙 마스크 {'ON' if args.soil_mask else 'off'} · "
          f"IMU 되돌리기 {'ON' if args.derotate else 'off'}\n")

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
             "/robot/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
             "/robot/depth@sensor_msgs/msg/Image[ignition.msgs.Image",
             "/robot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        gf = open(GT_FILE, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        procs.append(gtsub)
        time.sleep(5)

        rclpy.init()
        node = Node("ww_vo_diag")
        state = {"odom": None, "vo_x": 0.0, "vo_y": 0.0, "n": 0, "prev": None,
                 "depth": None, "mask": None, "imu_yaw": None, "prev_yaw": None,
                 "scale_used": [], "mask_frac": []}

        def on_depth(m):
            """깊이 프레임(32FC1, 미터). 중앙값(스케일용)과 지면 마스크(선택용)를 같이 만든다."""
            a = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
            h, w = a.shape
            c = a[h // 3: 2 * h // 3, w // 3: 2 * w // 3]
            fin = c[np.isfinite(c) & (c > 0.05) & (c < 3.0)]
            if fin.size:
                state["depth"] = float(np.median(fin))
            # 캘리브 평면(두둑 윗면) ±5cm 안의 픽셀만 = 흙. 식물(더 가까움)·고랑(더 멂)은 뺀다.
            state["mask"] = (np.isfinite(a) & (np.abs(a - CAL_H) < 0.05)).astype(np.float32)

        def on_imu(m):
            q = m.orientation
            state["imu_yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z))

        def on_img(m):
            ch = (m.step // m.width) if m.width else 3
            a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, ch)
            g = a[:, :, :3].mean(axis=2).astype(np.float32)
            if args.soil_mask and state["mask"] is not None and state["mask"].shape == g.shape:
                mk = state["mask"]
                frac = float(mk.mean())
                state["mask_frac"].append(frac)
                if frac > 0.05:                      # 흙이 너무 적으면 마스킹이 오히려 해가 된다
                    g = (g - g.mean()) * mk          # 평균 제거 후 마스킹 — 마스크 경계 계단을 줄인다
            # 전해상도 그대로 쓴다 — 절반으로 줄이면 픽셀당 0.9mm 라 적산 오차가 커진다
            prev, prev_yaw = state["prev"], state["prev_yaw"]
            state["prev"], state["prev_yaw"] = g, state["imu_yaw"]
            if prev is None:
                return
            if args.derotate and prev_yaw is not None and state["imu_yaw"] is not None:
                prev = derotate(prev, wrap_pi(state["imu_yaw"] - prev_yaw))
            drow, dcol = phase_shift(prev, g)
            # 픽셀당 거리: 고정값은 캘리브 평면(CAL_H)에서 잰 것이라, 실제로 본 거리에 비례해 늘린다.
            mpp = MM_PER_PX
            if args.depth_scale and state["depth"]:
                mpp = MM_PER_PX * (state["depth"] / CAL_H)
            state["scale_used"].append(mpp / MM_PER_PX)
            # 캘리브(022): world_x = cam_x − 0.457mm·(row−360) → 로봇이 +x 로 가면 지면은 row 증가 방향
            state["vo_x"] += -drow * mpp
            state["vo_y"] += -dcol * mpp
            state["n"] += 1

        def on_odom(m):
            p = m.pose.pose.position
            state["odom"] = (p.x, p.y)

        node.create_subscription(Image, "/robot/camera", on_img, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/odometry", on_odom, 10)
        if args.depth_scale or args.soil_mask:
            node.create_subscription(Image, "/robot/depth", on_depth, qos_profile_sensor_data)
        if args.derotate:
            from sensor_msgs.msg import Imu
            node.create_subscription(Imu, "/robot/imu", on_imu, 10)
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
        sc, mf = state["scale_used"], state["mask_frac"]
        extra = ""
        if args.depth_scale and sc:
            extra += f" · 스케일 배율 중앙 {np.median(sc):.2f}"
        if args.soil_mask and mf:
            extra += f" · 흙 픽셀 비율 중앙 {np.median(mf)*100:.0f}%"
        print(f"      시각 오도메트리 {d_vo*100:7.1f} cm  (오차 {abs(d_vo-d_gt)*100:+.1f}){extra}")

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
