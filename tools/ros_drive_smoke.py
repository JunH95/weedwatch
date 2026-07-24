#!/usr/bin/env python3
"""ROS 이관 Phase 0 — ign→ros_gz_bridge→rclpy 폐루프가 이 컴퓨터에서 도는지 증명.

ROS판 `make drive`: cmd_vel 을 **ROS 토픽(geometry_msgs/Twist)** 으로 발행해 로봇이 움직이고,
움직임을 **ROS 토픽(/odometry, nav_msgs/Odometry)** 으로 확인한다. 그 사이를 `ros_gz_bridge`
parameter_bridge 가 ign↔ROS 로 번역한다. 이게 되면 제어·인식을 ROS 노드로 옮길 토대가 증명된다.

설치 없음: 이미 있는 parameter_bridge 만 사용(ros_gz_sim 불요). 자기완결(sim·bridge 스폰→단언→정리).
실행:  ./scripts/env.sh python3 tools/ros_drive_smoke.py
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WW = Path(__file__).resolve().parent.parent
ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")

# 브리지 스펙: ]=ROS→ign(제어), [=ign→ROS(상태)
BRIDGE = [
    "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
    "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
]
V = 0.15          # m/s
DRIVE_S = 5.0     # 주행 시간
MOVED_MIN = 0.15  # 이만큼 이상 x 증가해야 통과


def _stop(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


class DriveProbe(Node):
    def __init__(self):
        super().__init__("ww_ros_drive_smoke")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odometry", self._odom, 10)
        self.x = None
        self.n = 0

    def _odom(self, m):
        self.x = m.pose.pose.position.x
        self.n += 1


def main():
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD],
        stdout=open("/tmp/ros_smoke_sim.log", "w"), stderr=subprocess.STDOUT,
        start_new_session=True)
    bridge = None
    try:
        # /cmd_vel 은 DiffDrive 의 구독자라 발행자(브리지)가 생기기 전엔 -l 에 안 뜬다 → /odometry 만 대기.
        for _ in range(40):
            t = subprocess.run([ENV, "ign", "topic", "-l"],
                               capture_output=True, text=True).stdout
            if "/odometry" in t:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("ign 토픽 안 뜸 (/odometry)")

        bridge = subprocess.Popen(
            [ENV, "ros2", "run", "ros_gz_bridge", "parameter_bridge", *BRIDGE],
            stdout=open("/tmp/ros_smoke_bridge.log", "w"), stderr=subprocess.STDOUT,
            start_new_session=True)
        time.sleep(3.0)                            # 브리지 디스커버리

        rclpy.init()
        node = DriveProbe()
        # odom 이 ROS 로 흘러오는지 먼저 확인(브리지 ign→ROS 검증)
        t0 = time.time()
        while node.x is None and time.time() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.x is None:
            raise RuntimeError("브리지가 /odometry 를 ROS 로 안 넘김 (ign→ROS 실패)")
        x_start = node.x
        print(f"게이트 A(ign→ROS): /odometry 수신 {node.n}건, x0={x_start:.3f}")

        # cmd_vel 을 ROS 로 발행(브리지 ROS→ign 검증) — 10Hz 로 유지
        tw = Twist(); tw.linear.x = V
        t0 = time.time()
        while time.time() - t0 < DRIVE_S:
            node.pub.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.1)
        node.pub.publish(Twist())                 # 정지
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        x_end = node.x
        moved = x_end - x_start
        print(f"게이트 B(ROS→ign): cmd_vel 발행 후 x1={x_end:.3f} · 이동 {moved:.3f}m")

        node.destroy_node(); rclpy.shutdown()
        if moved < MOVED_MIN:
            raise SystemExit(f"실패: {moved:.3f}m < {MOVED_MIN}m — ROS 명령이 로봇을 못 움직임")
        print(f"=== OK Phase 0 — ign↔ros_gz_bridge↔rclpy 폐루프 작동 ({moved:.2f}m 주행) ===")
    finally:
        _stop(bridge)
        _stop(sim)
        try:
            sim.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
            except Exception:
                pass


if __name__ == "__main__":
    main()
