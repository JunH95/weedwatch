#!/usr/bin/env python3
"""ROS 이관 Phase 1 — ww_cmd(ign-transport 직결 C++)를 대체하는 rclpy 제어 노드.

같은 제어 인터페이스를 **ROS 토픽**으로 낸다(ww_cmd 프로토콜 v/carriage/tool 과 1:1):
  /cmd_vel            geometry_msgs/Twist   주행
  /carriage<i>_cmd    std_msgs/Float64      툴 i Y 캐리지 목표 [m]
  /tool<i>_cmd        std_msgs/Float64      툴 i Z 도구 목표 [m] (0=접힘, 음수=하강)
  /odometry (구독)    nav_msgs/Odometry     위치 피드백(x,y,yaw)

ros_gz_bridge(parameter_bridge)가 이 ROS 토픽들을 ign 쪽으로 번역한다(bridge_args()).
ww_cmd 와 달리 상주 C++ 가 아니라 ROS 노드지만 지연은 동급(상주 발행자). 이게 제어의 ROS 화 핵심.

자가검증:  make ros-control   (주행 + 툴0 하강을 ROS 로 명령하고 지상진실로 확인)
"""
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

WW = Path(__file__).resolve().parent.parent
ENV = str(WW / "scripts" / "env.sh")


def bridge_args(n_tools: int):
    """parameter_bridge 인자: 제어(ROS→ign)와 상태(ign→ROS). ]=ROS→ign, [=ign→ROS."""
    a = [
        "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
    ]
    for i in range(n_tools):
        a.append(f"/carriage{i}_cmd@std_msgs/msg/Float64]ignition.msgs.Double")
        a.append(f"/tool{i}_cmd@std_msgs/msg/Float64]ignition.msgs.Double")
    return a


class WwControl(Node):
    """제어 상태 + ROS 발행/구독. field_run 등이 이 노드를 써서 주행·타격을 지휘한다."""

    def __init__(self, n_tools: int = 3):
        super().__init__("ww_control")
        self.n = n_tools
        self._vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._carr = [self.create_publisher(Float64, f"/carriage{i}_cmd", 10) for i in range(n_tools)]
        self._tool = [self.create_publisher(Float64, f"/tool{i}_cmd", 10) for i in range(n_tools)]
        self.create_subscription(Odometry, "/odometry", self._on_odom, 10)
        self.x = self.y = self.yaw = None
        self.odom_n = 0

    def _on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_n += 1

    # ── ww_cmd 프로토콜 대응 ────────────────────────────────
    def drive(self, lin: float, ang: float = 0.0):
        t = Twist(); t.linear.x = float(lin); t.angular.z = float(ang); self._vel.publish(t)

    def set_carriage(self, i: int, pos: float):
        m = Float64(); m.data = float(pos); self._carr[i].publish(m)

    def set_tool(self, i: int, pos: float):
        m = Float64(); m.data = float(pos); self._tool[i].publish(m)

    def stop(self):
        self.drive(0.0, 0.0)


# ── 자가검증 (Phase 1 게이트) ────────────────────────────────────────────────
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
WORLD_NAME = "robot_field_multi"
MODEL = "weedwatch"


def _stop(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def read_joint_gt(joint: str):
    """지상진실: ign joint_state 에서 관절 위치를 한 번 읽는다(검증용 — 제어는 ROS 로 함)."""
    topic = f"/world/{WORLD_NAME}/model/{MODEL}/joint_state"
    out = subprocess.run([ENV, "ign", "topic", "-e", "-t", topic, "-n", "1"],
                         capture_output=True, text=True, timeout=15).stdout
    # ignition.msgs.Model 텍스트: joint { name: "tool0_joint" ... axis1 { position: -0.15 } }
    blocks = out.split("joint {")
    for b in blocks:
        if f'"{joint}"' in b:
            for ln in b.splitlines():
                s = ln.strip()
                if s.startswith("position:"):
                    return float(s.split(":")[1])
    return None


def selftest():
    n = 3
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD],
        stdout=open("/tmp/ros_ctrl_sim.log", "w"), stderr=subprocess.STDOUT,
        start_new_session=True)
    bridge = None
    try:
        for _ in range(40):
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if "/odometry" in t:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("ign 토픽 안 뜸")
        bridge = subprocess.Popen(
            [ENV, "ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args(n)],
            stdout=open("/tmp/ros_ctrl_bridge.log", "w"), stderr=subprocess.STDOUT,
            start_new_session=True)
        time.sleep(3.0)

        rclpy.init()
        node = WwControl(n)
        t0 = time.time()
        while node.x is None and time.time() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.x is None:
            raise RuntimeError("브리지가 /odometry 를 ROS 로 안 넘김")
        x0 = node.x
        tool0_up = read_joint_gt("tool0_joint")
        print(f"시작: x0={x0:.3f}, tool0_joint(지상진실)={tool0_up}")

        # 게이트 A: 주행(ROS /cmd_vel)
        t0 = time.time()
        while time.time() - t0 < 4.0:
            node.drive(0.15)
            rclpy.spin_once(node, timeout_sec=0.1)
        node.stop()
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        moved = node.x - x0
        print(f"게이트 A(주행): 이동 {moved:.3f}m")

        # 게이트 B: 관절(ROS /tool0_cmd) — 툴0 하강 명령 후 지상진실 위치 확인
        node.set_tool(0, -0.15)
        for _ in range(20):
            node.set_tool(0, -0.15)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        tool0_dn = read_joint_gt("tool0_joint")
        print(f"게이트 B(관절): tool0_joint 하강 후(지상진실)={tool0_dn}")

        node.destroy_node(); rclpy.shutdown()
        ok = moved > 0.15 and tool0_dn is not None and tool0_dn < -0.05
        if not ok:
            raise SystemExit(f"실패: 주행 {moved:.3f}m / tool0 {tool0_dn} — ROS 제어 미작동")
        print(f"=== OK Phase 1 — ROS 제어 노드로 주행({moved:.2f}m)+관절(tool0={tool0_dn:.3f}) 작동 ===")
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
    selftest()
