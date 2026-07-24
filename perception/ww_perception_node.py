#!/usr/bin/env python3
"""ROS 이관 P2 — 인식 ROS 노드. detect_server(파일IO)를 대체한다.

카메라 토픽(sensor_msgs/Image, ros_gz_bridge 로 ign→ROS)을 구독 → best.pt 추론 → 잡초 world 좌표를
/weeds(geometry_msgs/PoseArray)로 발행. 제어 노드(ww_control)가 이 토픽을 받아 타격한다.

핵심: detect_server 의 추론(load_model·detect_fused·2카메라 융합)을 **그대로 import 재사용**한다 —
파이썬을 3.10 으로 통일(DECISIONS 038)했기에 한 프로세스에 torch+rclpy 가 공존해서 가능하다.
예전 파일IO 핸드셰이크(Gazebo <save> PNG → 디스크 → 폴링)가 통째로 사라진다.

실행(사람/하네스):  scripts/env.sh perception/condaenv/bin/python perception/ww_perception_node.py
자가검증:           make ros-percept
"""
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose

WW = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WW / "perception"))
from detect_server import load_model, detect_fused, CAM_DYS  # noqa: E402

ENV = str(WW / "scripts" / "env.sh")
CAM_TOPICS = ["/robot/camera", "/robot/camera1"]
WEEDS_TOPIC = "/weeds"
SAFE_DIST = 0.025


def image_to_rgb(m: Image) -> np.ndarray:
    """sensor_msgs/Image → RGB 배열(H,W,3). rgb8/rgba8/bgr8 처리."""
    ch = (m.step // m.width) if m.width else 3
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, ch)
    if ch == 4:
        a = a[:, :, :3]
    if m.encoding == "bgr8":
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a)


class Perception(Node):
    """카메라 토픽 → best.pt → /weeds(PoseArray, world 좌표)."""

    def __init__(self, safe_dist: float = SAFE_DIST, rate_hz: float = 5.0):
        super().__init__("ww_perception")
        self.model, self.device = load_model()
        self.frames = [None] * len(CAM_TOPICS)
        self.base = None
        self.safe = safe_dist
        self.n_pub = 0
        for i, t in enumerate(CAM_TOPICS):
            self.create_subscription(Image, t, lambda m, i=i: self._img(m, i), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odometry", self._odom, 10)
        self.pub = self.create_publisher(PoseArray, WEEDS_TOPIC, 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"ww_perception ready (dev={self.device})")

    def _img(self, m, i):
        try:
            self.frames[i] = image_to_rgb(m)
        except Exception as e:                       # 반쯤 온 프레임 등 → 다음 프레임
            self.get_logger().warn(f"img{i} skip: {e}")

    def _odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.base = (p.x, p.y, 0.0, yaw)

    def _tick(self):
        if self.base is None or any(f is None for f in self.frames):
            return
        frames = [(i, self.frames[i]) for i in range(len(self.frames))]
        dets = detect_fused(self.model, frames, self.base, self.device, safe_dist=self.safe)
        pa = PoseArray()
        pa.header.frame_id = "world"
        pa.header.stamp = self.get_clock().now().to_msg()
        for wx, wy, _a in dets:
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = float(wx), float(wy), 0.0
            pa.poses.append(ps)
        self.pub.publish(pa)
        self.n_pub += 1


# ── 자가검증 (P2 게이트) ─────────────────────────────────────────────────────
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
BED0_Y = 0.6


def bridge_args():
    """odom·cmd_vel·카메라 2대를 ros_gz_bridge 로. [=ign→ROS, ]=ROS→ign."""
    a = [
        "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
    ]
    for t in CAM_TOPICS:
        a.append(f"{t}@sensor_msgs/msg/Image[ignition.msgs.Image")
    return a


def _stop(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def selftest():
    from geometry_msgs.msg import Twist
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD],
        stdout=open("/tmp/ros_percept_sim.log", "w"), stderr=subprocess.STDOUT,
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
            [ENV, "ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args()],
            stdout=open("/tmp/ros_percept_bridge.log", "w"), stderr=subprocess.STDOUT,
            start_new_session=True)
        time.sleep(4.0)

        rclpy.init()
        node = Perception()
        collected = []
        node.create_subscription(PoseArray, WEEDS_TOPIC,
                                 lambda m: collected.append(m), 10)
        drive = node.create_publisher(Twist, "/cmd_vel", 10)

        # 두둑0 위를 천천히 주행하며 /weeds 수집 (카메라가 잡초를 지나가게)
        t0 = time.time()
        band = []
        while time.time() - t0 < 16.0:
            tw = Twist(); tw.linear.x = 0.12
            drive.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.05)
        drive.publish(Twist())
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        # 집계: 두둑0 밴드(|wy-0.6|<0.45) 안 검출
        all_dets = [(p.position.x, p.position.y) for m in collected for p in m.poses]
        band = [(x, y) for x, y in all_dets if abs(y - BED0_Y) < 0.45]
        msgs_with_dets = sum(1 for m in collected if m.poses)
        print(f"발행된 /weeds 메시지: {len(collected)} (검출 포함 {msgs_with_dets})")
        print(f"총 검출점 {len(all_dets)} · 두둑0 밴드 안 {len(band)}")
        node.destroy_node(); rclpy.shutdown()

        # 게이트: 여러 메시지에서 두둑 밴드 검출이 실제로 나온다(ROS 인식 파이프라인 작동)
        ok = msgs_with_dets >= 2 and len(band) >= 3
        if not ok:
            raise SystemExit(f"실패: 메시지{msgs_with_dets}·밴드검출{len(band)} — ROS 인식 미작동")
        print(f"=== OK P2 — 인식 ROS 노드가 카메라 토픽→best.pt→/weeds 발행 (밴드 {len(band)}검출) ===")
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
    if "--selftest" in sys.argv:
        selftest()
    else:
        rclpy.init()
        n = Perception()
        try:
            rclpy.spin(n)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()
