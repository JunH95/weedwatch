#!/usr/bin/env python3
"""시각 오도메트리 ROS 노드 — 카메라·깊이·IMU → `/ww/vo` (로봇 기준 이동 증분).

왜 있나 (DECISIONS 040·041): 이 로봇은 스키드 스티어라 제자리 회전에서 몸통이 계통적으로
미끄러지는데(90°마다 앞 0.27m·오른쪽 0.26m) **바퀴는 그걸 원리적으로 못 본다**(0.0cm 보고).
지면을 직접 보는 카메라만 본다 — 실측 86% 관측.

계약:
  구독  /robot/camera (Image) · /robot/depth (Image, 32FC1) · /robot/imu (Imu) · /odometry
  발행  /ww/vo (Vector3Stamped)  x=전방[m], y=좌[m] 증분 (누적 아님, 로봇 기준)

**회전 중에만 계산한다.** 융합이 회전 구간에서만 VO 를 쓰므로(041), 직진 내내 FFT·회전보간을
돌리는 건 순수 낭비다 — 실제로 그 부하 때문에 U턴이 제한 시간 안에 안 끝났다. 직진 구간에는
프레임만 기억해두고(싸다), |wz| 가 임계를 넘을 때 상관을 시작한다. 관심영역도 가운데 정사각형
(ROI_PX)만 쓴다 — 카메라가 아래를 보므로 가운데가 곧 흙이고, FFT 비용이 몇 배 준다.

**증분만 낸다.** 언제 이걸 믿을지는 융합하는 쪽(weedwatch_control.GyroOdom)이 정한다 —
직진 거리는 바퀴가 압도적으로 정확하고(0.8% vs 7~13%) VO 는 회전 미끄러짐에서만 이긴다(041).

실행: ./scripts/env.sh perception/condaenv/bin/python perception/ww_vo_node.py
      (torch 는 안 쓴다. numpy·scipy·rclpy 만 — 3.10 통일 덕에 한 파이썬에 다 있다)
"""
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vo import VoTracker  # noqa: E402

CAM_TOPIC, DEPTH_TOPIC, IMU_TOPIC = "/robot/camera", "/robot/depth", "/robot/imu"
ODOM_TOPIC, OUT_TOPIC = "/odometry", "/ww/vo"
TURN_WZ = 0.15      # 이 각속도를 넘으면 회전으로 보고 상관을 돌린다 (GyroOdom.TURN_WZ 와 같은 값)
# 관심영역 크롭은 **쓰지 않는다**. 회전 중에는 프레임마다 5~6° 씩 돌아 크롭 경계로 내용이
# 드나들고, 그러면 상관 피크가 엉뚱한 데 꽂힌다(실측: U턴 뒤 표류 43.7→221.9cm). 진단에서
# 검증한 설정이 전체 프레임이었으므로 그대로 간다 — 비용은 "회전 중에만 계산"으로 이미 줄었다.


class VoNode(Node):
    def __init__(self):
        super().__init__("ww_vo")
        self.tracker = VoTracker()
        self.depth = None
        self.imu_yaw = None
        self.wz = 0.0
        self.skipped = 0
        self.create_subscription(Image, CAM_TOPIC, self._on_img, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        self.pub = self.create_publisher(Vector3Stamped, OUT_TOPIC, 10)
        self.n_pub = 0
        self.get_logger().info("ww_vo — 지면 흐름으로 이동 증분 발행 (/ww/vo)")

    def _on_depth(self, m):
        try:
            self.depth = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
        except ValueError:
            self.depth = None

    def _on_odom(self, m):
        self.wz = m.twist.twist.angular.z

    def _on_imu(self, m):
        q = m.orientation
        self.imu_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_img(self, m):
        ch = (m.step // m.width) if m.width else 3
        try:
            rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, ch)
        except ValueError:
            return                                   # 반쯤 온 프레임 → 다음 것
        if abs(self.wz) <= TURN_WZ:
            # 직진 중: 융합이 어차피 휠을 쓴다. 프레임만 기억하고 무거운 계산은 건너뛴다.
            self.tracker.remember(rgb, self.depth, self.imu_yaw)
            self.skipped += 1
            return
        d = self.tracker.update(rgb, self.depth, self.imu_yaw)
        if d is None:
            return
        out = Vector3Stamped()
        out.header.stamp = m.header.stamp if m.header.stamp.sec else self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.vector.x, out.vector.y = float(d[0]), float(d[1])
        self.pub.publish(out)
        self.n_pub += 1


def main(args=None):
    rclpy.init(args=args)
    node = VoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
