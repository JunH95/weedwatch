#!/usr/bin/env python3
"""주행+관절 제어 노드 (rclpy) — ww_cmd(ign-transport 직결 C++) 대체, DECISIONS 038.

ww_cmd 프로토콜(v/carriage/tool)을 ROS 토픽으로 낸다:
  /cmd_vel            geometry_msgs/Twist   주행
  /carriage<i>_cmd    std_msgs/Float64      툴 i Y 캐리지 목표 [m]
  /tool<i>_cmd        std_msgs/Float64      툴 i Z 도구 목표 [m] (0=접힘, 음수=하강)
  /odometry (구독)    nav_msgs/Odometry     위치 피드백(x,y,yaw)

ros_gz_bridge(parameter_bridge)가 이 ROS 토픽을 ign 으로 번역한다. 상주 발행자라 지연은
ww_cmd(3.6us)와 동급. WwControl 은 coordinator 가 라이브러리로 써서 주행·타격을 지휘한다.
"""
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from weedwatch_control.maneuver import GyroOdom

# 실물 IMU 잔차 (BNO085 동적 오차 ~1°). 시뮬 IMU 의 orientation 은 노이즈 없는 참자세에서
# 계산되므로(적분 안 함) 그대로 쓰면 제어에 지상진실을 주는 셈이다 — DECISIONS 025 와 같이
# 잔차를 얹어 쓴다. 전처리로 되뺄 수 없는 실현값이라 성능을 실제로 깎는다.
IMU_BIAS_DEG, IMU_NOISE_DEG = 0.8, 0.5


def bridge_args(n_tools: int):
    """parameter_bridge 인자: 제어(ROS→ign)와 상태(ign→ROS). ]=ROS→ign, [=ign→ROS."""
    a = [
        "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        # IMU: 두둑 끝 회전에 필수. 스키드 스티어라 회전 중 바퀴가 긁혀 휠 오도메트리 yaw 가
        # 26°/회전 부풀고(diag_uturn), 방위를 되잡을 수 있는 온보드 센서는 이것뿐이다.
        "/robot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
    ]
    for i in range(n_tools):
        a.append(f"/carriage{i}_cmd@std_msgs/msg/Float64]ignition.msgs.Double")
        a.append(f"/tool{i}_cmd@std_msgs/msg/Float64]ignition.msgs.Double")
    return a


class WwControl(Node):
    """제어 상태 + ROS 발행/구독. coordinator 가 이 노드로 주행·타격을 지휘한다."""

    def __init__(self, n_tools: int = 3):
        super().__init__("ww_control")
        self.n = n_tools
        self._vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._carr = [self.create_publisher(Float64, f"/carriage{i}_cmd", 10) for i in range(n_tools)]
        self._tool = [self.create_publisher(Float64, f"/tool{i}_cmd", 10) for i in range(n_tools)]
        self.create_subscription(Odometry, "/odometry", self._on_odom, 10)
        self.create_subscription(Imu, "/robot/imu", self._on_imu, 10)
        # 시각 오도메트리 증분 — 회전 중에만 쓴다(DECISIONS 041). 노드가 없으면 안 오고,
        # 그러면 GyroOdom 이 휠만으로 돌아간다(vo_used 가 0 으로 남아 드러난다).
        self.create_subscription(Vector3Stamped, "/ww/vo", self._on_vo, 10)
        self.x = self.y = self.yaw = None          # 휠 오도메트리 원본 (직진 구간용)
        self.odom_n = self.imu_n = 0
        self.imu_yaw = None                        # IMU 방위 + 실물 잔차
        self._vo_acc = [0.0, 0.0]                  # 다음 odom 까지 쌓아둘 VO 증분 (전방, 좌)
        self._vo_fresh = False
        self.vo_n = 0
        self._rng = random.Random(7)
        self._imu_bias = math.radians(IMU_BIAS_DEG) * self._rng.choice((1, -1))
        # 자이로-오도메트리: 거리는 바퀴, 방위는 IMU. 스폰 자세를 원점으로 두고 노드가 채운다.
        self.gyro = GyroOdom()

    @staticmethod
    def _yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_imu(self, m):
        noise = math.radians(IMU_NOISE_DEG) * self._rng.gauss(0, 1)
        self.imu_yaw = self._yaw_of(m.orientation) + self._imu_bias + noise
        self.imu_n += 1

    def _on_vo(self, m):
        self._vo_acc[0] += m.vector.x
        self._vo_acc[1] += m.vector.y
        self._vo_fresh = True          # 이번 odom 주기에 **새 카메라 증분이 왔다**
        self.vo_n += 1

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = self._yaw_of(m.pose.pose.orientation)
        self.odom_n += 1
        # 카메라는 5Hz, odom 은 50Hz 다. 새 증분이 안 온 주기에 (0,0) 을 넘기면 융합이
        # "카메라가 정지를 봤다"로 읽어 **매번 슬립으로 오판**한다(실측: VO 사용 534회).
        # 신선한 증분이 있을 때만 넘긴다.
        vo = tuple(self._vo_acc) if self._vo_fresh else None
        self._vo_acc, self._vo_fresh = [0.0, 0.0], False
        self.gyro.update(p.x, p.y, self.yaw, m.twist.twist.linear.x, self.imu_yaw,
                         vo=vo, odom_wz=m.twist.twist.angular.z)

    def seed_pose(self, x, y, yaw=0.0):
        """자이로-오도메트리 원점을 world 스폰 자세로 맞춘다 (밭 기하를 아는 쪽이 준다).

        이걸 안 하면 추정이 "스폰 기준 상대"라, 절대 좌표로 검출을 앵커링하는 인식 노드와 어긋난다.
        """
        self.gyro.x, self.gyro.y, self.gyro.yaw = float(x), float(y), float(yaw)

    def est_pose(self):
        """제어가 믿는 자세 (자이로-오도메트리). 아직 센서가 없으면 None."""
        if self.odom_n == 0:
            return None
        return self.gyro.x, self.gyro.y, self.gyro.yaw

    # ── ww_cmd 프로토콜 대응 ────────────────────────────────
    def drive(self, lin: float, ang: float = 0.0):
        t = Twist(); t.linear.x = float(lin); t.angular.z = float(ang); self._vel.publish(t)

    def set_carriage(self, i: int, pos: float):
        m = Float64(); m.data = float(pos); self._carr[i].publish(m)

    def set_tool(self, i: int, pos: float):
        m = Float64(); m.data = float(pos); self._tool[i].publish(m)

    def stop(self):
        self.drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = WwControl()
    node.get_logger().info("ww_control 노드 대기 (coordinator 가 지휘)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
