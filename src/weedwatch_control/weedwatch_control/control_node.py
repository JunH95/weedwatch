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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


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
    """제어 상태 + ROS 발행/구독. coordinator 가 이 노드로 주행·타격을 지휘한다."""

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
