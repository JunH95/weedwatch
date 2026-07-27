#!/usr/bin/env python3
"""ROS 경로의 IMU/방위 진단 — 직결(ww_cmd)에서는 0.1° 인데 ROS 에서는 왜 깨지나.

`make turn`(ww_cmd 직결)은 U턴 방위 오차 0.2~1.2° 로 통과하는데, 같은 마뉴버 코드를 ROS 노드로
돌린 관통에서는 직진 중 추정이 −19° 로 틀어지고 90° 회전이 72° 에서 멈췄다. 전송만 다르므로
**브리지를 건너온 IMU 가 의심**이다. 여기서 사실만 잰다:

  · /robot/imu 가 실제로 오는가 · 몇 Hz 인가 (QoS 불일치면 0)
  · IMU yaw 와 휠 odom yaw 가 정지 상태에서 얼마나 다른가
  · 회전 중 IMU yaw 가 튀는가 (샘플 간 점프 크기)

실행:  ./scripts/env.sh python3 tools/diag_ros_imu.py     (sim+bridge 를 직접 띄운다)
"""
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")

SNIPPET = r'''
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

class D(Node):
    def __init__(self):
        super().__init__("diag_imu")
        self.imu, self.odom = [], []
        self.create_subscription(Imu, "/robot/imu", self._imu, 10)
        self.create_subscription(Odometry, "/odometry", self._odom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
    def _imu(self, m):
        self.imu.append((time.time(), yaw_of(m.orientation)))
    def _odom(self, m):
        self.odom.append((time.time(), yaw_of(m.pose.pose.orientation)))
    def drive(self, lin, ang):
        t = Twist(); t.linear.x = lin; t.angular.z = ang; self.pub.publish(t)

rclpy.init()
n = D()
def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.02)

spin(5.0)
n0_imu, n0_odom = len(n.imu), len(n.odom)
print(f"정지 5초: IMU {n0_imu} 개 ({n0_imu/5:.1f} Hz) · odom {n0_odom} 개 ({n0_odom/5:.1f} Hz)")
if n0_imu == 0:
    print("IMU 가 한 개도 안 옴 — 브리지/QoS 문제")
else:
    iy = [y for _, y in n.imu]
    oy = [y for _, y in n.odom] or [0.0]
    print(f"  정지 IMU yaw 범위 {math.degrees(min(iy)):+.2f}~{math.degrees(max(iy)):+.2f}° · "
          f"odom yaw {math.degrees(oy[-1]):+.2f}°")
    jumps = [abs(math.degrees(iy[i] - iy[i-1])) for i in range(1, len(iy))]
    print(f"  정지 IMU 샘플간 점프 최대 {max(jumps):.2f}°" if jumps else "")

# 제자리 회전 3초 — 두 소스가 어떻게 갈리나
n.imu.clear(); n.odom.clear()
n.drive(0.0, 0.5)
spin(3.0)
n.drive(0.0, 0.0)
spin(1.5)
if n.imu and n.odom:
    di = math.degrees(n.imu[-1][1] - n.imu[0][1])
    do = math.degrees(n.odom[-1][1] - n.odom[0][1])
    iy = [y for _, y in n.imu]
    jumps = [abs(math.degrees(iy[i] - iy[i-1])) for i in range(1, len(iy))]
    print(f"회전 3초(0.5rad/s): IMU Δ{di:+.1f}° · odom Δ{do:+.1f}° "
          f"(IMU {len(n.imu)} 샘플 {len(n.imu)/4.5:.0f} Hz, 샘플간 최대 점프 {max(jumps):.1f}°)")
rclpy.shutdown()
'''


def main():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    log = open("/tmp/ww_diag_imu_sim.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--iterations", "60000", WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    bridge = None
    try:
        time.sleep(6)
        sys.path.insert(0, str(WW / "src" / "weedwatch_control"))
        from weedwatch_control.control_node import bridge_args
        bridge = subprocess.Popen(
            [ENV, "ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args(3)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(5)
        snippet = Path("/tmp/ww_diag_imu_node.py")
        snippet.write_text(SNIPPET)
        r = subprocess.run([ENV, "python3", str(snippet)], capture_output=True, text=True, timeout=120)
        print(r.stdout.strip())
        if r.returncode != 0:
            print(r.stderr[-800:], file=sys.stderr)
    finally:
        for p in (bridge, sim):
            if p:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except (ProcessLookupError, AttributeError):
                    pass
        log.close()


if __name__ == "__main__":
    main()
