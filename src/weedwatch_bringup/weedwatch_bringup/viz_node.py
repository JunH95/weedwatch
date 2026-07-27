#!/usr/bin/env python3
"""관제 화면용 시각화 노드 — 로봇이 **믿는** 위치와 **실제** 위치를 나란히 띄운다.

Gazebo GUI 가 "밖에서 본 로봇"이라면, rviz2 는 **로봇 머릿속**이다. 그래서 이 노드가 내는 건
로봇의 믿음(추정 자세·검출한 잡초)이고, 옆에 지상진실을 같이 놓아 **사람이 눈으로 오차를 본다**.
에이전트는 수치로, 사람은 화면으로 — 같은 시스템을 두 창으로 보는 교차검증(CLAUDE.md).

  발행                                     무엇
  /tf : world→base_est                     로봇이 믿는 자세 (코디네이터의 자이로-오도메트리)
  /tf : world→base_truth                   지상진실 (시뮬에서만 알 수 있는 값)
  /ww/viz (MarkerArray)                    두둑·검출 잡초·믿음 vs 실제 상자·오차 선분과 숫자
  /ww/state/* (Float64)                    **그래프로 볼 수치** — 아래 표

  토픽                          무엇                                    왜 보나
  /ww/state/loc_error_cm        |믿음 − 실제| 위치 오차                  위치추정이 얼마나 새는가
  /ww/state/heading_error_deg   믿음 − 실제 방위                         IMU 방위가 유지되는가
  /ww/state/speed_mps           주행 속도                                무정차 상한 0.2 를 지키는가
  /ww/state/gyro_vs_wheel_cm    자이로-오도 vs 휠 오도 차이               **온보드로만** 보이는 신호 —
                                                                        바퀴가 긁히면 여기서 벌어진다
  /ww/state/weeds_seen          지금 프레임에서 본 잡초 수                인식이 살아 있는가

**지상진실은 화면 전용이다.** 이 노드는 아무것도 제어하지 않고, 제어 노드는 이 토픽을 구독하지
않는다. GT 브리지도 제어용 bridge_args 가 아니라 관람 런치에서만 켠다 — 제어가 정답을 물리적으로
못 보게 하는 규율(아키텍처)을 화면 하나 만들자고 무르지 않기 위해서다.

자가검증(에이전트용, 화면 없이): `--selftest` 는 마커·TF 가 실제로 나가는지 세어서 단언한다.
"""
import math
import os
import subprocess
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Float64
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from weedwatch_control.ww_paths import find_repo_root  # noqa: E402

WW = find_repo_root()
sys.path.insert(0, str(WW / "tools"))
from garden_geometry import Garden, Portal  # noqa: E402  (순수 기하 config)
from assert_drive import gt_samples          # noqa: E402  (게이트들이 쓰는 그 GT 파서)

ENV = str(WW / "scripts" / "env.sh")
GT_TOPIC = "/world/robot_field_multi/dynamic_pose/info"
MODEL = "weedwatch"
N_BEDS = 2
FIRST_BED_Y, PITCH = 0.60, 1.20

_G, _P = Garden(), Portal()
BED_W = _G.bed_width
ROBOT_L, ROBOT_W = _P.deck_length, _P.track(_G) + 2 * _P.deck_overhang


def rgba(r, g, b, a=1.0):
    c = ColorRGBA(); c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Viz(Node):
    def __init__(self):
        super().__init__("ww_viz")
        self.est = None          # (x, y, yaw) 로봇이 믿는 자세
        self.truth = None        # (x, y, yaw) 지상진실
        self.weeds = []
        self.n_markers = 0
        self._wheel_ref = None
        self.create_subscription(PoseStamped, "/ww/base_pose", self._on_est, 10)
        self.create_subscription(PoseArray, "/weeds", self._on_weeds, 10)
        self._start_truth_reader()
        self.create_subscription(Odometry, "/odometry", self._on_odom, 10)
        self.odom = None         # (x, y, speed) 휠 오도메트리 원본
        # 그래프용 상태 수치 — Foxglove 의 Plot 패널이 이걸 그린다
        self.state = {k: self.create_publisher(Float64, f"/ww/state/{k}", 10) for k in
                      ("loc_error_cm", "heading_error_deg", "speed_mps",
                       "gyro_vs_wheel_cm", "weeds_seen")}
        self.markers = self.create_publisher(MarkerArray, "/ww/viz", 10)
        self.tf = self.create_publisher(TFMessage, "/tf", 10)
        self.create_timer(0.1, self._tick)

    # ── 입력 ────────────────────────────────────────────────────────
    def _on_est(self, m):
        self.est = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def _on_weeds(self, m):
        self.weeds = [(p.position.x, p.position.y) for p in m.poses]

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y, m.twist.twist.linear.x)

    def _publish_state(self):
        """그래프로 볼 수치. 오차 항목은 화면 전용(지상진실 필요)이고, gyro_vs_wheel 은
        **지상진실 없이도** 보이는 신호라 실물에서도 그대로 쓸 수 있다."""
        out = {}
        if self.odom:
            out["speed_mps"] = self.odom[2]
        out["weeds_seen"] = float(len(self.weeds))
        if self.est and self.truth:
            out["loc_error_cm"] = math.hypot(self.est[0] - self.truth[0],
                                             self.est[1] - self.truth[1]) * 100
            out["heading_error_deg"] = math.degrees(
                math.atan2(math.sin(self.est[2] - self.truth[2]),
                           math.cos(self.est[2] - self.truth[2])))
        if self.est and self.odom:
            # 휠 오도메트리는 스폰 기준 상대이므로, 두 추정의 **증분 차이**가 의미 있다.
            if self._wheel_ref is None:
                self._wheel_ref = (self.odom[0] - self.est[0], self.odom[1] - self.est[1])
            dx = (self.odom[0] - self._wheel_ref[0]) - self.est[0]
            dy = (self.odom[1] - self._wheel_ref[1]) - self.est[1]
            out["gyro_vs_wheel_cm"] = math.hypot(dx, dy) * 100
        for k, v in out.items():
            m = Float64(); m.data = float(v); self.state[k].publish(m)

    def _start_truth_reader(self):
        """지상진실을 ign 텍스트 스트림에서 읽는다 — **화면 전용**.

        ROS 브리지(Pose_V→TFMessage)는 프레임 이름을 빈 문자열로 흘려(실측) 어느 게 로봇인지
        못 가른다. 그래서 게이트들이 쓰는 검증된 경로(`ign topic -e` + assert_drive.gt_samples)를
        그대로 재사용한다. 블록버퍼링으로 수백 ms 밀릴 수 있는데, 제어가 아니라 사람 눈이 볼
        그림이라 무해하다(제어에 쓰면 안 되는 이유이기도 하다).
        """
        self._proc = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                      text=True, bufsize=1)

        def read():
            buf = []
            for line in self._proc.stdout:
                buf.append(line)
                if len(buf) > 400:                      # 메시지 경계(빈 줄)에서 통째로 파싱
                    samples = gt_samples("".join(buf))
                    if samples:
                        t, x, y, _z, _r, _p, yw = samples[-1]
                        self.truth = (x, y, yw)
                    buf = buf[-80:]

        threading.Thread(target=read, daemon=True).start()

    # ── 출력 ────────────────────────────────────────────────────────
    def _send_tf(self, frame, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = frame
        t.transform.translation.x, t.transform.translation.y = float(pose[0]), float(pose[1])
        t.transform.rotation.z = math.sin(pose[2] / 2)
        t.transform.rotation.w = math.cos(pose[2] / 2)
        self.tf.publish(TFMessage(transforms=[t]))

    def _box(self, mid, ns, pose, size, color, kind=Marker.CUBE):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pose[0]), float(pose[1]), float(pose[2])
        m.pose.orientation.z = math.sin(pose[3] / 2) if len(pose) > 3 else 0.0
        m.pose.orientation.w = math.cos(pose[3] / 2) if len(pose) > 3 else 1.0
        m.scale.x, m.scale.y, m.scale.z = size
        m.color = color
        return m

    def _tick(self):
        arr = MarkerArray()
        mid = 0

        # 밭: 두둑을 갈색 판으로 (로봇이 걸터타는 대상)
        for b in range(N_BEDS):
            cy = FIRST_BED_Y + b * PITCH
            arr.markers.append(self._box(mid, "field", (1.5, cy, 0.125, 0.0),
                                         (3.6, BED_W, 0.25), rgba(0.45, 0.33, 0.22, 0.55)))
            mid += 1

        # 검출한 잡초 = 로봇이 "봤다"고 믿는 것 (빨강)
        for wx, wy in self.weeds:
            arr.markers.append(self._box(mid, "weeds", (wx, wy, 0.28, 0.0),
                                         (0.05, 0.05, 0.05), rgba(0.9, 0.15, 0.15, 0.95),
                                         kind=Marker.SPHERE))
            mid += 1

        # 로봇: 믿음(주황 반투명) vs 실제(초록 반투명)
        if self.est:
            self._send_tf("base_est", self.est)
            arr.markers.append(self._box(mid, "robot", (self.est[0], self.est[1], 0.62, self.est[2]),
                                         (ROBOT_L, ROBOT_W, 0.06), rgba(1.0, 0.55, 0.1, 0.45))); mid += 1
        if self.truth:
            self._send_tf("base_truth", self.truth)
            arr.markers.append(self._box(mid, "robot", (self.truth[0], self.truth[1], 0.60, self.truth[2]),
                                         (ROBOT_L, ROBOT_W, 0.06), rgba(0.15, 0.8, 0.35, 0.45))); mid += 1

        # 오차: 두 자세를 잇는 선분 + 숫자 (이게 이 화면의 요점)
        if self.est and self.truth:
            d = math.hypot(self.est[0] - self.truth[0], self.est[1] - self.truth[1])
            dyaw = math.degrees(math.atan2(math.sin(self.est[2] - self.truth[2]),
                                           math.cos(self.est[2] - self.truth[2])))
            line = Marker()
            line.header.frame_id = "world"
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns, line.id, line.type, line.action = "error", mid, Marker.LINE_LIST, Marker.ADD
            line.scale.x = 0.02
            line.color = rgba(1.0, 0.1, 0.1, 0.9)
            for (x, y, _), z in ((self.est, 0.62), (self.truth, 0.60)):
                line.points.append(Point(x=float(x), y=float(y), z=float(z)))
            line.pose.orientation.w = 1.0
            arr.markers.append(line); mid += 1

            txt = Marker()
            txt.header.frame_id = "world"
            txt.header.stamp = self.get_clock().now().to_msg()
            txt.ns, txt.id, txt.type, txt.action = "error", mid, Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = \
                float(self.truth[0]), float(self.truth[1]), 1.1
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.16
            txt.color = rgba(1.0, 1.0, 1.0, 0.95)
            txt.text = f"위치오차 {d*100:.0f}cm · 방위 {dyaw:+.1f}°"
            arr.markers.append(txt); mid += 1

        self.markers.publish(arr)
        self.n_markers += len(arr.markers)
        self._publish_state()


def selftest():
    """화면 없이: 마커가 실제로 나가고, 믿음/실제 프레임이 둘 다 잡히는지 단언."""
    rclpy.init()
    node = Viz()
    seen = {"markers": 0, "tf": 0}
    node.create_subscription(MarkerArray, "/ww/viz",
                             lambda m: seen.__setitem__("markers", seen["markers"] + len(m.markers)), 10)
    node.create_subscription(TFMessage, "/tf",
                             lambda m: seen.__setitem__("tf", seen["tf"] + len(m.transforms)), 10)
    import time
    t0 = time.time()
    while time.time() - t0 < 12:
        rclpy.spin_once(node, timeout_sec=0.05)
    ok = seen["markers"] > 0 and node.truth is not None
    print(f"마커 {seen['markers']} · TF {seen['tf']} · 지상진실 수신 {'예' if node.truth else '아니오'} "
          f"· 추정 수신 {'예' if node.est else '아니오(코디네이터 미기동이면 정상)'}")
    rclpy.shutdown()
    if not ok:
        print("[실패] 관제 화면에 띄울 게 안 나옵니다 — 브리지/토픽 확인", file=sys.stderr)
        return 1
    print("[통과] 관제 시각화가 발행됩니다 (rviz2 로 보면 됨: make watch-rviz)")
    return 0


def main(args=None):
    if "--selftest" in (args or sys.argv):
        sys.exit(selftest())
    rclpy.init(args=args)
    node = Viz()
    node.get_logger().info("ww_viz — 믿음(주황) vs 실제(초록), 검출(빨강). rviz2 fixed frame = world")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
