#!/usr/bin/env python3
"""관통 코디네이터 노드 — 자율주행 두뇌 (DECISIONS 036·038).

여러 두둑을 순회하며: 검출(/weeds, 인식 노드) → 담당 툴 배정 → 무정차 예측 타격 → 오라클 채점 →
artifacts/field_run.json 로깅. field_run_ros.py(하네스)를 **진짜 노드**로 승격 — sim·브리지·인식은
런치가 켜고(subprocess 안 함), 이 노드는 순수 ROS 노드로 제어/판단만 한다.

제어는 WwControl(weedwatch_control) 상속 — /cmd_vel + /carriage<i>_cmd + /tool<i>_cmd 발행,
/odometry 구독. 여기에 /weeds 구독 + /ww/base_pose 발행(참 world pose 앵커, 텔레포트 odom 누적 흡수)을
더한다. 스켈레톤 순회 로직은 워커 스레드에서 돌고 rclpy.spin 이 콜백을 처리한다.

전이 메모: 기하 상수·오라클·set_pose 는 아직 tools/ 에서 import(WW_ROOT). 직결 파일 정리 때 패키지로 이동.
"""
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped

WW = Path(os.environ.get("WW_ROOT", str(Path(__file__).resolve().parents[3])))
sys.path.insert(0, str(WW / "tools"))

from weedwatch_control.control_node import WwControl              # noqa: E402
import field_run as FR                                            # noqa: E402
from assert_row_stamp import TOOL_XS, BAND_CENTERS, V, STRIKE, RAISE, Z_SETTLE, N  # noqa: E402
from assert_row_stamp import weed_tool                            # noqa: E402
from make_field_world import bed_centers                          # noqa: E402

N_BEDS = FR.N_BEDS
X0, X1 = FR.X_DRIVE0, FR.X_DRIVE1
TOL_XY, SAFE_DIST = FR.TOL_XY, FR.SAFE_DIST
OUT = str(WW / "artifacts" / "field_run.json")


class Coordinator(WwControl):
    def __init__(self):
        super().__init__(N)
        self.latest_weeds = []
        self.create_subscription(PoseArray, "/weeds", self._on_weeds, 10)
        self.base_pub = self.create_publisher(PoseStamped, "/ww/base_pose", 10)
        self.result = None

    def _on_weeds(self, m):
        self.latest_weeds = [(p.position.x, p.position.y) for p in m.poses]

    def publish_base(self, x, y, yaw=0.0):
        ps = PoseStamped(); ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z = math.sin(yaw / 2); ps.pose.orientation.w = math.cos(yaw / 2)
        self.base_pub.publish(ps)

    def run_skeleton(self):
        centers = bed_centers(N_BEDS)
        result = {"field": {"n_beds": N_BEDS, "bed_centers": [round(c, 3) for c in centers],
                            "drive_x": [X0, X1]}, "beds": [], "started": True}
        t_start = time.time()
        # odom 붙을 때까지
        t0 = time.time()
        while self.x is None and time.time() - t0 < 15:
            time.sleep(0.1)
        for bed in range(N_BEDS):
            cy = centers[bed]
            FR.set_pose(X0 - 0.05, cy, 0.05, 0.0)
            self.publish_base(X0 - 0.05, cy); time.sleep(2.5)
            bed_log = {"bed": bed, "y": round(cy, 3), "reached": False,
                       "detected": [], "struck": [], "oracle_weeds": len(FR.oracle_weeds_for_bed(cy))}
            seen, active, pool = set(), [None] * N, [[] for _ in range(N)]
            ox_ref, ox = None, X0 - 0.05
            self.drive(V, 0.0)
            deadline = time.time() + (X1 - X0) / V / 0.15 + 30
            while time.time() < deadline:
                if self.x is None:
                    time.sleep(0.01); continue
                if ox_ref is None:
                    ox_ref = self.x
                ox = (X0 - 0.05) + (self.x - ox_ref)         # 참 world x (odom 두둑 간 누적 상대화)
                self.publish_base(ox, cy)
                for wx, wy in list(self.latest_weeds):
                    key = (round(wx / 0.06), round(wy / 0.06))
                    if key in seen or abs(wy - cy) > 0.45:
                        continue
                    i = weed_tool(wy - (cy - 0.6))
                    strike_x = wx - TOOL_XS[i]
                    if ox >= strike_x - V * Z_SETTLE:
                        continue
                    seen.add(key)
                    pool[i].append({"wx": wx, "wy": wy, "i": i, "strike_x": strike_x, "phase": 0})
                    bed_log["detected"].append([round(wx, 3), round(wy, 3)])
                for i in range(N):
                    if active[i] is None:
                        cand = [p for p in pool[i] if p["phase"] == 0 and p["strike_x"] > ox + 0.01]
                        if cand:
                            p = min(cand, key=lambda z: z["strike_x"])
                            active[i] = p; p["phase"] = 1
                            self.set_carriage(i, (p["wy"] - cy) - BAND_CENTERS[i])
                    else:
                        p = active[i]
                        if p["phase"] == 1 and ox >= p["strike_x"] - V * Z_SETTLE:
                            self.set_tool(i, STRIKE); p["phase"] = 2
                            bed_log["struck"].append([round(p["wx"], 3), round(p["wy"], 3)])
                        elif p["phase"] == 2 and ox >= p["strike_x"] + 0.06:
                            self.set_tool(i, RAISE); p["phase"] = 3; active[i] = None
                if ox >= X1:
                    break
                time.sleep(0.01)
            bed_log["reached"] = ox >= X1 - 0.05
            self.stop(); time.sleep(0.4)
            result["beds"].append(bed_log)
        self.stop()
        result["duration_s"] = round(time.time() - t_start, 1)

        # 사후 오라클 채점 (제어와 분리, GT)
        summ = {"struck": 0, "handed_to_human": 0, "missed": 0, "detected": 0}
        for bl in result["beds"]:
            cy = bl["y"]; crops = FR.crops_for_bed(cy)
            summ["detected"] += len(bl["detected"])
            bl["weeds"] = []
            for wx, wy in FR.oracle_weeds_for_bed(cy):
                near_crop = crops and min(math.hypot(wx - cx, wy - cyp) for cx, cyp in crops) < SAFE_DIST
                hit = any(math.hypot(wx - sx, wy - sy) <= TOL_XY for sx, sy in bl["struck"])
                outcome = "struck" if hit else ("handed_to_human" if near_crop else "missed")
                summ[outcome] += 1
                bl["weeds"].append({"x": round(wx, 3), "y": round(wy, 3), "outcome": outcome})
            bl["crops"] = [[round(cx, 3), round(cyp, 3)] for cx, cyp in crops]
        result["summary"] = summ
        result["coverage"] = {"beds_done": sum(1 for b in result["beds"] if b.get("reached")),
                              "beds_total": N_BEDS}
        Path(OUT).write_text(json.dumps(result, ensure_ascii=False, indent=2))
        self.result = result
        c = result["coverage"]
        self.get_logger().info(f"관통 완료: 두둑 {c['beds_done']}/{c['beds_total']} · "
                               f"검출 {summ['detected']} · 처리 {summ['struck']} · 로그 {OUT}")


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()

    def worker():
        try:
            node.run_skeleton()
        finally:
            rclpy.shutdown()

    th = threading.Thread(target=worker, daemon=True)
    th.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    th.join(timeout=5)
    node.destroy_node()


if __name__ == "__main__":
    main()
