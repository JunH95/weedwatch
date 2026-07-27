#!/usr/bin/env python3
"""관통 코디네이터 노드 — 자율주행 두뇌 (DECISIONS 036·038).

여러 두둑을 순회하며: 검출(/weeds, 인식 노드) → 담당 툴 배정 → 무정차 예측 타격 → 오라클 채점 →
artifacts/field_run.json 로깅. field_run_ros.py(하네스)를 **진짜 노드**로 승격 — sim·브리지·인식은
런치가 켜고(subprocess 안 함), 이 노드는 순수 ROS 노드로 제어/판단만 한다.

**두둑 사이는 실제로 돈다 (DECISIONS 040)** — 예전엔 `set_pose` 순간이동 치트(036)였다. 지금은
헤드랜드 U턴: 두둑을 벗어나 → 90° → 옆으로 pitch → 90° → 반대 방향으로 재진입. 그래서 패스가
방향을 번갈아 달리고(보스트로페돈), 툴 밴드·타격 예측·캐리지 명령이 전부 진행 방향 d(±1)를 탄다.
회전 방위는 IMU 가 준다 — 스키드 스티어라 휠 오도메트리 yaw 는 회전당 26° 부푼다(diag_uturn).

제어는 WwControl(weedwatch_control) 상속 — /cmd_vel + /carriage<i>_cmd + /tool<i>_cmd 발행,
/odometry 구독. 여기에 /weeds 구독 + /ww/base_pose 발행(참 world pose 앵커, 텔레포트 odom 누적 흡수)을
더한다. 순회 로직은 워커 스레드에서 돌고 rclpy.spin 이 콜백을 처리한다.

기하 상수·오라클·set_pose 는 weedwatch_control.params / weedwatch_sim.field 에서 가져온다 —
직결 코드(ww_cmd·assert_row_stamp)를 안 끌어온다(garden_geometry·oracle 순수 config 만 경유).
"""
import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped

WW = Path(os.environ.get("WW_ROOT", str(Path(__file__).resolve().parents[3])))

from weedwatch_control.control_node import WwControl              # noqa: E402
from weedwatch_control.maneuver import Maneuver, SWING_RADIUS, EXIT_MARGIN  # noqa: E402
from weedwatch_control.params import (                            # noqa: E402
    TOOL_XS, BAND_CENTERS, BASE_Y, V, STRIKE, RAISE, Z_SETTLE, N, weed_tool)
from weedwatch_sim.field import (                                 # noqa: E402
    oracle_weeds_for_bed, crops_for_bed, bed_centers)

N_BEDS = 2
X0, X1 = 0.2, 1.6                  # 두둑 주행 구간 (짧게 — 카메라+best.pt GPU 경합 느림, 036)
RIDGE_X = (-0.30, 3.30)            # 두둑(ridge) 의 x 범위 — 이 밖이 헤드랜드
SPAWN_X = 0.0                      # 월드가 로봇을 놓는 x (make_field_world: 두둑0 걸터탄 채 x=0)
X_EXIT_HI = RIDGE_X[1] + SWING_RADIUS + EXIT_MARGIN     # +x 쪽 회전 지점
X_EXIT_LO = RIDGE_X[0] - SWING_RADIUS - EXIT_MARGIN     # -x 쪽 회전 지점
TOL_XY = 0.08                      # "그 잡초를 맞게 타격" 반경
SAFE_DIST = 0.025                  # 작물 근접 잡초는 사람 몫(007)
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
                            "drive_x": [X0, X1], "transit": "headland_uturn"}, "beds": [], "started": True}
        t_start = time.time()
        # odom 붙을 때까지
        t0 = time.time()
        while self.x is None and time.time() - t0 < 15:
            time.sleep(0.1)
        # 추정 원점을 world 스폰 자세로 (월드가 로봇을 두둑0 걸터탄 x=0 에 놓는다). 이걸 맞춰야
        # 인식 노드가 받는 base_pose 가 절대 좌표가 된다 — 예전엔 순간이동이 이 정합을 대신했다.
        self.seed_pose(SPAWN_X, centers[0], 0.0)
        man = Maneuver(self.drive, self.est_pose, v=V)
        man.wait_pose()
        if self.gyro.degraded:
            self.get_logger().warn("IMU 가 없어 휠 yaw 로 폴백 — 회전당 26° 오차가 난다(diag_uturn)")

        for bed in range(N_BEDS):
            cy = centers[bed]
            d = 1 if bed % 2 == 0 else -1                 # 패스 진행 방향(+x / -x)
            sx, ex = (X0, X1) if d > 0 else (X1, X0)      # 이 패스의 시작/끝 x
            if bed > 0:
                # 두둑 사이 이동 = 실제 헤드랜드 U턴 (순간이동 치트 제거, DECISIONS 040).
                # 방위는 IMU, 거리는 바퀴(자이로-오도메트리) — 휠 yaw 로는 회전당 26° 틀어진다.
                x_exit = X_EXIT_HI if d < 0 else X_EXIT_LO
                self.get_logger().info(f"두둑 {bed-1}→{bed} 헤드랜드 U턴 (x_exit={x_exit:.2f})")
                man.uturn(x_exit, cy - centers[bed - 1], sx - 0.05 * d,
                          log=lambda m: self.get_logger().info(m))
            self.publish_base(sx, cy, 0.0 if d > 0 else math.pi); time.sleep(1.0)
            bed_log = {"bed": bed, "y": round(cy, 3), "dir": d, "reached": False,
                       "detected": [], "struck": [], "oracle_weeds": len(oracle_weeds_for_bed(cy))}
            seen, active, pool = set(), [None] * N, [[] for _ in range(N)]
            anchor, ox = None, sx
            self.drive(V, 0.0)
            deadline = time.time() + abs(X1 - X0) / V / 0.15 + 30
            while time.time() < deadline:
                est = self.est_pose()
                if est is None:
                    time.sleep(0.01); continue
                if anchor is None:
                    # 두둑0: 스폰 자세로 seed 했으니 추정이 곧 world x — 손대지 않는다.
                    # 두둑>0: U턴 중 몸통 미끄러짐(계통적, ~52cm)을 어떤 온보드 센서도 못 봐서
                    # 진입 시 기하로 x 를 다시 잡는다. **가정이지 관측이 아니다** — 절대 위치추정이
                    # 다음 단계인 이유. 패스 안의 상대 정확도(타격이 쓰는 값)는 이것과 무관하다.
                    anchor = 0.0 if bed == 0 else sx - est[0]
                ox = est[0] + anchor
                self.publish_base(ox, cy, 0.0 if d > 0 else math.pi)
                for wx, wy in list(self.latest_weeds):
                    key = (round(wx / 0.06), round(wy / 0.06))
                    if key in seen or abs(wy - cy) > 0.45:
                        continue
                    i = weed_tool(d * (wy - cy) + BASE_Y)          # 밴드는 로봇 기준(방향 반영)
                    strike_x = wx - d * TOOL_XS[i]                 # 툴이 잡초에 닿는 base x
                    if d * (ox - strike_x) >= -V * Z_SETTLE:       # 이미 지나침 → 못 친다
                        continue
                    seen.add(key)
                    pool[i].append({"wx": wx, "wy": wy, "i": i, "strike_x": strike_x, "phase": 0})
                    bed_log["detected"].append([round(wx, 3), round(wy, 3)])
                for i in range(N):
                    if active[i] is None:
                        cand = [p for p in pool[i] if p["phase"] == 0
                                and d * (p["strike_x"] - ox) > 0.01]
                        if cand:
                            p = min(cand, key=lambda z: d * z["strike_x"])
                            active[i] = p; p["phase"] = 1
                            self.set_carriage(i, d * (p["wy"] - cy) - BAND_CENTERS[i])
                    else:
                        p = active[i]
                        if p["phase"] == 1 and d * (ox - p["strike_x"]) >= -V * Z_SETTLE:
                            self.set_tool(i, STRIKE); p["phase"] = 2
                            bed_log["struck"].append([round(p["wx"], 3), round(p["wy"], 3)])
                        elif p["phase"] == 2 and d * (ox - p["strike_x"]) >= 0.06:
                            self.set_tool(i, RAISE); p["phase"] = 3; active[i] = None
                if d * (ox - ex) >= 0:
                    break
                time.sleep(0.01)
            bed_log["reached"] = d * (ox - ex) >= -0.05
            self.stop()
            # 패스가 끝나면 **모든 도구를 올린다**. 안 올리면 타격 도중 패스가 끝난 툴이 내려간 채
            # 남아, 헤드랜드로 나가는 2.8m 동안 두둑을 긁으며 로봇을 비튼다 — 실측으로 yaw 가 19°
            # 돌아갔고(IMU 는 정직하게 보고했다) U턴이 그 틀어진 방위 기준으로 돌아 재진입이 어긋났다.
            for i in range(N):
                self.set_tool(i, RAISE)
            time.sleep(1.0)
            result["beds"].append(bed_log)
        self.stop()
        result["duration_s"] = round(time.time() - t_start, 1)

        # 사후 오라클 채점 (제어와 분리, GT)
        summ = {"struck": 0, "handed_to_human": 0, "missed": 0, "detected": 0}
        for bl in result["beds"]:
            cy = bl["y"]; crops = crops_for_bed(cy)
            summ["detected"] += len(bl["detected"])
            bl["weeds"] = []
            for wx, wy in oracle_weeds_for_bed(cy):
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
