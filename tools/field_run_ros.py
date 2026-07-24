#!/usr/bin/env python3
"""ROS 이관 P3 — 관통(walking skeleton)을 ROS 노드 조합으로 (field_run.py 직결판 대체).

구성: Gazebo(ign) + ros_gz_bridge + 인식 노드(ww_perception, subprocess) + 이 코디네이터.
  · 검출  = /weeds (PoseArray, 인식 노드가 카메라→best.pt→발행)
  · 제어  = WwControl (rclpy: /cmd_vel + /carriage<i>_cmd + /tool<i>_cmd)
  · 앵커  = /ww/base_pose (코디네이터가 참 world pose 발행 → 인식이 이걸로 앵커링. 텔레포트로
            odom 이 두둑 간 누적돼 어긋나는 걸 로컬라이저 역할로 흡수)
타격 로직·오라클 채점은 field_run(직결판)에서 그대로 재사용. 로그는 같은 artifacts/field_run.json →
대시보드(make dashboard) 그대로. 게이트: 두둑 완주 AND 검출>0 (재현율로 안 막음, DECISIONS 036).

실행:  make field-run-ros   (scripts/env.sh python3 tools/field_run_ros.py)
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parent.parent
ENV = str(WW / "scripts" / "env.sh")
PNODE_PY = str(WW / "perception" / "condaenv" / "bin" / "python")
PNODE = str(WW / "perception" / "ww_perception_node.py")
sys.path.insert(0, str(WW / "tools"))

import rclpy                                            # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped    # noqa: E402

from ww_control import WwControl, bridge_args as ctrl_bridge  # noqa: E402
import field_run as FR                                  # noqa: E402  (oracle/crops/set_pose/상수 재사용)
from assert_row_stamp import TOOL_XS, BAND_CENTERS, V, STRIKE, RAISE, Z_SETTLE, N  # noqa: E402
from assert_row_stamp import weed_tool                  # noqa: E402
from make_field_world import bed_centers                # noqa: E402

N_BEDS = FR.N_BEDS
X0, X1 = FR.X_DRIVE0, FR.X_DRIVE1
TOL_XY, SAFE_DIST = FR.TOL_XY, FR.SAFE_DIST
WORLD = FR.WORLD
CAM_TOPICS = ["/robot/camera", "/robot/camera1"]
OUT = str(WW / "artifacts" / "field_run.json")


def bridge_args():
    a = ctrl_bridge(N)                                  # cmd_vel·odometry·carriage/tool
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


def run():
    centers = bed_centers(N_BEDS)
    sim = subprocess.Popen(
        [ENV, "ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD],
        stdout=open("/tmp/ww_frros_sim.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    bridge = percept = None
    result = {"field": {"n_beds": N_BEDS, "bed_centers": [round(c, 3) for c in centers],
                        "drive_x": [X0, X1]}, "beds": [], "started": True}
    t_start = time.time()
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
            stdout=open("/tmp/ww_frros_bridge.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
        percept = subprocess.Popen(
            [ENV, PNODE_PY, PNODE],
            stdout=open("/tmp/ww_frros_percept.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(7.0)                                 # 브리지 디스커버리 + best.pt 로드

        rclpy.init()
        ctrl = WwControl(N)
        weeds = {"latest": []}
        ctrl.create_subscription(PoseArray, "/weeds",
                                 lambda m: weeds.__setitem__("latest", [(p.position.x, p.position.y) for p in m.poses]), 10)
        base_pub = ctrl.create_publisher(PoseStamped, "/ww/base_pose", 10)

        def spin(dt):
            end = time.time() + dt
            while time.time() < end:
                rclpy.spin_once(ctrl, timeout_sec=0.02)

        def publish_base(x, y, yaw=0.0):
            ps = PoseStamped(); ps.header.frame_id = "world"
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            base_pub.publish(ps)

        # odom 붙을 때까지
        t0 = time.time()
        while ctrl.x is None and time.time() - t0 < 10:
            rclpy.spin_once(ctrl, timeout_sec=0.1)
        if ctrl.x is None:
            raise RuntimeError("/odometry 안 들어옴")

        for bed in range(N_BEDS):
            cy = centers[bed]
            FR.set_pose(X0 - 0.05, cy, 0.05, 0.0)
            publish_base(X0 - 0.05, cy); spin(2.5)
            bed_log = {"bed": bed, "y": round(cy, 3), "reached": False,
                       "detected": [], "struck": [], "oracle_weeds": len(FR.oracle_weeds_for_bed(cy))}
            seen, active, pool = set(), [None] * N, [[] for _ in range(N)]
            ox_ref, ox = None, X0 - 0.05
            ctrl.drive(V, 0.0)
            deadline = time.time() + (X1 - X0) / V / 0.15 + 30
            while time.time() < deadline:
                rclpy.spin_once(ctrl, timeout_sec=0.02)
                if ctrl.x is None:
                    continue
                if ox_ref is None:
                    ox_ref = ctrl.x
                ox = (X0 - 0.05) + (ctrl.x - ox_ref)    # 참 world x (odom 두둑 간 누적 상대화)
                publish_base(ox, cy)                    # 인식 앵커링용
                for wx, wy in weeds["latest"]:
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
                            ctrl.set_carriage(i, (p["wy"] - cy) - BAND_CENTERS[i])
                    else:
                        p = active[i]
                        if p["phase"] == 1 and ox >= p["strike_x"] - V * Z_SETTLE:
                            ctrl.set_tool(i, STRIKE); p["phase"] = 2
                            bed_log["struck"].append([round(p["wx"], 3), round(p["wy"], 3)])
                        elif p["phase"] == 2 and ox >= p["strike_x"] + 0.06:
                            ctrl.set_tool(i, RAISE); p["phase"] = 3; active[i] = None
                if ox >= X1:
                    break
            bed_log["reached"] = ox >= X1 - 0.05
            ctrl.stop(); spin(0.4)
            result["beds"].append(bed_log)

        ctrl.stop()
        result["duration_s"] = round(time.time() - t_start, 1)
        ctrl.destroy_node(); rclpy.shutdown()

        # 사후 오라클 채점 (제어와 분리, GT) — field_run 과 동일 규칙
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
    finally:
        for p in (percept, bridge):
            _stop(p)
        _stop(sim)
        try:
            sim.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
            except Exception:
                pass
    Path(OUT).write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return result


if __name__ == "__main__":
    print("=== 관통 P3(ROS) — 여러 두둑 자율 주행+검출+타격+로깅 (ROS 노드 조합) ===\n")
    r = run()
    s = r.get("summary", {}); cov = r.get("coverage", {})
    print(f"\n커버리지: 두둑 {cov.get('beds_done')}/{cov.get('beds_total')} 완주 · {r.get('duration_s')}s")
    print(f"검출: {s.get('detected')}개 · 처리 {s.get('struck')} · 사람몫 {s.get('handed_to_human')} · 놓침 {s.get('missed')}")
    print(f"로그: {OUT}")
    if cov.get("beds_done") != N_BEDS:
        sys.exit("실패: 두둑 완주 못 함")
    if s.get("detected", 0) == 0:
        sys.exit("실패: 검출 0")
    print("\n=== OK 관통(ROS) — ROS 노드 조합으로 밭을 자율 순회하고 잡초 찍고 데이터를 남겼다 ===")
