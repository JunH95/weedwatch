#!/usr/bin/env python3
"""관제 화면(rviz2)이 실제로 뭔가 띄우는지 단언 — `make viz-check`. 화면 없이.

나(에이전트)는 rviz2 를 못 본다. 그래서 "관제 화면 만들었습니다"를 화면 없이 검증한다:
sim + 브리지 + GT 브리지 + viz 노드를 켜고, **rviz 가 구독할 토픽에 실제로 내용이 흐르는지**를
센다. 사람은 `make watch-rviz` 로 같은 걸 눈으로 본다 (에이전트=수치, 사람=화면 교차검증).

게이트:
  0. 그래프용 상태 수치(/ww/state/*)가 발행된다 — Foxglove Plot 패널이 그릴 것
  1. /ww/viz 마커가 발행된다 (밭·로봇 상자)
  2. 지상진실이 화면 쪽으로 흐른다 (viz 노드가 ign 스트림에서 읽어온다)
  3. viz 노드가 world→base_truth TF 를 낸다 (fixed frame=world 에서 로봇이 보이려면 필요)
  4. 카메라 영상 토픽이 ROS 쪽에 있다 (rviz Image 패널이 볼 것)

여기서 코디네이터는 안 켠다 — 주행 없이도 화면 배선이 성립해야 한다. 믿음(base_est)은
코디네이터가 붙을 때만 나오므로 게이트가 아니라 보고 항목이다.
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
WORLD_NAME = "robot_field_multi"
GT_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"


class Fail(Exception):
    pass


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def main():
    print("=== 관제 시각화 배선 단언 (화면 없이) ===")
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    sys.path.insert(0, str(WW / "src" / "weedwatch_control"))
    from weedwatch_control.control_node import bridge_args

    log = open("/tmp/ww_viz_sim.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--headless-rendering",
                            "--iterations", "60000", WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    procs = [sim]
    try:
        time.sleep(7)
        cams = [f"{t}@sensor_msgs/msg/Image[ignition.msgs.Image"
                for t in ("/robot/camera", "/robot/camera1")]
        procs.append(subprocess.Popen(
            [ENV, "ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args(3), *cams],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        time.sleep(5)
        # 로봇을 살짝 굴린다 — dynamic_pose/info 는 **움직이는 것**의 자세라, 로봇이 서 있으면
        # 한 줄도 안 나온다(실측). 배선을 재려면 뭔가 움직이고 있어야 한다.
        # ign 쪽으로 직접 명령한다: `ros2 topic pub -1` 은 발행 직후 종료돼 디스커버리 전에
        # 메시지가 날아간다(실측 — 이것 때문에 "지상진실 안 옴"으로 오진했다).
        subprocess.run([ENV, "ign", "topic", "-t", "/cmd_vel", "-m", "ignition.msgs.Twist",
                        "-p", "linear: {x: 0.15}"], capture_output=True, timeout=20)
        viz = subprocess.Popen(
            [ENV, "bash", "-c",
             f"source {WW}/install/setup.bash && WW_ROOT={WW} "
             f"python3 {WW}/src/weedwatch_bringup/weedwatch_bringup/viz_node.py --selftest"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        procs.append(viz)
        time.sleep(6)                     # viz 노드가 사는 동안 토픽을 확인해야 한다
        topics = subprocess.run([ENV, "ros2", "topic", "list"],
                                capture_output=True, text=True, timeout=30).stdout
        out, _ = viz.communicate(timeout=90)
        print(out.strip())
        need = ("/robot/camera", "/ww/viz", "/ww/state/loc_error_cm",
                "/ww/state/speed_mps", "/ww/state/gyro_vs_wheel_cm", "/ww/state/weeds_seen")
        for t in need:
            if t not in topics:
                raise Fail(f"관제 화면이 볼 토픽이 없다: {t}")
        print(f"  관제 토픽 {len(need)}개 존재 (영상·마커·그래프용 상태 수치)")
        if viz.returncode != 0:
            raise Fail("viz 노드 자가검증 실패 (위 출력 참고)")
    finally:
        for p in reversed(procs):
            kill(p)
        time.sleep(0.5)
        log.close()
    print("\n[통과] 관제 화면 배선 성립 — 사람은 `make watch-rviz` 로 보면 된다.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fail as e:
        print(f"\n[실패] {e}", file=sys.stderr)
        sys.exit(1)
