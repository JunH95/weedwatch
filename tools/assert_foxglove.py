#!/usr/bin/env python3
"""Foxglove 관제 배선 단언 — `make foxglove-check`. 화면 없이.

Foxglove Studio 는 사람이 보는 창이고 나는 못 본다. 그래서 **그 창에 붙는 경로가 실제로 열려
있는지**를 대신 단언한다: foxglove_bridge 가 WebSocket 을 열고 프로토콜 핸드셰이크에 응답하는가,
그리고 거기서 볼 상태 수치(/ww/state/*)가 실제로 흐르는가.

WebSocket 핸드셰이크는 표준 라이브러리 소켓으로 직접 친다(의존성 추가 없음): HTTP Upgrade 를
보내고 **101 Switching Protocols** 와 `Sec-WebSocket-Protocol: foxglove.websocket.v1` 을 받으면
Foxglove Studio 가 붙을 수 있는 상태다.

선행 1회(사람):  sudo apt install ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap
"""
import base64
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
PORT = 8765
APT_HINT = ("sudo apt install ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap")


class Fail(Exception):
    pass


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def ws_handshake(host="127.0.0.1", port=PORT, timeout=5.0) -> str:
    """WebSocket 업그레이드를 직접 쳐서 응답 헤더를 돌려준다."""
    key = base64.b64encode(os.urandom(16)).decode()
    req = (f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
           f"Sec-WebSocket-Version: 13\r\n"
           f"Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n\r\n")
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.settimeout(timeout)
        s.sendall(req.encode())
        return s.recv(4096).decode(errors="ignore")


def main():
    print("=== Foxglove 관제 배선 단언 (화면 없이) ===")
    if not (Path("/opt/ros/humble/share/foxglove_bridge").exists()
            or shutil.which("foxglove_bridge")):
        raise Fail(f"foxglove_bridge 가 설치돼 있지 않습니다. 사람이 한 번 실행:\n    {APT_HINT}")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    sys.path.insert(0, str(WW / "src" / "weedwatch_control"))
    from weedwatch_control.control_node import bridge_args

    log = open("/tmp/ww_fox_sim.log", "w")
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
        procs.append(subprocess.Popen(
            [ENV, "bash", "-c",
             f"source {WW}/install/setup.bash && WW_ROOT={WW} "
             f"python3 {WW}/src/weedwatch_bringup/weedwatch_bringup/viz_node.py"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        foxlog = open("/tmp/ww_fox_bridge.log", "w")
        procs.append(subprocess.Popen(
            [ENV, "ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
            stdout=foxlog, stderr=subprocess.STDOUT, start_new_session=True))
        time.sleep(9)

        # 게이트 1 — WebSocket 이 열리고 Foxglove 프로토콜로 응답하는가
        try:
            resp = ws_handshake()
        except OSError as e:
            raise Fail(f"WebSocket({PORT}) 접속 실패: {e} — 브리지 로그 /tmp/ww_fox_bridge.log")
        first = resp.splitlines()[0] if resp else "(응답 없음)"
        if "101" not in first:
            raise Fail(f"업그레이드 거부: {first}")
        proto_ok = "foxglove.websocket.v1" in resp
        print(f"  게이트 1 WebSocket  {first.strip()} · 프로토콜 {'OK' if proto_ok else '불일치'}")
        if not proto_ok:
            raise Fail("Foxglove 프로토콜을 응답하지 않습니다")

        # 게이트 2 — 그 창에서 볼 상태 수치가 실제로 흐르는가
        topics = subprocess.run([ENV, "ros2", "topic", "list"],
                                capture_output=True, text=True, timeout=30).stdout
        need = ("/ww/state/loc_error_cm", "/ww/state/speed_mps", "/ww/state/weeds_seen",
                "/ww/viz", "/robot/camera")
        missing = [t for t in need if t not in topics]
        if missing:
            raise Fail(f"관제 창에서 볼 토픽이 없다: {missing}")
        print(f"  게이트 2 관제 토픽  {len(need)}개 존재 (그래프·마커·영상)")
    finally:
        for p in reversed(procs):
            kill(p)
        time.sleep(0.5)
        log.close()

    print(f"\n[통과] Foxglove Studio 가 붙을 수 있습니다.")
    print(f"       사람: make watch-foxglove → Studio 에서 ws://localhost:{PORT} 접속")
    print(f"       레이아웃: src/weedwatch_bringup/config/weedwatch.foxglove.json 를 Studio 에 import")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fail as e:
        print(f"\n[실패] {e}", file=sys.stderr)
        sys.exit(1)
