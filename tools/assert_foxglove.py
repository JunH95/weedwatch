#!/usr/bin/env python3
"""Foxglove 관제 배선 단언 — `make foxglove-check`. 화면 없이.

Foxglove Studio 는 사람이 보는 창이고 나는 못 본다. 그래서 **그 창에 붙는 경로가 실제로 열려
있는지**를 대신 단언한다: foxglove_bridge 가 WebSocket 을 열고 프로토콜 핸드셰이크에 응답하는가,
그리고 거기서 볼 상태 수치(/ww/state/*)가 실제로 흐르는가.

핸드셰이크는 **실제 WebSocket 클라이언트**(perception venv 의 `websockets`)로 친다. 손수 만든
HTTP Upgrade 는 어떤 변형을 써도 400 이었다 — 프로토콜 세부를 흉내내는 건 "Studio 가 붙는다"의
증거로 약하다. 붙은 뒤 서버가 보내는 **serverInfo** 와 채널 광고까지 받아야 확인한 것이다.

⚠️ 서브프로토콜 이름이 버전마다 다르다: foxglove_bridge **3.4.2 는 `foxglove.sdk.v1`**,
옛 문서의 `foxglove.websocket.v1` 만 보내면 400 으로 거부된다(실측). 둘 다 제시한다.

선행 1회(사람):  sudo apt install ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap
"""
import json
import os
import shutil
import signal
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


WS_PROBE = r'''
import asyncio, json, sys
import websockets

async def main():
    # 서브프로토콜 이름이 버전마다 다르다 — foxglove_bridge 3.4.2 는 "foxglove.sdk.v1" 이고
    # 옛 이름("foxglove.websocket.v1")만 보내면 **400 으로 거부**된다(실측). 둘 다 제시하고
    # 서버가 고르게 한다.
    async with websockets.connect("ws://127.0.0.1:%d",
                                  subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"],
                                  open_timeout=10) as ws:
        got = {"serverInfo": None, "channels": []}
        for _ in range(40):
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=5)
            except asyncio.TimeoutError:
                break
            if isinstance(msg, bytes):
                continue
            d = json.loads(msg)
            if d.get("op") == "serverInfo":
                got["serverInfo"] = d.get("name") or "(이름 미설정)"
                got["subprotocol"] = ws.subprotocol
            elif d.get("op") == "advertise":
                got["channels"] += [c["topic"] for c in d.get("channels", [])]
            if got["serverInfo"] and len(got["channels"]) > 5:
                break
        print(json.dumps(got))

asyncio.run(main())
'''


def ws_probe(py: str, port: int = PORT) -> dict:
    """진짜 WebSocket 클라이언트로 붙어 serverInfo·채널 광고를 받아온다."""
    script = Path("/tmp/ww_fox_probe.py")
    script.write_text(WS_PROBE % port)
    r = subprocess.run([py, str(script)], capture_output=True, text=True, timeout=60)
    if r.returncode != 0:
        raise Fail(f"WebSocket 접속 실패:\n{r.stderr[-500:]}")
    return json.loads(r.stdout.strip().splitlines()[-1])


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

        # 게이트 1 — Studio 처럼 실제로 붙어서 serverInfo 와 채널 광고를 받는가
        probe = ws_probe(str(WW / "perception" / "condaenv" / "bin" / "python"))
        if not probe.get("serverInfo"):
            raise Fail("붙긴 했으나 serverInfo 가 안 옵니다 — 프로토콜 불일치")
        chans = probe.get("channels", [])
        print(f"  게이트 1 접속       serverInfo='{probe['serverInfo']}' · 채널 {len(chans)}개 광고")
        want = [t for t in ("/ww/state/loc_error_cm", "/ww/viz", "/robot/camera") if t in chans]
        print(f"           그중 관제 핵심 토픽: {want}")
        if len(want) < 2:
            raise Fail(f"관제 토픽이 Studio 로 안 나갑니다 (광고된 채널: {chans[:10]})")

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
