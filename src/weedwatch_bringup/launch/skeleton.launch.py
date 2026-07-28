#!/usr/bin/env python3
"""관통(walking skeleton) 전체를 한 줄로 켠다 — DECISIONS 038 P4.

  Gazebo(ign) → ros_gz_bridge → 인식 노드(condaenv) → 코디네이터(제어+판단)

실행:
  source scripts/ros_env.sh
  ros2 launch weedwatch_bringup skeleton.launch.py gui:=true              # 매끈한 밭, 눈으로
  ros2 launch weedwatch_bringup skeleton.launch.py field:=dev gui:=true   # 현실 밭
  ros2 launch weedwatch_bringup skeleton.launch.py field:=dev vo:=true    # 카메라 융합까지

env.sh 환경(EGL·정리된 PYTHONPATH·ROS 오버레이) + 워크스페이스 install 을 상속받아 자식들이 돈다.
인식 노드만 condaenv 파이썬으로(torch), 나머지는 시스템 3.10. 코디네이터가 끝나면 전체 종료.
"""
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, TimerAction, RegisterEventHandler,
                            EmitEvent, DeclareLaunchArgument, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from weedwatch_control.control_node import bridge_args
from weedwatch_control.ww_paths import find_repo_root

# 저장소 루트는 환경변수 없이도 찾는다 — `ros2 launch` 를 직접 써도 되게(2026-07-27 버그).
WW = find_repo_root()
CONDA_PY = str(WW / "perception" / "condaenv" / "bin" / "python")
PERCEPT = str(WW / "perception" / "ww_perception_node.py")
VO_NODE = str(WW / "perception" / "ww_vo_node.py")
def _field_from_cli() -> str:
    """`ros2 launch ... field:=dev` 를 읽는다.

    월드 경로는 런치를 **만드는 시점**에 정해져야 해서(ExecuteProcess 의 cmd) 런치 인자 치환
    (LaunchConfiguration)으로는 늦다. 그래서 argv 를 직접 본다. 환경변수(WW_FIELD)도 받는다 —
    make 가 그쪽을 쓴다.
    """
    for a in sys.argv:
        if a.startswith("field:="):
            return a.split(":=", 1)[1].strip()
    return os.environ.get("WW_FIELD", "")


# 밭 선택: field:=dev 면 현실적인 밭(042). 기본은 기존 매끈한 밭(기준선).
FIELD = _field_from_cli()
if FIELD:
    WORLD = str(WW / "worlds" / f"field_{FIELD}.sdf")
    WORLD_NAME = f"field_{FIELD}"
else:
    WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
    WORLD_NAME = "robot_field_multi"
N_TOOLS = 3
CAM_TOPICS = ["/robot/camera", "/robot/camera1"]


def generate_launch_description():
    # 깊이도 브리지한다 — VO 가 "흙 픽셀만 고르기"에 쓴다(스케일이 아니라 선택, 041).
    bridge = (bridge_args(N_TOOLS)
              + [f"{t}@sensor_msgs/msg/Image[ignition.msgs.Image" for t in CAM_TOPICS]
              + ["/robot/depth@sensor_msgs/msg/Image[ignition.msgs.Image"])

    # gui:=false(기본) = 헤드리스(에이전트 수치 단언). gui:=true = Gazebo GUI(사람 관람 — 데스크톱).
    gui = LaunchConfiguration("gui")
    declare_gui = DeclareLaunchArgument("gui", default_value="false",
                                        description="Gazebo GUI 표시. 기본 headless(에이전트).")
    # 문서용 선언 — 실제 값은 위 _field_from_cli() 가 argv 에서 먼저 읽는다(월드 경로가 먼저 필요).
    declare_field = DeclareLaunchArgument("field", default_value="",
                                          description="밭: 빈값=매끈(기준선) · dev=현실 · main=정본")
    gazebo_headless = ExecuteProcess(
        condition=UnlessCondition(gui),
        cmd=["ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD], output="screen")
    gazebo_gui = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=["ign", "gazebo", "-r", WORLD], output="screen")   # 서버+GUI (사람이 봄)
    bridge_proc = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge], output="screen")])
    perception = TimerAction(period=7.0, actions=[ExecuteProcess(
        cmd=[CONDA_PY, PERCEPT], output="screen")])
    # 시각 오도메트리 — 회전 중 몸통 미끄러짐(바퀴가 못 보는 것)을 메운다. vo:=false 로 끄면
    # 예전처럼 휠+IMU 만으로 돈다(A/B 측정용).
    # 기본 off — 실측에서 이득을 못 봤다(아래 041 후속). 노드·융합 경로는 남겨두고 vo:=true 로 켠다.
    vo = LaunchConfiguration("vo")
    declare_vo = DeclareLaunchArgument("vo", default_value="false",
                                       description="시각 오도메트리 융합(실험). 기본 off — 041 후속 참고")
    vo_proc = TimerAction(period=7.0, actions=[ExecuteProcess(
        condition=IfCondition(vo), cmd=[CONDA_PY, VO_NODE], output="screen")])

    coord_node = Node(package="weedwatch_coordinator", executable="coordinator_node",
                      output="screen", additional_env={"FIELD": FIELD})
    coordinator = TimerAction(period=11.0, actions=[coord_node])

    # 관제 화면(rviz2) — 사람이 "로봇 머릿속"을 본다: 믿는 자세 vs 실제 자세, 검출한 잡초, 카메라.
    # 지상진실은 viz 노드가 ign 스트림에서 직접 읽는다(ROS 로 안 흘린다). 제어 노드가 구독할 수 있는
    # ROS 토픽으로 만들지 않는 게 요점 — "제어는 추정, 채점은 지상진실" 규율을 화면 때문에 무르지 않는다.
    rviz = LaunchConfiguration("rviz")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false",
                                         description="rviz2 관제 화면(사람용). 에이전트는 headless.")
    viz_node = TimerAction(period=8.0, actions=[Node(
        condition=IfCondition(rviz), package="weedwatch_bringup", executable="viz_node",
        output="screen")])
    rviz_proc = TimerAction(period=9.0, actions=[ExecuteProcess(
        condition=IfCondition(rviz),
        cmd=["rviz2", "-d", str(WW / "src" / "weedwatch_bringup" / "config" / "weedwatch.rviz")],
        output="screen")])

    # Foxglove — 그래프(Plot)·로봇 상태·영상·3D 를 한 창에서 보는 관제 도구 (PLAN Stage 7).
    # foxglove_bridge 가 WebSocket(8765)으로 ROS 그래프를 통째로 내보내고, Foxglove Studio
    # (데스크톱 앱 또는 app.foxglove.dev)가 붙는다. rviz 와 달리 **시계열 그래프**가 된다.
    # viz 노드도 같이 켠다 — /ww/state/* 수치와 믿음/실제 마커가 거기서 나온다.
    fox = LaunchConfiguration("foxglove")
    declare_fox = DeclareLaunchArgument("foxglove", default_value="false",
                                        description="foxglove_bridge(WebSocket 8765) 기동")
    fox_viz = TimerAction(period=8.0, actions=[Node(
        condition=IfCondition(fox), package="weedwatch_bringup", executable="viz_node",
        output="screen")])
    fox_bridge = TimerAction(period=8.0, actions=[ExecuteProcess(
        condition=IfCondition(fox),
        cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
        output="screen")])
    # foxglove:=true 는 **서버(브리지)만** 켠다. 보는 창은 Foxglove Studio(앱 또는 웹)다 —
    # 이 안내가 없으면 "켰는데 아무것도 안 뜬다"가 된다(2026-07-27 실제로 그랬다).
    fox_hint = TimerAction(period=10.0, actions=[LogInfo(condition=IfCondition(fox), msg=(
        "\n" + "=" * 72 +
        "\n  Foxglove 브리지가 떴다. **보는 창은 따로 연다** — 둘 중 하나:\n"
        "    · 웹  : https://app.foxglove.dev  →  Open connection  →  ws://localhost:8765\n"
        "    · 앱  : snap install foxglove-studio   (설치 후 같은 주소로 접속)\n"
        "  레이아웃 import: src/weedwatch_bringup/config/weedwatch.foxglove.json\n" +
        "=" * 72))])

    # 그래프만 보고 싶을 때 — 계정도 설치도 필요 없는 경로. Foxglove 는 계정을 요구하지만
    # rqt_plot 은 ROS 기본이라 이미 있고, PlotJuggler 를 깔면(apt, 계정 없음) 훨씬 낫다.
    plot = LaunchConfiguration("plot")
    declare_plot = DeclareLaunchArgument("plot", default_value="false",
                                         description="그래프 창(rqt_plot 또는 plotjuggler)")
    plot_viz = TimerAction(period=8.0, actions=[Node(
        condition=IfCondition(plot), package="weedwatch_bringup", executable="viz_node",
        output="screen")])
    plot_proc = TimerAction(period=10.0, actions=[ExecuteProcess(
        condition=IfCondition(plot),
        cmd=["bash", "-c",
             # PlotJuggler 가 있으면 그걸로(레이아웃·다축·줌 다 됨), 없으면 rqt_plot 으로 폴백.
             "if ros2 pkg prefix plotjuggler_ros >/dev/null 2>&1; then "
             "  ros2 run plotjuggler plotjuggler; "
             "else "
             "  echo '[안내] PlotJuggler 를 깔면 더 낫다: sudo apt install ros-humble-plotjuggler-ros'; "
             "  rqt_plot /ww/state/loc_error_cm/data /ww/state/heading_error_deg/data "
             "           /ww/state/speed_mps/data /ww/state/gyro_vs_wheel_cm/data; "
             "fi"],
        output="screen")])

    # 코디네이터가 끝나면(관통 완료) 런치 전체 종료
    shutdown_on_done = RegisterEventHandler(OnProcessExit(
        target_action=coord_node,
        on_exit=[EmitEvent(event=Shutdown(reason="관통 완료"))]))

    return LaunchDescription([declare_gui, declare_field, declare_rviz, declare_fox,
                             declare_plot, declare_vo,
                             gazebo_headless, gazebo_gui, bridge_proc, perception,
                             viz_node, rviz_proc, fox_viz, fox_bridge, fox_hint,
                             plot_viz, plot_proc, vo_proc,
                             coordinator, shutdown_on_done])
