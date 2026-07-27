#!/usr/bin/env python3
"""관통(walking skeleton) 전체를 한 줄로 켠다 — DECISIONS 038 P4.

  Gazebo(ign) → ros_gz_bridge → 인식 노드(condaenv) → 코디네이터(제어+판단)

실행(make ros-skeleton 가 감싼다):
  env.sh bash -c "source install/setup.bash && WW_ROOT=<repo> ros2 launch weedwatch_bringup skeleton.launch.py"

env.sh 환경(EGL·정리된 PYTHONPATH·ROS 오버레이) + 워크스페이스 install 을 상속받아 자식들이 돈다.
인식 노드만 condaenv 파이썬으로(torch), 나머지는 시스템 3.10. 코디네이터가 끝나면 전체 종료.
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, TimerAction, RegisterEventHandler,
                            EmitEvent, DeclareLaunchArgument)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from weedwatch_control.control_node import bridge_args

WW = Path(os.environ.get("WW_ROOT", str(Path.cwd())))
CONDA_PY = str(WW / "perception" / "condaenv" / "bin" / "python")
PERCEPT = str(WW / "perception" / "ww_perception_node.py")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
WORLD_NAME = "robot_field_multi"
N_TOOLS = 3
CAM_TOPICS = ["/robot/camera", "/robot/camera1"]


def generate_launch_description():
    bridge = bridge_args(N_TOOLS) + [f"{t}@sensor_msgs/msg/Image[ignition.msgs.Image" for t in CAM_TOPICS]

    # gui:=false(기본) = 헤드리스(에이전트 수치 단언). gui:=true = Gazebo GUI(사람 관람 — 데스크톱).
    gui = LaunchConfiguration("gui")
    declare_gui = DeclareLaunchArgument("gui", default_value="false",
                                        description="Gazebo GUI 표시. 기본 headless(에이전트).")
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
    coord_node = Node(package="weedwatch_coordinator", executable="coordinator_node", output="screen")
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

    # 코디네이터가 끝나면(관통 완료) 런치 전체 종료
    shutdown_on_done = RegisterEventHandler(OnProcessExit(
        target_action=coord_node,
        on_exit=[EmitEvent(event=Shutdown(reason="관통 완료"))]))

    return LaunchDescription([declare_gui, declare_rviz, gazebo_headless, gazebo_gui,
                             bridge_proc, perception, viz_node, rviz_proc,
                             coordinator, shutdown_on_done])
