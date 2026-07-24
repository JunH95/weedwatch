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
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from weedwatch_control.control_node import bridge_args

WW = Path(os.environ.get("WW_ROOT", str(Path.cwd())))
CONDA_PY = str(WW / "perception" / "condaenv" / "bin" / "python")
PERCEPT = str(WW / "perception" / "ww_perception_node.py")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
N_TOOLS = 3
CAM_TOPICS = ["/robot/camera", "/robot/camera1"]


def generate_launch_description():
    bridge = bridge_args(N_TOOLS) + [f"{t}@sensor_msgs/msg/Image[ignition.msgs.Image" for t in CAM_TOPICS]

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD],
        output="screen")
    bridge_proc = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge], output="screen")])
    perception = TimerAction(period=7.0, actions=[ExecuteProcess(
        cmd=[CONDA_PY, PERCEPT], output="screen")])
    coord_node = Node(package="weedwatch_coordinator", executable="coordinator_node", output="screen")
    coordinator = TimerAction(period=11.0, actions=[coord_node])

    # 코디네이터가 끝나면(관통 완료) 런치 전체 종료
    shutdown_on_done = RegisterEventHandler(OnProcessExit(
        target_action=coord_node,
        on_exit=[EmitEvent(event=Shutdown(reason="관통 완료"))]))

    return LaunchDescription([gazebo, bridge_proc, perception, coordinator, shutdown_on_done])
