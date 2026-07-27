#!/usr/bin/env python3
"""상주 시뮬 런치 — Gazebo(GUI) + 브리지 + 인식 노드를 켜두고 유지 (코디네이터 없음).

사람 관람·교차검증용: 한 번 켜두고, 다른 터미널에서 코디네이터를 붙여 주행을 주입한다.
직결판 sim-live + field-attach 를 ROS-native 로 대체(DECISIONS 038).

  터미널1:  make ros-sim      # 이 런치 — Gazebo GUI 창을 열어둔다 (닫기 전까지 삶)
  터미널2:  make ros-attach    # 코디네이터를 붙여 관통 주행 주입 → 터미널1에서 로봇이 움직인다

기본 gui:=true (관람 목적). 에이전트 헤드리스 단언은 make ros-skeleton(코디네이터까지 한 번에).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from weedwatch_control.control_node import bridge_args
from weedwatch_control.ww_paths import find_repo_root

# 저장소 루트는 환경변수 없이도 찾는다 — `ros2 launch` 를 직접 써도 되게(2026-07-27 버그).
WW = find_repo_root()
CONDA_PY = str(WW / "perception" / "condaenv" / "bin" / "python")
PERCEPT = str(WW / "perception" / "ww_perception_node.py")
WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
N_TOOLS = 3
CAM_TOPICS = ["/robot/camera", "/robot/camera1"]


def generate_launch_description():
    bridge = bridge_args(N_TOOLS) + [f"{t}@sensor_msgs/msg/Image[ignition.msgs.Image" for t in CAM_TOPICS]
    gui = LaunchConfiguration("gui")
    declare_gui = DeclareLaunchArgument("gui", default_value="true",
                                        description="Gazebo GUI 표시(관람). false 면 headless.")
    gazebo_gui = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=["ign", "gazebo", "-r", WORLD], output="screen")
    gazebo_headless = ExecuteProcess(
        condition=UnlessCondition(gui),
        cmd=["ign", "gazebo", "-s", "-r", "--headless-rendering", WORLD], output="screen")
    bridge_proc = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge], output="screen")])
    perception = TimerAction(period=7.0, actions=[ExecuteProcess(
        cmd=[CONDA_PY, PERCEPT], output="screen")])
    return LaunchDescription([declare_gui, gazebo_gui, gazebo_headless, bridge_proc, perception])
