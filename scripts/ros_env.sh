#!/usr/bin/env bash
# 사람이 쓰는 진입점 — **source 해서** 셸을 ROS 작업 상태로 만든다.
#
#   source scripts/ros_env.sh
#   ros2 launch weedwatch_bringup skeleton.launch.py foxglove:=true
#   ros2 topic echo /ww/state/loc_error_cm
#   rviz2 -d src/weedwatch_bringup/config/weedwatch.rviz
#
# `make ...` 타깃은 이걸 감싼 얇은 껍데기다(에이전트는 Bash 호출 사이에 셸 상태가 안 남아서
# 매번 자기완결적이어야 한다). 사람은 굳이 make 를 거칠 필요가 없다 — 한 번 source 하고
# 그 셸에서 평범하게 ros2 명령을 쓰면 된다.
#
# 그냥 `source /opt/ros/humble/setup.bash` 만 하면 안 되는 이유 (전부 이 컴퓨터에서 확인):
#   1. python3 가 miniforge 3.13 → rclpy import 실패 (ROS 는 3.10 용 빌드)
#   2. ~/.bashrc 가 남의 워크스페이스 4개를 PYTHONPATH 에 주입
#   3. EGL 기본값이 인텔 내장 그래픽을 잡아 카메라가 검게 나오거나 100배 느려진다
#   4. Gazebo 가 worlds/·models/ 를 못 찾는다
# env.sh(에이전트용 래퍼)와 **같은 파일을 공유**하므로 두 경로가 어긋날 수 없다.

# shellcheck disable=SC2296
_WW_SELF="${BASH_SOURCE[0]:-$0}"
WW="$(cd "$(dirname "${_WW_SELF}")/.." && pwd)"
export WW

# --- 1. 물려받은 환경 씻어내기 ---------------------------------------------
unset PYTHONPATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH \
      CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PKG_CONFIG_PATH \
      PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE \
      IGN_GAZEBO_RESOURCE_PATH IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export PYTHONNOUSERSITE=1

# --- 2. EGL 을 NVIDIA ICD 로 고정 -------------------------------------------
# libglvnd: 이 목록에 적힌 ICD 만 로드된다 → EGL 장치 0번이 RTX 4060 이 된다.
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
# Fortress 는 카메라를 서버에서 렌더한다. EGL 은 ogre2 전용 — ogre 로 폴백하면 조용히 죽는다.
export IGN_GAZEBO_RENDER_ENGINE=ogre2

# 병렬 실행이 DDS 로 서로 간섭하지 않게.
export ROS_DOMAIN_ID="${WW_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1

# --- 3. ROS 오버레이 (setup.bash 가 set -u 에서 넘어져서 잠시 끈다) ----------
_ww_had_u=0
case "$-" in *u*) _ww_had_u=1; set +u ;; esac
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$WW/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WW/install/setup.bash"
fi
[ "$_ww_had_u" = 1 ] && set -u
unset _ww_had_u _WW_SELF

# --- 4. 프로젝트 리소스 경로 -------------------------------------------------
export IGN_GAZEBO_RESOURCE_PATH="$WW/worlds:$WW/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
