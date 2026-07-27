#!/usr/bin/env bash
# 에이전트용 래퍼 — 한 방에 자기완결적으로 명령 하나를 실행한다.
#
#   ./scripts/env.sh python3 -c 'import rclpy; print("ok")'
#   ./scripts/env.sh ign gazebo -s -r --headless-rendering worlds/X.sdf
#
# Bash 호출 사이에 셸 상태가 안 남으므로(에이전트 제약) 매번 이걸 통과해야 한다.
# **사람은 이걸 쓸 필요가 없다** — 한 번만 `source scripts/ros_env.sh` 하고
# 그 셸에서 평범하게 `ros2 launch ...` / `rviz2` / `ros2 topic echo ...` 를 쓰면 된다.
#
# 환경 설정 자체는 scripts/ros_env.sh 한 곳에만 있다. 두 진입점이 같은 파일을 읽으므로
# "에이전트 경로에서만 되고 사람 경로에서는 안 되는" 상태가 생길 수 없다.
# 왜 이런 게 필요한지는 ros_env.sh 주석 참고 (python 3.13 그림자 · 남의 워크스페이스
# PYTHONPATH 주입 · EGL 이 인텔 내장을 잡음 · Gazebo 리소스 경로).

set -eo pipefail

_WW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$_WW_DIR/ros_env.sh"
unset _WW_DIR

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <command> [args...]" >&2
  echo "사람이라면:  source scripts/ros_env.sh   후 ros2 명령을 그대로 쓰세요" >&2
  exit 2
fi

exec "$@"
