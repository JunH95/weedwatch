#!/usr/bin/env bash
# perception(ML) 학습/평가 전용 진입점 — ROS 를 씻어낸 깨끗한 venv.
#
# ── 왜 씻나 ───────────────────────────────────────────────────────────────
# 이 컴퓨터의 ~/.bashrc 가 ROS 워크스페이스들의 python3.10 경로를 PYTHONPATH 에 밀어넣는다.
# venv 도 3.10 이라 그대로 두면 venv 의 torch/numpy 대신 ROS 쪽 패키지가 섞여 들어온다
# (가장 나쁜 버그 — 조용히 틀린 걸 import). 학습·평가는 ROS 가 전혀 필요 없으므로 다 씻고
# venv 만 본다. train/eval 을 결정적으로 재현하려면 이 격리가 맞다.
#
# ── ROS 노드로 쓸 땐 다른 진입점 ─────────────────────────────────────────
# venv 는 3.10 (rclpy 와 같은 ABI, DECISIONS 038) → 한 프로세스에 torch+rclpy 공존 가능.
# 그래서 인식을 ROS 노드로 돌릴 땐 ROS 를 씻지 않는다:  scripts/env.sh perception/condaenv/bin/python <node>
# (ROS 경로 유지 + venv 패키지 우선). 여기 env.sh 는 ROS 안 쓰는 학습·평가용.
#
# 사용:
#   perception/env.sh python train.py ...                     # 학습/평가 (ROS 씻김)
#   scripts/env.sh perception/condaenv/bin/python <ros_node>.py  # 인식 ROS 노드 (torch+rclpy)
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/condaenv"

if [ ! -x "$VENV/bin/python" ]; then
  echo "perception conda 환경이 없습니다. 먼저: make perception-venv" >&2
  exit 1
fi

# 상속받은 ROS/conda 오염을 씻는다. venv 는 자기 site-packages 만 봐야 한다.
unset PYTHONPATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH \
      CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PKG_CONFIG_PATH \
      PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PYTHONNOUSERSITE=1
export PATH="$VENV/bin:$PATH"

exec "$@"
