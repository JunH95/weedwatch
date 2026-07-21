#!/usr/bin/env bash
# perception(ML) 전용 진입점. ROS 쪽 scripts/env.sh 와 격리된다 — 다른 venv, 다른 세계.
#
# ── 왜 필요한가 ───────────────────────────────────────────────────────────
# 이 컴퓨터의 ~/.bashrc 가 ROS 워크스페이스 4개의 python3.10 경로를 PYTHONPATH 에 밀어넣는다.
# perception venv 도 3.10 이라, 그대로 두면 venv 의 torch/numpy 대신 ROS 쪽 패키지가 섞여
# 들어온다(가장 나쁜 종류의 버그 — 조용히 틀린 걸 import). 그래서 PYTHONPATH 와 ROS 변수를
# 씻고, venv 를 PATH 앞에 둔다.
#
# ── ROS 와의 계약은 import 가 아니라 디스크 파일이다 ──────────────────────
# CLAUDE.md: "ML쪽은 별도 venv. 둘을 import 로 잇지 마라." 학습 결과는 models/best.pt 로 나가고
# ROS 는 그 파일만 읽는다. 이 두 파이썬은 서로를 절대 import 하지 않는다.
#
# 사용:
#   perception/env.sh python train.py ...
#   perception/env.sh python -c "import torch; print(torch.cuda.is_available())"
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"

if [ ! -x "$VENV/bin/python" ]; then
  echo "perception venv 가 없습니다. 먼저: make perception-venv" >&2
  exit 1
fi

# 상속받은 ROS/conda 오염을 씻는다. venv 는 자기 site-packages 만 봐야 한다.
unset PYTHONPATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH \
      CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PKG_CONFIG_PATH \
      PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PYTHONNOUSERSITE=1
export PATH="$VENV/bin:$PATH"

exec "$@"
