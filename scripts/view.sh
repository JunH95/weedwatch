#!/usr/bin/env bash
# 사람이 눈으로 확인하는 용도. GUI 를 띄운다.
#
# ── env.sh 와 뭐가 다른가 ─────────────────────────────────────────────────
# env.sh 는 화면 없는(headless) 실행용이라 EGL 을 NVIDIA 로 고정하고 DISPLAY 를 안 쓴다.
# 이건 정반대다 — 사람의 X 세션(DISPLAY)에 창을 띄운다. 그래서 EGL 고정을 안 한다.
# GUI 는 사용자의 데스크톱 GPU 경로(GLX)를 그대로 쓴다.
#
# CLAUDE(에이전트)는 이걸 쓰면 안 된다. GUI 는 사람 전용이다.
#
# 사용법:
#   scripts/view.sh <월드파일>
#   make view WORLD=worlds/garden_ridge.sdf

set -eo pipefail

WW="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY 가 없습니다. GUI 는 데스크톱 앞에서만 뜹니다 (SSH 는 안 됨)." >&2
  exit 1
fi

WORLD="${1:?월드 파일이 필요합니다. 예: scripts/view.sh worlds/garden_ridge.sdf}"

# model:// 참조를 풀려면 모델·월드 폴더를 알려줘야 한다.
export IGN_GAZEBO_RESOURCE_PATH="$WW/worlds:$WW/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

# -g 없이 실행하면 서버+GUI 가 같이 뜬다 (사람이 3D 로 보고 돌려볼 수 있음).
exec ign gazebo -v 3 "$WORLD"
