#!/usr/bin/env bash
# CropCraft(정원 생성기) 전용 진입점. ROS 쪽의 scripts/env.sh 와 짝이지만 요구사항이 정반대다.
#
# ── 이 컴퓨터에는 파이썬이 세 개 있고, 서로 섞이면 안 된다 ─────────────────
#
#   누구                    버전    필요한 것            누가 쓰나
#   ─────────────────────────────────────────────────────────────────
#   miniforge (PATH 1순위)  3.13    (아무것도)           아무도. 방해만 함.
#   시스템 /usr/bin/python3 3.10    click, rclpy         ROS · cropcraft.py 바깥
#   Blender 번들            3.11    yaml,msgpack,PIL     cropcraft 안쪽 (blender --python)
#
# cropcraft.py 는 바깥에서 돌면서 안쪽 blender 를 subprocess 로 띄운다.
# 그래서 두 파이썬을 동시에 만족시켜야 한다.
#
# ── 왜 env.sh 를 못 쓰나 ──────────────────────────────────────────────────
# env.sh 는 PYTHONNOUSERSITE=1 로 user site 를 끈다 (ROS 파이썬을 깨끗하게 하려고).
# 그런데 Blender 의 의존성은 ~/.local/lib/python3.11 에 있다 — snap 이 읽기 전용이라
# pip 이 거기로 물러났기 때문이다. user site 를 끄면 Blender 가 그걸 못 본다.
# 요구가 정반대라서 진입점을 나눈다.
#
# ── PYTHONPATH 를 반드시 지워야 하는 이유 ────────────────────────────────
# cropcraft 는 blender 를 --python-use-system-env 로 띄운다. 그러면 Blender 가
# PYTHONPATH 를 존중하는데, 이 컴퓨터의 PYTHONPATH 에는 ROS 워크스페이스 4개의
# **python3.10** 경로가 들어 있다. Blender 는 3.11 이다. 3.10 용으로 컴파일된
# C 확장을 3.11 이 읽으려다 죽는다.
#
# ── 출력은 models/ 로 간다 ────────────────────────────────────────────────
# scripts/env.sh 가 IGN_GAZEBO_RESOURCE_PATH 를 "$WW/worlds:$WW/models" 로 고정한다
# (상속받은 환경을 씻는 과정에서 기존 값을 버린다). 그래서 Gazebo 가 model://<이름> 을
# 찾으려면 결과물이 models/ 안에 있어야 한다.
# 겸사겸사 third_party/cropcraft (남의 저장소)를 우리 산출물로 더럽히지 않게 된다.
#
# 사용법:
#   scripts/cropcraft.sh <설정.yaml>            → models/ 에 생성
#   scripts/cropcraft.sh <설정.yaml> -d <경로>   → 다른 곳에 생성

set -eo pipefail

WW="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CROPCRAFT="$WW/third_party/cropcraft"

if [ ! -d "$CROPCRAFT" ]; then
  echo "CropCraft 가 없습니다. 먼저: make cropcraft-install" >&2
  exit 1
fi

# 상속받은 환경을 씻는다. 단, user site 는 살린다 (Blender 의존성이 거기 있으므로).
unset PYTHONPATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH \
      CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PKG_CONFIG_PATH \
      PYTHONHOME PYTHONNOUSERSITE CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PATH="/snap/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <설정.yaml> [cropcraft 옵션...]" >&2
  exit 2
fi

CFG="$1"; shift
# 상대 경로로 줘도 되게 절대 경로로 바꾼다 (cd 하기 전에).
case "$CFG" in /*) ;; *) CFG="$PWD/$CFG" ;; esac

# 호출자가 -d 를 안 줬으면 models/ 로 보낸다.
case " $* " in
  *" -d "*|*" --output-dir "*) ;;
  *) mkdir -p "$WW/models"; set -- "$@" -d "$WW/models" ;;
esac

# cropcraft.py 는 자기 디렉토리 기준으로 core/ 와 assets/ 를 찾는다.
cd "$CROPCRAFT"

# 바깥 파이썬은 시스템 3.10 을 명시한다 (click 이 거기 있다).
# 맨 python3 는 miniforge 3.13 이라 click 이 없다.
exec /usr/bin/python3 cropcraft.py "$CFG" "$@"
