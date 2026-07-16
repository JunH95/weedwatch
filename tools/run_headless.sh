#!/usr/bin/env bash
# 헤드리스로 시뮬을 돌려서 카메라 프레임을 디스크에 남긴다.
#
# ── 왜 이 스크립트가 따로 필요한가 ──────────────────────────────────────────
# Gazebo Fortress의 카메라는 "누군가 그 사진을 보고 있을 때만" 렌더링한다.
# 아무도 토픽을 구독하지 않으면 카메라는 조용히 아무 일도 안 한다.
# 에러도, 경고도 없다. 그냥 사진이 0장 나온다.
#
# 실측 (2026-07-16):
#   구독자 없음 → PNG   0장
#   구독자 있음 → PNG 121장
#
# 월드 파일의 <save> 태그도 "렌더링" 과정 안에서 파일을 쓰기 때문에,
# 렌더링을 안 하면 저장도 같이 죽는다. 그래서 사진이 필요하면 반드시
# 구독자를 하나 붙여줘야 한다. 이 스크립트가 그 역할이다.
#
# 사용법:
#   tools/run_headless.sh <월드파일> <이미지토픽> <스텝수>
#   tools/run_headless.sh worlds/smoke.sdf /smoke/image 12000

set -euo pipefail

WORLD="${1:?월드 파일 경로가 필요합니다}"
TOPIC="${2:?이미지 토픽 이름이 필요합니다}"
ITERS="${3:-12000}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_SH="$HERE/scripts/env.sh"
LOG="$HERE/artifacts/sim.log"
mkdir -p "$HERE/artifacts"

# 죽다 만 서버가 남아 있으면 다음 실행이 낡은 상태를 재사용한다.
# 그러면 "통과"가 통과가 아니게 되므로 항상 먼저 청소한다.
# 대괄호가 중요하다: pkill -f 'ign gazebo'는 자기 자신의 명령줄에도
# 'ign gazebo'가 들어 있어서 스스로를 죽인다. '[i]gn'은 실제 프로세스만 잡는다.
pkill -f '[i]gn gazebo' 2>/dev/null || true
sleep 0.3

# 시뮬레이터 실행.
#   -s                    서버만 (렌더링은 서버 쪽에 있으므로 이걸로 충분)
#   -r                    필수. 없으면 시뮬이 일시정지 상태라 시계가 안 흐르고
#                         카메라가 한 장도 안 찍는다 — 증상이 EGL 고장과 똑같다.
#   --headless-rendering  화면 없이 GPU로 렌더링 (EGL)
#   --iterations N        N스텝 돌고 스스로 종료. 좀비 프로세스가 안 남는다.
"$ENV_SH" ign gazebo -s -r --headless-rendering --iterations "$ITERS" "$WORLD" \
  >"$LOG" 2>&1 &
SIM_PID=$!

# 렌더링 스레드가 ogre2 + EGL 컨텍스트를 만드는 데 1~2초 걸린다.
# 그 전에 구독하면 토픽이 아직 없다.
sleep 4

# 구독자를 붙인다 = 카메라에게 "찍어라"라고 말하는 것.
# 내용물은 안 쓰고 버린다. 목적은 오직 렌더링을 깨우는 것.
"$ENV_SH" ign topic -e -t "$TOPIC" >/dev/null 2>&1 &
SUB_PID=$!

# 스크립트가 중간에 죽어도 구독자는 반드시 정리한다.
trap 'kill $SUB_PID 2>/dev/null || true' EXIT

# 시뮬이 --iterations 만큼 다 돌 때까지 기다린다.
wait "$SIM_PID"
SIM_RC=$?

kill "$SUB_PID" 2>/dev/null || true

if [ "$SIM_RC" -ne 0 ]; then
  echo "시뮬레이터가 비정상 종료했습니다 (exit $SIM_RC). 로그 끝부분:" >&2
  tail -20 "$LOG" >&2
  exit "$SIM_RC"
fi
