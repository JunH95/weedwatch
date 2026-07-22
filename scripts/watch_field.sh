#!/usr/bin/env bash
# 사람 눈 관람용(데스크톱 전용): 사실적 밭(사면 두둑 + 고랑 + CropCraft 식물) 위를 로봇이 무정차
# 주행하며 오라클 잡초 좌표로 스탬핑하는 모습을 GUI 로 재생한다. watch-row 의 사실적 밭 버전.
# 표적은 아직 카메라 라이브가 아니라 정답 좌표(카메라 검출로 바꾸는 건 Phase 4b-3).
#
# CLAUDE(에이전트)는 못 씀 — GUI 는 사람 전용(DISPLAY 필요).
#
# 관람엔 카메라 렌더가 불필요하므로 sensors 플러그인을 뺀 임시 월드를 쓴다 — GUI(GLX)와 센서
# 렌더(EGL)가 한 프로세스에서 부딪히는 걸 원천 회피(watch-row 처럼 깔끔). 4b-3 라이브는 원본 사용.
#
# 사용법:  scripts/watch_field.sh
set -eo pipefail
WW="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY 가 없습니다. GUI 는 데스크톱 앞에서만 (SSH 안 됨)." >&2
  exit 1
fi

pkill -f "[i]gn gazebo" 2>/dev/null || true
sleep 0.5

WATCH="/tmp/ww_robot_field_watch.sdf"
sed '/ignition::gazebo::systems::Sensors/,/<\/plugin>/d' "$WW/worlds/robot_field.sdf" > "$WATCH"

echo "1) GUI 로 사실적 밭을 띄운다 (사면 두둑 + 고랑 + CropCraft 식물). 로봇이 안착한다..."
"$WW/scripts/env.sh" ign gazebo -r "$WATCH" &
SIM=$!
trap 'kill $SIM 2>/dev/null; pkill -f "[i]gn gazebo" 2>/dev/null' EXIT
sleep 8

echo "2) 오라클 잡초 좌표로 무정차 주행+스탬핑 재생 (실식물 사이를 달리며 툴 하강)..."
"$WW/scripts/env.sh" python3 "$WW/tools/drive_field.py" || true

echo ""
echo "재생이 끝났다. GUI 에서 로봇을 돌려보며 두둑·고랑·식물을 살펴봐도 되고, 창을 닫으면 종료."
echo "(다시 보려면: scripts/watch_field.sh)"
wait "$SIM"
