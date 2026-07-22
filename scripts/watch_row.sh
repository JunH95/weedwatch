#!/usr/bin/env bash
# 사람이 눈으로 보는 용도: robot_row 를 GUI 창으로 띄우고, 정답 좌표로 무정차 주행하며 잡초 위에
# 툴을 내려찍는 모습을 재생한다. `make row` 는 헤드리스(창 없음)라 단언만 찍는데, 이건 같은 제어
# (ww_cmd)로 GUI 에서 움직임을 보여준다. 채점은 make row 가 이미 헤드리스로 한다 — 여기는 관람용.
#
# CLAUDE(에이전트)는 이걸 쓰면 안 된다 — GUI 는 사람 전용(데스크톱 DISPLAY 필요, SSH 안 됨).
#
# env.sh 를 쓰는 이유: sim 과 ww_cmd 가 같은 ign-transport 환경이라야 붙는다(검증된 make row 경로와
# 동일). robot_row 는 카메라 센서가 없어 EGL 서버 렌더가 안 일어나므로, env.sh 의 EGL 고정은
# 무해하고 GUI 창은 데스크톱 GLX 로 그려진다. (만약 GUI 가 이상하면 view.sh 처럼 EGL 고정을 빼면 됨.)
#
# 사용법:  scripts/watch_row.sh
set -eo pipefail
WW="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY 가 없습니다. GUI 는 데스크톱 앞에서만 뜹니다 (SSH 는 안 됨)." >&2
  exit 1
fi

# 이전 좀비 정리
pkill -f "[i]gn gazebo" 2>/dev/null || true
sleep 0.5

echo "1) GUI 로 robot_row 를 띄운다 (물리 실행 -r). 창이 뜨고 로봇이 두둑에 안착한다..."
"$WW/scripts/env.sh" ign gazebo -r "$WW/worlds/robot_row.sdf" &
SIM=$!
trap 'kill $SIM 2>/dev/null; pkill -f "[i]gn gazebo" 2>/dev/null' EXIT
sleep 8   # GUI 창 + ign 토픽이 뜰 때까지

echo "2) ww_cmd 로 무정차 주행+스탬핑 재생 (앞으로 가며 빨강 잡초 위에 툴 하강, 초록 작물은 건너뜀)..."
"$WW/scripts/env.sh" python3 "$WW/tools/drive_row.py" || true

echo ""
echo "재생이 끝났다. GUI 에서 로봇을 돌려보며 살펴봐도 되고, 창을 닫으면 종료된다."
echo "(다시 보려면: scripts/watch_row.sh)"
wait "$SIM"
