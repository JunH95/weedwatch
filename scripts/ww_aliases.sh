#!/usr/bin/env bash
# weedwatch 별칭 — ~/.bashrc 에서 source 해두면 터미널마다 긴 명령을 안 쳐도 된다.
#
#   source ~/projects/weedwatch/scripts/ww_aliases.sh
#
# 이 파일은 **정의만 한다**. 셸을 열 때 환경을 바꾸지 않으므로(conda·PATH 그대로),
# 다른 프로젝트 작업에 영향이 없다. `ww` 를 쳐야 그때 ROS 환경으로 들어간다.

export WW_HOME="${WW_HOME:-$HOME/projects/weedwatch}"

# ww        저장소로 이동 + ROS 환경 준비 (터미널마다 한 번)
#           별칭 정의도 **매번 다시 읽는다** — git pull 로 새 명령이 생겨도 터미널을 새로 안 열어도 된다.
#           (실제로 wwplot 을 추가한 뒤 "명령을 찾을 수 없습니다"가 났다 — 셸이 옛 정의를 들고 있어서.)
ww() {
  cd "$WW_HOME" || return 1
  source "$WW_HOME/scripts/ros_env.sh"
  source "$WW_HOME/scripts/ww_aliases.sh"
  echo "weedwatch 준비됨: $(python3 -V 2>&1), ROS $ROS_DISTRO  (명령 목록: wwhelp)"
}

# wwb       코드 받아서 빌드 (git pull 후에만 필요 — 매번 아님)
wwb() { ww && git pull --ff-only && colcon build && echo "빌드 완료"; }

# 관람 (사람 눈) — 인자로 밭을 고른다: wwsim (매끈) · wwsim dev (현실 밭) · wwsim main (정본)
_wwfield() { [ -n "$1" ] && { make -s "worlds/field_$1.sdf" && echo "field:=$1"; }; }
wwsim()  { ww && ros2 launch weedwatch_bringup skeleton.launch.py gui:=true      $(_wwfield "$1"); }
wwfox()  { ww && ros2 launch weedwatch_bringup skeleton.launch.py foxglove:=true $(_wwfield "$1"); }
wwrviz() { ww && ros2 launch weedwatch_bringup skeleton.launch.py rviz:=true     $(_wwfield "$1"); }
wwplot() { ww && ros2 launch weedwatch_bringup skeleton.launch.py plot:=true     $(_wwfield "$1"); }
wwall()  { ww && ros2 launch weedwatch_bringup skeleton.launch.py gui:=true rviz:=true foxglove:=true $(_wwfield "$1"); }

# 상태 들여다보기 ────────────────────────────────────────────────────────────
wwerr()  { ww && ros2 topic echo /ww/state/loc_error_cm; }     # 위치추정 오차 실시간
wwtop()  { ww && ros2 topic list; }

# 검증 (수치) ────────────────────────────────────────────────────────────────
wwtest() { ww && make test; }          # 순수 단위 (밀리초)
wwturn() { ww && make turn; }          # 두둑 끝 U턴 게이트
wwrun()  { ww && make ros-skeleton; }  # 관통 전체 (헤드리스)

# 정리 — 시뮬은 한 번에 하나만 돌아야 한다. 좀비가 남으면 추정이 통째로 망가진다.
wwkill() { ww && make clean-sim; echo "시뮬·노드 정리됨"; }

wwhelp() {
  cat <<'EOF'
weedwatch 명령
  ww        저장소 + ROS 환경 준비 (터미널마다 한 번)
  wwb       git pull + colcon build (코드 받았을 때만)

  보기 (뒤에 밭 이름을 붙일 수 있다: wwsim dev / wwsim main)
    wwsim     Gazebo — 밖에서 본 로봇
    wwplot    그래프 — 위치오차·방위·속도·슬립 (rqt_plot/PlotJuggler, 계정 불필요)
    wwfox     Foxglove 브리지 — 창은 app.foxglove.dev (계정 필요) 또는 Lichtblick
    wwrviz    rviz2 — 믿는 위치(주황) vs 실제 위치(초록)
    wwall     셋 다 (느림)
    wwerr     위치추정 오차 실시간 숫자

  검증
    wwtest    순수 단위 테스트          wwturn   두둑 끝 U턴 게이트
    wwrun     관통 전체(헤드리스)        wwkill   시뮬 정리 (한 번에 하나만!)

  참고
    · 끝날 때 나오는 "process has died ... exit code -15" 는 **정상 종료**다.
      관통이 끝나면 런치가 나머지를 SIGTERM 으로 내리는데, 브리지가 그걸 그냥 종료로 처리한다.
    · 새 명령이 안 잡히면 ww 를 한 번 더 쳐라 (별칭을 다시 읽는다).
EOF
}
