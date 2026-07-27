#!/usr/bin/env bash
# weedwatch 별칭 — ~/.bashrc 에서 source 해두면 터미널마다 긴 명령을 안 쳐도 된다.
#
#   source ~/projects/weedwatch/scripts/ww_aliases.sh
#
# 이 파일은 **정의만 한다**. 셸을 열 때 환경을 바꾸지 않으므로(conda·PATH 그대로),
# 다른 프로젝트 작업에 영향이 없다. `ww` 를 쳐야 그때 ROS 환경으로 들어간다.

export WW_HOME="${WW_HOME:-$HOME/projects/weedwatch}"

# ww        저장소로 이동 + ROS 환경 준비 (터미널마다 한 번)
ww() { cd "$WW_HOME" && source "$WW_HOME/scripts/ros_env.sh" && echo "weedwatch 준비됨: $(python3 -V), ROS $ROS_DISTRO"; }

# wwb       코드 받아서 빌드 (git pull 후에만 필요 — 매번 아님)
wwb() { ww && git pull --ff-only && colcon build && echo "빌드 완료"; }

# 관람 (사람 눈) ─────────────────────────────────────────────────────────────
wwsim()  { ww && ros2 launch weedwatch_bringup skeleton.launch.py gui:=true; }        # Gazebo
wwfox()  { ww && ros2 launch weedwatch_bringup skeleton.launch.py foxglove:=true; }   # 그래프·상태
wwrviz() { ww && ros2 launch weedwatch_bringup skeleton.launch.py rviz:=true; }       # 3D 믿음vs실제
wwall()  { ww && ros2 launch weedwatch_bringup skeleton.launch.py gui:=true rviz:=true foxglove:=true; }

# 상태 들여다보기 ────────────────────────────────────────────────────────────
wwerr()  { ww && ros2 topic echo /ww/state/loc_error_cm; }     # 위치추정 오차 실시간
wwtop()  { ww && ros2 topic list; }

# 검증 (수치) ────────────────────────────────────────────────────────────────
wwtest() { ww && make test; }          # 순수 단위 (밀리초)
wwturn() { ww && make turn; }          # 두둑 끝 U턴 게이트
wwrun()  { ww && make ros-skeleton; }  # 관통 전체 (헤드리스)

# 정리 — 시뮬은 한 번에 하나만 돌아야 한다. 좀비가 남으면 추정이 통째로 망가진다.
wwkill() { ww && make clean-sim && pkill -f 'ros2 launch' 2>/dev/null; echo "시뮬 정리됨"; }

wwhelp() {
  cat <<'EOF'
weedwatch 명령
  ww        저장소 + ROS 환경 준비 (터미널마다 한 번)
  wwb       git pull + colcon build (코드 받았을 때만)

  보기
    wwsim     Gazebo — 밖에서 본 로봇
    wwfox     Foxglove 브리지 — 그래프·로봇 상태 (창은 app.foxglove.dev 에서 ws://localhost:8765)
    wwrviz    rviz2 — 믿는 위치(주황) vs 실제 위치(초록)
    wwall     셋 다 (느림)
    wwerr     위치추정 오차 실시간 숫자

  검증
    wwtest    순수 단위 테스트          wwturn   두둑 끝 U턴 게이트
    wwrun     관통 전체(헤드리스)        wwkill   시뮬 정리 (한 번에 하나만!)
EOF
}
