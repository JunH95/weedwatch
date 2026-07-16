# 이 저장소의 모든 명령은 scripts/env.sh 를 통과한다.
# 이유는 그 파일 맨 위 주석 참고 — 요약하면 이 컴퓨터의 python3 는 ROS가 못 쓰는
# 버전이고, 남의 워크스페이스 4개가 환경변수에 섞여 들어온다.

ENV := ./scripts/env.sh
# 12초 분량. 렌더링 스레드가 ogre2+EGL 컨텍스트를 만드는 데 1~2초 걸리므로
# 그보다 넉넉히 줘야 한다. (make는 값 뒤 공백까지 변수에 넣으므로 주석은 윗줄에)
SMOKE_ITERS ?= 12000

.PHONY: help doctor test smoke garden clean-sim clean

help:
	@echo "make doctor    - 환경이 멀쩡한지 단언 (파이썬 3.10 / rclpy / EGL / NVIDIA)"
	@echo "make test      - 순수 단위 테스트 (시뮬·GPU 불필요, 밀리초)"
	@echo "make smoke     - 헤드리스 GPU 렌더링 전 과정 + 게이트 2개 단언"
	@echo "make garden    - 주말농장 지형 생성 + 렌더 (기하학 눈으로 확인용)"
	@echo "make clean-sim - 좀비 ign 서버 정리"

# 산수로 답할 수 있는 건 시뮬로 확인하지 않는다. 느리고 불안정하고 GPU가 필요하니까.
# "로봇이 두둑을 탈 수 있나"는 산수다. 밀리초 안에 끝난다.
test:
	@$(ENV) python3 -m pytest tests/ -q

# 지형 생성 + 렌더. 기하학이 말이 되는지 사람(과 에이전트)이 눈으로 보는 용도.
# 진짜 검증은 make test 가 한다 — 눈으로 보는 건 확장이 안 된다.
garden: clean-sim
	@$(ENV) python3 tools/make_garden_world.py > worlds/garden_ridge.sdf
	@$(ENV) ign sdf -k worlds/garden_ridge.sdf
	@rm -rf artifacts/garden && mkdir -p artifacts/garden
	@tools/run_headless.sh worlds/garden_ridge.sdf /garden/inspect $(SMOKE_ITERS)
	@$(ENV) python3 tools/assert_render.py artifacts/garden

# 환경 건강검진. 뭔가 이상하면 여기서 먼저 걸린다.
doctor:
	@echo "== 파이썬 인터프리터 =="
	@$(ENV) python3 -c 'import sys; assert sys.version_info[:2]==(3,10), f"파이썬이 {sys.version.split()[0]} 입니다. 3.10 이어야 합니다"; print("  python", sys.version.split()[0])'
	@echo "== ROS =="
	@$(ENV) python3 -c 'import rclpy; print("  rclpy ok")'
	@echo "== 남의 워크스페이스가 안 새어들어왔는가 =="
	@$(ENV) python3 -c 'import sys; bad=[p for p in sys.path if any(w in p for w in ("rmf_ws","movebot_ws","colcon_ws","micro_ros_ws","miniforge"))]; assert not bad, f"새어들어옴: {bad}"; print("  sys.path 깨끗함")'
	@echo "== 이미지 처리 =="
	@$(ENV) python3 -c 'import numpy, PIL; print("  numpy", numpy.__version__, "/ pillow", PIL.__version__)'
	@echo "== 시뮬레이터 =="
	@$(ENV) sh -c 'ign gazebo --version | head -1 | sed "s/^/  /"'
	@echo "== EGL이 NVIDIA로 고정됐는가 =="
	@$(ENV) sh -c 'test -f "$$__EGL_VENDOR_LIBRARY_FILENAMES" && echo "  $$__EGL_VENDOR_LIBRARY_FILENAMES"'
	@echo "== X 없이 GPU가 보이는가 =="
	@$(ENV) sh -c 'test -c /dev/nvidia0 && echo "  /dev/nvidia0 있음 (권한 0666, X 세션 불필요)"'
	@echo "doctor: OK"

# 이 프로젝트 전체가 성립하는지를 묻는 시험.
# 통과 = "사람이 안 보고 있어도 시뮬을 돌려서 진짜 GPU 사진을 받아올 수 있다"
smoke: clean-sim
	@rm -rf artifacts/smoke && mkdir -p artifacts/smoke
	@echo "== 헤드리스 렌더링 ($(SMOKE_ITERS) 스텝) =="
	@tools/run_headless.sh worlds/smoke.sdf /smoke/image $(SMOKE_ITERS)
	@echo "== 단언 =="
	@$(ENV) python3 tools/assert_render.py artifacts/smoke

# ign 서버는 테스트가 죽어도 살아남는다. 그 상태로 다음 테스트를 돌리면
# 낡은 서버를 재사용해서 "통과"가 통과가 아니게 된다. 항상 먼저 청소한다.
#
# 대괄호가 중요하다: pkill -f 'ign gazebo' 는 자기 자신의 명령줄에도
# 'ign gazebo' 가 들어 있어서 스스로를 죽인다. '[i]gn' 은 진짜 프로세스만 잡는다.
clean-sim:
	@pkill -f '[i]gn gazebo' 2>/dev/null || true
	@pkill -f '[i]gn-gazebo-server' 2>/dev/null || true
	@sleep 0.3

clean: clean-sim
	@rm -rf artifacts build install log
