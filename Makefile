# 이 저장소의 모든 명령은 scripts/env.sh 를 통과한다.
# 이유는 그 파일 맨 위 주석 참고 — 요약하면 이 컴퓨터의 python3 는 ROS가 못 쓰는
# 버전이고, 남의 워크스페이스 4개가 환경변수에 섞여 들어온다.

ENV := ./scripts/env.sh
# 12초 분량. 렌더링 스레드가 ogre2+EGL 컨텍스트를 만드는 데 1~2초 걸리므로
# 그보다 넉넉히 줘야 한다. (make는 값 뒤 공백까지 변수에 넣으므로 주석은 윗줄에)
SMOKE_ITERS ?= 12000

.PHONY: help doctor test smoke garden drive joints straddle camera view blender-gpu cropcraft clean-sim clean

# 사람이 GUI 로 직접 3D 확인. 데스크톱 앞에서만 (SSH 불가).
# 에이전트의 헤드리스 검증과 별개 — 이건 사람 눈용이다.
WORLD ?= worlds/garden_ridge.sdf
view:
	@scripts/view.sh $(WORLD)

help:
	@echo "make doctor      - 환경이 멀쩡한지 단언 (파이썬 3.10 / rclpy / EGL / NVIDIA / Blender GPU)"
	@echo "make blender-gpu - Blender Cycles가 GPU를 쓰도록 켠다 (한 번만)"
	@echo "make test      - 순수 단위 테스트 (시뮬·GPU 불필요, 밀리초)"
	@echo "make smoke     - 헤드리스 GPU 렌더링 전 과정 + 게이트 2개 단언"
	@echo "make garden    - 주말농장 지형 생성 + 렌더 (기하학 눈으로 확인용)"
	@echo "make drive     - diff-drive 로 cmd_vel 주행 단언 (물리만, GPU 불필요)"
	@echo "make joints    - Y/Z 관절이 명령 위치에 mm 정밀 도달하는지 단언 (물리만)"
	@echo "make straddle  - 두둑 걸터타고 주행 — 포탈 설계가 물리로 성립하는지 단언 (물리만)"
	@echo "make camera    - 로봇 하방 카메라가 두둑을 보고 프레임 발행 — 2게이트 (GPU 필요)"
	@echo "make view WORLD=... - GUI 를 띄워 사람이 직접 3D 로 확인 (데스크톱 전용)"
	@echo "make cropcraft   - CropCraft 를 고정 SHA 로 가져오고 의존성 설치"
	@echo "make clean-sim - 좀비 ign 서버 정리"

# 산수로 답할 수 있는 건 시뮬로 확인하지 않는다. 느리고 불안정하고 GPU가 필요하니까.
# "로봇이 두둑을 탈 수 있나"는 산수다. 밀리초 안에 끝난다.
test:
	@$(ENV) python3 -m pytest tests/ -q

# CropCraft(정원 생성기)를 고정된 커밋으로 가져온다.
# SHA 를 박아두는 이유: 데이터셋은 (설정 + 시드 + CropCraft SHA + Blender 버전)의 함수다.
# 넷 중 하나라도 흐르면 어제 만든 정원을 오늘 다시 못 만든다.
CROPCRAFT_SHA := 7128cd2acade50cc4a5a1761210b55989ab62527

cropcraft:
	@if [ ! -d third_party/cropcraft ]; then \
	  mkdir -p third_party && git clone -q https://github.com/Romea/cropcraft.git third_party/cropcraft; \
	fi
	@cd third_party/cropcraft && git fetch -q --depth 1 origin $(CROPCRAFT_SHA) 2>/dev/null; \
	  git checkout -q $(CROPCRAFT_SHA)
	@echo "CropCraft $(CROPCRAFT_SHA) 준비됨"
	@# Blender 번들 파이썬(3.11)에 의존성을 넣는다. snap 이 읽기 전용이라 pip 이
	@# ~/.local/lib/python3.11 로 물러난다 — 그래서 cropcraft.sh 가 user site 를 켜둔다.
	@/snap/blender/current/5.0/python/bin/python3.11 -m pip install --user -q \
	  pyyaml msgpack pillow appdirs && echo "Blender 파이썬 의존성 설치됨"

# Blender Cycles GPU 켜기. 한 번만 하면 ~/.config/blender 에 저장돼서 계속 유지된다.
# 안 하면 Cycles가 아무 말 없이 CPU로 렌더링한다 — 이게 이 프로젝트에서 가장 조용한 함정이다.
blender-gpu:
	@blender --background --python tools/blender_gpu.py -- setup 2>/dev/null | grep -vE '^(Blender|Read|found)' 

# 지형 생성 + 렌더. 기하학이 말이 되는지 사람(과 에이전트)이 눈으로 보는 용도.
# 진짜 검증은 make test 가 한다 — 눈으로 보는 건 확장이 안 된다.
garden: clean-sim
	@$(ENV) python3 tools/make_garden_world.py > worlds/garden_ridge.sdf
	@$(ENV) ign sdf -k worlds/garden_ridge.sdf
	@rm -rf artifacts/garden && mkdir -p artifacts/garden
	@tools/run_headless.sh worlds/garden_ridge.sdf /garden/inspect $(SMOKE_ITERS)
	@$(ENV) python3 tools/assert_render.py artifacts/garden

# diff-drive 주행 단언 (Tier 2 — 물리만, 렌더/GPU 불필요).
# cmd_vel 로 로봇이 실제로 움직이는가. 게이트 2개: 플러그인이 명령 속도를 보고하나 AND
# 지상진실(pose/info)로 몸통이 물리적으로 그만큼 움직였나. odom 만 보면 바퀴가 헛돌아도
# 거짓 통과한다 — 실제로 그 함정(빔이 낮아 바퀴가 떠서 헛돎)을 밟았고 지상진실이 잡았다.
drive: clean-sim
	@$(ENV) python3 tools/assert_drive.py

# Y/Z 관절 위치 제어 단언 (Tier 2 — 물리만). 성공 기준("잡초 위 ±2cm")을 물리적으로
# 가능하게 하는 게 이 프리즘 관절들이다 (DECISIONS 006): 차체는 대충, 관절이 mm 를 준다.
# "잡초가 죽었나" 는 시뮬 못 함 — 여기서 재는 건 "막대가 그 위치·깊이에 정확히 갔나".
joints: clean-sim
	@$(ENV) python3 tools/assert_joints.py

# 두둑 걸터타고 주행 (Tier 2 — 물리만). 포탈 설계(바퀴는 고랑, 몸통은 두둑 위 터널)가
# 물리로 성립하는지 — DECISIONS 006 의 핵심 주장이자 Stage 1 "두둑을 탈 수 있는가" 위험의 완결.
straddle: clean-sim
	@$(ENV) python3 tools/assert_straddle.py

# 로봇 하방 카메라 검증 (Tier 3 — GPU 렌더링 필요). 카리지에 강체 고정된 카메라가
# 두둑을 내려다보고 프레임을 발행하는가. smoke 와 같은 2게이트(검지 않음 AND NVIDIA).
# Stage 2 의 마지막 DONE 항목이자 인식(Stage 3)의 관문.
camera: clean-sim
	@rm -rf artifacts/camera && mkdir -p artifacts/camera
	@tools/run_headless.sh worlds/robot_camera.sdf /robot/camera $(SMOKE_ITERS)
	@$(ENV) python3 tools/assert_render.py artifacts/camera

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
	@echo "== Blender =="
	@blender --version 2>/dev/null | head -1 | sed 's/^/  /'
	@echo "== Blender Cycles GPU (안 켜져 있으면 경고 없이 10배 느려진다) =="
	@# grep 에 파이프하면 종료 코드가 가려져서 GPU가 꺼져 있어도 doctor 가 통과한다.
	@# 실패할 수 없는 검사는 검사가 아니므로, 출력은 파일로 받고 종료 코드는 살린다.
	@blender --background --python tools/blender_gpu.py -- check >/tmp/ww_gpu.log 2>&1; \
	  rc=$$?; grep -E '백엔드|활성 장치|사용 중|통과|실패|고치려면' /tmp/ww_gpu.log | sed 's/^/  /'; \
	  exit $$rc
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
