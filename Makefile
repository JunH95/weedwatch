# 이 저장소의 모든 명령은 scripts/env.sh 를 통과한다.
# 이유는 그 파일 맨 위 주석 참고 — 요약하면 이 컴퓨터의 python3 는 ROS가 못 쓰는
# 버전이고, 남의 워크스페이스 4개가 환경변수에 섞여 들어온다.

ENV := ./scripts/env.sh
# 12초 분량. 렌더링 스레드가 ogre2+EGL 컨텍스트를 만드는 데 1~2초 걸리므로
# 그보다 넉넉히 줘야 한다. (make는 값 뒤 공백까지 변수에 넣으므로 주석은 윗줄에)
SMOKE_ITERS ?= 12000

.PHONY: help doctor test smoke garden drive joints straddle tilt tilt-stamp shake camera dataset bake perception-venv train eval-model stamp-targets stamp row watch-row percept-render percept percept-calib field-render watch-field row-live overlay ww-cmd view blender-gpu cropcraft aihub clean-sim clean

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
	@echo "make dataset   - Stage3 학습데이터 스모크: CropCraft 로 1시드 RGB+마스크 생성+검증 (GPU)"
	@echo "make bake      - Stage3 데이터셋 bake: 전 시드로 train/eval 세트 구축 (증분, GPU, 오래 걸림)"
	@echo "make perception-venv - ML용 격리 venv(conda) + torch(CUDA) + 세그 라이브러리 설치"
	@echo "make train     - Stage3 4클래스 세그 모델 학습 → models/best.pt (GPU, bake 먼저)"
	@echo "make eval-model- Stage3 평가 게이트: held-out eval 에서 잡초·옥수수 IoU/recall 단언"
	@echo "make stamp-targets - Stage4 인식→미터좌표: 잡초 검출률·타격 위치오차 단언 (held-out)"
	@echo "make stamp     - Stage4 스탬핑: 두둑 위 잡초에 도구 끝 얹기 |도구-잡초|<2cm 단언 (물리)"
	@echo "make row       - Stage4-3 무정차 행 스윕: 주행하며 임의(x,y) 잡초 타격 <2cm + 작물무접촉 (물리)"
	@echo "make watch-row - (데스크톱) make row 를 GUI 창으로 재생 — 주행+스탬핑을 눈으로 봄"
	@echo "make field-render - Stage4-3 P4b: 사실적 밭(사면두둑+고랑+CropCraft) 카메라 렌더 2게이트 (GPU)"
	@echo "make watch-field - (데스크톱) 사실적 밭 위 주행+스탬핑을 GUI 로 재생"
	@echo "make row-live  - Stage4-3 P4b-3: 로봇이 제 카메라로 본 잡초만으로 주행 타격 (GPU, 자율)"
	@echo "make percept-render - Stage4-3 P4a: 로봇 카메라가 CropCraft 사실적 두둑 렌더 2게이트 (GPU)"
	@echo "make percept   - Stage4-3 P4a: 로봇 카메라 렌더에 best.pt 라이브 추론 → 오라클 대조 검출률 (GPU)"
	@echo "make overlay   - 인식 결과를 눈으로: 원본|예측+타격점 오버레이 PNG (사람 검증용)"
	@echo "make ww-cmd    - Stage4-3 주행 중 제어용 상주 명령 프로세스 빌드 (ign topic -p 는 1초라 못 씀)"
	@echo "make view WORLD=... - GUI 를 띄워 사람이 직접 3D 로 확인 (데스크톱 전용)"
	@echo "make cropcraft   - CropCraft 를 고정 SHA 로 가져오고 의존성 설치"
	@echo "make aihub AIHUB_KEY=키 - AI Hub 527 쇠비름 검증세트(~3GB) 다운로드 (승인 필요)"
	@echo "make clean-sim - 좀비 ign 서버 정리"

# 산수로 답할 수 있는 건 시뮬로 확인하지 않는다. 느리고 불안정하고 GPU가 필요하니까.
# "로봇이 두둑을 탈 수 있나"는 산수다. 밀리초 안에 끝난다.
test:
	@$(ENV) python3 -m pytest tests/ -q

# CropCraft(정원 생성기)를 고정된 커밋으로 가져온다.
# SHA 를 박아두는 이유: 데이터셋은 (설정 + 시드 + CropCraft SHA + 4클래스 패치 + Blender 버전)의
# 함수다. 다섯 중 하나라도 흐르면 어제 만든 정원을 오늘 다시 못 만든다.
CROPCRAFT_SHA := 7128cd2acade50cc4a5a1761210b55989ab62527

# AI Hub 527 쇠비름 검증세트(~3GB) 다운로드 (sim-to-real 평가용, Stage 3).
# 사람 몫: aihub.or.kr API 키 발급 + 527 승인 (내국인). 재배포 금지 → data/aihub/ 는 gitignore.
#   사용: make aihub AIHUB_KEY=<발급키>
aihub:
	@scripts/fetch_aihub.sh $(AIHUB_KEY)

# Stage 3 학습 데이터: CropCraft 내장 렌더가 정원을 RGB(images/) + 세그멘테이션 마스크
# (masks/, 흙 검정·작물 초록·잡초 빨강)로 두 번 렌더한다. GPU(Cycles+EEVEE) 필요.
# 마스크가 실제로 작물/잡초를 가르는지 assert_dataset.py 가 픽셀단위로 단언.
dataset:
	@scripts/cropcraft.sh configs/train_garden.yaml
	@$(ENV) python3 tools/assert_dataset.py

# Stage 3-2 데이터셋 bake: 여러 시드로 CropCraft 를 돌려 학습/평가 세트를 models/dataset/ 에
# 쌓는다(증분·idempotent — 있는 시드는 건너뜀). eval 시드는 보호(configs/eval_seeds.txt).
# GPU 렌더라 오래 걸린다(수백 장 = 수십 분~시간). 백그라운드로 돌릴 것.
bake:
	@$(ENV) python3 tools/bake_dataset.py train configs/train_seeds.txt
	@$(ENV) python3 tools/bake_dataset.py eval  configs/eval_seeds.txt
	@$(ENV) python3 tools/assert_dataset.py models/dataset/train
	@$(ENV) python3 tools/assert_dataset.py models/dataset/eval

# perception(ML) 격리 venv. 시스템 3.10 에 python3-venv 가 없어 conda 로 만든다(sudo 불필요).
# torch/torchvision 은 CUDA 휠이라 별도 index(cu124 = 드라이버 595 호환). ROS 와 완전 격리.
perception-venv:
	conda create -y -p perception/.venv python=3.11 pip
	@# PYTHONNOUSERSITE=1 필수: conda 의 pip 은 Blender user-site(~/.local/lib/python3.11 의
	@# PIL·yaml 등)를 "이미 설치됨"으로 보고 venv 에 안 넣는다. 그러면 env.sh(user-site off)에서
	@# import 가 깨진다. user-site 를 끄고 설치해야 venv 가 자기완결이 된다.
	PYTHONNOUSERSITE=1 perception/.venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
	PYTHONNOUSERSITE=1 perception/.venv/bin/pip install -r perception/requirements.txt
	@perception/env.sh python -c "import torch,segmentation_models_pytorch as smp; print('torch',torch.__version__,'cuda',torch.cuda.is_available(),'smp',smp.__version__)"

# Stage 3-2c 학습: 4클래스 세그 모델(smp U-Net) → models/best.pt (GPU). make bake 먼저.
#   손실 0.6·가중CE+0.4·Dice, inverse-sqrt 가중 (DECISIONS 015). TRAIN_ARGS 로 인자 전달.
train:
	perception/env.sh python perception/train.py $(TRAIN_ARGS)

# Stage 3-2d 평가 게이트: held-out eval 에서 잡초·옥수수 IoU/recall 단언(전체정확도 금지).
eval-model:
	perception/env.sh python perception/eval_model.py --gate

# Stage 4-1 인식->미터좌표: best.pt 로 잡초 검출 -> 인스턴스 중심 -> 픽셀->미터 변환.
# held-out eval 에서 검출률과 타격 위치오차(예측 중심 vs GT 중심)를 단언.
stamp-targets:
	perception/env.sh python perception/stamp_targets.py --gate

# Stage 4-2 스탬핑: 두둑 위 알려진 잡초에 캐리지+도구를 움직여 도구 끝을 얹고,
# 실측(지상진실 base + achieved joint_state)으로 |도구-잡초|<2cm 를 단언. 물리, GPU 불필요.
stamp:
	@$(ENV) python3 tools/assert_stamp.py

# Stage 4-3 선결조건: 주행 중 폐루프 제어용 상주 명령/상태 프로세스.
#
# 왜 필요한가 (실측): `ign topic -p` 는 발행당 1.055초가 걸린다(5회 중앙값). 프로세스 기동 +
# 디스커버리를 매번 반복하기 때문이다. 0.25 m/s 면 명령 하나에 26cm — 허용오차 2cm 의 13배다.
# 즉 CLI 로는 주행 중 제어가 불가능하다. 4-2 가 이걸 안 밟은 건 로봇이 서 있었기 때문이다.
# ww_cmd 는 디스커버리를 한 번만 하고 상주한다: 명령 쓰기 3.6us (약 29만 배).
#
# colcon 이 아니라 g++ 한 줄로 짓는다 — src/ 는 ROS 패키지 전용이고(넣으면 colcon 이 깨진다)
# 이건 ROS 노드가 아니라 ign-transport 직결 도구다.
build/ww_cmd: tools/ww_cmd/ww_cmd.cc
	@mkdir -p build
	@$(ENV) sh -c 'g++ -O2 -o $@ $< $$(pkg-config --cflags --libs ignition-transport11 ignition-msgs8)'
	@echo "build/ww_cmd 빌드됨"

ww-cmd: build/ww_cmd

# Stage 4-3 Phase 2 무정차 행 스윕: 로봇이 두둑을 걸터탄 채 +x 무정차 주행하며 임의 (x,y) 잡초를
# 타격. 카메라가 먼저 보고(인과공개) 담당 툴(엇갈린 X)이 지나갈 때 예측 하강. 제어=odom(ww_cmd),
# 채점=지상진실(별도 구독). 게이트: X정렬<2cm·안티크리프·iterations·작물무접촉·odom↔GT표류 보고.
# 물리(Tier 2, GPU 불필요)지만 ww_cmd 빌드가 선행돼야 한다.
row: build/ww_cmd clean-sim
	@$(ENV) python3 tools/assert_row_stamp.py

# 사람 눈 관람용(데스크톱 전용): make row 와 같은 무정차 주행+스탬핑을 GUI 창으로 재생한다.
# 헤드리스 단언이 아니라 "움직이는 걸 눈으로 본다". CLAUDE(에이전트)는 못 씀 — DISPLAY 필요.
watch-row: build/ww_cmd
	@scripts/watch_row.sh

# Stage 4-3 Phase 4a 기반: 로봇 카메라가 CropCraft 사실적 두둑(model://oracle_test)을 GPU 렌더하는가.
# 학습 카메라와 정합(Phase 3)된 위에서만 라이브 인식이 정직하다. 2게이트(검지않음+NVIDIA, assert_render).
# 두둑을 z=0.25 로 올려 카메라가 식물 위 0.33m(학습 스케일)에서 봄. detect_server 가 이 렌더를 소비.
percept-render: clean-sim
	@rm -rf artifacts/camera && mkdir -p artifacts/camera
	@tools/run_headless.sh worlds/robot_percept.sdf /robot/camera $(SMOKE_ITERS)
	@$(ENV) python3 tools/assert_render.py artifacts/camera

# Stage 4-3 Phase 4a 정적 라이브 인식: 로봇 카메라가 렌더한 CropCraft 두둑에 best.pt 라이브 추론 →
# 잡초 world 좌표 → 오라클(정답) 대조. sim카메라→인식 다리(카메라 정합 Phase 3 위). ML venv(torch)라
# perception/env.sh 로 돌리고 sim 은 subprocess. 게이트: 렌더 2게이트 + 시야 안 target 검출률.
percept: clean-sim
	@perception/env.sh python perception/assert_percept.py --gate

# 픽셀→world 매핑 재캘리브 (detect_server MPP 상수 검증. 색 마커 월드).
percept-calib: clean-sim
	@perception/env.sh python perception/calibrate_camera.py

# Stage 4-3 Phase 4b 필드 에셋 (models/ 는 gitignore 산출물 — 재생성). 사면 사다리꼴 두둑 +
# 충돌 제거한 CropCraft 정원(주행 중 잎에 안 부딪히게). garden_field 는 oracle_test(make cropcraft) 필요.
models/ridge/model.sdf: tools/make_ridge.py tools/garden_geometry.py
	@$(ENV) python3 tools/make_ridge.py
models/garden_field/model.sdf: tools/make_garden_field.py
	@$(ENV) python3 tools/make_garden_field.py

# Stage 4-3 Phase 4b-1: 사실적 주행 월드(사면 두둑+고랑+CropCraft 식물)에서 로봇 카메라 렌더 2게이트.
field-render: models/ridge/model.sdf models/garden_field/model.sdf clean-sim
	@rm -rf artifacts/camera && mkdir -p artifacts/camera
	@tools/run_headless.sh worlds/robot_field.sdf /robot/camera $(SMOKE_ITERS)
	@$(ENV) python3 tools/assert_render.py artifacts/camera

# 사람 눈 관람용(데스크톱): 사실적 밭 위를 로봇이 주행하며 스탬핑(오라클 좌표). watch-row 의 실사 버전.
watch-field: build/ww_cmd models/ridge/model.sdf models/garden_field/model.sdf
	@scripts/watch_field.sh

# Stage 4-3 Phase 4b-3 주행 라이브 온-루프: 로봇이 제 카메라(best.pt via detect_server)로 본 잡초만으로
# 주행하며 타격. 오라클은 제어서 빠지고 채점만. 게이트: 렌더 2게이트 + 검출→타격 재현율 + 작물무접촉.
row-live: build/ww_cmd models/ridge/model.sdf models/garden_field/model.sdf clean-sim
	@$(ENV) python3 tools/assert_field_live.py

# 사람 검증용: held-out 정원에 모델을 돌려 [원본 | 예측+타격점] 오버레이 PNG 생성.
# 단언이 아니라 눈으로 보는 용도 → artifacts/perception_overlay.png 를 열어 본다.
overlay:
	perception/env.sh python perception/overlay.py

cropcraft:
	@if [ ! -d third_party/cropcraft ]; then \
	  mkdir -p third_party && git clone -q https://github.com/Romea/cropcraft.git third_party/cropcraft; \
	fi
	@cd third_party/cropcraft && git fetch -q --depth 1 origin $(CROPCRAFT_SHA) 2>/dev/null; \
	  git checkout -q -f $(CROPCRAFT_SHA)
	@# 고정 SHA 위에 weedwatch 4클래스 라벨 패치 적용 (DECISIONS 016). third_party/ 는
	@# gitignore 라 CropCraft 수정을 직접 커밋 못 함 → 패치로 추적한다. checkout -f 로 매번
	@# 깨끗한 SHA 로 되돌린 뒤 적용하므로 idempotent. 재현성 = CropCraft SHA + 이 패치.
	@cd third_party/cropcraft && git apply --whitespace=nowarn "$(CURDIR)/patches/cropcraft-4class-labels.patch"
	@echo "CropCraft $(CROPCRAFT_SHA) + 4클래스 라벨 패치 준비됨"
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

# Stage 5 Tier 1 기울기 선검증 (Tier 2 — 물리만). 로봇이 크로스슬로프에서 실제로 기울고,
# DART 가 기운 접촉에서 안 터지고, IMU 가 그 기울기를 읽는가. 게이트 2개: IMU(센서) AND
# 지상진실(물리). 지금껏 어떤 월드에도 imu-system 이 없어 IMU 가 발행조차 안 됐다(DECISIONS 025).
# 이게 통과해야 Stage 5 흔들림 보정 하네스로 넘어간다.
worlds/robot_tilt.sdf: tools/make_tilt_world.py
	@$(ENV) python3 tools/make_tilt_world.py > worlds/robot_tilt.sdf
tilt: worlds/robot_tilt.sdf clean-sim
	@$(ENV) python3 tools/assert_tilt.py

# Stage 5 기울기 보정 스탬핑 A/B (Tier 2 — 물리만). 로봇이 8° 기운 채 잡초를 찍는다:
# 무보정(수평 가정)이면 도구가 기운 축으로 하강해 옆으로 빗나가고(>2cm), IMU 자세로 보정하면
# 잡초 위 2cm 안(히트). 제어=IMU(센서, 실물 잔차 얹음), 채점=지상진실 FK. "흔들려도 맞춘다"의 증명.
worlds/robot_tilt_stamp.sdf: tools/make_tilt_world.py
	@$(ENV) python3 tools/make_tilt_world.py stamp > worlds/robot_tilt_stamp.sdf
tilt-stamp: worlds/robot_tilt_stamp.sdf clean-sim
	@$(ENV) python3 tools/assert_tilt_stamp.py

# Stage 5 Tier 2 Step A (Tier 2 — 물리만). 로봇이 절차적 흙덩이 밭을 주행. 게이트 3개: 완주+안정
# (DART 범프 안 터짐) AND 실제로 흔들림(자세 시변) AND IMU 가 그 시변 자세를 GT 대로 추적. 주행 중
# 타격 보정(Step B) 전에 기반을 세운다.
worlds/robot_shake.sdf: tools/make_shake_world.py
	@$(ENV) python3 tools/make_shake_world.py > worlds/robot_shake.sdf
shake: worlds/robot_shake.sdf clean-sim
	@$(ENV) python3 tools/assert_shake.py

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
