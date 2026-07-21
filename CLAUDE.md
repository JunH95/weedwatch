# weedwatch

주말농장(취미 텃밭)을 대신 돌보는 자율 제초 로봇을 **시뮬레이션 안에서** 만든다.
카메라로 작물과 잡초를 구별해 **잡초 위에 정밀하게 서서 스탬핑**한다. 실제 로봇은 만들지 않는다.

**이 프로젝트의 핵심 제약: Claude가 만들고 Claude가 스스로 검증한다.**
사람이 GUI를 들여다보지 않는다. 따라서 **CLI에서 단언(assert)할 수 없는 기능은 존재할 수 없다.**

전체 계획: `docs/PLAN.md` · 결정 기록: `docs/DECISIONS.md` · 현재 위치: `STATUS.md`

---

## 🚨 이것부터 읽어라 — 모르면 세션을 통째로 날린다

### 1. 모든 명령은 `./scripts/env.sh`를 통과해야 한다

```bash
./scripts/env.sh python3 ...      # ✅
python3 ...                        # ❌ 조용히 틀린 파이썬을 쓴다
```

이 컴퓨터의 `python3`는 **miniforge의 3.13.13**이고, `/usr/bin/python3`(3.10.12)를 가린다.
ROS 2 Humble은 3.10용으로 빌드돼 있어서 `import rclpy`가 이렇게 실패한다:

```
The C extension '..._rclpy_pybind11.cpython-313-x86_64-linux-gnu.so' isn't present
```

`~/.bashrc`가 남의 워크스페이스 4개(rmf_ws, movebot_ws, colcon_ws, micro_ros_ws)를
`PYTHONPATH`에 밀어넣는 것도 `env.sh`가 씻어낸다.

### 2. 시뮬레이터는 `ign gazebo`다. `gz sim`이 **아니다**

```bash
ign gazebo ...    # ✅ Ignition Fortress 6.17.0 — 우리가 쓰는 것
gz sim ...        # ❌ "Invalid arguments" — /usr/bin/gz는 Gazebo Classic의 도구다
```

이 컴퓨터엔 이름이 Gazebo인 게 셋 있고 둘은 함정이다.
인터넷 문서 대부분이 `gz sim`이라고 써 있지만 그건 최신 Gazebo(Harmonic+) 얘기다.
**Harmonic을 설치하면 안 된다** — `ros-gz-bridge`와 사용자의 `~/rmf_ws`를 제거해버린다.

### 3. Bash 호출 사이에 셸 상태가 남지 않는다

`cd`도 `export`도 `source`도 다음 호출에 안 넘어간다. 모든 명령을 **자기완결적으로** 써라.
그래서 `env.sh`가 있고, `Makefile` 타깃이 있다.

### 4. 검증 계약 (이 프로젝트의 존재 이유)

- **단언 출력을 붙이지 않은 "작동합니다"는 금지.** 명령을 돌리고 출력을 보여줘라.
- **불안정한(flaky) 테스트는 실패한 테스트다.** 임계값을 조용히 낮추지 마라.
- **테스트를 고쳐서 통과시키지 마라.** 코드를 고쳐라.
- **`configs/eval_seeds.txt`와 임계 상수는 보호 대상이다.** 모델을 바꾸는 커밋에서 같이
  건드리지 마라. 골대를 옮길 수 있는 에이전트는 언젠가 옮긴다.

---

## 명령어

```bash
make doctor   # 환경이 멀쩡한지 단언 (파이썬 3.10 / rclpy / EGL / NVIDIA)
make smoke    # 헤드리스 GPU 렌더링 전 과정 + 게이트 2개 단언
make clean-sim # 좀비 ign 서버 정리
```

**테스트 전에 항상 `clean-sim`.** 죽다 만 `ign` 서버가 살아남아서 다음 실행이 낡은 상태를
재사용하면, 통과가 통과가 아니게 된다.

---

## 검증된 환경 사실 (전부 이 컴퓨터에서 직접 확인함)

| 항목 | 값 |
|---|---|
| OS / ROS | Ubuntu 22.04.5 / ROS 2 **Humble** (`/opt/ros/humble`) |
| 시뮬 | **Ignition Fortress 6.17.0** (`ign gazebo`) |
| GPU | RTX 4060 Laptop **8GB** · 드라이버 595.71.05 · CUDA 13.2 |
| 파이썬 | ROS용 `/usr/bin/python3` **3.10.12** (conda 3.13은 격리) |
| 렌더 | ogre2 + EGL (헤드리스) |

### 🔴 EGL은 기본값으로 두면 **인텔 내장 그래픽**을 잡는다

이 컴퓨터의 EGL 장치 목록에 RTX 4060이 원래 **없었다**:

```
Found Num EGL Devices: 2
  #0 /dev/dri/card1              ← 인텔 UHD
  #1 EGL_MESA_device_software    ← llvmpipe (소프트웨어 렌더러)
GL_RENDERER = Mesa Intel(R) UHD Graphics
```

gz-sim#1272(미해결)이 항상 0번을 고르고, llvmpipe는 **검은 화면**을 낸다(gz-sim#1116).
`env.sh`가 NVIDIA ICD만 로드하도록 고정해서 해결했다:

```bash
__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
# → Found Num EGL Devices: 1 / GL_RENDERER = NVIDIA GeForce RTX 4060 Laptop GPU ✅
```

**그래서 모든 시뮬 테스트는 게이트가 2개다**: 픽셀이 검지 않은가 **AND** NVIDIA가 그렸는가.
픽셀만 보면 거짓 통과한다 — llvmpipe는 *멀쩡해 보이는* 그림을 100배 느리게 그린다.
(`/dev/nvidia*`는 권한이 0666이라 X 없이도 접근된다. `usermod -aG render`는 불필요.)

---

## Gazebo Fortress 함정 (직접 데어본 것들)

### `-r`을 빼먹으면 아무 일도 안 일어난다
시뮬은 **기본이 일시정지**다. `-r` 없이 돌리면 시계가 안 흘러서 카메라가 한 장도 안 찍는데,
증상이 EGL 고장과 똑같아 보인다.

### 카메라는 **구독자가 있을 때만** 렌더링한다 ⚠️
실측: 구독자 없음 → PNG **0장**. 구독자 있음 → PNG **121장**. 에러 메시지는 **한 줄도 없다.**
`<save>`도 렌더 안에서 일어나므로 같이 죽는다. 헤드리스로 프레임이 필요하면 **반드시 구독자를 붙여라.**

### `ign topic -p` 는 발행 한 번에 **1초**가 걸린다 ⚠️
실측 1.055초(5회 중앙값). 프로세스 기동 + 디스커버리 + 광고 + 종료를 매번 반복해서다.
**로봇이 서 있을 땐 무해하지만 주행 중엔 곧 위치 오차다** — 0.25m/s 면 명령 하나에 26cm 이고
성공 허용오차는 2cm 다. CLI 로는 주행 중 폐루프 제어가 원천 불가능하다.
→ 상주 프로세스 `build/ww_cmd`(`make ww-cmd`)를 써라. 명령 쓰기 3.6us.
`ign topic -e` 로 상태를 파이프로 받는 것도 피하라 — 블록버퍼링으로 50Hz 스트림이 수백 ms 밀린다.

### 바퀴는 즉시 서지만 몸통은 미끄러진다 ⚠️
`cmd_vel` 0 을 주면 DiffDrive 가 바퀴를 바로 세우고, 몸통은 관성으로 더 간다.
**오도메트리는 그 미끄러짐을 못 본다**(바퀴 회전에서 속도를 뽑으므로). 0.3m/s 에서 지상진실은
3.47cm 가는데 odom 은 0.51cm 라고 보고한다. 정지 정밀도를 odom 으로 재면 거짓 통과한다.
제동거리 ≈ 0.40·v² (실측 피팅). 자세한 표는 `STATUS.md` Stage 4-3.

### 렌더 스레드는 비동기다
ogre2 + EGL 컨텍스트 만드는 데 1~2초 걸린다. 물리는 그보다 빨리 끝날 수 있다.
`--iterations`는 실시간 배속을 존중하므로(30000 스텝 = 30초 시뮬 ≈ 33초 실제)
센서가 뜰 시간을 벌려면 충분한 스텝을 줘라.

### 표준 실행법
```bash
./scripts/env.sh ign gazebo -s -r --headless-rendering --iterations N worlds/X.sdf
#                            │  │  └ EGL 헤드리스 (ogre2 전용 — ogre로 폴백하면 조용히 죽음)
#                            │  └ 필수: 안 주면 일시정지 상태
#                            └ 서버만 (렌더링은 서버에 있으므로 -s로 충분)
```

---

## Blender 함정

### Cycles는 기본이 **CPU**다. 경고 없이.
새로 깔면 `compute_device_type = 'NONE'`이고, 스크립트에서 `scene.cycles.device = 'GPU'`라고
써놔도 **조용히 무시되고 CPU로 돈다.** 에러도 경고도 로그도 없다. 그냥 10배 느릴 뿐이다.
환경변수로 우회할 방법이 없다 — 설정 파일을 한 번 만들어두는 게 유일한 방법이다.

```bash
make blender-gpu    # 한 번만. OPTIX로 켜고 userpref.blend 에 저장
```
렌더링하는 코드는 `tools/blender_gpu.py check`로 먼저 확인할 것. 조용히 CPU로 도는 걸 막는다.

### 측정된 렌더 시간 (이 컴퓨터, 2026-07-16)
1920×1080 / 32샘플 / 식물 800개 / OPTIX 기준 **장당 11.2초** (장면 빌드 8초는 여러 장에 분산됨).
→ 논문 헤드라인 재현(1,050장) ≈ **3.5시간**. 하룻밤이면 된다.
GPU vs CPU는 이 크기에서 1.7배지만, 작은 장면이라 시작 오버헤드가 지배한 수치다.

### 버전은 5.0.1 고정. 마음대로 올리지 말 것.
`snap install blender --channel=5.0/stable --classic` → 5.0.1 (번들 파이썬 3.11.13).
- apt는 3.0.1 → CropCraft 최소 요구(4.0) 미달
- **4.2 / 4.5 LTS는 실제로 깨진다** — CropCraft가 쓰는 `BLENDER_EEVEE`가 4.2에서
  `BLENDER_EEVEE_NEXT`로 개명됐다가 5.0에서 되돌아왔다
- 채널을 고정해야 snap 자동 업데이트가 5.2/5.3으로 밀어버리지 않는다

---

## Gazebo 자산 함정

### 메시 포맷은 **OBJ**다
Fortress 로더는 DAE/OBJ/STL만 지원하고(glTF **없음**), PBR 재질을 실을 수 있는 건 OBJ+MTL뿐인데,
Blender 5.0이 DAE export를 없앴다. 이 삼각형의 유일한 해가 OBJ다.
Blender가 내보낸 OBJ의 **첫 줄 `# Blender` 주석을 지우지 마라** — 로더가 그걸로 분기한다.

### `<material><script>`는 무시된다 → 전부 **검게** 렌더된다
Ogre1 스타일 머티리얼을 gz-sim이 버리고 diffuse {0,0,0,1}로 메시 자체 재질까지 덮어쓴다.
`<pbr><metal><albedo_map>` + `<double_sided>true`를 써야 한다. (CropCraft 출력이 여기 걸린다.)

---

## 저장소 구조

```
scripts/env.sh     # 유일한 진입점
tools/             # 단언 스크립트 (ROS 패키지 아님)
worlds/            # SDF 월드
src/               # ROS 2 패키지만 — 학습 코드를 넣으면 colcon이 깨진다
perception/        # ML 코드. 별도 venv. ROS와의 계약은 디스크 파일(models/best.pt)이지
                   #   공유 import가 아니다
configs/           # train_seeds.txt / eval_seeds.txt — 코드가 아니라 데이터로 커밋
```

**파이썬이 둘이고 경계는 단단하다.** ROS쪽은 `/usr/bin/python3` 3.10만 쓴다(rclpy가 도는
유일한 인터프리터). ML쪽은 별도 venv. 둘을 import로 잇지 마라.

---

## 이 프로젝트에서 하면 안 되는 것

- **GUI 실행 금지** — `rviz2`, `gzclient`, `-g` 없는 `ign gazebo`(GUI가 뜬다). 보는 사람이 없다.
- **AI Hub 데이터를 커밋하지 마라** — 재배포 금지 라이선스다. 지표와 모델만 공개 가능.
- **`gz sim`, Harmonic, Docker 개발환경** — 전부 검토 후 기각됐다. 이유는 `docs/DECISIONS.md`.
- **시뮬 결과를 발표된 로봇 성능과 나란히 비교하지 마라** — 재는 물리량이 다르다
  (우리는 "잡초 위에 섰나", 저쪽은 "잡초가 죽었나"). 단위 변환이 존재하지 않는다.
