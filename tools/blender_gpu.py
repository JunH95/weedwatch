"""Blender의 Cycles 렌더러가 GPU를 쓰도록 켠다. 그리고 켜졌는지 검사한다.

── 왜 이게 필요한가 (이 프로젝트에서 가장 조용한 함정) ────────────────────
Blender를 새로 깔면 Cycles는 **CPU로 렌더링**한다. GPU를 쓰려면 사람이
Edit > Preferences > System > Cycles Render Devices 에서 한 번 클릭해줘야 한다.

문제: 우리에겐 GUI가 없다. 그리고 안 켜면 **아무 경고도 안 뜬다.**
스크립트에서 `scene.cycles.device = 'GPU'` 라고 써놔도 조용히 무시되고 CPU로 돈다.
에러도, 경고도, 로그 한 줄도 없다. 그냥 24코어 CPU로 10배 느리게 렌더링할 뿐이다.
밤새 돌려놓고 아침에 "왜 아직도 안 끝났지?" 하게 되는 종류의 버그다.

실측 (2026-07-16, 이 컴퓨터):
    compute_device_type = 'NONE'   ← GPU 꺼짐
    has_active_device   = False
    그런데 RTX 4060 은 OPTIX/CUDA 둘 다 멀쩡히 보였다. 켜준 적이 없었을 뿐.

환경변수로 우회할 방법은 **없다** (Cycles 소스에 CYCLES_DEVICE 같은 건 없다).
유일한 방법은 설정 파일(~/.config/blender/<버전>/config/userpref.blend)을
한 번 만들어두는 것이고, 그걸 이 스크립트가 한다.

── OPTIX vs CUDA ────────────────────────────────────────────────────────
둘 다 NVIDIA GPU를 쓰지만 OPTIX 는 RTX 카드의 레이트레이싱 전용 코어(RT core)를
쓴다. 광선 추적이 본업인 Cycles 에서는 보통 훨씬 빠르다. RTX 4060 은 RT 코어가
있으므로 OPTIX 를 쓰고, 없으면 CUDA 로 물러난다.

사용법:
    blender --background --python tools/blender_gpu.py -- setup   # 한 번만
    blender --background --python tools/blender_gpu.py -- check   # 매번 (테스트에서)
"""

import sys

import bpy

# 선호 순서. 앞에 있는 걸 먼저 시도한다.
BACKENDS = ("OPTIX", "CUDA")


def _prefs():
    return bpy.context.preferences.addons["cycles"].preferences


def setup() -> int:
    """GPU를 켜고 설정 파일에 저장한다. 한 번만 하면 된다."""
    prefs = _prefs()

    for backend in BACKENDS:
        try:
            prefs.compute_device_type = backend
        except TypeError:
            print(f"  {backend}: 이 빌드에서 지원 안 함")
            continue

        prefs.refresh_devices()
        # 이 백엔드에 해당하는 GPU만 켠다.
        # CPU는 끈다 — GPU와 CPU를 섞으면 동기화 비용 때문에 오히려 느려질 수 있고,
        # 무엇보다 "GPU가 진짜 도는가"를 검사하기 애매해진다.
        gpus = [d for d in prefs.devices if d.type == backend]
        if not gpus:
            print(f"  {backend}: 장치 없음")
            continue

        for d in prefs.devices:
            d.use = d.type == backend

        if not prefs.has_active_device():
            print(f"  {backend}: 켰는데 has_active_device 가 여전히 False")
            continue

        bpy.ops.wm.save_userpref()
        print(f"GPU 켜짐: {backend}")
        for d in gpus:
            print(f"  - {d.name}")
        print(f"설정 저장됨 → {bpy.utils.resource_path('USER')}/config/userpref.blend")
        return 0

    print("실패: 쓸 수 있는 GPU 백엔드가 없습니다.", file=sys.stderr)
    print("  nvidia-smi 가 되는지, Blender가 GPU를 보는지 확인하세요.", file=sys.stderr)
    return 1


def check() -> int:
    """GPU가 진짜 켜져 있는지 검사한다. 렌더링 전에 매번.

    이게 통과해야만 '밤새 렌더링'이 몇 시간이 아니라 몇 시간으로 끝난다.
    """
    prefs = _prefs()
    backend = prefs.compute_device_type
    active = prefs.has_active_device()
    used = [d.name for d in prefs.devices if d.use]

    print(f"  백엔드   : {backend}")
    print(f"  활성 장치: {active}")
    print(f"  사용 중  : {used}")

    if backend == "NONE" or not active:
        print(
            "실패: Cycles가 CPU로 렌더링합니다. 경고 없이 10배 느려집니다.\n"
            "  고치려면: make blender-gpu",
            file=sys.stderr,
        )
        return 1

    if not any("NVIDIA" in n.upper() for n in used):
        print(f"실패: NVIDIA GPU가 사용 목록에 없습니다: {used}", file=sys.stderr)
        return 1

    print(f"통과: Cycles가 {backend}로 GPU 렌더링합니다.")
    return 0


if __name__ == "__main__":
    # blender --background --python 파일.py -- <인자>
    # '--' 뒤가 우리 인자다. Blender 자기 인자와 섞이지 않게 하는 관례.
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    mode = argv[0] if argv else "check"
    sys.exit(setup() if mode == "setup" else check())
