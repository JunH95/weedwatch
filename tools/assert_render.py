#!/usr/bin/env python3
"""헤드리스 렌더링이 진짜로, 그리고 GPU에서 일어났는지 검사한다.

── 왜 검사를 두 번 하는가 ────────────────────────────────────────────────
게이트 1 (그림): 사진이 나왔고, 검지 않고, 단색이 아닌가?
게이트 2 (장치): 그걸 그린 게 NVIDIA인가?

게이트 1만 보면 **거짓 통과**한다. 이유:

이 컴퓨터의 EGL(그래픽 초기화 규약)은 기본 설정에서 RTX 4060을 아예 못 찾는다.
대신 인텔 내장 그래픽과 llvmpipe(CPU로 그림을 그리는 소프트웨어 렌더러)만 보인다.
그런데 llvmpipe는 **멀쩡해 보이는 그림을 100배 느리게** 그린다.
즉 "사진 나왔나?"만 물으면 통과한다. 프로젝트 전체가 조용히 CPU 위에서
기어다니는데도 초록불이 켜진다.

그래서 "무엇이 그렸는가"를 반드시 같이 묻는다.

사용법:
    tools/assert_render.py artifacts/smoke
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

# Ignition이 렌더링 엔진 초기화 기록을 남기는 곳. 여기에 어떤 GPU를 잡았는지 적힌다.
OGRE_LOG = Path.home() / ".ignition" / "rendering" / "ogre2.log"

# 픽셀값은 0(검정)~255(흰색).
MIN_MEAN = 8.0  # 평균이 이보다 낮으면 사실상 검은 화면 = 렌더링 실패
MIN_STD = 6.0  # 표준편차 = 화면이 얼마나 다채로운가. 단색으로 칠해졌으면 0에 가깝다.


def fail(msg: str):
    print(f"실패: {msg}", file=sys.stderr)
    sys.exit(1)


def gate_pixels(frame_dir: Path) -> Path:
    """게이트 1 — 사진이 나왔고 내용이 있는가."""
    if not frame_dir.is_dir():
        fail(f"{frame_dir} 디렉토리가 없습니다. 시뮬이 사진을 한 장도 안 썼습니다.")

    frames = sorted(frame_dir.glob("*.png"))
    if not frames:
        fail(
            f"{frame_dir} 는 있는데 PNG가 없습니다.\n"
            "  가장 흔한 원인 두 가지:\n"
            "  1) 이미지 토픽을 구독하는 쪽이 없었다.\n"
            "     Fortress 카메라는 누가 보고 있을 때만 렌더링한다. 아무도 없으면\n"
            "     에러 한 줄 없이 그냥 아무것도 안 한다. tools/run_headless.sh 를 쓸 것.\n"
            "  2) ign gazebo 에 -r 을 빼먹었다.\n"
            "     시뮬은 기본이 일시정지라 시계가 안 흐르고 카메라가 안 찍는다."
        )

    # 마지막 프레임을 본다. 첫 프레임은 장면이 다 뜨기 전에 찍혔을 수 있다.
    frame = frames[-1]
    arr = np.asarray(Image.open(frame).convert("RGB"), dtype=np.float32)
    mean, std = float(arr.mean()), float(arr.std())

    print(f"  사진 장수   : {len(frames)}")
    print(f"  검사 대상   : {frame.name}")
    print(f"  평균/표준편차: {mean:.2f} / {std:.2f}")

    if mean < MIN_MEAN:
        fail(
            f"화면이 검습니다 (평균 {mean:.2f} < {MIN_MEAN}).\n"
            "  ogre2가 EGL을 못 잡고 소프트웨어 렌더러로 흘러간 전형적 증상입니다.\n"
            "  env.sh 의 __EGL_VENDOR_LIBRARY_FILENAMES 설정을 확인하세요."
        )
    if std < MIN_STD:
        fail(
            f"화면이 단색입니다 (표준편차 {std:.2f} < {MIN_STD}).\n"
            "  배경색만 칠해졌고 물체가 하나도 안 그려졌다는 뜻입니다."
        )
    return frame


def gate_device() -> str:
    """게이트 2 — 그걸 그린 게 NVIDIA인가. 이게 없으면 게이트 1은 거짓 통과다."""
    if not OGRE_LOG.exists():
        fail(f"{OGRE_LOG} 가 없어서 어떤 GPU가 그렸는지 증명할 수 없습니다. 통과시키지 않습니다.")

    vendor = renderer = None
    for line in OGRE_LOG.read_text(errors="replace").splitlines():
        if m := re.search(r"GL_VENDOR\s*=\s*(.+)", line):
            vendor = m.group(1).strip()
        elif m := re.search(r"GL_RENDERER\s*=\s*(.+)", line):
            renderer = m.group(1).strip()

    if vendor is None:
        fail(f"{OGRE_LOG} 에 GL_VENDOR 줄이 없습니다. 렌더링 엔진이 아예 안 떴습니다.")

    print(f"  GL_VENDOR   : {vendor}")
    print(f"  GL_RENDERER : {renderer}")

    if "nvidia" not in vendor.lower():
        fail(
            f"NVIDIA가 아니라 '{vendor}' ({renderer}) 가 그렸습니다.\n"
            "  이 컴퓨터의 EGL은 기본값으로 두면 인텔 내장 그래픽을 먼저 잡습니다.\n"
            "  그림은 멀쩡해 보이지만 100배 느립니다. env.sh 에서 아래를 고정하세요:\n"
            "    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        )
    return f"{vendor} / {renderer}"


def main() -> None:
    frame_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts/smoke")

    print("게이트 1 — 그림")
    frame = gate_pixels(frame_dir)
    print("게이트 2 — 장치")
    device = gate_device()

    print()
    print("통과: 화면 없이 GPU 렌더링이 확인되었습니다.")
    print(f"  {frame.name} ← {device}")
    print("  → 에이전트가 GUI 없이 시뮬을 돌리고 진짜 사진을 받아올 수 있습니다.")


if __name__ == "__main__":
    main()
