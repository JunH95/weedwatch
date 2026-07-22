#!/usr/bin/env python3
"""로봇 down_cam 의 픽셀→월드 매핑을 색 마커로 직접 캘리브한다 (Stage 4-3 Phase 4a 보조).

detect_server.py 의 MPP/중심 상수가 load-bearing 이라 재현 가능해야 한다. worlds/robot_calib.sdf
는 알려진 world 좌표에 원색 마커 3개(빨강=중심, 파랑=+0.08x, 초록=+0.08y)를 둔다. 여기서 렌더 →
각 마커의 픽셀 중심을 색으로 찾아 2D 아핀(픽셀→world)을 푼다. 오라클 노이즈 없이 정확.

기대 결과(이 머신): 중심≈(640,360), 0.457mm/px, +x→-row, +y→-col. detect_server 상수와 대조.

실행:  perception/env.sh python perception/calibrate_camera.py
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

WW = Path(__file__).resolve().parents[1]
ENVSH = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_calib.sdf")
CAMDIR = WW / "artifacts" / "camera"
MARKERS = {"c": (0.22, 0.60), "x": (0.30, 0.60), "y": (0.22, 0.68)}  # world (robot_calib.sdf 와 일치)
STEP = 0.08  # 마커 오프셋 (m)


def _centroid(mask):
    ys, xs = np.where(mask)
    return (xs.mean(), ys.mean()) if len(xs) >= 5 else None


def main():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    for f in CAMDIR.glob("*.png"):
        f.unlink()
    CAMDIR.mkdir(parents=True, exist_ok=True)
    log = open("/tmp/ww_calib.log", "w")
    sim = subprocess.Popen([ENVSH, "ign", "gazebo", "-s", "-r", "--headless-rendering",
                            "--iterations", "12000", WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    csub = None
    try:
        time.sleep(6)
        csub = subprocess.Popen([ENVSH, "ign", "topic", "-e", "-t", "/robot/camera"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(12)
    finally:
        for p in (csub, sim):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except (ProcessLookupError, AttributeError):
                pass
        log.close()
    time.sleep(1)

    frames = sorted(CAMDIR.glob("*.png"))
    if not frames:
        sys.exit("FAIL 프레임 없음 — 렌더/구독 실패")
    im = np.asarray(Image.open(frames[-1]).convert("RGB")).astype(np.int32)
    R, G, B = im[..., 0], im[..., 1], im[..., 2]
    pix = {
        "c": _centroid((R > 150) & (G < 90) & (B < 90)),
        "x": _centroid((B > 150) & (R < 90) & (G < 90)),
        "y": _centroid((G > 150) & (R < 90) & (B < 90)),
    }
    for k, v in pix.items():
        print(f"  마커 {k}: world={MARKERS[k]} pixel={None if v is None else (round(v[0],1),round(v[1],1))}")
    if not all(pix.values()):
        sys.exit("FAIL 마커를 못 찾음 — 색/배치 확인")

    uc, vc = pix["c"]
    Jpix = np.array([[pix["x"][0] - uc, pix["y"][0] - uc],
                     [pix["x"][1] - vc, pix["y"][1] - vc]], float)  # world→pixel (STEP 단위)
    A = STEP * np.linalg.inv(Jpix)  # pixel→world
    mpp = float(np.linalg.norm(A[:, 0]))
    print(f"\n픽셀→월드 아핀 A = {np.round(A, 6).tolist()}")
    print(f"이미지 중심 (uc,vc) = ({uc:.1f},{vc:.1f}) = world {MARKERS['c']}")
    print(f"MPP ≈ {mpp*1000:.3f} mm/px")
    # 검산
    ok = True
    for k, (wx, wy) in MARKERS.items():
        dw = A @ np.array([pix[k][0] - uc, pix[k][1] - vc])
        rx, ry = MARKERS["c"][0] + dw[0], MARKERS["c"][1] + dw[1]
        e = ((rx - wx) ** 2 + (ry - wy) ** 2) ** 0.5
        print(f"  검산 {k}: 복원 ({rx:+.3f},{ry:+.3f}) vs 실제 ({wx},{wy}) 오차 {e*1000:.2f}mm")
        ok = ok and e < 0.003
    print("\ndetect_server.py 상수와 대조:  MPP, 중심(640,360), +x→-row/+y→-col")
    print("OK 캘리브 재현" if ok else "주의: 복원 오차 큼 — detect_server 상수 재확인")


if __name__ == "__main__":
    main()
