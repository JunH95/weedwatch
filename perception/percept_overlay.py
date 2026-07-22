#!/usr/bin/env python3
"""사람 눈 검증용: 로봇 카메라 실제 렌더 + best.pt 잡초 예측 + 오라클 정답을 한 장에 겹친다.

make percept 가 낸 artifacts/camera 프레임에 best.pt 를 돌려, 예측 잡초(빨강 반투명) + 검출 중심
(하늘색 +) + 오라클 target(초록 원=시야안·노랑=시야밖) 을 그린다 → artifacts/percept_overlay.png.
단언이 아니라 눈으로 "카메라가 이걸 봤고 AI 가 이걸 찾았다" 를 확인하는 용도.

실행:  perception/env.sh python perception/percept_overlay.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
WW = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(WW / "tools"))
from detect_server import load_model, _predict, detect_frame, MPP, CU, CV, CAM_DX, WEED  # noqa
from oracle import load as oracle_load  # noqa

CAMDIR = WW / "artifacts" / "camera"
OUT = WW / "artifacts" / "percept_overlay.png"
INCLUDE_OFF = (-1.12, 0.10)
BASE = (0.0, 0.60, 0.0, 0.0)   # 정지 spawn (make percept 와 동일)


def world_to_pixel(wx, wy, cam_x, cam_y):
    row = CV - (wx - cam_x) / MPP
    col = CU - (wy - cam_y) / MPP
    return col, row


def main():
    frames = sorted(CAMDIR.glob("*.png"))
    if not frames:
        sys.exit("artifacts/camera 에 프레임이 없음 — 먼저 make percept")
    fr = frames[-1]
    model, device = load_model()
    pred = _predict(model, str(fr), device)
    dets = detect_frame(model, str(fr), BASE, device)
    cam_x, cam_y = BASE[0] + CAM_DX, BASE[1]

    base_img = Image.open(fr).convert("RGB")
    ov = base_img.copy()
    # 잡초 예측 픽셀을 빨강 반투명
    red = np.zeros((*pred.shape, 4), np.uint8)
    red[pred == WEED] = (255, 40, 40, 110)
    ov = Image.alpha_composite(ov.convert("RGBA"), Image.fromarray(red)).convert("RGB")
    d = ImageDraw.Draw(ov)

    # 검출 중심 (하늘색 +)
    for wx, wy, _ in dets:
        u, v = world_to_pixel(wx, wy, cam_x, cam_y)
        d.line([(u - 8, v), (u + 8, v)], fill=(80, 220, 255), width=2)
        d.line([(u, v - 8), (u, v + 8)], fill=(80, 220, 255), width=2)

    # 오라클 target (초록 원=시야안, 노랑=시야밖)
    HX, HY = CV * MPP, CU * MPP
    og = oracle_load(str(WW / "models" / "oracle_test.json"))
    for w in og.weeds:
        tx, ty = w.x + INCLUDE_OFF[0], w.y + INCLUDE_OFF[1]
        inview = abs(tx - cam_x) < HX and abs(ty - cam_y) < HY
        u, v = world_to_pixel(tx, ty, cam_x, cam_y)
        if -20 < u < CU * 2 + 20 and -20 < v < CV * 2 + 20:
            color = (60, 255, 60) if inview else (255, 230, 40)
            d.ellipse([u - 16, v - 16, u + 16, v + 16], outline=color, width=3)

    ov.save(OUT)
    print(f"저장: {OUT}")
    print(f"  로봇 카메라 프레임: {fr.name}")
    print(f"  빨강 반투명 = best.pt 잡초 예측 | 하늘색 + = 검출 중심 | 초록 원 = 오라클 정답(시야안) | 노랑 = 시야밖")
    print(f"  검출 blob {len(dets)}개, 오라클 target {len(og.weeds)}개")


if __name__ == "__main__":
    main()
