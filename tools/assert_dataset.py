#!/usr/bin/env python3
"""학습 데이터셋(RGB + 세그멘테이션 마스크) 검증 — Stage 3-1.

CropCraft 내장 렌더가 configs/train_garden.yaml 로부터 만든 images/ + masks/ 쌍이
학습에 쓸 만한가를 단언한다. "렌더가 나왔다"만으로는 부족 — 마스크가 실제로 작물/잡초를
가르는지(3클래스 색이 정확하고, 두 클래스가 다 존재하는지) 확인해야 한다.

게이트:
  1. images 장수 == masks 장수 > 0
  2. 모든 마스크 픽셀이 정확히 4클래스 색(흙 검정 / 콩 초록 / 옥수수 파랑 / 잡초 빨강) — 양자화 확인
  3. 데이터셋 전체에 콩·옥수수·잡초 픽셀이 모두 존재 (한 클래스라도 0이면 그 클래스 학습 불가)

색은 configs/train_garden.yaml 의 label_colors 와 일치해야 한다 (DECISIONS 016, 4클래스).
실행:  ./scripts/env.sh python3 tools/assert_dataset.py   (make dataset 가 호출)
"""
from __future__ import annotations

import glob
import sys
from pathlib import Path

import numpy as np
from PIL import Image

WW = Path(__file__).resolve().parents[1]
RENDER = WW / "models" / "render"
# 순서 = configs/train_garden.yaml label_colors. 흙/콩/옥수수/잡초.
CLASSES = {"흙(배경)": (0, 0, 0), "콩": (0, 255, 0), "옥수수": (0, 0, 255), "잡초": (255, 0, 0)}
# 픽셀이 반드시 존재해야 하는 클래스(인덱스): 콩·옥수수·잡초 (흙은 당연히 있음).
REQUIRE_PRESENT = {"콩": 1, "옥수수": 2, "잡초": 3}


def main():
    imgs = sorted(glob.glob(str(RENDER / "images" / "*.jpg")))
    masks = sorted(glob.glob(str(RENDER / "masks" / "*.png")))
    print(f"=== 데이터셋 검증: {RENDER} ===")
    print(f"  이미지 {len(imgs)}장 / 마스크 {len(masks)}장")

    errs = []
    if not (len(imgs) == len(masks) > 0):
        print(f"❌ 이미지·마스크 장수 불일치 또는 0 ({len(imgs)} vs {len(masks)})", file=sys.stderr)
        sys.exit(1)

    palette = np.array(list(CLASSES.values()))          # (3,3)
    totals = np.zeros(len(CLASSES), dtype=np.int64)
    off_palette_total = 0
    for m in masks:
        arr = np.array(Image.open(m).convert("RGB")).reshape(-1, 3)
        # 각 픽셀을 3색 중 최근접에 매칭, 정확 일치인지 확인 (양자화 검증)
        d = np.abs(arr[:, None, :].astype(int) - palette[None, :, :]).sum(-1)  # (N,3)
        nearest = d.argmin(1)
        off = (d.min(1) > 0).sum()                       # 3색 중 어느 것과도 정확히 안 맞는 픽셀
        off_palette_total += int(off)
        totals += np.bincount(nearest, minlength=len(CLASSES))

    tot = totals.sum()
    print("  클래스별 픽셀 비율:")
    for (name, _), c in zip(CLASSES.items(), totals):
        print(f"    {name:8s}: {100*c/tot:5.2f}%  ({c:,})")
    print(f"  팔레트 밖 픽셀: {off_palette_total:,} (양자화 후 0 이어야)")

    if off_palette_total > 0:
        errs.append(f"마스크에 {len(CLASSES)}클래스 밖 색 {off_palette_total} 픽셀 — 양자화 실패")
    for name, idx in REQUIRE_PRESENT.items():
        if totals[idx] == 0:
            errs.append(f"{name} 픽셀이 0 — 마스크가 {name}을(를) 라벨 못함")

    if errs:
        print("\n❌ 데이터셋 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print("\n=== ✅ 데이터셋 통과 — RGB+마스크 쌍이 흙/콩/옥수수/잡초를 픽셀단위로 가른다 ===")


if __name__ == "__main__":
    main()
