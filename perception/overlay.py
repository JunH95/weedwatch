#!/usr/bin/env python3
"""인식 결과를 눈으로 보게 오버레이 PNG 를 만든다 (사람 검증용).

held-out eval 프레임 몇 장에 best.pt 를 돌려, 각 행에 [원본 RGB | 예측 오버레이] 를 나란히
붙인다. 오버레이는 예측 클래스(콩 초록/옥수수 파랑/잡초 빨강)를 반투명으로 칠하고, 검출한
잡초 인스턴스의 중심(스탬핑 타격점)을 흰 십자로 표시한다. 즉 "모델이 이 정원을 어떻게 보고
어디를 찍을지"를 그대로 보여준다.

출력: artifacts/perception_overlay.png (gitignore). 파일을 그냥 열어 보면 된다.
사용: perception/env.sh python perception/overlay.py [--n 3]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw
from scipy import ndimage

import segmentation_models_pytorch as smp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from seg_data import (  # noqa: E402
    CLASS_COLORS, NUM_CLASSES, WEED, _MEAN, _STD, list_pairs,
)

WW = HERE.parent
MIN_AREA_PX = 400   # stamp_targets 와 동일: 스탬핑 대상 잡초 최소 크기


def predict(model, img_path, device):
    arr = np.asarray(Image.open(img_path).convert("RGB"), dtype=np.float32)
    x = (arr / 255.0 - _MEAN) / _STD
    x = torch.from_numpy(x.transpose(2, 0, 1).copy()).float().unsqueeze(0).to(device)
    with torch.no_grad():
        return model(x).argmax(1)[0].cpu().numpy()


def colorize(mask):
    out = np.zeros((*mask.shape, 3), dtype=np.uint8)
    for c in range(NUM_CLASSES):
        out[mask == c] = CLASS_COLORS[c]
    return out


def weed_centroids_px(mask):
    lbl, n = ndimage.label(mask == WEED)
    pts = []
    for i in range(1, n + 1):
        ys, xs = np.where(lbl == i)
        if len(xs) >= MIN_AREA_PX:
            pts.append((xs.mean(), ys.mean()))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(WW / "models" / "dataset" / "eval"))
    ap.add_argument("--ckpt", default=str(WW / "models" / "best.pt"))
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--out", default=str(WW / "artifacts" / "perception_overlay.png"))
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    pairs = list_pairs(Path(args.data))
    if not pairs:
        sys.exit(f"평가 데이터 없음: {args.data}")
    # 프레임이 다양하게 보이도록 고르게 뽑는다
    step = max(1, len(pairs) // args.n)
    picks = pairs[::step][:args.n]

    ck = torch.load(args.ckpt, map_location=device, weights_only=False)
    model = smp.Unet(encoder_name=ck.get("encoder", "resnet34"), encoder_weights=None,
                     in_channels=3, classes=NUM_CLASSES).to(device)
    model.load_state_dict(ck["model"])
    model.eval()

    rows = []
    for ip, _ in picks:
        rgb = Image.open(ip).convert("RGB")
        mask = predict(model, ip, device)
        pred_rgb = Image.fromarray(colorize(mask))
        # 반투명 합성: 원본 어둡게 + 예측 색
        blend = Image.blend(rgb, pred_rgb, 0.55)
        draw = ImageDraw.Draw(blend)
        for cx, cy in weed_centroids_px(mask):
            r = 9
            draw.line([(cx - r, cy), (cx + r, cy)], fill=(255, 255, 255), width=3)
            draw.line([(cx, cy - r), (cx, cy + r)], fill=(255, 255, 255), width=3)
        # [원본 | 오버레이] 가로로
        w, h = rgb.size
        row = Image.new("RGB", (w * 2 + 8, h), (20, 20, 20))
        row.paste(rgb, (0, 0))
        row.paste(blend, (w + 8, 0))
        rows.append(row)

    W = rows[0].width
    H = sum(r.height for r in rows) + 8 * (len(rows) - 1)
    canvas = Image.new("RGB", (W, H), (20, 20, 20))
    y = 0
    for r in rows:
        canvas.paste(r, (0, y))
        y += r.height + 8
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.out)
    n_weeds = sum(len(weed_centroids_px(predict(model, ip, device))) for ip, _ in picks)
    print(f"오버레이 저장: {args.out}")
    print(f"  프레임 {len(picks)}장, 검출한 스탬핑 대상 잡초 {n_weeds}개(흰 십자)")
    print("  왼쪽=원본 정원, 오른쪽=모델 예측(콩 초록/옥수수 파랑/잡초 빨강)+타격점")


if __name__ == "__main__":
    main()
