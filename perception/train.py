#!/usr/bin/env python3
"""4클래스 세그멘테이션 학습 (Stage 3-2c) → models/best.pt (ROS 와의 디스크 계약).

근거 레시피 (DECISIONS 015, docs/REFERENCES.md):
  손실 = 0.6·가중CE + 0.4·Dice (콩+잡초 PMC11136954) · CE 가중 = inverse-sqrt (PMC13275391)
  지표 = per-class IoU·recall (전체 정확도 금지 — 흙 75%라 무의미)
모델 = smp U-Net (resnet34, imagenet 사전학습). RTX 4060 8GB → AMP(mixed precision).

models/dataset/train 을 학습하고, 그 안에서 **시드 기준** val 을 분리한다(같은 정원의 비슷한
프레임이 train/val 로 갈리는 누수 방지). eval_seeds 로 만든 models/dataset/eval 은 최종 게이트라
학습에서 절대 안 본다. best = val 잡초 IoU 최고 체크포인트.

사용: perception/env.sh python perception/train.py --epochs 40
"""
from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

import segmentation_models_pytorch as smp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from seg_data import (  # noqa: E402
    CLASS_NAMES, NUM_CLASSES, WEED, SegDataset, class_pixel_counts,
    inverse_sqrt_weights, list_pairs, split_by_seed,
)
from metrics import confusion, per_class_iou_recall  # noqa: E402

WW = HERE.parent


def evaluate(model, loader, device):
    model.eval()
    cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=np.int64)
    with torch.no_grad():
        for img, lbl in loader:
            pred = model(img.to(device)).argmax(1).cpu().numpy().ravel()
            cm += confusion(pred, lbl.numpy().ravel(), NUM_CLASSES)
    return per_class_iou_recall(cm)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(WW / "models" / "dataset" / "train"))
    ap.add_argument("--out", default=str(WW / "models" / "best.pt"))
    ap.add_argument("--encoder", default="resnet34")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=4)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    # 재현성: 시드 고정 + cuDNN 결정적. 모델(models/best.pt)은 gitignore 되는 산출물이지만,
    # (데이터 + 이 코드 + 시드)로 결정적으로 재생성돼야 한다 — 데이터셋과 같은 재현 계약.
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    def worker_init(wid):
        s = args.seed + wid
        np.random.seed(s)
        random.seed(s)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        print("⚠️ CUDA 없음 — CPU 로 학습(매우 느림). GPU 확인 필요.", file=sys.stderr)

    pairs = list_pairs(Path(args.data))
    if not pairs:
        sys.exit(f"학습 데이터 없음: {args.data} (make bake 먼저)")
    tr_pairs, va_pairs = split_by_seed(pairs)
    if not va_pairs:
        sys.exit("val 분리 결과가 비었다 — 시드가 val_every(10) 배수를 안 가짐")
    print(f"train {len(tr_pairs)}장 / val {len(va_pairs)}장 (시드 분리, val = seed%10==0)")

    counts = class_pixel_counts(tr_pairs)
    weights = inverse_sqrt_weights(counts)
    print("클래스 픽셀:", dict(zip(CLASS_NAMES, counts.tolist())))
    print("CE 가중(inverse-sqrt):", dict(zip(CLASS_NAMES, np.round(weights, 3).tolist())))

    g = torch.Generator().manual_seed(args.seed)
    tr = DataLoader(SegDataset(tr_pairs, augment=True), batch_size=args.batch,
                    shuffle=True, num_workers=args.workers, drop_last=True,
                    worker_init_fn=worker_init, generator=g)
    va = DataLoader(SegDataset(va_pairs, augment=False), batch_size=args.batch,
                    shuffle=False, num_workers=args.workers)

    model = smp.Unet(encoder_name=args.encoder, encoder_weights="imagenet",
                     in_channels=3, classes=NUM_CLASSES).to(device)
    w = torch.tensor(weights, dtype=torch.float32, device=device)
    ce = nn.CrossEntropyLoss(weight=w)
    dice = smp.losses.DiceLoss(mode="multiclass")

    def loss_fn(logits, lbl):
        return 0.6 * ce(logits, lbl) + 0.4 * dice(logits, lbl)

    opt = torch.optim.AdamW(model.parameters(), lr=args.lr)
    use_amp = device == "cuda"
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)

    best = -1.0
    for ep in range(1, args.epochs + 1):
        model.train()
        tot = 0.0
        for img, lbl in tr:
            img, lbl = img.to(device), lbl.to(device)
            opt.zero_grad()
            with torch.amp.autocast("cuda", enabled=use_amp):
                loss = loss_fn(model(img), lbl)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            tot += loss.item()

        iou, recall = evaluate(model, va, device)
        msg = " · ".join(f"{n} IoU {iou[i]:.3f}/R {recall[i]:.3f}"
                         for i, n in enumerate(CLASS_NAMES))
        print(f"[{ep:2d}/{args.epochs}] loss {tot/len(tr):.4f} | {msg}", flush=True)
        if iou[WEED] > best:
            best = iou[WEED]
            Path(args.out).parent.mkdir(parents=True, exist_ok=True)
            torch.save({"model": model.state_dict(), "encoder": args.encoder,
                        "classes": CLASS_NAMES, "val_iou": iou.tolist(),
                        "val_recall": recall.tolist()}, args.out)
            print(f"     ↑ best (val 잡초 IoU {best:.3f}) 저장 → {args.out}", flush=True)

    print(f"=== 학습 끝. best val 잡초 IoU {best:.3f} → {args.out} ===")


if __name__ == "__main__":
    main()
