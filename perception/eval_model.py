#!/usr/bin/env python3
"""학습 모델을 held-out eval 시드에서 평가 + 게이트 (Stage 3-2d/3-3).

models/best.pt 를 models/dataset/eval(= eval_seeds, 학습이 한 번도 안 본 정원)에서 돌려
per-class IoU·recall 을 재고, **희소 클래스(잡초·옥수수)**가 임계 이상인지 단언한다.
전체 정확도는 게이트로 쓰지 않는다 — 흙 75%라 "전부 흙"이 이미 ~75%다(DECISIONS 015).

🔒 임계값은 보호 대상(골대). 모델을 바꾸는 커밋에서 임계를 같이 낮추지 마라(검증 계약).
사용:
  perception/env.sh python perception/eval_model.py            # 리포트만
  perception/env.sh python perception/eval_model.py --gate     # 단언(게이트)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

import segmentation_models_pytorch as smp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from seg_data import (  # noqa: E402
    CLASS_NAMES, MAIZE, NUM_CLASSES, WEED, SegDataset, list_pairs,
)
from metrics import confusion, per_class_iou_recall  # noqa: E402

WW = HERE.parent

# 🔒 게이트 임계값 — 2026-07-21 시드 baseline(held-out) 에서 보정. 관측치:
#    잡초 IoU 0.904 / recall 0.965 · 옥수수 IoU 0.926 / recall 0.985 (make train, seed 0).
#    임계는 관측치보다 ~0.05-0.08 아래 — 학습 변동(AMP/CUDA 잔여 비결정성)은 통과시키되,
#    희소 클래스를 무시하는 회귀(IoU 급락)는 잡는다. 모델 바꾸는 커밋에서 함께 낮추지 마라(골대).
GATES = {
    ("weed", "IoU"): 0.85,
    ("weed", "recall"): 0.90,
    ("maize", "IoU"): 0.85,
    ("maize", "recall"): 0.90,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(WW / "models" / "dataset" / "eval"))
    ap.add_argument("--ckpt", default=str(WW / "models" / "best.pt"))
    ap.add_argument("--batch", type=int, default=4)
    ap.add_argument("--gate", action="store_true", help="임계 미달 시 실패로 종료")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    if not Path(args.ckpt).exists():
        sys.exit(f"체크포인트 없음: {args.ckpt} (make train 먼저)")
    pairs = list_pairs(Path(args.data))
    if not pairs:
        sys.exit(f"평가 데이터 없음: {args.data} (make bake 먼저)")

    ck = torch.load(args.ckpt, map_location=device, weights_only=False)
    model = smp.Unet(encoder_name=ck.get("encoder", "resnet34"), encoder_weights=None,
                     in_channels=3, classes=NUM_CLASSES).to(device)
    model.load_state_dict(ck["model"])
    model.eval()

    loader = DataLoader(SegDataset(pairs, augment=False), batch_size=args.batch)
    cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=np.int64)
    with torch.no_grad():
        for img, lbl in loader:
            pred = model(img.to(device)).argmax(1).cpu().numpy().ravel()
            cm += confusion(pred, lbl.numpy().ravel(), NUM_CLASSES)
    iou, recall = per_class_iou_recall(cm)

    print(f"=== 평가: {args.ckpt} on {args.data} ({len(pairs)}장, held-out) ===")
    for i, n in enumerate(CLASS_NAMES):
        print(f"  {n:6s}: IoU {iou[i]:.3f}  recall {recall[i]:.3f}")
    print(f"  mIoU {iou.mean():.3f}  (전체 정확도는 게이트로 안 씀 — 흙 지배)")

    metrics = {"weed": {"IoU": iou[WEED], "recall": recall[WEED]},
               "maize": {"IoU": iou[MAIZE], "recall": recall[MAIZE]}}
    fails = []
    for (cls, kind), thr in GATES.items():
        val = metrics[cls][kind]
        mark = "✅" if val >= thr else "❌"
        print(f"  게이트 {cls} {kind} {val:.3f} ≥ {thr:.2f} {mark}")
        if val < thr:
            fails.append(f"{cls} {kind} {val:.3f} < {thr:.2f}")

    if args.gate and fails:
        print("\n❌ 게이트 실패:\n    - " + "\n    - ".join(fails), file=sys.stderr)
        sys.exit(1)
    if args.gate:
        print("\n=== ✅ 게이트 통과 — 희소 클래스(잡초·옥수수) 임계 이상 ===")


if __name__ == "__main__":
    main()
