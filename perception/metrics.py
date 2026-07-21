#!/usr/bin/env python3
"""세그멘테이션 지표 — per-class IoU·recall (혼동행렬 기반).

전체(global) 정확도는 **일부러 안 넣는다**: 흙이 ~75% 라 "전부 흙" 예측이 이미 ~75% 정확도라서,
소수 클래스(잡초·옥수수)의 실패를 숨긴다(DECISIONS 015, PMC13275391). 게이트는 희소 클래스의
IoU·recall 을 직접 본다.
"""
from __future__ import annotations

import numpy as np


def confusion(pred: np.ndarray, target: np.ndarray, n: int) -> np.ndarray:
    """1D 정수 pred/target → (n,n) 혼동행렬 [행=정답, 열=예측]."""
    k = target.astype(np.int64) * n + pred.astype(np.int64)
    return np.bincount(k, minlength=n * n).reshape(n, n)


def per_class_iou_recall(cm: np.ndarray):
    """혼동행렬 → (iou[n], recall[n]). 분모 0 은 1로 막아 nan 방지."""
    tp = np.diag(cm).astype(np.float64)
    fp = cm.sum(0) - tp
    fn = cm.sum(1) - tp
    iou = tp / np.maximum(tp + fp + fn, 1.0)
    recall = tp / np.maximum(tp + fn, 1.0)
    return iou, recall
