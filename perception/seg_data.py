#!/usr/bin/env python3
"""4클래스 세그멘테이션 데이터셋 로더 (Stage 3-2). models/dataset/<split>/{images,masks}.

마스크는 configs/train_garden.yaml 의 label_colors 로 렌더된 RGB PNG(양자화됨):
  흙=검정 · 콩=초록 · 옥수수=파랑 · 잡초=빨강. 이 색을 클래스 인덱스(0..3)로 바꾼다.
색 팔레트는 assert_dataset.py · train_garden.yaml 과 반드시 일치해야 한다(DECISIONS 016).
이게 어긋나면 라벨이 조용히 틀린다 — 그래서 한 곳(여기)에서만 정의하고 아래 자기점검을 둔다.
"""
from __future__ import annotations

import glob
import random
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

# ── 클래스 계약 (configs/train_garden.yaml label_colors 와 일치) ──────────────
CLASS_NAMES = ["soil", "bean", "maize", "weed"]
CLASS_COLORS = np.array([[0, 0, 0], [0, 255, 0], [0, 0, 255], [255, 0, 0]], dtype=np.int16)
NUM_CLASSES = len(CLASS_NAMES)
WEED = 3
MAIZE = 2  # 희소 클래스들 — 게이트가 이 둘을 직접 겨눈다

# ImageNet 정규화 (smp 인코더가 imagenet 사전학습이라 이 통계로 맞춘다)
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def mask_rgb_to_index(arr: np.ndarray) -> np.ndarray:
    """(H,W,3) RGB → (H,W) 클래스 인덱스. 최근접 팔레트 색(양자화돼 있어 보통 정확 일치)."""
    d = np.abs(arr[:, :, None, :].astype(np.int16) - CLASS_COLORS[None, None, :, :]).sum(-1)
    return d.argmin(-1).astype(np.int64)


def list_pairs(split_dir: Path) -> list[tuple[str, str]]:
    """(이미지.jpg, 마스크.png) 쌍 목록. 스템(seed<N>_frame_xxxx)으로 짝짓는다."""
    pairs = []
    for ip in sorted(glob.glob(str(Path(split_dir) / "images" / "*.jpg"))):
        mp = Path(split_dir) / "masks" / (Path(ip).stem + ".png")
        if mp.exists():
            pairs.append((ip, str(mp)))
    return pairs


def seed_of(path: str) -> int:
    """seed<N>_frame_xxxx → N. 시드 기준 train/val 분리에 쓴다(같은 정원 프레임 누수 방지)."""
    return int(Path(path).name.split("_")[0].replace("seed", ""))


def split_by_seed(pairs, val_every: int = 10):
    """시드 기준으로 train/val 분리. seed % val_every == 0 인 시드가 val.
    프레임이 아니라 시드로 나눠야, 같은 정원의 비슷한 프레임이 train/val 에 갈리지 않는다."""
    tr, va = [], []
    for ip, mp in pairs:
        (va if seed_of(ip) % val_every == 0 else tr).append((ip, mp))
    return tr, va


class SegDataset(Dataset):
    def __init__(self, pairs, augment: bool = False):
        self.pairs = pairs
        self.augment = augment

    def __len__(self):
        return len(self.pairs)

    def _aug(self, img: np.ndarray, lbl: np.ndarray):
        # 기하: 좌우/상하 뒤집기 + 90도 회전 (top-down 이라 회전 대칭이 자연스럽다)
        if random.random() < 0.5:
            img, lbl = img[:, ::-1, :], lbl[:, ::-1]
        if random.random() < 0.5:
            img, lbl = img[::-1, :, :], lbl[::-1, :]
        k = random.randint(0, 3)
        if k:
            img, lbl = np.rot90(img, k, (0, 1)), np.rot90(lbl, k, (0, 1))
        # 광도: 밝기 스케일 (자연광 변동을 흉내)
        if random.random() < 0.5:
            img = np.clip(img * random.uniform(0.8, 1.2), 0, 255)
        return np.ascontiguousarray(img), np.ascontiguousarray(lbl)

    def __getitem__(self, i):
        ip, mp = self.pairs[i]
        img = np.asarray(Image.open(ip).convert("RGB"), dtype=np.float32)          # HWC 0..255
        lbl = mask_rgb_to_index(np.asarray(Image.open(mp).convert("RGB")))          # HW 0..3
        if self.augment:
            img, lbl = self._aug(img, lbl)
        img = (img / 255.0 - _MEAN) / _STD
        img = torch.from_numpy(img.transpose(2, 0, 1).copy()).float()              # CHW
        return img, torch.from_numpy(lbl.copy()).long()


def class_pixel_counts(pairs) -> np.ndarray:
    """클래스별 픽셀 수 (손실 가중 계산용). inverse-sqrt 가중의 입력."""
    counts = np.zeros(NUM_CLASSES, dtype=np.int64)
    for _, mp in pairs:
        idx = mask_rgb_to_index(np.asarray(Image.open(mp).convert("RGB")))
        counts += np.bincount(idx.ravel(), minlength=NUM_CLASSES)
    return counts


def inverse_sqrt_weights(counts: np.ndarray) -> np.ndarray:
    """가중 = 1/sqrt(빈도), 평균 1로 정규화. raw inverse-freq(과벌점)·median-freq(과소가중)
    대신 inverse-sqrt (PMC13275391). 0 픽셀 클래스는 빈도 최소값으로 막는다."""
    freq = counts / max(counts.sum(), 1)
    freq = np.maximum(freq, 1e-6)
    w = 1.0 / np.sqrt(freq)
    return w / w.mean()
