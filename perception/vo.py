#!/usr/bin/env python3
"""시각 오도메트리 — 하방 카메라가 보는 지면 흐름으로 **바퀴가 못 보는 이동**을 잰다.

DECISIONS 041 의 측정이 이 파일의 설계를 정했다:

  · 위상상관(FFT)으로 프레임 간 평행이동을 픽셀로 찾는다. 특징점 검출이 필요 없어 흙처럼
    반복적인 질감에서도 동작한다.
  · **깊이는 스케일이 아니라 선택에 쓴다.** 밭은 평면이 아니라 "픽셀당 몇 mm"가 하나로 존재하지
    않는다 — 중앙값 깊이로 스케일을 고쳤더니 65cm 주행을 18cm 로 봤다. 대신 캘리브 평면
    (두둑 윗면) ±5cm 픽셀만 남기면 고정 mm/px 가 그대로 맞다. 회전 관측 27%→86%.
  · **회전은 IMU 로 되돌린 뒤 상관한다.** 위상상관은 평행이동만 찾으므로 영상이 돌면 못 따라간다.

이 모듈은 순수 계산이다(ROS·시뮬 모름). ROS 노드(`ww_vo_node.py`)와 진단(`tools/diag_vo.py`)이
같은 코드를 쓰게 하려고 뺐다 — 진단에서 잰 성능이 실제 노드 성능이어야 하니까.
"""
from __future__ import annotations

import math

import numpy as np

MM_PER_PX = 0.457e-3     # m/px, 두둑 윗면에서 캘리브 (DECISIONS 022, 복원오차 0)
CAL_H = 0.33             # 그 값을 잰 거리 [m] (카메라 → 두둑 윗면)
SOIL_BAND = 0.05         # 캘리브 평면 ±이만큼을 "흙"으로 본다 [m]
MIN_SOIL_FRAC = 0.05     # 흙이 이보다 적으면 마스킹이 오히려 해가 된다


def phase_shift(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """두 그레이 이미지 사이 평행이동 (drow, dcol). 서브픽셀(3점 포물선) 보간 포함."""
    A, B = np.fft.rfft2(a), np.fft.rfft2(b)
    R = A * np.conj(B)
    mag = np.abs(R)
    mag[mag == 0] = 1e-9
    r = np.fft.irfft2(R / mag, s=a.shape)
    i0, j0 = np.unravel_index(np.argmax(r), r.shape)

    def refine(prev, peak, nxt):
        d = prev - 2 * peak + nxt
        return 0.0 if d == 0 else 0.5 * (prev - nxt) / d

    h, w = r.shape
    di = refine(r[(i0 - 1) % h, j0], r[i0, j0], r[(i0 + 1) % h, j0])
    dj = refine(r[i0, (j0 - 1) % w], r[i0, j0], r[i0, (j0 + 1) % w])
    drow = (i0 if i0 < h // 2 else i0 - h) + di
    dcol = (j0 if j0 < w // 2 else j0 - w) + dj
    return float(drow), float(dcol)


def derotate(img: np.ndarray, dyaw: float) -> np.ndarray:
    """영상을 중심 기준으로 dyaw 만큼 돌린다 (IMU 가 준 회전을 상쇄)."""
    if abs(dyaw) < math.radians(0.05):
        return img
    from scipy import ndimage
    return ndimage.rotate(img, math.degrees(dyaw), reshape=False, order=1, mode="nearest")


def soil_mask(depth: np.ndarray, cal_h: float = CAL_H, band: float = SOIL_BAND) -> np.ndarray:
    """캘리브 평면 근처(=흙) 픽셀 마스크. 식물(더 가까움)·고랑(더 멂)을 뺀다."""
    return (np.isfinite(depth) & (np.abs(depth - cal_h) < band)).astype(np.float32)


def to_gray(img: np.ndarray) -> np.ndarray:
    """RGB(H,W,3|4) 또는 이미 흑백(H,W) 둘 다 받는다 — 테스트·재사용이 쉬워진다."""
    if img.ndim == 2:
        return img.astype(np.float32)
    return img[:, :, :3].mean(axis=2).astype(np.float32)


class VoTracker:
    """프레임을 넣으면 **로봇 기준 이동 증분**(전방 m, 좌 m)을 돌려준다.

    반환은 증분이지 누적이 아니다 — 융합하는 쪽(GyroOdom)이 언제 이 값을 믿을지 정한다
    (직진은 바퀴가 더 정확하고, 회전 미끄러짐은 이것만 본다 — 041).
    """

    def __init__(self, mm_per_px: float = MM_PER_PX):
        self.mpp = mm_per_px
        self._prev = None
        self._prev_yaw = None
        self.frames = 0
        self.masked_frames = 0

    def _prep(self, rgb, depth):
        g = to_gray(rgb)
        if depth is not None and depth.shape == g.shape:
            mk = soil_mask(depth)
            if mk.mean() > MIN_SOIL_FRAC:
                g = (g - g.mean()) * mk       # 평균 제거 후 마스킹 — 경계 계단을 줄인다
                self.masked_frames += 1
        return g

    def remember(self, rgb: np.ndarray, depth=None, imu_yaw=None):
        """상관 없이 **프레임만 기억**한다 — 직진 구간처럼 VO 를 안 쓰는 동안 비용을 아낀다.
        다음에 update() 를 부르면 이 프레임이 직전 프레임이 된다."""
        self._prev, self._prev_yaw = self._prep(rgb, depth), imu_yaw

    def update(self, rgb: np.ndarray, depth: np.ndarray | None = None,
               imu_yaw: float | None = None):
        """(d_forward, d_left) [m] 또는 직전 프레임이 없으면 None."""
        g = self._prep(rgb, depth)
        prev, prev_yaw = self._prev, self._prev_yaw
        self._prev, self._prev_yaw = g, imu_yaw
        if prev is None:
            return None
        if imu_yaw is not None and prev_yaw is not None:
            dyaw = math.atan2(math.sin(imu_yaw - prev_yaw), math.cos(imu_yaw - prev_yaw))
            prev = derotate(prev, dyaw)
        drow, dcol = phase_shift(prev, g)
        self.frames += 1
        # 캘리브(022): +row → base 전방(−x) 방향이라 부호가 뒤집힌다.
        return -drow * self.mpp, -dcol * self.mpp
