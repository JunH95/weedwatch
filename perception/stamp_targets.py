#!/usr/bin/env python3
"""인식을 미터 좌표로 바꾸고 그 정확도를 단언한다 (Stage 4-1).

best.pt 로 잡초를 검출하고, 연결요소로 잡초 인스턴스를 나눈 뒤, 각 중심을 카메라 기하로
카메라상대 미터 오프셋(이미지 중심 기준)으로 변환한다. 이 오프셋이 "툴을 얼마나 움직여
잡초 위에 설까"의 입력이다. Stage 4-2 가 이 값을 캐리지·주행 명령으로 쓴다.

카메라 기하는 configs/train_garden.yaml 의 render.camera 에서 읽는다(단일 출처).
top-down(pitch=0), 높이 H, 정사각 FOV(Blender camera.angle = radians(fov_deg)).
지면 footprint = 2·H·tan(fov/2), m_per_px = footprint / resolution.

원근 왜곡 주의: 키 있는 잡초는 가장자리에서 약간 밀린다. 다만 4-1 은 같은 이미지의 예측 vs GT
비교라 이 왜곡이 양쪽에 똑같이 걸려 상쇄된다(측정하는 건 분할 위치오차). 절대좌표 왜곡은
4-2(로봇 월드좌표 변환)에서 다룬다.

최소 크기: 400px(약 26mm 지름)보다 작은 잡초 blob 은 타깃에서 뺀다. 툴 막대가 12mm 이므로
그 두 배(약 2.5cm)를 스탬핑 대상 잡초의 최소 크기로 잡는다. 그보다 작은 건 다음 순찰에 잡는다.

사용:
  perception/env.sh python perception/stamp_targets.py           # 리포트
  perception/env.sh python perception/stamp_targets.py --gate    # 단언
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import torch
import yaml
from PIL import Image
from scipy import ndimage

import segmentation_models_pytorch as smp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from seg_data import (  # noqa: E402
    NUM_CLASSES, WEED, _MEAN, _STD, mask_rgb_to_index, list_pairs,
)

WW = HERE.parent

# 게이트 임계 (보호 대상, 골대). 2026-07-21 baseline(min_area 400) 관측치에서 보정:
# 검출률 0.822, 위치오차 중앙 1.4mm / p90 13mm. 임계는 관측치보다 여유를 두되(변동 허용),
# 검출을 못 하거나 좌표가 크게 어긋나는 회귀는 잡는다. 모델 바꾸는 커밋에서 함께 낮추지 마라.
MIN_AREA_PX = 400
MATCH_RADIUS_M = 0.04       # 예측·GT 중심이 이 안이면 같은 잡초로 본다
GATES = {
    "detection_rate": 0.75,
    "median_err_m": 0.004,
    "p90_err_m": 0.020,
}


def load_geom(cfg_path: Path):
    cfg = yaml.safe_load(cfg_path.read_text())
    cam = cfg["render"]["camera"]
    res_x = int(cfg["render"]["resolution_x"])
    res_y = int(cfg["render"]["resolution_y"])
    footprint = 2.0 * float(cam["height"]) * math.tan(math.radians(float(cam["fov_deg"])) / 2.0)
    return footprint / res_x, res_x, res_y


def weed_instances(mask_idx: np.ndarray, m_per_px: float, res_x: int, res_y: int,
                   min_area: int = MIN_AREA_PX):
    """잡초 클래스 연결요소를 [(ox_m, oy_m, area_px)] 로. ox/oy 는 이미지 중심 기준 미터 오프셋."""
    lbl, n = ndimage.label(mask_idx == WEED)
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(lbl == i)
        if len(xs) < min_area:
            continue
        ox = (xs.mean() - res_x / 2.0) * m_per_px
        oy = (ys.mean() - res_y / 2.0) * m_per_px
        out.append((ox, oy, int(len(xs))))
    return out


def predict_mask(model, img_path: str, device: str) -> np.ndarray:
    arr = np.asarray(Image.open(img_path).convert("RGB"), dtype=np.float32)
    x = (arr / 255.0 - _MEAN) / _STD
    x = torch.from_numpy(x.transpose(2, 0, 1).copy()).float().unsqueeze(0).to(device)
    with torch.no_grad():
        return model(x).argmax(1)[0].cpu().numpy()


def match(pred, gt):
    """GT 중심마다 반경 내 가장 가까운 예측을 1:1 로 짝짓는다. (검출수, 오차[m] 목록, 오탐수)."""
    used = [False] * len(pred)
    errs = []
    detected = 0
    for gx, gy, _ in gt:
        best, bd = -1, MATCH_RADIUS_M
        for j, (px, py, _) in enumerate(pred):
            if used[j]:
                continue
            d = math.hypot(px - gx, py - gy)
            if d < bd:
                bd, best = d, j
        if best >= 0:
            used[best] = True
            detected += 1
            errs.append(bd)
    return detected, errs, used.count(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(WW / "models" / "dataset" / "eval"))
    ap.add_argument("--ckpt", default=str(WW / "models" / "best.pt"))
    ap.add_argument("--cfg", default=str(WW / "configs" / "train_garden.yaml"))
    ap.add_argument("--min-area", type=int, default=MIN_AREA_PX)
    ap.add_argument("--gate", action="store_true")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    m_per_px, res_x, res_y = load_geom(Path(args.cfg))
    pairs = list_pairs(Path(args.data))
    if not pairs:
        sys.exit(f"평가 데이터 없음: {args.data}")

    ck = torch.load(args.ckpt, map_location=device, weights_only=False)
    model = smp.Unet(encoder_name=ck.get("encoder", "resnet34"), encoder_weights=None,
                     in_channels=3, classes=NUM_CLASSES).to(device)
    model.load_state_dict(ck["model"])
    model.eval()

    tot_gt = tot_det = tot_fp = tot_pred = 0
    all_errs = []
    for ip, mp in pairs:
        gt = weed_instances(mask_rgb_to_index(np.asarray(Image.open(mp).convert("RGB"))),
                            m_per_px, res_x, res_y, args.min_area)
        pred = weed_instances(predict_mask(model, ip, device), m_per_px, res_x, res_y, args.min_area)
        det, errs, fp = match(pred, gt)
        tot_gt += len(gt); tot_pred += len(pred); tot_det += det; tot_fp += fp
        all_errs += errs

    errs = np.array(all_errs) if all_errs else np.array([0.0])
    det_rate = tot_det / max(tot_gt, 1)
    precision = tot_det / max(tot_pred, 1)
    med = float(np.median(errs)); p90 = float(np.percentile(errs, 90)); mx = float(errs.max())

    print(f"스탬프 타깃 평가: {args.ckpt} on {args.data} ({len(pairs)}장)")
    print(f"  카메라 {res_x}px, {m_per_px*1000:.3f} mm/px (지면 footprint {m_per_px*res_x:.3f} m)")
    print(f"  GT 잡초 {tot_gt}, 예측 {tot_pred}, 검출 {tot_det}, 오탐 {tot_fp}")
    print(f"  검출률 {det_rate:.3f}, 정밀도 {precision:.3f}")
    print(f"  위치오차({len(all_errs)}쌍): 중앙 {med*1000:.1f}mm, p90 {p90*1000:.1f}mm, 최대 {mx*1000:.1f}mm")

    checks = {
        "detection_rate": (det_rate, det_rate >= GATES["detection_rate"]),
        "median_err_m": (med, med <= GATES["median_err_m"]),
        "p90_err_m": (p90, p90 <= GATES["p90_err_m"]),
    }
    fails = []
    for k, (v, ok) in checks.items():
        op = ">=" if k == "detection_rate" else "<="
        print(f"  게이트 {k} {v:.4f} {op} {GATES[k]} {'OK' if ok else '미달'}")
        if not ok:
            fails.append(f"{k} {v:.4f}")

    if args.gate and fails:
        print("게이트 실패: " + ", ".join(fails), file=sys.stderr)
        sys.exit(1)
    if args.gate:
        print("게이트 통과: 검출을 정확한 타격 좌표로 변환한다")


if __name__ == "__main__":
    main()
