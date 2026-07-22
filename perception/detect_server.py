#!/usr/bin/env python3
"""로봇 카메라 프레임에 best.pt 를 라이브 추론해 잡초 world 좌표를 뽑는다 (Stage 4-3 Phase 4).

ROS(3.10)↔ML(3.11 venv) 경계는 디스크 파일이다(공유 import 금지, perception/env.sh 참고). 카메라
프레임은 Gazebo down_cam 의 <save> 태그로 artifacts/camera/*.png 에 떨어진다(구독자 있어야 렌더 —
CLAUDE.md 함정). 이 모듈은 그 PNG 를 읽어 best.pt 로 잡초를 검출하고, 카메라 기하로 world 좌표를 낸다.

── 픽셀→월드 매핑 (worlds/robot_calib.sdf 색 마커로 직접 캘리브, 오차 0) ──────────────
카메라는 base 전방(0.22,0,0.58)에 정하방 고정. 이미지 중심(640,360)=카메라 직하점.
  +row(이미지 아래) → base 전방(-x),  +col(이미지 우) → base 좌(-y),  0.457mm/px.
  base 프레임 오프셋:  dx_base = -MPP·(row-360),  dy_base = -MPP·(col-640)
  world (yaw 반영):    (x,y) = cam_xy + R(yaw)·(dx_base, dy_base)
정지(4a)면 yaw≈0. 주행(4b)이면 base yaw 로 회전. 높이 정합(Phase 3): 두둑 z=0.25, 카메라 0.33m 위.

사용:
  detect_frame(model, png, base_pose) → [(wx, wy, area_px)]   # import 재사용 (assert_percept, 4b)
  perception/env.sh python detect_server.py --watch <dir> --out <file> --base x y z yaw   # 상주(4b)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from scipy import ndimage

import segmentation_models_pytorch as smp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from seg_data import _MEAN, _STD, NUM_CLASSES, WEED  # noqa: E402

WW = HERE.parent

# ── 카메라 기하 (worlds/robot_calib.sdf 캘리브 결과. 바꾸려면 재캘리브) ──
MPP = 0.000457              # m/px (두둑 z=0.25 위 0.33m). calibrate_camera 로 유도.
CU, CV = 640.0, 360.0       # 이미지 중심 (1280×720)
CAM_DX, CAM_DZ = 0.22, 0.58  # base 기준 카메라 X/Z 오프셋 (links.json camera_world)
MIN_AREA_PX = 400           # 이보다 작은 잡초 blob 제외 (≈2.5cm, stamp_targets 와 동일 규율)


def load_model(ckpt: str = None, device: str = None):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    ckpt = ckpt or str(WW / "models" / "best.pt")
    ck = torch.load(ckpt, map_location=device, weights_only=False)
    model = smp.Unet(encoder_name=ck.get("encoder", "resnet34"), encoder_weights=None,
                     in_channels=3, classes=NUM_CLASSES).to(device)
    model.load_state_dict(ck["model"])
    model.eval()
    return model, device


def _predict(model, img_path: str, device: str) -> np.ndarray:
    arr = np.asarray(Image.open(img_path).convert("RGB"), dtype=np.float32)
    x = (arr / 255.0 - _MEAN) / _STD
    x = torch.from_numpy(x.transpose(2, 0, 1).copy()).float().unsqueeze(0).to(device)
    with torch.no_grad():
        return model(x).argmax(1)[0].cpu().numpy()


def filter_weed_mask(mask: np.ndarray, k: int = 5) -> np.ndarray:
    """잡초 마스크 정리 (4b-2): 형태학적 열림(얇은 잎경계·흙 오검출 노이즈 제거) + 닫힘(잎 가림으로
    쪼개진 한 잡초 파편을 하나로 병합). k×k 구조요소. best.pt 원시 마스크는 blob 이 과분할·노이즈가
    많아(4a 오버레이) 그대로면 로봇이 엉뚱한 데/한 잡초를 여러 번 찍는다."""
    st = np.ones((k, k), bool)
    return ndimage.binary_closing(ndimage.binary_opening(mask, st), st)


def detect_frame(model, img_path: str, base_pose, device: str = "cuda",
                 min_area: int = MIN_AREA_PX, filter_noise: bool = True):
    """프레임 → [(world_x, world_y, area_px)]. base_pose=(x,y,z,yaw) 지상진실.

    잡초 클래스 연결요소 중심을 카메라 기하로 world 좌표화. yaw 로 base 전방/좌 오프셋을 회전.
    filter_noise=True 면 형태학 필터(4b-2)로 노이즈·과분할을 정리한 뒤 인스턴스를 뽑는다.
    """
    bx, by, bz, byaw = base_pose
    cam_x = bx + math.cos(byaw) * CAM_DX
    cam_y = by + math.sin(byaw) * CAM_DX
    c, s = math.cos(byaw), math.sin(byaw)
    pred = _predict(model, img_path, device)
    mask = pred == WEED
    if filter_noise:
        mask = filter_weed_mask(mask)
    lbl, n = ndimage.label(mask)
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(lbl == i)
        if len(xs) < min_area:
            continue
        col, row = xs.mean(), ys.mean()
        dx_base = -MPP * (row - CV)      # base 전방(+x)
        dy_base = -MPP * (col - CU)      # base 좌(+y)
        wx = cam_x + c * dx_base - s * dy_base
        wy = cam_y + s * dx_base + c * dy_base
        out.append((wx, wy, int(len(xs))))
    return out


def _latest_png(d: Path):
    pngs = sorted(d.glob("*.png"), key=lambda p: p.stat().st_mtime)
    return pngs[-1] if pngs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", help="폴링할 프레임 디렉토리 (상주 모드, 4b)")
    ap.add_argument("--out", help="검출 world 좌표를 쓸 파일 (라인당 'x y area')")
    ap.add_argument("--base", nargs=4, type=float, metavar=("X", "Y", "Z", "YAW"),
                    default=[0.0, 0.6, 0.0, 0.0], help="지상진실 base pose (정지 4a 는 고정)")
    ap.add_argument("--ckpt", default=None)
    ap.add_argument("--once", help="한 프레임만 검출해 stdout 에 출력하고 종료")
    args = ap.parse_args()
    model, device = load_model(args.ckpt)

    if args.once:
        for wx, wy, a in detect_frame(model, args.once, tuple(args.base), device):
            print(f"{wx:.4f} {wy:.4f} {a}")
        return

    if args.watch:
        wd = Path(args.watch)
        last = None
        print("R detect_server ready", flush=True)  # 핸드셰이크
        while True:
            f = _latest_png(wd)
            if f and f != last:
                last = f
                dets = detect_frame(model, str(f), tuple(args.base), device)
                if args.out:
                    Path(args.out).write_text("\n".join(f"{x:.4f} {y:.4f} {a}" for x, y, a in dets))
                print(f"D {len(dets)} {f.name}", flush=True)
            time.sleep(0.05)


if __name__ == "__main__":
    main()
