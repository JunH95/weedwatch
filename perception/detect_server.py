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
from seg_data import _MEAN, _STD, NUM_CLASSES, WEED, MAIZE  # noqa: E402

WW = HERE.parent
BEAN = 1  # seg_data 클래스: 0 흙 · 1 콩 · 2 옥수수(MAIZE) · 3 잡초(WEED)

# 기하는 tools/garden_geometry 단일 출처에서 읽는다. ROS↔ML 경계 규율(공유 import 금지)은 **rclpy 등
# ROS 의존**을 안 섞는다는 뜻이고, garden_geometry 는 math+dataclass 뿐인 순수 모듈이라 안전하다.
# 하드코딩했다가 "카메라가 두둑 전체를 본다"는 틀린 상수가 굳어 재현율 상한 0.65 를 만든 적이 있다
# (DECISIONS 026) — 그래서 이제 읽어온다. assert_percept 도 이미 tools/ 를 import 한다.
sys.path.insert(0, str(WW / "tools"))
from garden_geometry import Garden, Portal  # noqa: E402

_G, _P = Garden(), Portal()

# ── 카메라 기하 (worlds/robot_calib.sdf 캘리브 + garden_geometry) ──
MPP = _P.camera_mpp                       # m/px (두둑 위 0.33m). calibrate_camera 로 유도.
CU, CV = _P.camera_w / 2, _P.camera_h / 2  # 이미지 중심 (1280×720)
CAM_DX, CAM_DZ = _P.camera_x, _P.camera_z()   # base 기준 카메라 X/Z 오프셋
CAM_DYS = _P.camera_ys(_G)                # 카메라별 base 기준 Y 오프셋. n=2 → [-0.225, +0.225]
MIN_AREA_PX = 400           # 이보다 작은 잡초 blob 제외 (≈2.5cm, stamp_targets 와 동일 규율)

# 두 카메라 겹침 구간(0.135m)에서 같은 잡초를 두 번 잡는다. 이 반경 안이면 하나로 합친다.
# 5cm 인 이유: 겹침 안 잡초는 두 카메라에서 서로 반대쪽으로 시차가 생겨(캐노피 높이×off-nadir 거리)
# 완전히 같은 좌표로 안 떨어진다. 너무 키우면 진짜로 가까운 별개 잡초를 합쳐버린다 — 그 균형점.
DEDUP_R = 0.05

# ── 깊이 (Stage 5) ────────────────────────────────────────────────────────────
# 카메라에서 캘리브 평면(두둑 윗면)까지 거리. MPP 는 이 평면에서 잰 값이라, 다른 높이의 픽셀은
# 오프셋을 depth/H 로 스케일해야 한다 — 그게 시차 보정의 전부다.
CAL_H = _P.camera_height_above_bed(_G)     # ≈0.33 m
# 실물 D405 열화 파라미터 (데이터시트 337029-017 Table 4-14: D401/D405, ≤0.5m, Z-accuracy ±2%).
DEPTH_RMS_FRAC = 0.02      # 거리 비례 노이즈 (0.33m 에서 6.6mm)
DEPTH_FLYING_FRAC = 0.25   # 잎 경계 픽셀 중 flying pixel 비율 (스펙 밖, 스테레오 고질)
DEPTH_DROPOUT_FRAC = 0.02  # 무효 픽셀 비율. 실험실 fill rate ≥99% 지만 야외는 더 나쁘다
# 타격 가능한 잡초의 높이 상한 [m]. 이보다 높은 픽셀은 잡초 blob 안에 있어도 **칠 수 있는 잡초가
# 아니다** — 실측: 콩 캐노피 22.9cm, 마스크에 섞여 들어온 픽셀 13.9~20.8cm vs 진짜 납작한 잡초
# 0.8~1.1cm. 근거: 점 타격이 듣는 건 BBCH ≤12(본잎 2장, Langsenkamp 2014)이고 우리 종은 쇠비름
# 1.0~2.6cm · 마디풀 2.3~8.0cm. 5cm 면 그 위를 넉넉히 덮으면서 작물 캐노피는 확실히 배제한다.
# 주의: 이건 "키로 잡초를 판별"하는 Tertill 규칙이 아니다. 잡초 판별은 세그멘테이션(종·형태)이
# 하고, 높이는 **이미 잡초로 판정된 것이 칠 수 있는 단계인가**만 가른다 (DECISIONS 007·027).
MAX_STRIKE_H = 0.05


def load_model(ckpt: str = None, device: str = None):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    ckpt = ckpt or str(WW / "models" / "best.pt")
    ck = torch.load(ckpt, map_location=device, weights_only=False)
    model = smp.Unet(encoder_name=ck.get("encoder", "resnet34"), encoder_weights=None,
                     in_channels=3, classes=NUM_CLASSES).to(device)
    model.load_state_dict(ck["model"])
    model.eval()
    return model, device


def _predict(model, img, device: str) -> np.ndarray:
    """img: 파일 경로/Path 또는 이미 로드된 RGB 배열(H,W,3). ROS 노드는 카메라 토픽 배열을 바로 넣는다."""
    if isinstance(img, np.ndarray):
        arr = img.astype(np.float32)
    else:
        arr = np.asarray(Image.open(img).convert("RGB"), dtype=np.float32)
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
                 min_area: int = MIN_AREA_PX, filter_noise: bool = True,
                 safe_dist: float = 0.0, cam_dy: float = 0.0, depth=None):
    """프레임 → [(world_x, world_y, area_px)]. base_pose=(x,y,z,yaw) 지상진실.

    잡초 클래스 연결요소 중심을 카메라 기하로 world 좌표화. yaw 로 base 전방/좌 오프셋을 회전.
    filter_noise=True 면 형태학 필터(4b-2)로 노이즈·과분할을 정리.
    **작물 회피(safe_dist)**: 콩·옥수수(best.pt 작물 클래스)에서 이 거리 안의 잡초는 뺀다 — 점타격이
    작물을 칠 위험. DECISIONS 007 safe-remove(작물 코앞 잡초는 사람 몫). 오라클 아닌 로봇 제 인식으로.
    **cam_dy**: 이 프레임을 찍은 카메라의 base 기준 Y 오프셋(DECISIONS 026, n=2 → ∓0.225). 카메라마다
    직하점이 달라서 이걸 안 넣으면 바깥 카메라 검출이 통째로 0.225m 밀린다.
    **depth**: (H,W) 미터 배열(ww_depth). 주면 시차 보정을 한다 — 잎이 높이 h 에 떠 있어 생기는
    투영 밀림(화면 가장자리에서 최대 ~4cm)을 실측 거리로 되짚는다. None 이면 기존 평지 가정.
    """
    bx, by, _bz, byaw = base_pose
    c, s = math.cos(byaw), math.sin(byaw)
    cam_x = bx + c * CAM_DX - s * cam_dy      # 카메라 직하점 (body→world 회전)
    cam_y = by + s * CAM_DX + c * cam_dy
    pred = _predict(model, img_path, device)
    mask = pred == WEED
    if filter_noise:
        mask = filter_weed_mask(mask)
    dist_crop, safe_px = None, 0.0
    if safe_dist > 0:                                       # 작물 회피 켤 때만 계산
        crop = (pred == BEAN) | (pred == MAIZE)
        dist_crop = ndimage.distance_transform_edt(~crop)  # 각 픽셀 → 최근접 작물 픽셀 거리(px)
        safe_px = safe_dist / MPP
    lbl, n = ndimage.label(mask)
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(lbl == i)
        if len(xs) < min_area:
            continue
        col, row = xs.mean(), ys.mean()
        if dist_crop is not None and dist_crop[int(round(row)), int(round(col))] < safe_px:
            continue                                        # 작물 코앞 → safe-remove (007)
        if depth is None:
            # 평지 가정: 모든 픽셀이 캘리브 평면(두둑 윗면)에 있다고 본다. 잎이 높이 h 에 떠 있으면
            # 직하점에서 멀수록 밀려 보인다(가장자리에서 최대 ~4cm) — 그게 지금 오차의 주범이다.
            dx_base = -MPP * (row - CV)
            dy_base = -MPP * (col - CU)
        else:
            # 시차 보정: 픽셀 오프셋을 실제 거리로 스케일한다. MPP 는 거리 CAL_H 에서 잰 값이라
            # 거리 d 인 픽셀의 참 오프셋 = MPP·Δpx·(d/CAL_H). 잎이 높으면 d<CAL_H → 오프셋이 줄어든다.
            dd = depth[ys, xs]
            ok = np.isfinite(dd) & (dd > 0)                 # 무효 픽셀(구멍) 제외
            # 높이 게이트: 지면에서 MAX_STRIKE_H 위 픽셀은 뺀다. 세그가 작물 캐노피를 잡초로
            # 흘린 픽셀(실측 14~21cm)이 중심을 끌고 가는 걸 막는다 — 깊이만이 할 수 있는 일.
            ok &= (CAL_H - dd) <= MAX_STRIKE_H
            if ok.sum() < max(8, 0.05 * len(xs)):
                continue                                    # 지면 높이 픽셀이 거의 없음 = 칠 대상 아님
            scale = (dd[ok] / CAL_H).astype(np.float64)
            dx_base = float(np.mean(-MPP * (ys[ok] - CV) * scale))
            dy_base = float(np.mean(-MPP * (xs[ok] - CU) * scale))
        wx = cam_x + c * dx_base - s * dy_base
        wy = cam_y + s * dx_base + c * dy_base
        out.append((wx, wy, int(len(xs))))
    return out


def load_depth(path):
    """ww_depth 가 내린 원본 깊이 프레임 → (H,W) float32 미터. 형식 [u32 w][u32 h][f32...]."""
    hdr = np.fromfile(path, np.uint32, 2)
    w, h = int(hdr[0]), int(hdr[1])
    d = np.fromfile(path, np.float32, offset=8)
    if d.size != w * h:
        raise ValueError(f"깊이 크기 불일치: {d.size} != {w}x{h}")
    return d.reshape(h, w)


def degrade_depth(depth, rng, edge_mask=None):
    """시뮬 깊이를 실물 D405 답게 망가뜨린다 (전처리로 되돌릴 수 없는 잔차를 남긴다).

    왜 필요한가: gz 깊이는 렌더 기하에서 나온 값이라 거의 완벽하다. 완벽한 센서로 좋은 결과를 내면
    실물에서 무너지는 걸 못 잡는다(IMU orientation 에서 똑같은 함정을 밟았다 — DECISIONS 025 보정).

    근거 (Intel D400 시리즈 데이터시트 337029-017, Table 4-14 — D401/D405, ≤0.5m, 80% ROI):
      · Z-accuracy(절대오차) **±2%** → 0.33m 에서 ±6.6mm. **거리 비례**(우리 sim 은 거리 무관 3mm 고정)
      · Fill rate ≥99% — 단 **실험실 조건**이다. 야외 직사광은 IR 패턴을 씻어 구멍이 훨씬 많다.
    스펙에 없지만 스테레오의 고질인 것:
      · **flying pixel** — 얇은 잎 경계에서 깊이가 잎과 뒤 지면 사이 엉뚱한 값으로 튄다.
        잡초 잎이 얇아서 우리한테 특히 아프다.
    """
    d = depth.astype(np.float32, copy=True)
    valid = np.isfinite(d) & (d > 0)
    # ① 거리 비례 노이즈 (스펙 2%)
    d[valid] += rng.normal(0.0, DEPTH_RMS_FRAC * d[valid]).astype(np.float32)
    # ② 잎 경계 flying pixel — 경계 픽셀 일부를 잎과 배경 사이 값으로 대체
    if edge_mask is not None and DEPTH_FLYING_FRAC > 0:
        idx = np.flatnonzero(edge_mask.ravel() & valid.ravel())
        if idx.size:
            pick = idx[rng.random(idx.size) < DEPTH_FLYING_FRAC]
            if pick.size:
                far = float(np.nanmedian(d[valid])) if valid.any() else 0.0
                t = rng.random(pick.size).astype(np.float32)      # 잎↔배경 사이 아무 데나
                flat = d.ravel()
                flat[pick] = flat[pick] * (1 - t) + far * t
    # ③ 무효 픽셀(구멍) — 야외 IR 씻김. 실험실 fill rate 는 ≥99% 지만 밭은 더 나쁘다.
    if DEPTH_DROPOUT_FRAC > 0:
        holes = rng.random(d.shape) < DEPTH_DROPOUT_FRAC
        d[holes] = np.nan
    return d


def merge_detections(dets, radius: float = DEDUP_R):
    """겹침 구간에서 두 카메라가 같은 잡초를 두 번 잡은 걸 하나로 (순수 함수 — Tier 1 단언 대상).

    면적 큰 것부터 잡고 radius 안의 검출을 흡수한다. 위치는 면적 가중 평균(큰 blob 이 신뢰도 높음),
    면적은 max — 같은 잡초를 두 번 본 것이지 두 개가 아니므로 더하지 않는다.
    입력 [(wx, wy, area)] → 출력 [(wx, wy, area)] (면적 내림차순).
    """
    out: list[list[float]] = []
    for wx, wy, a in sorted(dets, key=lambda d: -d[2]):
        for o in out:
            if math.hypot(wx - o[0], wy - o[1]) <= radius:
                tot = o[2] + a
                o[0] = (o[0] * o[2] + wx * a) / tot
                o[1] = (o[1] * o[2] + wy * a) / tot
                o[2] = max(o[2], a)
                break
        else:
            out.append([wx, wy, float(a)])
    return [(x, y, int(a)) for x, y, a in out]


def detect_fused(model, frames, base_pose, device: str = "cuda", depths=None, **kw):
    """카메라 여러 대의 프레임을 한 번에 → 융합된 [(wx, wy, area)].

    frames: [(cam_index, png_path)]. 각 프레임을 그 카메라의 Y 오프셋(CAM_DYS)으로 world 화한 뒤
    겹침 중복을 합친다. 한 대로는 두둑 폭의 65% 밖에 못 봤다(DECISIONS 026) — 이 함수가 나머지를 준다.
    """
    alld = []
    for ci, path in frames:
        cam_dy = CAM_DYS[ci] if ci < len(CAM_DYS) else 0.0
        dep = (depths or {}).get(ci)
        alld += detect_frame(model, str(path), base_pose, device, cam_dy=cam_dy, depth=dep, **kw)
    return merge_detections(alld)


def _latest_png(d: Path):
    # 두 번째 최신 반환 — 가장 최신 PNG 는 sim 이 지금 쓰는 중일 수 있어(반쯤 쓰인 파일 → PIL 크래시).
    pngs = sorted(d.glob("*.png"), key=lambda p: p.stat().st_mtime)
    if len(pngs) >= 2:
        return pngs[-2]
    return pngs[-1] if pngs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", help="폴링할 프레임 디렉토리. 카메라 여러 대면 쉼표로 "
                    "인덱스 순서대로 (예: artifacts/camera,artifacts/camera1). 상주 모드(4b)")
    ap.add_argument("--out", help="검출 world 좌표를 쓸 파일 (라인당 'x y area')")
    ap.add_argument("--base", nargs=4, type=float, metavar=("X", "Y", "Z", "YAW"),
                    default=[0.0, 0.6, 0.0, 0.0], help="지상진실 base pose (정지 4a 는 고정)")
    ap.add_argument("--odom-file", help="주행(4b-3): 이 파일의 현재 odom_x 를 읽어 base 를 앵커링. "
                    "제어=odom 규율(GT 아님). base_y 는 --base 의 Y 사용.")
    ap.add_argument("--safe-dist", type=float, default=0.0,
                    help="작물(콩·옥수수) 이 거리[m] 안의 잡초는 뺀다(safe-remove, 007). 0=끔.")
    ap.add_argument("--ckpt", default=None)
    ap.add_argument("--once", help="한 프레임만 검출해 stdout 에 출력하고 종료")
    args = ap.parse_args()
    model, device = load_model(args.ckpt)
    base_y = args.base[1]

    if args.once:
        for wx, wy, a in detect_frame(model, args.once, tuple(args.base), device):
            print(f"{wx:.4f} {wy:.4f} {a}")
        return

    def read_odom():
        """odom 파일에서 base (x, y) 를 읽는다. 'x' 한 값이면 y 는 --base 의 Y(단일 두둑).
        'x y' 두 값이면 둘 다 쓴다 — 관통(P3)에서 두둑마다 base_y 가 달라서다(DECISIONS 036)."""
        try:
            parts = Path(args.odom_file).read_text().split()
            x = float(parts[0])
            y = float(parts[1]) if len(parts) > 1 else base_y
            return x, y
        except (FileNotFoundError, ValueError, IndexError):
            return None

    if args.watch:
        wds = [Path(p.strip()) for p in args.watch.split(",") if p.strip()]
        last = None
        print(f"R detect_server ready ({len(wds)} cam)", flush=True)  # 핸드셰이크
        while True:
            # 카메라별 최신 프레임을 모은다. 대표(0번) 프레임이 바뀔 때만 한 사이클 돈다 —
            # 두 카메라가 같은 15Hz 라 프레임이 거의 동시에 떨어진다(정합 오차 무시 가능).
            frames = [(i, f) for i, wd in enumerate(wds) if (f := _latest_png(wd)) is not None]
            if not frames or frames[0][1] == last:
                time.sleep(0.03); continue
            last = frames[0][1]
            if args.odom_file:                      # 주행: odom 으로 base_x(,y) 앵커링
                o = read_odom()
                if o is None:
                    time.sleep(0.02); continue
                base = (o[0], o[1], 0.0, 0.0)
            else:
                base = tuple(args.base)
            try:
                dets = detect_fused(model, frames, base, device, safe_dist=args.safe_dist)
            except Exception as e:                  # 반쯤 쓰인 PNG 등 → 스킵(다음 프레임)
                print(f"E skip {frames[0][1].name}: {e}", flush=True)
                time.sleep(0.02); continue
            if args.out:                            # 융합된 world 검출 (하네스가 소비)
                Path(args.out).write_text(
                    f"# {base[0]:.4f}\n" + "\n".join(f"{x:.4f} {y:.4f} {a}" for x, y, a in dets))
            print(f"D {len(dets)} @x={base[0]:.2f} cam{len(frames)} {frames[0][1].name}", flush=True)
            time.sleep(0.03)


if __name__ == "__main__":
    main()
