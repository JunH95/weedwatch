#!/usr/bin/env python3
"""작물 vs 잡초 눈으로 보는 참조 이미지 (사용자 요청 — GUI 색칠 원기둥으로는 구분 불가).

학습 데이터(models/dataset)의 렌더 RGB + 정답 마스크를 나란히 붙인다. 왼쪽=카메라가 보는 것,
오른쪽=정답(초록 콩·파랑 옥수수·빨강 잡초). watch-strikes/watch-jam 의 색칠 원기둥은 위치 표식일
뿐이고, 진짜 CropCraft 식물이 어떻게 생겼는지는 여기서 본다.  출력: artifacts/species_reference.png
"""
import glob, os, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "perception"))
from seg_data import CLASS_COLORS  # noqa: E402

WW = os.path.join(os.path.dirname(__file__), "..")


def _mask(ip):
    return os.path.join(WW, "models/dataset/train/masks", os.path.basename(ip).replace(".jpg", ".png"))


def _font(sz):
    for p in ("/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()


def main():
    imgs = sorted(glob.glob(os.path.join(WW, "models/dataset/train/images/*.jpg")))
    picks = []
    for ip in imgs:
        mp = _mask(ip)
        if not os.path.exists(mp):
            continue
        m = np.asarray(Image.open(mp).convert("RGB")).reshape(-1, 3)
        lbl = np.abs(m[:, None, :] - CLASS_COLORS[None, :, :]).sum(2).argmin(1)
        f = [(lbl == k).mean() for k in range(4)]
        if f[1] > 0.02 and f[3] > 0.015:
            picks.append((ip, mp))
        if len(picks) >= 3:
            break
    if not picks:
        sys.exit("학습 이미지가 없습니다 — 먼저 make bake")

    TH, TW, gap, top = 360, 640, 12, 96
    W, H = TW * 2 + gap * 3, top + len(picks) * (TH + gap) + gap
    cv = Image.new("RGB", (W, H), (245, 245, 240)); d = ImageDraw.Draw(cv)
    font, fbig = _font(20), _font(26)
    d.text((gap, 10), "weedwatch — 작물 vs 잡초 참조 (왼쪽 원본 | 오른쪽 정답)", fill=(20, 20, 20), font=fbig)
    for i, (nm, c) in enumerate([("콩=작물", (0, 200, 0)), ("옥수수=작물", (40, 40, 230)),
                                 ("잡초", (230, 30, 30)), ("흙", (40, 40, 40))]):
        x = gap + i * 230
        d.rectangle([x, 56, x + 26, 80], fill=c, outline=(0, 0, 0)); d.text((x + 32, 56), nm, fill=(20, 20, 20), font=font)
    pal = np.array([[40, 40, 40], [0, 200, 0], [40, 40, 230], [230, 30, 30]], dtype="uint8")
    y = top
    for ip, mp in picks:
        arr = np.asarray(Image.open(mp).convert("RGB")); h, w = arr.shape[:2]
        lbl = np.abs(arr.reshape(-1, 3)[:, None, :] - CLASS_COLORS[None, :, :]).sum(2).argmin(1)
        cv.paste(Image.open(ip).convert("RGB").resize((TW, TH)), (gap, y))
        cv.paste(Image.fromarray(pal[lbl].reshape(h, w, 3)).resize((TW, TH), Image.NEAREST), (gap * 2 + TW, y))
        d.text((gap + 6, y + 6), "카메라가 보는 것", fill=(255, 255, 0), font=font)
        d.text((gap * 2 + TW + 6, y + 6), "정답: 초록=콩 파랑=옥수수 빨강=잡초", fill=(255, 255, 0), font=font)
        y += TH + gap
    out = os.path.join(WW, "artifacts", "species_reference.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv.save(out)
    print(f"저장: {out}")


if __name__ == "__main__":
    main()
