#!/usr/bin/env python3
"""작물·잡초 종별 표본 (make species). 각 종을 정하방으로 하나씩 렌더해 이름과 함께 한 장에.
사용자 요청: "각 작물과 잡초 이름 해서 그거 하나만 딱" — 색 마스크 없이, 눈으로 종 구분용.
render_species.py(Blender) 가 artifacts/species/<종>.png 를 먼저 만들어야 한다."""
import os, sys
from PIL import Image, ImageDraw, ImageFont
WW = os.path.join(os.path.dirname(__file__), "..")
SP = [("bean", "콩", "작물", (0, 150, 0)), ("maize", "옥수수", "작물", (0, 150, 0)),
      ("polygonum", "마디풀", "잡초", (200, 30, 30)), ("portulaca", "쇠비름", "잡초", (200, 30, 30)),
      ("taraxacum", "민들레", "잡초", (200, 30, 30))]


def _fnt(s):
    for p in ("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
              "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, s)
    return ImageFont.load_default()


def main():
    sd = os.path.join(WW, "artifacts", "species")
    if not all(os.path.exists(os.path.join(sd, f"{s[0]}.png")) for s in SP):
        sys.exit("종별 렌더 없음 — 먼저: blender --background --python tools/render_species.py")
    TS, pad, top, labh = 300, 14, 64, 60
    W = len(SP) * TS + (len(SP) + 1) * pad
    H = top + TS + labh + pad
    cv = Image.new("RGB", (W, H), (248, 247, 243)); d = ImageDraw.Draw(cv)
    d.text((pad, 16), "weedwatch — 종별 표본 (로봇 카메라가 보는 정하방)", fill=(20, 20, 20), font=_fnt(28))
    x = pad
    for key, kr, cls, c in SP:
        cv.paste(Image.open(os.path.join(sd, f"{key}.png")).convert("RGB").resize((TS, TS)), (x, top))
        d.rectangle([x, top + TS, x + TS, top + TS + labh], fill=c)
        d.text((x + 10, top + TS + 5), kr, fill=(255, 255, 255), font=_fnt(26))
        d.text((x + 10, top + TS + 35), f"{cls} · {key}", fill=(255, 255, 255), font=_fnt(16))
        x += TS + pad
    out = os.path.join(WW, "artifacts", "species_reference.png")
    cv.save(out); print("저장:", out)


if __name__ == "__main__":
    main()
