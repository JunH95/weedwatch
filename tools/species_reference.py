#!/usr/bin/env python3
"""작물 vs 잡초 참조 (make species). 학습 렌더 RGB + 정답 마스크를 나란히.
옥수수 많은 장면 · 잡초 많은 장면 · 콩 많은 장면을 골라, 실제 CropCraft 식물과 4클래스 정답을 보여준다.
한계: 마스크가 잡초 3종을 다 빨강으로 뭉뚱그려 종별 구분은 안 된다(종별 개별 렌더는 별도 — TODO).
출력: artifacts/species_reference.png"""
import glob, os, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "perception"))
from seg_data import CLASS_COLORS  # noqa: E402
WW = os.path.join(os.path.dirname(__file__), "..")
def _mp(ip): return os.path.join(WW,"models/dataset/train/masks",os.path.basename(ip).replace(".jpg",".png"))
def _frac(mp):
    m=np.asarray(Image.open(mp).convert("RGB")).reshape(-1,3)
    l=np.abs(m[:,None,:]-CLASS_COLORS[None,:,:]).sum(2).argmin(1)
    return [(l==k).mean() for k in range(4)]
def _fnt(s):
    for p in ("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf","/usr/share/fonts/truetype/nanum/NanumGothic.ttf","/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p): return ImageFont.truetype(p,s)
    return ImageFont.load_default()
def main():
    imgs=[i for i in sorted(glob.glob(os.path.join(WW,"models/dataset/train/images/*.jpg"))) if os.path.exists(_mp(i))]
    if not imgs: sys.exit("학습 이미지 없음 — make bake 먼저")
    scenes=[(max(imgs,key=lambda i:_frac(_mp(i))[2]),"옥수수(작물)가 있는 장면"),
            (max(imgs,key=lambda i:_frac(_mp(i))[3]),"잡초가 많은 장면"),
            (max(imgs,key=lambda i:_frac(_mp(i))[1]),"콩(작물)이 많은 장면")]
    pal=np.array([[45,40,35],[0,200,0],[40,40,235],[230,30,30]],dtype="uint8")
    TH,TW,gap,top=340,600,12,150
    W,H=TW*2+gap*3, top+len(scenes)*(TH+gap)+gap
    cv=Image.new("RGB",(W,H),(248,247,243)); d=ImageDraw.Draw(cv)
    d.text((gap,12),"weedwatch — 작물 vs 잡초 (왼쪽: 카메라가 보는 것 · 오른쪽: 정답)",fill=(20,20,20),font=_fnt(28))
    d.text((gap,52),"작물 = 콩(넓고 둥근 잎) · 옥수수(길쭉한 잎)   |   잡초 = 마디풀·쇠비름·민들레 3종(마스크는 다 빨강)",fill=(60,60,60),font=_fnt(18))
    for i,(nm,c) in enumerate([("콩=초록",(0,200,0)),("옥수수=파랑",(40,40,235)),("잡초=빨강",(230,30,30)),("흙=검정",(45,40,35))]):
        x=gap+i*260; d.rectangle([x,86,x+24,108],fill=c,outline=(0,0,0)); d.text((x+30,86),nm,fill=(20,20,20),font=_fnt(19))
    y=top
    for ip,title in scenes:
        arr=np.asarray(Image.open(_mp(ip)).convert("RGB")); h,w=arr.shape[:2]
        l=np.abs(arr.reshape(-1,3)[:,None,:]-CLASS_COLORS[None,:,:]).sum(2).argmin(1)
        cv.paste(Image.open(ip).convert("RGB").resize((TW,TH)),(gap,y))
        cv.paste(Image.fromarray(pal[l].reshape(h,w,3)).resize((TW,TH),Image.NEAREST),(gap*2+TW,y))
        d.text((gap+6,y+6),title,fill=(255,255,0),font=_fnt(20)); y+=TH+gap
    out=os.path.join(WW,"artifacts","species_reference.png"); os.makedirs(os.path.dirname(out),exist_ok=True)
    cv.save(out); print("저장:",out)
if __name__=="__main__": main()
