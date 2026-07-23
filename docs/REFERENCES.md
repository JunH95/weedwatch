# 참고문헌 (References)

프로젝트의 **중요 결정을 뒷받침하는 근거**를 출처(URL)와 함께 쌓는다. 노션 참고자료로
그대로 옮길 수 있는 형태. 새 근거 조사는 여기에 카테고리별로 추가한다.
정책: 중요 결정은 임의로 말고 근거를 서치해서 정한다(memory `evidence-grounding-proactive`).

각 항목: **[제목](URL)** — 한줄 시사점 *(신뢰도)*.

---

## 잡초 밀도·현장 잡초압 (agronomy) — DECISIONS 015

- **[Does weed diversity mitigate yield losses? (PMC11272534)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11272534/)**
  — 실제 밭 잡초는 식물 바이오매스의 중앙값 4%(범위 1–27%), 둘째 사이트 8%. 저/고 잡초압을
  150/300 plants/m² 로 조작. → 우리 ~4.55% 픽셀이 현실적임을 뒷받침. *(high)*
- **[Common Purslane — SARE, Manage Weeds on Your Farm](https://www.sare.org/publications/manage-weeds-on-your-farm/common-purslane/)**
  — 쇠비름은 매트형: 흩어진 몇 포기가 이듬해 거의 카펫. 국소 피복은 밭평균이 낮아도 ~100%.
  → 균일 산포가 아니라 군집/겹침 배치가 현실적. *(medium)*
- **[Early-season weed competition — Iowa State ICM](https://crops.extension.iastate.edu/encyclopedia/early-season-weed-competition)**
  — 줄에서 12.5cm 내, 출현기 pigweed 2포기/줄m 가 12.3% 감수(늦게 나면 1.9%).
  → 잡초를 줄 근처·조기 발생에 집중 모델링할 근거. *(high)*
- **[Critical periods of competition in corn — Iowa State ICM](https://crops.extension.iastate.edu/encyclopedia/critical-periods-competition-corn)**
  — 잡초 키 2in(무손실)→4in(2%)→6in(6%). 어린(자엽~5cm) 잡초가 표적 → 잡초 픽셀 footprint 가
  작아 잡초 비율이 낮게 유지되는 이유. *(high)*
- **[경남지역 밭 잡초 발생분포 및 군락변화 — KISTI ScienceON](https://scienceon.kisti.re.kr/srch/selectPORSrchArticle.do?cn=JAKO201530856745995&dbt=NART)**
  — 한국 밭: 쇠비름 배추서 8.1% 우점, 바랭이 옥수수 11.3%·콩 13.2%. 우리 잡초셋 타당,
  단일종 우점 <~15%. *(medium)*

## weed-seg 데이터셋 클래스 밸런스 (dataset-benchmark) — DECISIONS 015

- **[Real-time Semantic Segmentation of Crop and Weed — Milioto et al., ICRA 2018 (arXiv:1709.06764)](https://ar5iv.labs.arxiv.org/html/1709.06764)**
  — 정통 Bonn/Sugar-Beets top-down: 흙 97.8% / 작물 1.7% / 잡초 0.7%. 가장 비교가능한 실벤치마크
  보다 우리 잡초가 ~6배 많다. *(high)*
- **[Active learning for crop-weed semantic segmentation (arXiv:2404.02580)](https://ar5iv.labs.arxiv.org/html/2404.02580)**
  — Corn-Weed 잡초 4.0%(≈우리 4.55%), Sugarbeet 0.2%·PhenoBench 0.5%(저단). 우리 잡초비율은
  정상~높음. *(high)*

## 불균형 학습: 손실·가중·지표 (loss-imbalance) — DECISIONS 015

- **[Class imbalance aware seg for weed & tobacco — PMC13275391](https://pmc.ncbi.nlm.nih.gov/articles/PMC13275391/)**
  — inverse-**sqrt** 빈도 가중(bg/crop/weed=1.232/2.634/2.253) + Lovász+가중CE 가 ablation 우승
  (mIoU 84.99). 전체정확도(aAcc 95.93)가 소수클래스 오류(mIoU 84.99)를 숨김. 잡초 19.7% 상단 앵커. *(high)*
- **[Bean seedlings & weeds via improved ERFNet — PMC11136954](https://pmc.ncbi.nlm.nih.gov/articles/PMC11136954/)**
  — **콩+잡초 특화**: 0.6·CE + 0.4·Dice → 잡초 IoU 86.88 / recall 94.39. 우리 벤치마크용 구체 목표. *(high)*
- **[Class-Balanced Loss (Effective Number) — Cui et al., CVPR 2019 (arXiv:1901.05555)](https://arxiv.org/pdf/1901.05555)**
  — 유효표본수 가중 w=(1-β)/(1-βⁿ), β=0.999–0.9999·γ=0.5 로 rare-class 가중 상한. 3-클래스
  거친 문제에 적합. *(high)*
- **[Lightweight Multispectral Crop-Weed Segmentation (arXiv:2505.07444)](https://arxiv.org/html/2505.07444)**
  — 포컬 γ=2, α=클래스가중. 포컬을 단독이 아니라 Dice/CE 와 조합. *(medium)*

## 희소클래스 증강 (augmentation) — DECISIONS 015

- **[Augment to Segment: Pixel-Level Imbalance in Wheat Disease/Pest (arXiv:2509.09961)](https://arxiv.org/html/2509.09961)**
  — naive copy-paste +0.21 IoU, **블렌딩/정제 시 +2.81 IoU / +2.84 acc**. 붙이기 자체보다 블렌딩이 핵심. *(high)*
- **[Simple Copy-Paste is a Strong Augmentation (arXiv:2012.07177)](https://arxiv.org/abs/2012.07177)**
  — copy-paste 이득은 가장 희소한 클래스에 집중(+3.6 mask AP, rare LVIS). 잡초 종별 균형 샘플링과 병행. *(high)*

## 합성데이터·sim-to-real (synthetic) — DECISIONS 015

- **[CropCraft: Procedural World Generator for Agricultural Sim (arXiv:2511.02417)](https://arxiv.org/abs/2511.02417)**
  — **우리가 쓰는 생성기.** 잡초 밀도는 튜닝 노브지만 순수합성 ~10% mIoU sim-to-real 갭,
  실사진 몇 장 추가로만 닫힘. *(high)*
- **[Synthetic images for weed detection in cotton — PMC9666527](https://pmc.ncbi.nlm.nih.gov/articles/PMC9666527/)**
  — 실밭이 75% 잔디여도 합성은 **일부러 33/33/33 균등** 강제. 실셋 없을 때 희소클래스 과표현의 선례.
  단 강한 실셋 있으면 합성 이득 미미. *(high)*
- **[Synthetic Examples Improve Generalization for Rare Classes — Beery, WACV 2020 (arXiv:1904.05916)](https://arxiv.org/abs/1904.05916)**
  — 합성 rare-class 추가가 실 rare-class 오류를 줄임. **다양성(포즈·조명·텍스처)이 개수보다 이득 견인.**
  *(방향성만 — PDF 파싱 실패, 초록 기준)*
- **[Generating Diverse Agricultural Data — Klein et al., CVPR-W 2024 (arXiv:2403.18351)](https://arxiv.org/html/2403.18351)**
  — 현실적 밀도로 맞춰도 잡초 클래스는 ~0.35 mIoU 에 갇힘(작물 >0.81). **밀도 맞추기만으론 잡초 안 오름.**
  per-leaf 텍스처는 도움되나 극단 랜덤화는 해로움. *(high)*
- **[Deep CNNs for Convolvulus sepium in sugar beet — PMC7059384](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7059384/)**
  — 합성+실 병합이 잡초 mAP 0.751→0.829(+0.078). 소량 실셋과 섞으면 합성이 값함. *(high)*

---

## 제거 방식: 점 타격(튜브 스탬프)의 근거 — DECISIONS 002·007·009·027

- **[Tube Stamp for mechanical intra-row individual Plant Weed Control — Langsenkamp et al., CIGR 2014 (오스나브뤼크 응용과학대)](https://www.hs-osnabrueck.de/fileadmin/HSOS/Homepages/COALA/Veroeffentlichungen/2014-CIGR_2014_Tube_Stamp_for_mechanical_intra-row_individual_Plant_Weed_Control.pdf)**
  — BoniRob 스탬프의 **원 기구 논문**. 스탬프 지름 **11mm**, 흙 속 **약 47mm 관입**, 70W BLDC.
  효과 범위 **"BBCH 12 까지는 남는 식생이 없거나 극히 적다"**(= 본잎 2장까지). 개발 동기: *"당근처럼
  빽빽한 줄작물은 작물 손상 위험 때문에 개체별 제초 방법이 아예 없었다."* 결정적 장점으로 **"흙에
  절단면을 내지 않고 넓은 흙 교란도 없다 — 그런 교란은 새 잡초 발아를 자극한다"** 를 명시.
  → 점 타격을 고르는 **진짜 근거**(시뮬 편의가 아니라 흙 교란 회피 + 11mm 선택성). *(high)*
  · [Semantic Scholar](https://www.semanticscholar.org/paper/Tube-Stamp-for-mechanical-intra-row-individual-Weed-Langsenkamp-Sellmann/7a4ffc5f151d4697c52abe78f51e003212ca9bbd)
- **[Farm Robot Learns What Weeds Look Like, Smashes Them — Popular Science](https://www.popsci.com/meet-bonirob-plant-breeding-weed-smashing-robot/)**
  — 우리가 쓰던 **"당근밭 ~90%"** 수치의 출처. Deepfield 홍보담당 Birgit Schulz 발언으로 *"당근 재배
  시험에서 90% 이상"*, 2cm 간격 당근에 잡초 ~20포기/m, 초당 약 2포기, 큰 잡초는 여러 번 내리침.
  **주의: 사내 홍보 발언을 언론이 옮긴 것이지 동료심사 시험이 아니다.** *(medium-low — 아래 주의 참고)*
  · [Digital Trends](https://www.digitaltrends.com/cool-tech/weed-picking-robot/)
- **[Carbon Robotics LaserWeeder](https://carbonrobotics.com/laserweeder)**
  — 레이저 제초 **상용**. 시간당 20만 포기. 다만 **약 $500,000** 이고 크고 평평한 밭이라야 수지가 맞는다.
  → 대농 규모 기술. 우리 BOM(~$2,900)·평균 65W·640Wh 배터리·텃밭 눈안전과 안 맞음. *(medium)*
- **[RootWave — 고주파 전기 제초](https://rootwave.com/)**
  — 전기 제초 **상용**(과수원·포도밭, 트랙터 구동). 고주파로 잡초를 안에서부터 끓인다.
  → 방식은 유효하나 젖은 텃밭 고전압 안전·비용에서 취미 규모와 안 맞음. *(medium)*

## 신뢰도 주의 (인용 시 조심)

- **CropAndWeed (WACV 2023)** — 명시적 흙/작물/잡초 픽셀비율 **미공개**. 픽셀밸런스 수치 인용 금지;
  "soil-dominated, 8,034장/74종" 정성만 사용. <https://github.com/cropandweed/cropandweed-dataset>
- **Beery WACV 2020** — 본문 PDF 파싱 실패. 수치 말고 **방향성**만 인용.
- **Russian dandelion 밀도 8.9–22.2/m²** ([S1161030117301843](https://www.sciencedirect.com/science/article/pii/S1161030117301843))
  — **재배** 밀도(야생 잡초 아님), 저신뢰. 대략 order-of-magnitude 로만.
- **KOCW 잡초방제 강의자료** (종자은행 6만–10만 seeds/m², 출현 1천–5천/m²) — 강의자료, **배경**용
  (1차 정량 앵커 아님). <http://kocw-n.xcache.kinxcdn.com/data/document/2021/wonkwang/limjongok0113/03.pdf>
- **BoniRob "당근밭 90%"** — 동료심사 시험이 아니라 **Deepfield 홍보담당 발언**을 언론이 옮긴 것.
  기구·효과범위는 Langsenkamp 2014(동료심사)를 1차 근거로 쓰고, 90% 는 **보조 인용**으로만.
  게다가 그 팀은 이후 스탬핑을 버리고 호미날로 갔다(Farming Revolution, DECISIONS 009).
