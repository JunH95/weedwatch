#!/usr/bin/env bash
# AI Hub 527 "정밀농업 농기계 잡초 인식" 에서 **쇠비름(Portulaca oleracea) 검증 세트**만
# 내려받는다 (~3GB). 전체(470GB)가 아니다.
#
# ── 왜 쇠비름만 ─────────────────────────────────────────────────────────────
# 우리 CropCraft 합성 잡초 `portulaca` 와 **같은 종**이다. 그래서 "합성으로 배운 쇠비름을
# 실제 한국 쇠비름 사진에서 알아보는가"(sim-to-real 단일 클래스 전이)를 잴 수 있다.
# 527 은 개체 표본(한 프레임에 잡초 하나, 폰 촬영, 작물 없음)이라 crop-vs-weed 판별은
# 이 데이터로 테스트 못 한다 — 주장 스코프를 거기까지만.
#
# ── 파일 (aihubshell -mode l -datasetkey 527 로 확인, 2026-07-20) ────────────
#   VS9_쇠비름.zip  이미지 3GB  filekey 51432   (Validation 원천)
#   VL9_쇠비름.zip  라벨  3MB   filekey 51418   (Validation 라벨)
#
# ── 사전 준비 (사람 몫 — 내국인만) ──────────────────────────────────────────
#   1. aihub.or.kr 회원가입 → API 키 발급 (-aihubapikey 용)
#   2. 527 데이터셋 "승인신청" → 승인완료 대기
#   그 다음: AIHUB_KEY=<발급키> scripts/fetch_aihub.sh   (또는 make aihub)
#
# ⚠️ 재배포 금지. data/aihub/ 는 gitignore — 이미지는 절대 커밋/공개 안 한다. 지표·모델만.

set -euo pipefail

KEY="${AIHUB_KEY:-${1:-}}"
if [ -z "$KEY" ]; then
  echo "AI Hub API 키가 필요합니다." >&2
  echo "  사용: AIHUB_KEY=<키> $0     (키는 aihub.or.kr 회원가입 후 발급 + 527 승인 필요)" >&2
  exit 2
fi

WW="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$WW/data/aihub"
mkdir -p "$DEST"

echo "AI Hub 527 쇠비름 검증세트(~3GB) → $DEST"
cd "$DEST"
aihubshell -aihubapikey "$KEY" -mode d -datasetkey 527 -filekey 51432,51418

echo "다운로드 완료. 압축을 풀어 원천/라벨을 확인하세요 (data/aihub/ 안, 커밋 금지)."
