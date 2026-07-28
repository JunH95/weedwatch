#!/usr/bin/env python3
"""밭 명세 — 크기와 **현실성**을 파라미터로 (DECISIONS 042).

지금까지 밭은 자로 그은 듯 반듯했다: 고랑은 무한 평면, 두둑은 단면을 그대로 밀어낸 완전 균일.
040(U턴 재진입 0.1~2.8cm)·041(시각 오도메트리) 숫자가 전부 그 위에서 나왔고, 그게 현실에서
성립하는지는 모른다. 실제 배토기로 만든 두둑은 폭도 높이도 중심선도 흔들린다.

이 파일은 **순수 config**다 — 시뮬도 렌더도 모른다. 월드 생성기가 이걸 읽어 메시·충돌을 만들고,
테스트가 이걸 읽어 "정말 울퉁불퉁해졌나"를 산수로 단언한다.

── 밭 사다리 (042) ─────────────────────────────────────────────────────────
  SMOOTH  두둑 2 × 4m, 현실성 0      빠른 회귀 — 코드가 안 깨졌나
  DEV     두둑 2 × 2m, 현실성 전부   매 수정. 여기서 다 고친다
  MAIN    두둑 6 × 7m + 창고         단계 끝에만. 최종 판정 · **보호 대상**

**작게 만들되 쉽게 만들지 않는다.** DEV 는 MAIN 의 한 조각이다 — 현실성 파라미터가 **같고**
두둑 수와 길이만 작다. 크기와 난이도를 같이 줄이면 사다리가 아니라 자기기만이 된다.

**MAIN 은 보호 대상이다.** 제어·모델을 고치는 커밋에서 이 값을 같이 바꾸지 않는다. 정본이
실패하면 밭이 아니라 로봇을 고친다(`configs/eval_seeds.txt` 와 같은 규율).
"""
from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from garden_geometry import Garden  # noqa: E402

_G = Garden()


@dataclass(frozen=True)
class FieldSpec:
    """밭 하나의 명세. 같은 seed + 같은 값 = 같은 밭(재현 가능)."""

    name: str
    n_beds: int = 2
    bed_length: float = 4.0          # 두둑 길이 [m] (x 방향)
    x0: float = -0.30                # 두둑 시작 x
    first_bed_y: float = 0.60
    pitch: float = 1.20              # 두둑 중심 간격 = 두둑폭 0.9 + 고랑폭 0.3
    headland: float = 1.60           # 두둑 끝 바깥 여유 [m] — U턴이 필요로 하는 공간

    # ── 현실성 (0 이면 지금까지의 반듯한 밭) ────────────────────────────
    width_var: float = 0.0           # 두둑 폭 변이 진폭 [m] (±)
    height_var: float = 0.0          # 두둑 높이 변이 진폭 [m] (±)
    meander: float = 0.0             # 중심선 사행 진폭 [m] (±)
    meander_wave: float = 2.5        # 사행 파장 [m] — 이보다 짧으면 톱니처럼 부자연스럽다
    clod_density: float = 0.0        # 고랑 흙덩이 [개/m] (바퀴 경로에)
    clod_height: tuple = (0.03, 0.06)  # 흙덩이 높이 범위 [m] (shake 월드 실측 범위)
    cross_slope_deg: float = 0.0     # 밭 전체 가로경사 [°]

    # ── 구조물·내용물 ───────────────────────────────────────────────────
    home_base: bool = False          # 창고(홈 베이스) 배치 — 도킹·복귀가 쓸 것
    # ⚠️ **아직 월드 생성에 안 붙었다.** 잡초 배치는 CropCraft 정원(oracle_test)이 정하고 있고,
    # 그 정원은 잡초를 두둑 중앙 ±22.5cm 에만 심는다 — 그래서 카메라 2대의 값어치(전폭 커버,
    # DECISIONS 026)가 이 밭들에서는 측정에 안 드러난다. 정원 생성까지 파라미터화할 때 잇는다.
    weed_full_width: bool = False    # (선언만 — 위 주석 참고)
    seed: int = 42

    # ── 파생 ────────────────────────────────────────────────────────────
    @property
    def bed_centers(self) -> list[float]:
        return [self.first_bed_y + i * self.pitch for i in range(self.n_beds)]

    @property
    def x1(self) -> float:
        return self.x0 + self.bed_length

    @property
    def area_m2(self) -> float:
        """밭 면적 [m²] — 커버리지-용량 실험(039)이 쓸 값."""
        return self.n_beds * self.pitch * self.bed_length

    @property
    def realistic(self) -> bool:
        return any((self.width_var, self.height_var, self.meander,
                    self.clod_density, self.cross_slope_deg))

    def rng(self, salt: str = "") -> random.Random:
        """이름·소금별로 갈라지되 재현되는 난수원 — 두둑마다 다른 모양이 seed 로 고정된다."""
        return random.Random(f"{self.seed}:{self.name}:{salt}")

    # ── 두둑 단면의 x 별 변이 (생성기와 테스트가 같은 함수를 쓴다) ──────
    def profile(self, bed: int, x: float) -> tuple[float, float, float]:
        """두둑 bed 의 위치 x 에서 (중심 y, 윗변 폭, 높이).

        변이는 **저주파 정현파 합**이다 — 난수를 픽셀마다 뿌리면 톱니가 되고, 그건 배토기가
        만드는 모양이 아니다. 실제 두둑은 완만하게 굽고 두꺼워졌다 얇아진다.
        """
        cy = self.bed_centers[bed]
        r = self.rng(f"bed{bed}")
        ph = [r.uniform(0, 2 * math.pi) for _ in range(3)]
        if self.meander:
            cy += self.meander * math.sin(2 * math.pi * x / self.meander_wave + ph[0])
        w = _G.bed_width
        if self.width_var:
            w += self.width_var * math.sin(2 * math.pi * x / (self.meander_wave * 1.7) + ph[1])
        h = _G.bed_height
        if self.height_var:
            h += self.height_var * math.sin(2 * math.pi * x / (self.meander_wave * 0.9) + ph[2])
        return cy, w, h

    def clods(self, bed: int) -> list[tuple[float, float, float]]:
        """고랑 흙덩이 [(x, y, 높이)] — 바퀴가 지나는 두 고랑에 시드 고정으로 뿌린다."""
        if not self.clod_density:
            return []
        r = self.rng(f"clod{bed}")
        cy = self.bed_centers[bed]
        half_track = _G.bed_width / 2 + _G.furrow_width / 2      # 바퀴 중심까지
        out = []
        n = max(1, int(self.clod_density * self.bed_length))
        for side in (-1, +1):
            for _ in range(n):
                x = r.uniform(self.x0, self.x1)
                y = cy + side * half_track + r.uniform(-0.06, 0.06)
                out.append((x, y, r.uniform(*self.clod_height)))
        return out


# ── 밭 사다리 (042) ────────────────────────────────────────────────────────
# 현실성 값의 근거: 두둑 폭 ±5cm 는 걸터타기 여유 11cm/쪽(034)의 절반을 먹는 크기 —
# "여유가 정말 충분한가"를 시험하는 값이다. 높이 ±3cm 와 흙덩이 3~6cm 는 shake 월드 실측 범위.
# 사행 ±4cm 는 두둑 폭의 4% 로, 사람이 만든 두둑에서 흔한 정도.
_REAL = dict(width_var=0.05, height_var=0.03, meander=0.04,
             clod_density=1.5, cross_slope_deg=3.0, weed_full_width=True)

SMOOTH = FieldSpec(name="smooth", n_beds=2, bed_length=4.0)
DEV = FieldSpec(name="dev", n_beds=2, bed_length=2.0, **_REAL)
MAIN = FieldSpec(name="main", n_beds=6, bed_length=7.0, home_base=True, **_REAL)

PRESETS = {f.name: f for f in (SMOOTH, DEV, MAIN)}


def get(name: str) -> FieldSpec:
    if name not in PRESETS:
        raise SystemExit(f"모르는 밭: {name} (있는 것: {', '.join(PRESETS)})")
    return PRESETS[name]


def scaled(spec: FieldSpec, n_beds: int, bed_length: float, name: str | None = None) -> FieldSpec:
    """현실성은 그대로 두고 **크기만** 바꾼다 — 사다리의 핵심 규율을 코드로 강제."""
    return replace(spec, n_beds=n_beds, bed_length=bed_length,
                   name=name or f"{spec.name}{n_beds}x{bed_length:g}")


if __name__ == "__main__":
    for f in PRESETS.values():
        print(f"{f.name:7} 두둑 {f.n_beds}줄 × {f.bed_length:.1f}m = {f.area_m2:5.1f}m² · "
              f"현실성 {'ON ' if f.realistic else 'off'} "
              f"(폭±{f.width_var*100:.0f} 높이±{f.height_var*100:.0f} 사행±{f.meander*100:.0f}cm "
              f"흙덩이 {f.clod_density}/m 경사 {f.cross_slope_deg}°) "
              f"{'· 창고' if f.home_base else ''}")
