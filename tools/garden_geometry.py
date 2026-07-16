#!/usr/bin/env python3
"""주말농장 지형과 포탈 로봇의 기하학. 순수 계산만 — 시뮬도 GPU도 필요 없다.

여기가 두둑 폭·고랑 폭·로봇 트랙 같은 숫자들이 **한 곳에서만** 정의되는 자리다.
월드 생성기(make_garden_world.py)도, 테스트도, 나중의 URDF도 전부 여기서 읽어간다.
숫자가 두 곳에 적히는 순간 반드시 어긋난다.

── 왜 별도 모듈인가 ─────────────────────────────────────────────────────
"로봇이 두둑을 탈 수 있나"는 물리 시뮬레이션이 필요 없는 질문이다. 산수다.
산수로 답할 수 있는 걸 시뮬로 확인하면, 느리고, 불안정하고, GPU가 필요하다.
그래서 기하학은 여기서 밀리초 만에 검사하고, 시뮬은 산수로 못 푸는 것만 맡는다.
(계획서의 Tier 1 — 순수 단위 테스트)
"""

from __future__ import annotations

from dataclasses import dataclass

# ── 농촌진흥청 농사로 기준 ────────────────────────────────────────────────
#   두둑 높이 20~30cm · 고랑 폭 30cm 내외 · 평이랑 90~120cm
#   상추 줄간격×포기간격 20×20cm
# ⚠️ 상추 초장(20~25cm)은 농진청 1차 자료로 확인되지 않은 작업 추정치다.
#    포탈 높이가 여기서 나오므로 인용 전 재확인 필요. (docs/PLAN.md §10)


@dataclass(frozen=True)
class Garden:
    """밭의 기하학."""

    bed_width: float = 0.90  # 두둑 폭
    bed_height: float = 0.25  # 두둑 높이 (고랑 바닥 기준)
    furrow_width: float = 0.30  # 고랑 폭
    bed_length: float = 4.00
    n_beds: int = 2
    crop_height: float = 0.25  # 상추. 미검증 추정치.

    @property
    def pitch(self) -> float:
        """두둑 하나 + 고랑 하나. 다음 두둑 중심까지의 거리."""
        return self.bed_width + self.furrow_width

    @property
    def bed_centers(self) -> list[float]:
        """각 두둑 중심의 y좌표. y=0 기준 좌우 대칭."""
        return [(i - (self.n_beds - 1) / 2) * self.pitch for i in range(self.n_beds)]

    def bed_span(self, i: int) -> tuple[float, float]:
        """i번 두둑이 차지하는 y구간 (왼쪽, 오른쪽)."""
        c = self.bed_centers[i]
        return c - self.bed_width / 2, c + self.bed_width / 2

    def is_over_bed(self, y: float, half_width: float = 0.0) -> bool:
        """y 위치의 물체가 (폭을 감안해) 어떤 두둑과 겹치는가."""
        return any(
            lo - half_width < y < hi + half_width
            for lo, hi in (self.bed_span(i) for i in range(self.n_beds))
        )


@dataclass(frozen=True)
class Portal:
    """포탈형 로봇의 기하학. docs/DECISIONS.md 006 참고."""

    wheel_dia: float = 0.22
    wheel_width: float = 0.08
    clearance: float = 0.60  # 고랑 바닥 ~ 빔 아랫면
    beam_height: float = 0.08

    def track(self, g: Garden) -> float:
        """좌우 바퀴 중심 거리.

        바퀴는 각 고랑의 한가운데 있어야 한다. 두둑 가장자리가 ±(bed_width/2)이고
        고랑이 거기서 furrow_width 만큼 뻗으므로 고랑 중심은
        bed_width/2 + furrow_width/2, 트랙은 그 두 배 = bed_width + furrow_width.
        """
        return g.bed_width + g.furrow_width

    def overall_width(self, g: Garden) -> float:
        return self.track(g) + self.wheel_width

    def wheel_ys(self, g: Garden, robot_y: float) -> tuple[float, float]:
        """robot_y 에 선 로봇의 좌우 바퀴 중심 y좌표."""
        half = self.track(g) / 2
        return robot_y - half, robot_y + half

    def wheel_slack(self, g: Garden) -> float:
        """바퀴가 고랑 안에서 좌우로 갖는 여유. 한쪽 기준."""
        return (g.furrow_width - self.wheel_width) / 2

    def headroom(self, g: Garden) -> float:
        """빔 아랫면과 작물 꼭대기 사이 여유. 음수면 작물을 친다."""
        return self.clearance - (g.bed_height + g.crop_height)

    def required_clearance(self, g: Garden, margin: float = 0.10) -> float:
        """두둑 + 작물 + 여유."""
        return g.bed_height + g.crop_height + margin


def parking_y(g: Garden, bed_index: int = 0) -> float:
    """로봇이 서야 하는 y좌표 = 두둑 중심.

    y=0 이 아니다. 두둑이 짝수 개면 y=0 은 고랑 한가운데라서,
    거기 세우면 바퀴가 두둑을 밟고 몸통이 고랑 위에 뜬다 — 설계와 정확히 반대다.
    """
    return g.bed_centers[bed_index]
