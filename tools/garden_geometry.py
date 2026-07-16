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
    """포탈형 로봇의 기하학. docs/DECISIONS.md 006, 010, 012 참고.

    이 dataclass 가 로봇 치수의 단일 출처다. URDF 도, Blender 몸체 생성기
    (tools/robot_body.py)도, 테스트도 전부 여기서 읽는다. 숫자가 두 곳에 적히면
    반드시 어긋난다. (docs/DECISIONS.md 012 — Blender bpy 를 이 숫자로 파라메트릭 생성)

    실루엣은 AVO/Aigen 을 인용한다 (docs/DECISIONS.md 010):
    "바퀴 달린 테이블 + 그 아래 매달린 도구." 태양광 데크가 상판이고, 다리·바퀴·
    캐리지·도구가 전부 그 아래 매달린다. 상판 위로 튀어나오는 건 아무것도 없다.
    """

    # ── 구동/기하 (걸터타기 검증에 쓰이는 것들) ──────────────────────────
    wheel_dia: float = 0.22
    wheel_width: float = 0.08
    clearance: float = 0.60  # 고랑 바닥 ~ 빔(=데크) 아랫면
    beam_height: float = 0.08

    # ── 태양광 데크 = 상판 (AVO 실루엣의 핵심) ───────────────────────────
    # 트랙(120cm)보다 좌우로 10cm씩 내밀어 128 → 140cm. 그 오버행이 "카트"가 아니라
    # "기계"로 보이게 한다 (AVO 는 바퀴 밖으로 22~37cm 내민다). 패널 면적도 +9%.
    deck_length: float = 0.75  # 주행 방향 (앞뒤)
    deck_overhang: float = 0.10  # 트랙 대비 한쪽 오버행 → 전폭 = track + 2*overhang
    deck_thickness: float = 0.03  # ETFE 라미네이트 (유리 아님 — 무게 때문, 010)

    # ── 몸통: 양쪽 사이드 포드 + 얇은 상판 데크 (AVO 실루엣의 진짜 비밀) ──
    # "책상"을 벗어나는 핵심: 몸통이 위에 작게 있고 다리가 길게 뻗으면 책상이 된다.
    # 대신 몸통을 바퀴 바로 위까지 크게 내린 "사이드 포드" 두 덩어리로 만들고,
    # 그 사이(두둑 위)는 비워서 터널을 만든다. 다리가 거의 안 보이고 바퀴가 포드에
    # 직접 붙은 "덩어리 기계"가 된다. 가운데를 비우니 작물도 안 침범한다.
    pod_drop: float = 0.42      # 데크 아랫면에서 포드가 내려오는 높이 (바퀴 가까이까지)
    pod_width: float = 0.22     # 각 포드의 폭 (y 방향, 고랑+바퀴를 덮음)
    body_inset: float = 0.05    # 데크 가장자리에서 상판 안쪽까지

    # ── 다리 (이제 짧다 — 포드 아랫면 → 바퀴 축만) ──────────────────────
    leg_width: float = 0.06

    # ── Y 캐리지 (빔을 따라 좌우로 = 두둑 폭을 훑음) ─────────────────────
    carriage_travel: float = 0.45  # 중심에서 한쪽으로 ±0.45 (두둑 90cm 커버)
    carriage_size: float = 0.10  # 캐리지 본체 한 변

    # ── Z 축 (막대가 아래로 = 점 타격 + 카메라 하강) ────────────────────
    # docs/DECISIONS.md 009: 점 타격, 로봇팔 아님. 직선 레일 하나.
    z_travel: float = 0.35  # 데크 아래에서 두둑 위까지 내려가는 행정
    tool_rod_dia: float = 0.012  # 1.2cm 막대 (BoniRob 수치 인용)

    # ── 카메라 (캐리지에 강체 고정, 아래를 봄) ──────────────────────────
    # 캐리지에 붙어서 툴 팁이 항상 같은 픽셀에 온다 → 헤드리스 픽셀 단언이 됨.
    camera_height_above_bed: float = 0.35  # 두둑 윗면에서 카메라까지 (px/cm 계산의 그 35cm)

    # ── 배터리 베이 (앞쪽, 무게중심 낮게 — Aigen 인용) ──────────────────
    # 128cm 폭에 고랑 30cm 위 60cm 높이라 넘어지기 쉽다. 무게를 낮고 앞에 둔다.
    battery_size: float = 0.12

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

    # ── 몸체 생성기(robot_body.py)와 URDF 가 읽는 파생 치수 ──────────────

    def deck_width(self, g: Garden) -> float:
        """데크(상판) 전폭 = 트랙 + 양쪽 오버행."""
        return self.track(g) + 2 * self.deck_overhang

    def deck_top_z(self) -> float:
        """데크 윗면 높이 (고랑 바닥 기준). 로봇에서 제일 높은 지점."""
        return self.clearance + self.beam_height + self.deck_thickness

    def leg_height(self) -> float:
        """다리 길이 = 데크 아랫면 ~ 바퀴 축."""
        return self.clearance + self.beam_height - self.wheel_dia / 2

    def is_tippy(self, g: Garden) -> bool:
        """넘어지기 쉬운 형상인가 (높이 > 전폭). 배터리를 낮게 둬야 하는 근거."""
        return self.deck_top_z() > self.deck_width(g)


def parking_y(g: Garden, bed_index: int = 0) -> float:
    """로봇이 서야 하는 y좌표 = 두둑 중심.

    y=0 이 아니다. 두둑이 짝수 개면 y=0 은 고랑 한가운데라서,
    거기 세우면 바퀴가 두둑을 밟고 몸통이 고랑 위에 뜬다 — 설계와 정확히 반대다.
    """
    return g.bed_centers[bed_index]
