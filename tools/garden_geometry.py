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

import math
from dataclasses import dataclass

# ── 농촌진흥청 농사로 기준 ────────────────────────────────────────────────
#   두둑 높이 20~30cm · 고랑 폭 30cm 내외 · 평이랑 90~120cm
#   상추 줄간격×포기간격 20×20cm
# 주의: 상추 초장(20~25cm)은 농진청 1차 자료로 확인되지 않은 작업 추정치다.
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
    # 바퀴-흙 마찰계수 (단일 출처, DECISIONS 031·032). **작동 가능한 최악**에 맞춘 보수적 값이다 —
    # 젖은/마른 흙을 다 수치화하는 대신, 가장 미끄러운(젖은) 조건에서 되면 나머지는 다 된다(사용자).
    # 근거: 실측 바퀴-흙 외부 마찰 ~0.4(en17040966). 이보다 나쁜 진창은 어떤 바퀴 로봇도 못 가는
    # **작동 한계**라 스펙 밖(비 직후 미출동). 지금까지는 <surface> 미설정 → 엔진 기본(~1, 마른 흙급).
    wheel_mu: float = 0.4    # 구름 방향(전후)
    wheel_mu2: float = 0.4   # 옆 방향(좌우). 크로스슬로프 아래로 미끄러짐을 지배 — 보수적으로 같게
    clearance: float = 0.60  # 고랑 바닥 ~ 빔(=데크) 아랫면
    beam_height: float = 0.08

    # ── 태양광 데크 = 상판 (AVO 실루엣의 핵심) ───────────────────────────
    # 트랙(120cm)보다 좌우로 10cm씩 내밀어 128 → 140cm. 그 오버행이 "카트"가 아니라
    # "기계"로 보이게 한다 (AVO 는 바퀴 밖으로 22~37cm 내민다). 패널 면적도 +9%.
    deck_length: float = 1.30  # 주행 방향 (앞뒤). 폭(track 1.20)보다 길게 = 책상 아닌 로버/차
                               # 비율. 휠베이스도 0.45→1.0 로 늘어 피치 안정↑ (회전은 scrub↑ — 검증)
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
    # 멀티툴(DECISIONS 020): 두둑 90cm 를 n_tools 밴드로 나눠 각 툴이 자기 밴드만 짧게 훑는다.
    # 단일 캐리지가 90cm 를 건너려면 1.9s (48cm/s) 라 카메라 리드(0.2m/s 에서 1.55s)를 넘겨
    # x-근접·y-원거리 최악배치에서 잡초를 놓친다. 밴드를 나누면 각 툴 이동이 짧아 여유가 생긴다.
    # carriage_travel 은 이제 "어느 툴이든 닿는 최대 Y 반경"(사이드 페어링 clearance 근거) — 밴드
    # 중심 ±0.30 + 밴드 반폭 ±0.15 = ±0.45 로 값은 유지. 실제 관절 행정은 tool_band_half.
    carriage_travel: float = 0.45  # 최대 Y 반경 (페어링 안쪽면 clear 근거). 관절 행정 아님.
    carriage_size: float = 0.10  # 캐리지 본체 한 변
    n_tools: int = 3  # 점 타격 툴 개수 (독립 Y + 독립 Z). DECISIONS 020.
    tool_x0: float = -0.09  # 맨 앞 툴의 X (카메라 뒤). 기존 단일툴 값.
    tool_stagger_x: float = 0.18  # 툴 간 X 간격. 캐리지 X-길이(carriage_size·1.4=0.14)보다
                                  # 넓어야 Y 범위가 겹치는 순간에도 캐리지끼리 안 부딪힌다.
    camera_x: float = 0.22  # 하방 카메라 X (base 고정 전방 팔). DECISIONS 006.

    # ── 카메라 대수 (DECISIONS 026) ─────────────────────────────────────
    # D405 한 대(87°)로는 두둑 폭 90cm 를 못 덮는다: 두둑 위 0.33m 에서 가로 발자국이 0.585m 라
    # 35%(양쪽 각 15.8cm)가 사각이었다 — 툴은 ±0.45 까지 닿는데 눈이 안 닿아 완벽한 검출기라도
    # 재현율 상한이 0.65 였다. 한 대로 덮으려면 두둑 위 0.474m 가 필요한데 빔이 0.60m 라 불가능.
    # 두 대를 툴 밴드와 같은 균등분할 위치(±bed_width/4)에 두면 겹침 0.135m 로 전폭을 덮는다.
    n_cameras: int = 2
    camera_hfov: float = 1.5184   # D405 87°. make_urdf 가 이 단일 출처를 읽는다.
    camera_w: int = 1280
    camera_h: int = 720
    camera_mpp: float = 0.000457  # 캘리브 실측 m/px (perception/detect_server). 기하값보다 보수적.
    # 프레임률은 **정밀도 게이트가 아니라 시뮬 비용 노브**다. 2대로 늘리자 Tier-3 sim 이 0.200x →
    # 0.031x 로 6.5배 느려져(8GB GPU 에서 1280×720 렌더타깃 2개 + best.pt 추론 경합) 주행 하네스가
    # 데드라인 안에 완주를 못 했다. 필요량은 산수로 나온다: 0.2m/s 에서 주행방향 발자국 0.329m 를
    # 겹치며 훑으려면 ~1.3Hz 면 충분하다. 5Hz 는 4cm 마다 한 장 = 8배 여유.
    camera_rate: int = 5

    # ── Z 축 (막대가 아래로 = 점 타격 + 카메라 하강) ────────────────────
    # docs/DECISIONS.md 009: 점 타격, 로봇팔 아님. 직선 레일 하나.
    z_travel: float = 0.35  # 데크 아래에서 두둑 위까지 내려가는 행정
    tool_rod_dia: float = 0.012  # 1.2cm 막대 (BoniRob 수치 인용)
    tool_rod_fraction: float = 0.55  # 막대 물리 길이 = z_travel 의 이 비율

    @property
    def tool_rod_len(self) -> float:
        """점 타격 막대의 물리 길이. 시각 메시(robot_body)·충돌·관성(make_urdf)이
        모두 이 값을 읽어야 한다. z_travel(행정, 0.35)을 막대 길이로 쓰면 충돌·관성이
        시각 막대보다 길어져 접힘 자세에서 두둑을 파고든다 (적대적 검증에서 잡힘)."""
        return self.z_travel * self.tool_rod_fraction

    # ── 멀티툴 밴드 기하 (DECISIONS 020) ────────────────────────────────

    def tool_band_centers(self, g: Garden) -> list[float]:
        """각 툴이 담당하는 Y 밴드의 중심 (로봇 중심=두둑 중심 기준). 두둑 폭을 균등 분할.
        n_tools=3, bed_width=0.90 → [-0.30, 0.0, +0.30]."""
        step = g.bed_width / self.n_tools
        return [-g.bed_width / 2 + step / 2 + i * step for i in range(self.n_tools)]

    def tool_band_half(self, g: Garden) -> float:
        """한 툴의 Y 관절 행정(밴드 반폭). = bed_width/(2·n_tools). n=3 → 0.15."""
        return g.bed_width / (2 * self.n_tools)

    def tool_xs(self) -> list[float]:
        """각 툴의 X. 엇갈려(stagger) 독립 Y 범위가 겹쳐도 캐리지끼리 안 부딪히게.
        n=3 → [-0.09, -0.19, -0.29]. 맨 앞(카메라에 가장 가까운) 툴이 리드 최소."""
        return [self.tool_x0 - i * self.tool_stagger_x for i in range(self.n_tools)]

    def tool_lead(self, i: int) -> float:
        """툴 i 와 카메라의 X 차이 = 주행으로 메워야 하는 정렬 거리(리드). 카메라가 먼저 봄.
        n=3 → [0.31, 0.41, 0.51]. 뒤쪽 툴일수록 리드가 커 시간 여유가 오히려 많다."""
        return self.camera_x - self.tool_xs()[i]

    def band_of(self, g: Garden, y: float) -> int:
        """로봇 중심 기준 y 위치의 잡초를 담당하는 툴 인덱스. 밴드 경계로 가른다."""
        step = g.bed_width / self.n_tools
        idx = int((y + g.bed_width / 2) // step)
        return max(0, min(self.n_tools - 1, idx))

    # ── 카메라 (base 고정 전방 팔) 파생 위치 ────────────────────────────

    def camera_z(self) -> float:
        """하방 카메라의 월드 z (고랑 바닥 기준). 빔 바로 아래에 매단다.
        robot_body 의 cam_z 와 make_urdf 센서 pose 가 이 단일 출처를 읽어야 어긋나지 않는다."""
        beam_bottom = self.clearance  # 빔 아랫면 = clearance (터널 천장)
        return beam_bottom - 0.02

    def camera_height_above_bed(self, g: Garden) -> float:
        """두둑 윗면에서 카메라까지 높이 (파생). n=3 기준 ≈0.33m — 스펙 0.35 근접.
        학습 카메라(train_garden.yaml)를 이 값에 정합해야 온-루프 인식이 정직하다(Phase 3)."""
        return self.camera_z() - g.bed_height

    # ── 카메라 커버리지 (DECISIONS 026) ─────────────────────────────────

    def camera_ys(self, g: Garden) -> list[float]:
        """카메라 N대의 Y 중심 (로봇 중심 기준). 툴 밴드와 같은 균등분할 공식.
        n=2, bed_width=0.90 → [-0.225, +0.225]."""
        step = g.bed_width / self.n_cameras
        return [-g.bed_width / 2 + step / 2 + i * step for i in range(self.n_cameras)]

    def camera_footprint_w(self, g: Garden) -> float:
        """한 대가 지면에서 덮는 가로(두둑 폭 방향) 길이 [m]. 캘리브 MPP 기준 — 기하값(화각)보다
        작아서 보수적이다. 인식이 실제로 쓰는 스케일이므로 커버리지 판단은 이쪽이 정직하다."""
        return self.camera_w * self.camera_mpp

    def camera_footprint_h(self, g: Garden) -> float:
        """한 대가 덮는 세로(주행 방향) 길이 [m]. 주행이 x 를 메우므로 커버리지 제약은 아니다."""
        return self.camera_h * self.camera_mpp

    def camera_footprint_w_geometric(self, g: Garden) -> float:
        """화각·높이로 계산한 가로 발자국. 캘리브 MPP 와 교차검증용(둘이 크게 벌어지면 재캘리브)."""
        return 2 * self.camera_height_above_bed(g) * math.tan(self.camera_hfov / 2)

    def camera_coverage_half(self, g: Garden) -> float:
        """카메라들이 합쳐 덮는 반폭. 툴이 닿는 bed_width/2 이상이어야 '보는 만큼만 친다'가 성립."""
        return max(self.camera_ys(g)) + self.camera_footprint_w(g) / 2

    def camera_overlap(self, g: Garden) -> float:
        """인접 카메라 발자국의 겹침 [m]. 0 이하면 두둑 가운데 사각(blind strip)이 생긴다."""
        ys = self.camera_ys(g)
        if len(ys) < 2:
            return 0.0
        return self.camera_footprint_w(g) - (ys[1] - ys[0])

    # ── 카메라 (멀티툴 이후 base 고정 전방 팔, 두둑 전체를 봄) ────────────
    # 단일툴 때는 캐리지에 붙여 "툴이 항상 같은 픽셀"(DECISIONS 006)을 썼지만, 캐리지가
    # n_tools 개가 되면 어디 붙일지 모호해진다. 카메라를 base 전방 팔로 옮겨 두둑 폭 전체를
    # 내려다본다(고정 카메라 + 다중 툴 = Andela/ecoRobotix 실제 아키텍처). 툴 팁은 이제
    # FK(base GT + carriage_i + tool_i)로 구하므로 픽셀 고정 불변식이 없어도 단언은 성립한다.
    # 높이는 파생값 — camera_z()(빔 바로 아래) 에서 두둑 윗면(bed_height)을 뺀다. 단일 출처.

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
