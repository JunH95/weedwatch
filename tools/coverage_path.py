#!/usr/bin/env python3
"""보스트로페돈 커버리지 경로 계획 (관통 P1, DECISIONS 036). 순수 로직 — 시뮬 없이 산수·테스트.

로봇이 두둑 여러 줄을 지그재그로 다 훑는 경로를 웨이포인트로 낸다. 두둑 하나를 걸터타고 그 길이를
쭉 달린 뒤(카메라 2대가 두둑 폭 전체를 봄 → 그 두둑 커버 완료), 헤드랜드(밭 끝 빈 공간)에서 돌아
옆 두둑으로 옮겨 반대 방향으로 달린다.

── 경로 구조 ──────────────────────────────────────────────────────────────────
  pass   : 한 두둑을 걸터타고 그 길이를 달림 (커버가 실제로 일어나는 구간)
  transit: 두둑 끝 → 헤드랜드로 나감 → 옆으로 pitch(1.2m) 이동 → 다음 두둑에 재정렬
경로 = pass, transit, pass, transit, ... (두둑 수만큼 pass, 그 사이 transit)

⚠️ transit(두둑 끝 회전·재진입)이 P1 의 최대 난제다 — 걸터탄 로봇이 25cm 두둑을 벗어나 돌고 다시
걸터타는 건 물리적으로 까다롭다. 이 파일은 **경로(어디로 갈지)만** 낸다. 그 경로를 로봇이 물리적으로
해낼 수 있나(재진입 성공)는 P3(실행)에서 검증하고, 안 되면 스켈레톤은 재배치 치트로 우회한다(036).

실행: import 해서 씀. 단독 실행하면 경로를 텍스트로 출력.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from garden_geometry import Garden, Portal  # noqa: E402


@dataclass
class Waypoint:
    x: float
    y: float
    kind: str          # "pass_start" | "pass_end" | "transit"
    bed: int           # 이 웨이포인트가 속한(또는 향하는) 두둑 인덱스


def boustrophedon(g: Garden, p: Portal, x0: float, x1: float, headland: float = 0.8):
    """두둑 n_beds 줄을 지그재그로 훑는 웨이포인트 목록.

    x0~x1 = 두둑의 심긴 구간(주행 길이). headland = 밭 양 끝의 회전용 빈 공간.
    두둑을 y 오름차순으로 훑되 방향을 번갈아: 0번 +x, 1번 −x, ...
    반환: [Waypoint]. 연속 웨이포인트 사이를 로봇이 직선 주행한다고 본다.
    """
    centers = g.bed_centers                       # 로봇 중심(=두둑 중심) y 목록
    wps: list[Waypoint] = []
    for i, cy in enumerate(centers):
        forward = (i % 2 == 0)                    # 짝수 두둑은 +x, 홀수는 −x
        a, b = (x0, x1) if forward else (x1, x0)
        wps.append(Waypoint(a, cy, "pass_start", i))
        wps.append(Waypoint(b, cy, "pass_end", i))
        if i + 1 < len(centers):                  # 다음 두둑으로 transit
            ny = centers[i + 1]
            # 두둑 끝(b)에서 헤드랜드로 나가고 → 옆으로 이동 → 다음 두둑 시작점에 정렬
            out_x = b + headland if forward else b - headland
            wps.append(Waypoint(out_x, cy, "transit", i + 1))   # 헤드랜드로
            wps.append(Waypoint(out_x, ny, "transit", i + 1))   # 옆 두둑 열로
    return wps


def path_length(wps: list[Waypoint]) -> float:
    d = 0.0
    for a, b in zip(wps, wps[1:]):
        d += ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5
    return d


def covered_beds(wps: list[Waypoint]) -> set[int]:
    """실제로 pass(걸터타고 길이 주행)한 두둑 = 커버된 두둑."""
    return {w.bed for w in wps if w.kind == "pass_end"}


if __name__ == "__main__":
    G, P = Garden(), Portal()
    wps = boustrophedon(G, P, x0=0.2, x1=3.0)
    print(f"두둑 {G.n_beds}줄 · 웨이포인트 {len(wps)}개 · 총 경로 {path_length(wps):.1f}m")
    for w in wps:
        print(f"  ({w.x:5.2f}, {w.y:+5.2f}) {w.kind:11s} bed{w.bed}")
