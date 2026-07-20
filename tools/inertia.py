#!/usr/bin/env python3
"""링크 관성 텐서 계산. 순수 산수 — 시뮬도 GPU도 필요 없다 (Tier 1).

── 왜 이게 필요한가 ────────────────────────────────────────────────────
diff-drive 로 로봇을 돌리려면 각 링크의 질량뿐 아니라 **관성 텐서**가 물리적으로
말이 돼야 한다. 처음 URDF 는 전부 ixx=iyy=izz=0.01 이라는 자리채움 값이었는데,
20kg·폭 1.4m 몸통의 실제 yaw 관성(izz)은 ~7.5 라 **750배 작았다.** 그러면 아주 작은
토크에도 몸통이 팽이처럼 돌아 diff-drive 가 불안정해진다. 관성은 산수로 정확히 낼 수
있으니 (garden_geometry 와 같은 철학) 자리채움 대신 여기서 계산한다.

── URDF <inertial> 규약 ────────────────────────────────────────────────
  <origin xyz=COM>  = 링크 원점 기준 질량중심(COM) 위치
  <inertia ...>     = **COM 프레임에서** 본 관성 텐서
관성 행렬은
    | ixx  ixy  ixz |     ixx = ∫(y²+z²)dm,  ixy = -∫xy dm, ...
    | ixy  iyy  iyz |
    | ixz  iyz  izz |
우리 형상은 x=0, y=0 평면에 대칭이라 곱관성(ixy/ixz/iyz)은 0 으로 떨어진다.
그래도 대칭을 가정하지 않고 곱관성까지 평행축 정리로 합산한다 — 0 이 나오는 걸
test 가 확인한다 (대칭을 코드가 우연히 깨도 잡히게).
"""

from __future__ import annotations

from dataclasses import dataclass

# 관성 텐서 6성분 순서: (ixx, iyy, izz, ixy, ixz, iyz)
Tensor = tuple[float, float, float, float, float, float]


def box_inertia(mass: float, sx: float, sy: float, sz: float) -> Tensor:
    """축정렬 직육면체(변 sx,sy,sz)의 중심 기준 관성. 곱관성 0."""
    ixx = mass / 12.0 * (sy * sy + sz * sz)
    iyy = mass / 12.0 * (sx * sx + sz * sz)
    izz = mass / 12.0 * (sx * sx + sy * sy)
    return (ixx, iyy, izz, 0.0, 0.0, 0.0)


def cylinder_inertia(mass: float, radius: float, length: float, axis: str) -> Tensor:
    """실린더의 중심 기준 관성. axis 는 대칭축('x'|'y'|'z').

    대칭축 둘레: I = ½ m r²   (바퀴가 굴러갈 때 모터가 가속시키는 관성)
    직교 방향  : I = 1/12 m (3r² + L²)
    """
    i_axis = 0.5 * mass * radius * radius
    i_perp = mass / 12.0 * (3.0 * radius * radius + length * length)
    if axis == "x":
        return (i_axis, i_perp, i_perp, 0.0, 0.0, 0.0)
    if axis == "y":
        return (i_perp, i_axis, i_perp, 0.0, 0.0, 0.0)
    if axis == "z":
        return (i_perp, i_perp, i_axis, 0.0, 0.0, 0.0)
    raise ValueError(f"axis 는 x/y/z 여야 합니다: {axis!r}")


@dataclass(frozen=True)
class Part:
    """합성체의 한 조각. tensor 는 그 조각 자신의 COM 기준."""

    mass: float
    com: tuple[float, float, float]  # 링크 원점 기준 이 조각의 COM
    tensor: Tensor  # 이 조각의 COM 기준 관성


def combine(parts: list[Part]) -> tuple[float, tuple[float, float, float], Tensor]:
    """여러 조각을 합쳐 (총질량, 합성 COM, 합성 COM 기준 관성 텐서) 반환.

    평행축 정리로 각 조각의 관성을 **합성 COM** 으로 옮겨 더한다.
    합성 COM 기준으로 표현해야 URDF <inertial><origin> 규약과 맞는다.
    """
    m_total = sum(p.mass for p in parts)
    if m_total <= 0:
        raise ValueError("총 질량이 0 이하입니다")
    cx = sum(p.mass * p.com[0] for p in parts) / m_total
    cy = sum(p.mass * p.com[1] for p in parts) / m_total
    cz = sum(p.mass * p.com[2] for p in parts) / m_total

    ixx = iyy = izz = ixy = ixz = iyz = 0.0
    for p in parts:
        dx, dy, dz = p.com[0] - cx, p.com[1] - cy, p.com[2] - cz
        pxx, pyy, pzz, pxy, pxz, pyz = p.tensor
        ixx += pxx + p.mass * (dy * dy + dz * dz)
        iyy += pyy + p.mass * (dx * dx + dz * dz)
        izz += pzz + p.mass * (dx * dx + dy * dy)
        # 곱관성 규약: ixy = -∫xy dm → 평행축 항도 -m·dx·dy
        ixy += pxy - p.mass * dx * dy
        ixz += pxz - p.mass * dx * dz
        iyz += pyz - p.mass * dy * dz

    return m_total, (cx, cy, cz), (ixx, iyy, izz, ixy, ixz, iyz)
