#!/usr/bin/env python3
"""리드↔타격오차 곡선 — "관측에서 타격까지 얼마나 죽은 추측으로 가느냐"가 오차를 정하는가.

── 왜 이걸 재나 ────────────────────────────────────────────────────────────
Step B 가 현실 밭에서 23cm 로 실패했고, 성분을 보니 거의 전부 **전후**(이동거리 추정)였다.
바퀴가 흙덩이에서 20% 헛돌기 때문인데, 그 20% 가 곱해지는 길이가 **관측→타격 구간**이다.

지금 코디네이터는 잡초를 **처음 본 순간** 타격 위치를 확정하고 다시 보지 않는다
(coordinator_node 의 `seen` 집합). 그래서 추측 구간이 최대가 된다:

    카메라 발자국 720px × 0.457mm = 0.329m → base 기준 x ∈ [0.056, 0.384]
    툴 x = -0.09 / -0.27 / -0.45

    첫 관측(먼 가장자리)에 고정 → 0.47 / 0.65 / 0.83 m
    마지막 관측(가까운 가장자리)에 재계획 → 0.15 / 0.33 / 0.51 m

**재관측이 값어치가 있으려면 오차가 이 길이에 따라 줄어야 한다.** 안 줄면 원인이 다른 데
있는 것이고(기구·기울기·타이밍), 코디네이터를 고쳐봐야 헛수고다. 그걸 먼저 가른다.

측정은 Step B 하네스를 그대로 쓴다(같은 밭·같은 제어·같은 채점). 바꾸는 건 표적을 알려주는
거리 하나뿐이다 — 다른 걸 같이 바꾸면 곡선이 무엇의 곡선인지 알 수 없다.

실행:  make lead              개발 밭(기본)
       make lead FIELD=main   정본 밭
"""
import os
import sys
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "tools"))
from assert_step_b import P, TOOL_XS, V, Fail, run, score  # noqa: E402
from field_spec import get as get_field  # noqa: E402
from garden_geometry import Garden  # noqa: E402

G = Garden()

# 실제 기하가 허용하는 세 지점. 임의의 값이 아니라 **카메라가 잡초를 볼 수 있는 구간의 끝과 끝**이다.
FOOT_H = P.camera_h * P.camera_mpp                 # 주행방향 발자국 0.329m
NEAR = P.camera_x - FOOT_H / 2                     # 시야 근접 가장자리 = 마지막으로 볼 수 있는 순간
FAR = P.camera_x + FOOT_H / 2                      # 시야 원거리 가장자리 = 처음 보이는 순간
LEADS = [(0.55, "현재(첫 관측에 고정)"), (P.camera_x, "카메라 중심"), (NEAR, "마지막 관측(재관측)")]


def main():
    name = os.environ.get("FIELD") or "dev"
    spec = get_field(name)
    print("=== 리드↔타격오차 곡선 (Tier 2, 렌더 없음) ===")
    print(f"    밭 {name} · 속도 {V} m/s · 카메라 발자국 {FOOT_H*100:.1f}cm "
          f"(x {NEAR*100:+.1f} ~ {FAR*100:+.1f}cm)")
    print(f"    툴 x = {', '.join(f'{t*100:+.0f}' for t in TOOL_XS)}cm\n")

    rows = []
    for ahead, label in LEADS:
        adv = [ahead - t for t in TOOL_XS]
        gt_file = f"/tmp/ww_lead_{name}_{int(ahead*1000)}_gt.log"
        plans, joints = run(spec, True, gt_file, ahead=ahead)
        scored = score(plans, joints, gt_file)
        ds = [d for _, d, e in scored if e is None]
        print(f"  [리드 {ahead*100:5.1f}cm — {label}]  추측 구간 "
              f"{'/'.join(f'{a*100:.0f}' for a in adv)}cm")
        for p, d, err in scored:
            if err:
                print(f"    왼쪽{p['left']:+.2f}m (툴{p['i']}) — {err}")
            else:
                print(f"    왼쪽{p['left']:+.2f} (툴{p['i']}) → {d*100:6.2f}cm"
                      f" | 전후 {p['err_along']*100:+6.1f} 좌우 {p['err_lat']*100:+6.1f}cm"
                      f" | 이동 추정 {p['est_advance']*100:5.1f} vs 실제 {p['gt_advance']*100:5.1f}cm"
                      f" ({(p['est_advance']/p['gt_advance']-1)*100:+5.1f}%)")
        if not ds:
            raise Fail(f"리드 {ahead*100:.0f}cm 에서 채점된 표적이 하나도 없다")
        mx, av = max(ds) * 100, sum(ds) / len(ds) * 100
        slip = [abs(p["est_advance"] / p["gt_advance"] - 1) for p, d, e in scored if e is None]
        rows.append((ahead, label, av, mx, len(ds), len(scored),
                     sum(slip) / len(slip) * 100))
        print(f"    → 평균 {av:.2f}cm · 최대 {mx:.2f}cm · 채점 {len(ds)}/{len(scored)}\n")

    print("  리드[cm]  추측구간[cm]      평균오차  최대오차  이동추정오차")
    for ahead, label, av, mx, n, tot, sl in rows:
        adv = [ahead - t for t in TOOL_XS]
        print(f"  {ahead*100:7.1f}  {'/'.join(f'{a*100:.0f}' for a in adv):>14}  "
              f"{av:7.2f}  {mx:7.2f}  {sl:11.1f}%   {label}")

    base, short = rows[0], rows[-1]
    gain = base[2] / short[2] if short[2] > 0 else float("inf")
    print(f"\n  재관측 이득: 평균 오차 {base[2]:.2f} → {short[2]:.2f}cm ({gain:.1f}배)")
    print(f"  추측 구간 비:  {(base[0]-TOOL_XS[0])/(short[0]-TOOL_XS[0]):.1f}배 "
          f"(툴0) ~ {(base[0]-TOOL_XS[-1])/(short[0]-TOOL_XS[-1]):.1f}배 (툴{len(TOOL_XS)-1})")

    # 판정: 오차가 추측 구간에 **비례**하는가. 비례하면 재관측이 곧바로 값어치다.
    if short[2] >= base[2]:
        print("\n=== 결론: 리드를 줄여도 오차가 안 준다 — 원인은 추측 구간이 아니다. "
              "기구·기울기·타이밍을 따로 봐야 한다. ===")
        return 1
    print(f"\n=== 결론: 오차가 추측 구간을 따라간다 ({gain:.1f}배 감소) — "
          f"코디네이터의 '첫 관측 고정'을 '마지막 관측 재계획'으로 바꾸는 게 옳다. ===")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fail as e:
        print(f"\n실패: {e}")
        sys.exit(1)
