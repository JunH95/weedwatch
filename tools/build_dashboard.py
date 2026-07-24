#!/usr/bin/env python3
"""관통 P4 — 텃밭 관리 대시보드 (자기완결 웹페이지, DECISIONS 036).

field_run.py 가 낸 artifacts/field_run.json(로봇이 밭에서 한 일)을 읽어, 사람이 브라우저로 보는
단일 HTML 을 만든다. 밭 지도(SVG, 잡초별 판정 색)+통계 타일. 데이터는 내가(에이전트) 로그 쿼리로
검증하고, 화면은 사람이 본다(프로젝트 규율). 나중 실물용 관리 앱(FastAPI+프론트+제어)의 자리표시.

지도·통계는 Python 이 서버측에서 생성(JS 렌더 아님) — 에이전트가 결과를 확인할 수 있게.
실행:  make dashboard   →  artifacts/dashboard.html
"""
import json
import os
import sys

WW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG = os.path.join(WW, "artifacts", "field_run.json")
OUT = os.path.join(WW, "artifacts", "dashboard.html")

OUTCOME = {  # 판정 → (색, 라벨). 007 의 3분류.
    "struck": ("#3f9e57", "자율 처리"),
    "handed_to_human": ("#c98a1e", "사람에게 넘김"),
    "missed": ("#c0473a", "놓침"),
}


def field_map_svg(r) -> str:
    """밭 top-down 지도. x=주행방향(가로), y=두둑(세로). 잡초=판정색 점, 작물=흐린 초록."""
    xs = [X for bl in r["beds"] for w in bl["weeds"] for X in (w["x"],)]
    ys = [w["y"] for bl in r["beds"] for w in bl["weeds"]]
    if not xs:
        return '<p class="muted">표시할 밭 데이터 없음</p>'
    x0, x1 = min(0, min(xs)) - 0.1, max(xs) + 0.3
    y0, y1 = min(ys) - 0.3, max(ys) + 0.3
    W, H, pad = 900, 300, 30
    def sx(x): return pad + (x - x0) / (x1 - x0) * (W - 2 * pad)
    def sy(y): return H - pad - (y - y0) / (y1 - y0) * (H - 2 * pad)
    parts = [f'<svg viewBox="0 0 {W} {H}" width="100%" style="max-width:{W}px" role="img" aria-label="밭 지도">']
    # 두둑 띠 (흙)
    centers = r["field"]["bed_centers"]
    for cy in centers:
        top, bot = sy(cy + 0.45), sy(cy - 0.45)
        parts.append(f'<rect x="{sx(x0)+2:.0f}" y="{top:.0f}" width="{W-2*pad-4:.0f}" '
                     f'height="{bot-top:.0f}" rx="6" fill="var(--soil)" opacity="0.5"/>')
        parts.append(f'<text x="{pad-6:.0f}" y="{sy(cy)+4:.0f}" text-anchor="end" '
                     f'class="axlbl">두둑</text>')
    # 주행 구간 표시
    dx0, dx1 = r["field"]["drive_x"]
    parts.append(f'<line x1="{sx(dx0):.0f}" y1="{H-8}" x2="{sx(dx1):.0f}" y2="{H-8}" '
                 f'stroke="var(--accent)" stroke-width="2"/>')
    parts.append(f'<text x="{sx((dx0+dx1)/2):.0f}" y="{H-12}" text-anchor="middle" class="axlbl">주행 {dx0}~{dx1}m</text>')
    # 작물 (흐린 초록 작은 점)
    for bl in r["beds"]:
        for cx, cyp in bl.get("crops", []):
            if x0 <= cx <= x1:
                parts.append(f'<circle cx="{sx(cx):.1f}" cy="{sy(cyp):.1f}" r="2.4" fill="#2f7d4e" opacity="0.35"/>')
    # 잡초 (판정색 점)
    for bl in r["beds"]:
        for w in bl["weeds"]:
            c = OUTCOME.get(w["outcome"], ("#888", ""))[0]
            parts.append(f'<circle cx="{sx(w["x"]):.1f}" cy="{sy(w["y"]):.1f}" r="5" fill="{c}" '
                         f'stroke="#0003" stroke-width="0.5"><title>{w["outcome"]} ({w["x"]},{w["y"]})</title></circle>')
    parts.append("</svg>")
    return "\n".join(parts)


def build():
    r = json.load(open(LOG, encoding="utf-8"))
    s = r["summary"]; cov = r["coverage"]
    tiles = [
        ("커버리지", f'{cov["beds_done"]}/{cov["beds_total"]}', "두둑 완주", "--accent"),
        ("검출한 잡초", f'{s["detected"]}', "카메라가 본 것", "--ink"),
        ("자율 처리", f'{s["struck"]}', "로봇이 찍음", OUTCOME["struck"][0]),
        ("사람에게 넘김", f'{s["handed_to_human"]}', "작물 근접", OUTCOME["handed_to_human"][0]),
        ("놓침", f'{s["missed"]}', "탐지 실패", OUTCOME["missed"][0]),
        ("소요 시간", f'{r.get("duration_s","?")}', "초 (시뮬)", "--ink"),
    ]
    tile_html = "\n".join(
        f'<div class="tile"><div class="k">{k}</div><div class="v" style="color:{c}">{v}</div>'
        f'<div class="u">{u}</div></div>' for k, v, u, c in tiles)
    legend = " ".join(
        f'<span class="lg"><i style="background:{c}"></i>{lab}</span>' for c, lab in OUTCOME.values())

    html = f'''<title>텃밭 관리 — 자율 제초 리포트</title>
<style>
  :root{{--ground:#f4f2ea;--surface:#fbfaf5;--line:#ddd8c8;--ink:#1c2018;--muted:#6b7060;
    --soil:#b79b74;--accent:#7a8b3c;--mono:ui-monospace,"SFMono-Regular",Menlo,monospace;
    --sans:"Pretendard","Apple SD Gothic Neo","Noto Sans KR",system-ui,sans-serif;}}
  @media(prefers-color-scheme:dark){{:root{{--ground:#12160f;--surface:#1a2015;--line:#2c3423;
    --ink:#e7ebe0;--muted:#8f978a;--soil:#5a4a34;--accent:#9fb356;}}}}
  :root[data-theme="light"]{{--ground:#f4f2ea;--surface:#fbfaf5;--line:#ddd8c8;--ink:#1c2018;--muted:#6b7060;--soil:#b79b74;--accent:#7a8b3c;}}
  :root[data-theme="dark"]{{--ground:#12160f;--surface:#1a2015;--line:#2c3423;--ink:#e7ebe0;--muted:#8f978a;--soil:#5a4a34;--accent:#9fb356;}}
  *{{box-sizing:border-box}}
  body{{margin:0;background:var(--ground);color:var(--ink);font-family:var(--sans);
    line-height:1.6;padding:2rem 1rem}}
  .wrap{{max-width:960px;margin:0 auto}}
  h1{{font-size:1.7rem;margin:0 0 .2rem;letter-spacing:-.02em}}
  .sub{{color:var(--muted);margin:0 0 1.6rem}}
  .tiles{{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:.7rem;margin-bottom:1.8rem}}
  .tile{{background:var(--surface);border:1px solid var(--line);border-radius:12px;padding:.9rem 1rem}}
  .tile .k{{font-size:.78rem;color:var(--muted)}}
  .tile .v{{font-family:var(--mono);font-size:1.9rem;font-weight:700;font-variant-numeric:tabular-nums}}
  .tile .u{{font-size:.74rem;color:var(--muted)}}
  .card{{background:var(--surface);border:1px solid var(--line);border-radius:14px;padding:1.2rem;margin-bottom:1.4rem;overflow-x:auto}}
  .card h2{{font-size:1rem;margin:0 0 .8rem}}
  .axlbl{{font:11px var(--mono);fill:var(--muted)}}
  .legend{{display:flex;gap:1.2rem;flex-wrap:wrap;margin-top:.8rem;font-size:.85rem;color:var(--muted)}}
  .lg{{display:inline-flex;align-items:center;gap:.4rem}}
  .lg i{{width:11px;height:11px;border-radius:50%;display:inline-block}}
  .note{{background:color-mix(in srgb,var(--accent) 8%,var(--surface));border-left:3px solid var(--accent);
    border-radius:0 10px 10px 0;padding:.9rem 1.1rem;color:var(--ink);font-size:.9rem}}
  .muted{{color:var(--muted)}}
  footer{{color:var(--muted);font-size:.8rem;text-align:center;margin-top:1.5rem}}
</style>
<div class="wrap">
  <h1>텃밭 관리 — 자율 제초 리포트</h1>
  <p class="sub">로봇이 카메라로 잡초를 찾아 스스로 처리한 결과. (시뮬레이션, 관통 프로토타입)</p>
  <div class="tiles">{tile_html}</div>
  <div class="card">
    <h2>밭 지도 — 두둑 위 잡초와 처리 결과</h2>
    {field_map_svg(r)}
    <div class="legend">{legend}<span class="lg"><i style="background:#2f7d4e;opacity:.5"></i>작물</span></div>
  </div>
  <div class="note">
    <b>정직하게</b> — 이건 관통 프로토타입이다. 로봇이 두둑을 자율로 훑고 카메라로 잡초를 찾아 찍는
    전 과정이 돌아가고, 이 리포트가 그 결과다. 다만 자율 처리율은 아직 낮다(카메라가 채점 밖 잡초도
    찾고, 잎-밑동 오차가 있음). 최종 목표는 실물 로봇용 관리 앱(실시간 확인 + 작동 제어)이고, 지금은
    그 자리표시다.
  </div>
  <footer>weedwatch · 자율 제초 로봇 시뮬레이션</footer>
</div>'''
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, "w", encoding="utf-8").write(html)
    print(f"저장: {OUT}")
    print(f"  커버리지 {cov['beds_done']}/{cov['beds_total']} · 검출 {s['detected']} · "
          f"처리 {s['struck']} · 사람몫 {s['handed_to_human']} · 놓침 {s['missed']}")


if __name__ == "__main__":
    if not os.path.exists(LOG):
        sys.exit("field_run.json 없음 — 먼저 make field-run")
    build()
