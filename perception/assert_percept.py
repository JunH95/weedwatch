#!/usr/bin/env python3
"""정적 라이브 인식 단언 — 로봇 카메라가 실제 렌더한 CropCraft 두둑에 best.pt 를 라이브 추론해
잡초 world 좌표를 뽑고, CropCraft 오라클(정답 좌표)과 비교한다 (Tier 3, GPU, Stage 4-3 Phase 4a).

4-1(stamp_targets)은 학습 렌더에 오프라인 추론 + 렌더 마스크를 GT 로 썼다. 여기는 **로봇 down_cam
의 실제 렌더 프레임**에 라이브 추론 + **오라클 world 좌표**를 GT 로 쓴다 — sim카메라→인식 다리를,
카메라 정합(Phase 3) 위에서 정직하게 잇는다. 픽셀→world 매핑은 색 마커로 직접 캘리브(오차 0).

게이트: ① 렌더 2게이트(검지않음 AND NVIDIA, assert_render 재사용) ② 시야 안 오라클 target 검출률
③ 검출↔정답 위치오차(중앙·p90). 카메라 footprint 가 좁아(0.33×0.59m) 정적 1프레임은 target 몇 개만
본다 — 전 두둑 커버리지는 주행(Phase 4b)이 맡는다. 여기선 "실제 카메라로 그 자리 잡초를 맞게 봤나".

실행:  perception/env.sh python perception/assert_percept.py [--gate]   (make percept)
"""
from __future__ import annotations

import argparse
import math
import re
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WW = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(WW / "tools"))
from detect_server import (load_model, detect_frame, detect_fused,  # noqa: E402
                           load_depth, degrade_depth,
                           MPP, CU, CV, CAM_DX, CAM_DYS)
from oracle import load as oracle_load  # noqa: E402
from assert_drive import parse_messages, g, quat_to_rpy  # noqa: E402
import assert_render  # noqa: E402  (렌더 2게이트 재사용)

ENVSH = str(WW / "scripts" / "env.sh")
WORLD = str(WW / "worlds" / "robot_percept.sdf")
GT_TOPIC = "/world/robot_percept/dynamic_pose/info"
GT_FILE = "/tmp/ww_percept_gt.log"
CAMDIR = WW / "artifacts" / "camera"                      # 카메라 0 (기존 이름 유지, DECISIONS 026)
CAMDIRS = [CAMDIR] + [WW / "artifacts" / f"camera{i}" for i in range(1, len(CAM_DYS))]
CAM_TOPICS = ["/robot/camera"] + [f"/robot/camera{i}" for i in range(1, len(CAM_DYS))]
ORACLE = str(WW / "models" / "oracle_test.json")
INCLUDE_OFF = (-1.12, 0.10)   # robot_percept.sdf 의 oracle_test include (dx,dy). 잡초 world = 오라클+이것.

# 매칭 반경 = "그 잡초를 찾았나"의 척도. 8cm 인 이유(정직): 이 절대-좌표 검증은 4-1(상대오프셋,
# 원근 상쇄, 1.4mm)과 달리 ① best.pt 블롭은 캐노피 중심, 오라클은 줄기 밑동 ② footprint 가장자리
# 잡초의 시차(높이·오프셋 비례) ③ 콩 가림 이 안 상쇄된다. 잡초 캐노피 스프레드(~5-15cm)를 감안하면
# 8cm 안이면 "그 잡초를 맞게 식별". 정밀 타격(<2cm)은 Phase 2(오라클 좌표)·카메라-상대 제어(4b)가
# 별도 증명 — 여기는 인식 다리 검증이지 절대 서브센치 정밀이 아니다. 위치오차는 보고(informational).
MATCH_RADIUS = 0.08
# 게이트 임계 (보호 대상. 모델 바꾸는 커밋서 함께 낮추지 마라). 검출률만 게이트, 위치오차는 보고.
GATES = {"recall": 0.75}


class Fail(Exception):
    pass


def _stop(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def render_and_capture():
    """percept 월드 렌더 + GT 캡처 + 카메라 프레임 저장. base pose 반환."""
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    for d in CAMDIRS:
        d.mkdir(parents=True, exist_ok=True)
        for f in d.glob("*.png"):
            f.unlink()
    for i in range(len(CAM_DYS)):
        dd = WW / "artifacts" / f"depth{i}"
        dd.mkdir(parents=True, exist_ok=True)
        for f in dd.glob("*.bin"):
            f.unlink()
    log = open("/tmp/ww_percept.log", "w")
    sim = subprocess.Popen([ENVSH, "ign", "gazebo", "-s", "-r", "--headless-rendering",
                            "--iterations", "15000", WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    gsub, csubs = None, []
    try:
        time.sleep(6)
        gf = open(GT_FILE, "w")
        gsub = subprocess.Popen([ENVSH, "ign", "topic", "-e", "-t", GT_TOPIC],
                                stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        # 카메라는 구독자가 있을 때만 렌더한다(CLAUDE.md) → 대수만큼 구독자를 붙여야 둘 다 찍힌다.
        csubs = [subprocess.Popen([ENVSH, "ign", "topic", "-e", "-t", t],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                  start_new_session=True) for t in CAM_TOPICS]
        # 깊이 원본 수집(Stage 5). <save> 는 8비트 시각화라 못 쓴다 → ign-transport 직결 상주 구독자.
        dtopics = ",".join("/robot/depth" if i == 0 else f"/robot/depth{i}" for i in range(len(CAM_DYS)))
        csubs.append(subprocess.Popen([str(WW / "build" / "ww_depth"), "--topics", dtopics,
                                       "--out", str(WW / "artifacts")],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                      start_new_session=True))
        time.sleep(14)
    finally:
        for p in [gsub, *csubs]:
            _stop(p)
        try:
            gf.close()
        except NameError:
            pass
        _stop(sim)
        try:
            sim.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    time.sleep(1)
    # base GT (마지막 완결 메시지)
    txt = Path(GT_FILE).read_text(errors="ignore")
    for m in reversed(parse_messages(txt)):
        poses = m.get("pose")
        if poses is None:
            continue
        if isinstance(poses, dict):
            poses = [poses]
        for p in poses:
            if isinstance(p, dict) and p.get("name") == "weedwatch":
                q = p.get("orientation", {})
                yaw = quat_to_rpy(g(q, "x"), g(q, "y"), g(q, "z"), g(q, "w") or 1.0)[2]
                return (g(p, "position", "x"), g(p, "position", "y"), g(p, "position", "z"), yaw)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gate", action="store_true")
    args = ap.parse_args()
    print("=== 정적 라이브 인식 단언 (GPU 렌더 + best.pt 라이브) ===\n")

    base = render_and_capture()
    if base is None:
        raise Fail("base 지상진실을 못 읽음 — 렌더/GT 실패")
    print(f"base GT: ({base[0]:.3f},{base[1]:.3f},{base[2]:.3f}) yaw={base[3]:.3f}")

    # 게이트 1: 렌더 2게이트 (검지않음 AND NVIDIA)
    errs = []
    try:
        assert_render.gate_pixels(CAMDIR)
        assert_render.gate_device()
        print("게이트1 렌더: 검지않음 AND NVIDIA — OK")
    except SystemExit:
        errs.append("렌더 2게이트 실패 (검은화면 또는 non-NVIDIA)")
    except Exception as e:
        errs.append(f"렌더 게이트 예외: {e}")

    # 검출 — 카메라 전부의 최신 프레임을 융합 (DECISIONS 026)
    latest = []
    for ci, d in enumerate(CAMDIRS):
        fs = sorted(d.glob("*.png"))
        if not fs:
            errs.append(f"카메라{ci} 프레임 없음 ({d.name}) — 구독자/렌더 실패")
            continue
        latest.append((ci, fs[-1]))
    if not latest:
        raise Fail("어느 카메라도 프레임을 안 냄 — 구독자/렌더 실패")
    model, device = load_model()

    # RGB 프레임 seq → 같은 seq 의 깊이 프레임 (시각 정합은 5Hz 에서 4cm 어긋나 못 씀)
    def depth_for(ci, png):
        m = re.search(r"_(\d+)\.png$", png.name)
        if not m:
            return None
        f = WW / "artifacts" / f"depth{ci}" / f"{m.group(1)}.bin"
        return load_depth(f) if f.exists() else None

    depths = {ci: depth_for(ci, png) for ci, png in latest}
    have = sum(1 for v in depths.values() if v is not None)
    rng = np.random.default_rng(7)
    degraded = {ci: (degrade_depth(v, rng) if v is not None else None) for ci, v in depths.items()}

    dets_flat = detect_fused(model, latest, base, device)                      # A: 평지 가정(기존)
    dets_deep = detect_fused(model, latest, base, device, depths=depths)       # B: 깨끗한 깊이
    dets_real = detect_fused(model, latest, base, device, depths=degraded)     # C: 실물다운 깊이
    dets = dets_deep
    print(f"검출: 카메라 {len(latest)}대 융합 → 평지 {len(dets_flat)} · 깊이 {len(dets_deep)} · 실물깊이 {len(dets_real)}개 "
          f"(깊이 프레임 {have}/{len(latest)})")

    # 시야 안 오라클 target (footprint 반폭: x=v축 CV·MPP, y=u축 CU·MPP)
    HX, HY = CV * MPP, CU * MPP
    og = oracle_load(ORACLE)
    targets = [(w.x + INCLUDE_OFF[0], w.y + INCLUDE_OFF[1]) for w in og.weeds]

    def in_cam(tx, ty, cam_dy):
        """이 카메라 한 대의 발자국 안인가 (yaw≈0 인 정적 검증이라 회전 무시)."""
        return abs(tx - (base[0] + CAM_DX)) < HX and abs(ty - (base[1] + cam_dy)) < HY

    # A/B: 한 대만 썼을 때 vs 전부 (커버리지 이득을 수치로 — 이게 2대를 단 이유다)
    inview_one = [t for t in targets if in_cam(t[0], t[1], CAM_DYS[0])]
    inview = [t for t in targets if any(in_cam(t[0], t[1], dy) for dy in CAM_DYS)]
    gain = len(inview) - len(inview_one)
    print(f"시야 안 오라클 target: 카메라1대 {len(inview_one)}개 → {len(CAM_DYS)}대 {len(inview)}개 "
          f"(+{gain}개, 한 대당 발자국 {HY*2*100:.0f}cm 폭)")

    # 매칭: 각 target 마다 가장 가까운 검출
    errors = []
    detected = 0
    for tx, ty in inview:
        if not dets:
            break
        d = min(math.hypot(wx - tx, wy - ty) for wx, wy, _ in dets)
        if d <= MATCH_RADIUS:
            detected += 1
            errors.append(d)
        print(f"  target ({tx:+.3f},{ty:+.3f}): 최근접검출 {d*100:.2f}cm {'검출' if d<=MATCH_RADIUS else '놓침'}")
    # target 에 안 붙은 검출 (clutter taraxacum = 오라클에 없는 방해물 잡초, 또는 오검출)
    extra = 0
    for wx, wy, _ in dets:
        if not inview or min(math.hypot(wx - tx, wy - ty) for tx, ty in inview) > MATCH_RADIUS:
            extra += 1

    recall = detected / len(inview) if inview else 0.0
    errors.sort()
    med = errors[len(errors) // 2] if errors else float("nan")
    p90 = errors[min(len(errors) - 1, int(0.9 * len(errors)))] if errors else float("nan")

    # ── Stage 5 A/B: 깊이 시차 보정이 위치오차를 실제로 줄이나 ──────────────
    def err_stats(dd):
        es = []
        for tx, ty in inview:
            if not dd:
                continue
            m = min(math.hypot(wx - tx, wy - ty) for wx, wy, _ in dd)
            if m <= MATCH_RADIUS:
                es.append(m)
        es.sort()
        if not es:
            return float("nan"), float("nan"), 0
        return es[len(es) // 2], es[-1], len(es)

    print("\n[Stage 5 A/B] 깊이 시차 보정 — 매칭된 target 위치오차")
    for lab, dd in (("평지 가정(기존)", dets_flat), ("깊이 보정", dets_deep), ("깊이 보정 + 실물 열화", dets_real)):
        med, mx, k = err_stats(dd)
        print(f"    {lab:<22}: 중앙 {med*100:5.2f}cm · 최대 {mx*100:5.2f}cm · 매칭 {k}/{len(inview)}")

    print(f"\n검출률(<= {MATCH_RADIUS*100:.0f}cm): {detected}/{len(inview)} = {recall:.2f} (게이트 {GATES['recall']})")
    print(f"[보고] 매칭된 target 위치오차: 중앙 {med*100:.2f}cm · p90 {p90*100:.2f}cm "
          f"(절대-좌표 캐노피-vs-밑동+시차 — 정밀은 Phase2/4b)")
    print(f"[보고] target 밖 검출 {extra}개 (clutter taraxacum = 오라클 미채점 방해물 포함 — 002/003)")

    if args.gate:
        if not inview:
            errs.append("시야 안 target 이 0 — 두둑 오프셋(INCLUDE_OFF) 확인")
        if recall < GATES["recall"]:
            errs.append(f"검출률 {recall:.2f} < {GATES['recall']} (<= {MATCH_RADIUS*100:.0f}cm)")
        # 카메라를 늘린 값을 실제로 받고 있는가 — 대수만큼 프레임이 나와야 한다.
        if len(latest) != len(CAM_DYS):
            errs.append(f"카메라 {len(CAM_DYS)}대 중 {len(latest)}대만 프레임을 냄 — 구독자/렌더 확인")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    if errs:
        print("\nFAIL 정적 라이브 인식 실패:\n    - " + "\n    - ".join(errs), file=sys.stderr)
        sys.exit(1)
    print(f"\n=== OK 정적 라이브 인식 통과 — 로봇 카메라 실제 렌더에서 best.pt 가 잡초 {detected}/{len(inview)} 를 "
          f"{MATCH_RADIUS*100:.0f}cm 안으로 찾음 (오라클 대조) ===")


if __name__ == "__main__":
    main()
