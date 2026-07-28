#!/usr/bin/env python3
"""Step B — **달리면서** 울퉁불퉁한 밭에 타격한다. 무보정 vs IMU 보정 A/B (Tier 2, 렌더 없음).

Stage 5 가 설계해두고 036 전환 때 로드맵에서 사라졌던 항목이다(042 에서 되살림). 지금까지의 타격
검증은 전부 조건이 하나씩 빠져 있었다:

  make stamp        정지 · 매끈한 밭          → <0.15cm
  make row          주행 · 매끈한 밭          → 6/6 <0.15cm
  make tilt-stamp   정지 · 8° 경사            → 무보정 4.26cm / 보정 0.10cm
  **여기**          주행 · 경사 + 흙덩이 + 두둑 높이 변이 (현실 밭)

── 왜 이 조합이어야 하나 ────────────────────────────────────────────────────
Stage 5 Step A 결론이 이미 말했다: *"흙덩이만으로는 무보정 오차도 1.2cm 라 2cm 안에 들어와 A/B 가
안 갈린다 → 크로스슬로프 + 흙덩이를 합쳐야 한다."* 개발 밭(042)이 정확히 그 조합이다(경사 3° +
흙덩이 1.5/m + 두둑 높이 ±3cm).

── 무엇을 격리하나 (첫 시도가 틀렸다) ──────────────────────────────────────
처음엔 표적을 **절대 world 좌표**로 줬다. 결과가 8.5→34.6cm 로 x 를 따라 커졌다 — 기울기 탓이면
일정해야 하는데 그렇지 않았다. 원인은 명백했다: **로봇이 자기 위치를 모르므로 절대 좌표를 줘도 못
찾는다**(043 의 옆 이탈 14.4cm·절대 x 90cm 가 그대로 타격 오차가 된다). 그건 위치추정 실험이지
타격 실험이 아니다.

그래서 **카메라가 보는 방식**으로 준다: "지금 네 앞 0.55m, 왼쪽 0.12m 에 잡초가 있다."
로봇은 그 순간부터 **자기 오도메트리로만** 표적을 좇아 1.5초 뒤 타격한다 — row-live 와 같은 계약이다.
이러면 장기 표류가 빠지고 **기구 + 단기 오도메트리 + 기울기**만 남는다. 그게 Step B 의 물음이다.

제어는 온보드(휠 거리 + IMU)만 쓴다. 채점은 지상진실 base + achieved joint_state → FK.
IMU 는 실물 잔차(bias 0.8°·노이즈 0.5°)를 얹어 쓴다 — 전처리 없이(025).

게이트:
  1. 보정 타격이 전부 2cm 안       (실물 성공 기준)
  2. 무보정이 **유의하게 더 나쁘다** — A/B 가 안 갈리면 이 실험은 아무것도 안 말한 것이다
  3. 완주                          (끼임·전복 없음)

실행:  make step-b            개발 밭(기본)
       make step-b FIELD=main 정본 밭
"""
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
sys.path.insert(0, str(WW / "tools"))
from assert_drive import gt_samples  # noqa: E402
from field_spec import get as get_field  # noqa: E402
from garden_geometry import Garden, Portal  # noqa: E402

from weedwatch_control.maneuver import GyroOdom, heading_correction  # noqa: E402

sys.path.insert(0, str(WW / "perception"))
from vo import VoTracker  # noqa: E402  (진단·노드와 같은 코드)

G, P = Garden(), Portal()
N = P.n_tools
TOOL_XS = P.tool_xs()
BAND_CENTERS = P.tool_band_centers(G)
TIP_DZ = 0.3075          # base 기준 도구 끝 z (tool_pos=0) — assert_row_stamp 와 같은 값
V = 0.20
STRIKE, RAISE = -0.15, 0.0
Z_SETTLE = 0.180
HIT_CM = 2.0             # 실물 성공 기준
IMU_BIAS_DEG, IMU_NOISE_DEG = 0.8, 0.5
WW_CMD = str(WW / "build" / "ww_cmd")


class Fail(Exception):
    pass


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def rot(off, r, p, y):
    """R = Rz(y)·Ry(p)·Rx(r) — 몸통이 기울고 돌아간 상태의 body→world."""
    ox, oy, oz = off
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    x1, y1, z1 = ox, oy * cr - oz * sr, oy * sr + oz * cr
    x2, y2, z2 = x1 * cp + z1 * sp, y1, -x1 * sp + z1 * cp
    return (x2 * cy - y2 * sy, x2 * sy + y2 * cy, z2)


def tip_world(base, i, cpos, tpos):
    """실측 도구끝 — base(x,y,z,roll,pitch,yaw) + 전체회전 FK. 기울기를 채점에 정직히 반영."""
    bx, by, bz, r, p, y = base
    ox, oy, oz = rot((TOOL_XS[i], BAND_CENTERS[i] + cpos, TIP_DZ + tpos), r, p, y)
    return (bx + ox, by + oy, bz + oz)


def carriage_uncorrected(i, wy, base_y):
    """지금 시스템: 로봇이 수평이라 가정한다."""
    return (wy - base_y) - BAND_CENTERS[i]


def carriage_corrected(i, wy, base_y, base_z, roll, bed_top):
    """IMU roll 로 옆밀림을 상쇄 — 도구가 기운 축으로 두둑 윗면에 닿는 조건을 역산(tilt_stamp 와 동일)."""
    oy = math.cos(roll) * (wy - base_y + math.tan(roll) * (bed_top - base_z))
    return oy - BAND_CENTERS[i]


class WwCmd:
    """ww_cmd 상주 프로세스 — O(odom)·I(IMU)·J(관절). GT 는 안 본다."""

    def __init__(self, proc):
        self.proc = proc
        self.lock = threading.Lock()
        self.odom = self.imu = None
        self.joints = []
        self.ready = threading.Event()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for raw in self.proc.stdout:
            line = raw.rstrip("\n")
            if not line:
                continue
            tag, p = line[0], line.split()
            try:
                if tag == "R":
                    self.ready.set()
                elif tag == "O":
                    with self.lock:
                        self.odom = tuple(float(v) for v in p[1:7])
                elif tag == "I":
                    with self.lock:
                        self.imu = tuple(float(v) for v in p[1:5])
                elif tag == "J":
                    vals = [float(v) for v in p[2:2 + 2 * N]]
                    if len(vals) == 2 * N:
                        with self.lock:
                            self.joints.append((float(p[1]), vals[:N], vals[N:]))
            except (IndexError, ValueError):
                continue

    def snap(self):
        with self.lock:
            return self.odom, self.imu

    def send(self, s):
        try:
            self.proc.stdin.write(s + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass


# 표적은 **로봇 기준 상대 좌표**다 — 카메라가 "앞 A m, 왼쪽 L m 에 잡초" 라고 말해주는 상황.
# AHEAD 는 카메라 선행거리(0.31m)보다 넉넉히 크게 잡아 하강 리드(180ms)가 들어갈 여유를 준다.
REL_TARGETS = [(0.55, -0.28), (0.55, 0.0), (0.55, +0.28), (0.55, -0.14)]


def reveal_xs(spec, n):
    """표적이 드러나는 지점(로봇 추정 x) — 두둑 위에 고루 흩는다."""
    return [spec.x0 + spec.bed_length * f for f in (0.20, 0.38, 0.56, 0.74)][:n]


def run(spec, corrected: bool, gt_file: str):
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    world = str(WW / "worlds" / f"field_{spec.name}.sdf")
    world_name = f"field_{spec.name}"
    rng = random.Random(11)
    imu_bias = math.radians(IMU_BIAS_DEG) * rng.choice((1, -1))
    gyro = GyroOdom(x0=spec.x0 + 0.30, y0=spec.bed_centers[0], yaw0=0.0)
    plans = []
    for (ahead, left), rx in zip(REL_TARGETS, reveal_xs(spec, len(REL_TARGETS))):
        i = P.band_of(G, left)                          # 담당 툴 = 그 좌우 밴드
        plans.append({"ahead": ahead, "left": left, "i": i, "reveal_x": rx,
                      "phase": 0, "t_reveal": None, "t_strike": None,
                      # base 가 reveal 이후 이만큼 더 가면 도구 끝이 표적에 닿는다
                      "advance": ahead - TOOL_XS[i]})

    log = open(f"/tmp/ww_stepb_{spec.name}_sim.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--iterations", "120000", world],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    gtsub = wwp = None
    try:
        deadline = time.time() + 30
        gt_topic = f"/world/{world_name}/dynamic_pose/info"
        while time.time() < deadline:
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if gt_topic in t and "/odometry" in t:
                break
            time.sleep(0.5)
        else:
            raise Fail("토픽이 안 떴습니다")
        gf = open(gt_file, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", gt_topic],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        wwp = subprocess.Popen([ENV, WW_CMD, "--world", world_name, "--model", "weedwatch",
                                "--n-tools", str(N)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, text=True, bufsize=1,
                               start_new_session=True)
        ww = WwCmd(wwp)
        if not ww.ready.wait(timeout=20):
            raise Fail("ww_cmd 준비 신호 없음")
        time.sleep(2.5)                                  # 스폰 안착

        end_x = max(p["reveal_x"] + p["advance"] for p in plans) + 0.35
        ww.send(f"v {V:.3f} 0")
        t_end = time.time() + (end_x - spec.x0) / V / 0.12 + 40
        while time.time() < t_end:
            odom, imu = ww.snap()
            if odom is None:
                time.sleep(0.01); continue
            iy = ir = None
            if imu is not None:
                noise = math.radians(IMU_NOISE_DEG) * rng.gauss(0, 1)
                iy = imu[3] + imu_bias + noise
                ir = imu[1] + imu_bias                    # roll 에도 같은 잔차(전처리 없음)
            x, y, yaw = gyro.update(odom[1], odom[2], odom[3], odom[4], iy)
            ww.send(f"v {V:.3f} {heading_correction(yaw, 0.0):.4f}")

            for p in plans:
                if p["phase"] == 0 and x >= p["reveal_x"]:
                    # "카메라가 지금 봤다" — 이 순간의 상대 좌표로만 좇는다(절대 좌표 안 씀).
                    # 보정: 기운 몸통에서 도구가 옆으로 밀리는 만큼 캐리지를 미리 당긴다.
                    bed_h = spec.profile(0, x)[2]
                    cpos = (carriage_corrected(p["i"], p["left"], 0.0, 0.0, ir or 0.0, bed_h)
                            if corrected else carriage_uncorrected(p["i"], p["left"], 0.0))
                    ww.send(f"carriage {p['i']} {cpos:.4f}")
                    p["cpos"], p["phase"], p["t_reveal"] = cpos, 1, odom[0]
                    p["x_reveal"] = x
                elif p["phase"] == 1 and x >= p["x_reveal"] + p["advance"] - V * Z_SETTLE:
                    ww.send(f"tool {p['i']} {STRIKE:.4f}")
                    p["phase"], p["t_strike"], p["x_strike"] = 2, odom[0], x
                elif p["phase"] == 2 and x >= p["x_reveal"] + p["advance"] + 0.06:
                    ww.send(f"tool {p['i']} {RAISE:.4f}")
                    p["phase"] = 3
            if x >= end_x:
                break
            time.sleep(0.01)
        ww.send("v 0 0")
        time.sleep(1.0)
        with ww.lock:
            joints = list(ww.joints)
    finally:
        kill(wwp); kill(gtsub); time.sleep(0.4); kill(sim)
        try:
            sim.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return plans, joints


def score(plans, joints, gt_file):
    """채점 — 표적의 world 좌표는 **공개 시점의 지상진실 자세**에서 유도한다.

    "카메라가 그때 본 것"이 곧 그 순간 로봇 앞 A·왼쪽 L 지점이다. 제어는 그 상대값만 받았고,
    채점은 그게 실제로 어디였는지(GT)와 도구가 어디 닿았는지(GT+관절 FK)를 비교한다.
    """
    gt = gt_samples(Path(gt_file).read_text(errors="ignore"))
    if len(gt) < 20:
        raise Fail(f"GT 샘플 부족({len(gt)})")
    out = []
    for p in plans:
        if p["t_strike"] is None or p["t_reveal"] is None:
            out.append((p, None, "하강 안 함"))
            continue
        g0 = min(gt, key=lambda s: abs(s[0] - p["t_reveal"]))       # 공개 순간 실제 자세
        yaw0 = g0[6]
        tx = g0[1] + p["ahead"] * math.cos(yaw0) - p["left"] * math.sin(yaw0)
        ty = g0[2] + p["ahead"] * math.sin(yaw0) + p["left"] * math.cos(yaw0)
        g = min(gt, key=lambda s: abs(s[0] - p["t_strike"]))
        j = min(joints, key=lambda z: abs(z[0] - p["t_strike"])) if joints else None
        if j is None:
            out.append((p, None, "관절 샘플 없음"))
            continue
        base = (g[1], g[2], g[3], g[4], g[5], g[6])
        tip = tip_world(base, p["i"], j[1][p["i"]], j[2][p["i"]])
        p["tx"], p["ty"] = tx, ty
        # 오차를 성분으로 나눈다 — 추측 대신 어디서 오는지 보이게.
        p["gt_advance"] = math.hypot(g[1] - g0[1], g[2] - g0[2])   # 실제 이동 거리
        p["est_advance"] = p.get("x_strike", float("nan")) - p.get("x_reveal", float("nan"))
        p["roll_deg"] = math.degrees(g[4])
        p["dyaw_deg"] = math.degrees(math.atan2(math.sin(g[6] - yaw0), math.cos(g[6] - yaw0)))
        p["err_along"] = (tip[0] - tx) * math.cos(yaw0) + (tip[1] - ty) * math.sin(yaw0)
        p["err_lat"] = -(tip[0] - tx) * math.sin(yaw0) + (tip[1] - ty) * math.cos(yaw0)
        out.append((p, math.hypot(tip[0] - tx, tip[1] - ty), None))
    return out


def main():
    name = os.environ.get("FIELD") or "dev"
    spec = get_field(name)
    print("=== Step B — 달리면서 울퉁불퉁한 밭에 타격 (Tier 2, 렌더 없음) ===")
    print(f"    밭 {name}: 두둑 높이±{spec.height_var*100:.0f}cm · 흙덩이 {spec.clod_density}/m · "
          f"경사 {spec.cross_slope_deg}° · 속도 {V} m/s")
    print(f"    표적은 **로봇 기준 상대**(카메라가 보는 방식) · 제어=온보드"
          f"{' + 시각 오도메트리(슬립 감지)' if USE_VO else ''} · 채점=지상진실 FK\n")

    results = {}
    for corrected in (False, True):
        tag = "보정  " if corrected else "무보정"
        gt_file = f"/tmp/ww_stepb_{name}_{'corr' if corrected else 'raw'}_gt.log"
        plans, joints = run(spec, corrected, gt_file)
        rows = score(plans, joints, gt_file)
        results[corrected] = rows
        print(f"  [{tag}]")
        for p, d, err in rows:
            if err:
                print(f"    표적 앞{p['ahead']:.2f}m 왼쪽{p['left']:+.2f}m — {err}")
            else:
                mark = "HIT " if d * 100 < HIT_CM else "MISS"
                print(f"    앞{p['ahead']:.2f} 왼쪽{p['left']:+.2f} (툴{p['i']}) → 오차 {d*100:6.2f}cm {mark}"
                      f" | 전후 {p['err_along']*100:+6.1f} 좌우 {p['err_lat']*100:+6.1f}cm"
                      f" | 이동 추정 {p['est_advance']*100:.0f} vs 실제 {p['gt_advance']*100:.0f}cm"
                      f" | roll {p['roll_deg']:+.1f}° Δyaw {p['dyaw_deg']:+.1f}°")
        print()

    def stats(rows):
        ds = [d for _, d, e in rows if e is None]
        return (max(ds) * 100 if ds else float("nan"),
                sum(ds) / len(ds) * 100 if ds else float("nan"), len(ds), len(rows))

    raw_max, raw_avg, raw_n, raw_tot = stats(results[False])
    cor_max, cor_avg, cor_n, cor_tot = stats(results[True])
    print(f"  {'무보정':<8} 최대 {raw_max:6.2f}cm · 평균 {raw_avg:6.2f}cm · 채점 {raw_n}/{raw_tot}")
    print(f"  {'보정':<8} 최대 {cor_max:6.2f}cm · 평균 {cor_avg:6.2f}cm · 채점 {cor_n}/{cor_tot}")

    if cor_n < cor_tot or raw_n < raw_tot:
        raise Fail("표적 중 일부가 채점 불가 (하강 안 함 / 관절 샘플 없음) — 주행이 완주 못 했다")
    if cor_max >= HIT_CM:
        raise Fail(f"게이트 1 실패: 보정해도 최대 {cor_max:.2f}cm — 현실 밭에서 2cm 를 못 지킨다")
    if raw_avg <= cor_avg * 1.5:
        raise Fail(f"게이트 2 실패: 무보정({raw_avg:.2f}cm)과 보정({cor_avg:.2f}cm)이 안 갈린다 — "
                   f"이 지형이 보정을 시험하지 못한다(더 험한 밭이 필요)")
    print(f"\n[통과] 현실 밭에서도 IMU 보정이 타격을 2cm 안으로 지킨다 "
          f"(무보정 대비 {raw_avg/cor_avg:.1f}배 정확).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fail as e:
        print(f"\n[실패] {e}", file=sys.stderr)
        sys.exit(1)
