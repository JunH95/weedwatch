#!/usr/bin/env python3
"""헤드랜드 U턴 단언 (Tier 2 물리, 렌더 없음) — `make turn`.

두둑 사이 이동을 순간이동 치트(036)가 아니라 **실제 주행**으로 한다:
두둑을 벗어나 → 헤드랜드에서 좌회전 → 옆으로 pitch → 좌회전 → 옆 두둑에 다시 걸터탄다.

제어 코드는 프로덕션과 **같은 것**을 쓴다 — `weedwatch_control.maneuver`(코디네이터가 쓰는
모듈)를 여기서도 import 해서 돌린다. 다른 건 전송뿐이다(여기는 ww_cmd 직결, 프로덕션은 ROS).
제어에 들어가는 신호는 온보드 센서(휠 오도메트리 거리 + IMU 방위)뿐이고, 채점은 별도 프로세스가
받는 지상진실(GT)이다. ww_cmd 는 GT 를 구독조차 하지 않는다.

── 무엇을 재는 시점인가 (중요) ─────────────────────────────────────────────
게이트는 **두둑에 들어서는 순간**(x가 두둑 끝을 지날 때)의 자세를 본다. 그게 U턴이 실제로
제어하는 값이기 때문이다. 그 뒤 두둑 위를 달리는 동안의 옆 밀림은 U턴 탓이 아니라 **방위
잔차 × 거리**다(IMU bias 0.8° 로 2.9m 달리면 4cm) — 그건 주행 중 카메라로 닫아야 하는
별개 루프라, 여기서는 게이트가 아니라 **보고**한다. 두 개를 한 숫자로 섞으면 U턴이 잘못한
건지 방위 잔차가 누적된 건지 못 가른다(실제로 섞어 봤더니 반복 간 2.5~6.4cm 로 흔들렸다).

게이트 (전부 이미 측정된 물리에서 나온 값):
  1. 진입 y 오차 < 5cm      — 걸터타기 여유 11cm/쪽(034)의 절반
  2. 진입 yaw 오차 < 8°     — 그 이상이면 포드가 두둑 옆면에 낀다 (diag_turn 실측)
  3. 끼임 없음               — 재진입 구간 roll/pitch 진동 < 8°, 두둑 위를 실제로 주행
  4. IMU 를 실제로 썼다      — 휠 yaw 폴백으로 조용히 나빠지지 않았나 (폴백이면 26°/회전)
  5. (보고) 진입 후 옆 밀림 · 추정↔GT 표류 — 회전 중 몸통 미끄러짐은 온보드로 관측 불가

실행:  make turn                    매끈한 밭(기준선)
       make turn FIELD=dev          현실적인 밭 — 굽은 두둑·흙덩이·경사 (DECISIONS 042)
       (colcon 빌드 선행 — 프로덕션 모듈을 import 하므로)

**밭을 바꿔도 게이트는 그대로다.** 임계값을 밭에 맞춰 낮추면 "현실적인 밭에서도 된다"가 공허해진다.
현실 밭에서 실패하면 그게 결과다 — 무엇을 고쳐야 하는지 알려주는 실패다.
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

from weedwatch_control.maneuver import (  # noqa: E402  (colcon install 경유 — 프로덕션과 동일 코드)
    GyroOdom, Maneuver, SWING_RADIUS, EXIT_MARGIN, wrap)

FIELD = os.environ.get("FIELD", "")           # 빈 값 = 기존 매끈한 밭(기준선)
MODEL = "weedwatch"
WW_CMD = str(WW / "build" / "ww_cmd")
V = 0.20

if FIELD:
    from field_spec import get as get_field                     # noqa: E402
    _F = get_field(FIELD)
    WORLD = str(WW / "worlds" / f"field_{FIELD}.sdf")
    WORLD_NAME = f"field_{FIELD}"
    SPAWN = (_F.x0 + 0.30, _F.bed_centers[0], 0.0)
    BED0_Y, BED1_Y = _F.bed_centers[0], _F.bed_centers[1]
    RIDGE_X_END = _F.x1
    ENTRY_X = _F.x1 - 0.30                                      # 재진입 후 두둑 위 지점
    FIELD_DESC = (f"{FIELD} — 두둑 폭±{_F.width_var*100:.0f} 높이±{_F.height_var*100:.0f} "
                  f"사행±{_F.meander*100:.0f}cm · 흙덩이 {_F.clod_density}/m · 경사 {_F.cross_slope_deg}°")
else:
    WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
    WORLD_NAME = "robot_field_multi"
    SPAWN = (0.0, 0.60, 0.0)
    BED0_Y, BED1_Y = 0.60, 1.80
    RIDGE_X_END = 3.30
    ENTRY_X = 2.20
    FIELD_DESC = "매끈한 밭 (기준선)"

GT_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
GT_FILE = "/tmp/ww_turn_gate_gt.log"
X_EXIT = RIDGE_X_END + SWING_RADIUS + EXIT_MARGIN

GATE_Y_CM = 5.0
GATE_YAW_DEG = 8.0
GATE_TILT_DEG = 8.0

# 실물 IMU 잔차 — 시뮬 IMU 는 참자세라 오차 0 이다. 그대로 쓰면 제어에 GT 를 주는 셈(025).
IMU_BIAS_DEG, IMU_NOISE_DEG = 0.8, 0.5


class Fail(Exception):
    pass


class WwCmd:
    """ww_cmd 상주 프로세스 — O(오도메트리)·I(IMU) 스트림. GT 는 안 본다."""

    def __init__(self, proc):
        self.proc = proc
        self.lock = threading.Lock()
        self.odom = self.imu = None
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
            except (IndexError, ValueError):
                continue

    def snapshot(self):
        with self.lock:
            return self.odom, self.imu

    def drive(self, lin, ang=0.0):
        try:
            self.proc.stdin.write(f"v {lin:.4f} {ang:.4f}\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def run():
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    rng = random.Random(7)
    imu_bias = math.radians(IMU_BIAS_DEG) * rng.choice((1, -1))
    gyro = GyroOdom(x0=SPAWN[0], y0=SPAWN[1], yaw0=SPAWN[2])
    marks = []

    log = open("/tmp/ww_turn_gate_sim.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--iterations", "140000", WORLD],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    gtsub = wwp = None
    try:
        deadline = time.time() + 25
        while time.time() < deadline:
            topics = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if GT_TOPIC in topics and "/odometry" in topics:
                break
            time.sleep(0.5)
        else:
            raise Fail("토픽이 안 떴습니다 — 시뮬 초기화 실패")

        gf = open(GT_FILE, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        wwp = subprocess.Popen([ENV, WW_CMD, "--world", WORLD_NAME, "--model", MODEL],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, text=True, bufsize=1,
                               start_new_session=True)
        ww = WwCmd(wwp)
        if not ww.ready.wait(timeout=20):
            raise Fail("ww_cmd 준비(R) 신호가 안 왔습니다")
        time.sleep(2.0)

        def pose():
            odom, imu = ww.snapshot()
            if odom is None:
                return None
            iy = None
            if imu is not None:      # 실물 잔차를 얹어서 쓴다 — 시뮬 IMU 를 GT 로 쓰지 않기 위해
                iy = imu[3] + imu_bias + math.radians(IMU_NOISE_DEG) * rng.gauss(0, 1)
            return gyro.update(odom[1], odom[2], odom[3], odom[4], iy)

        m = Maneuver(ww.drive, pose, v=V)
        m.wait_pose()
        if gyro.degraded:
            raise Fail("IMU 가 안 옵니다 — 월드에 imu-system 플러그인이 있는지 확인")

        def note(msg):
            odom, _ = ww.snapshot()
            marks.append((msg, odom[0]))
            print(f"  {msg}", flush=True)

        m.uturn(X_EXIT, BED1_Y - BED0_Y, ENTRY_X, log=note)
        ww.drive(0.0, 0.0)
        time.sleep(1.0)
        est = pose()
    finally:
        kill(wwp); kill(gtsub)
        time.sleep(0.5)
        kill(sim)
        try:
            sim.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return est, gyro, marks


def main():
    print("=== 헤드랜드 U턴 (Tier 2 물리) ===")
    print(f"    밭: {FIELD_DESC}")
    print(f"    두둑0(y={BED0_Y:.2f}) → 헤드랜드 x={X_EXIT:.2f} → 좌회전 U턴 → 두둑1(y={BED1_Y:.2f}) 재진입 x={ENTRY_X:.2f}")
    print("    제어=온보드(휠 거리 + IMU 방위) · 채점=지상진실(별도 프로세스)\n")
    est, gyro, marks = run()

    gt = gt_samples(Path(GT_FILE).read_text(errors="ignore"))
    if len(gt) < 50:
        raise Fail(f"GT 샘플 부족({len(gt)}) — 채점 불가")
    t_reentry = marks[-2][1] if len(marks) >= 2 else gt[0][0]
    seg = [s for s in gt if s[0] >= t_reentry] or gt[-50:]
    end = gt[-1]

    # 두둑에 들어서는 순간 = 재진입 구간에서 x 가 두둑 끝을 처음 지날 때. U턴이 제어하는 값은 여기까지다.
    entry = next((s for s in seg if s[1] <= RIDGE_X_END), None)
    if entry is None:
        raise Fail(f"두둑 위로 재진입하지 못했습니다 (최종 GT x={end[1]:.2f} > 두둑 끝 {RIDGE_X_END})")

    y_err_cm = abs(entry[2] - BED1_Y) * 100
    yaw_err_deg = abs(math.degrees(wrap(entry[6] - math.pi)))
    roll_pp = math.degrees(max(s[4] for s in seg) - min(s[4] for s in seg))
    pitch_pp = math.degrees(max(s[5] for s in seg) - min(s[5] for s in seg))
    drift_cm = math.hypot(est[0] - end[1], est[1] - end[2]) * 100
    pass_slide_cm = abs(end[2] - entry[2]) * 100
    pass_len = abs(end[1] - entry[1])

    print(f"\n  두둑 진입 순간 GT: x={entry[1]:+.3f} y={entry[2]:+.3f} yaw={math.degrees(entry[6]):+.1f}°")
    print(f"  {'게이트 1 진입 y 오차':<22}{y_err_cm:>7.1f} cm  (< {GATE_Y_CM})")
    print(f"  {'게이트 2 진입 yaw 오차':<22}{yaw_err_deg:>7.1f} °   (< {GATE_YAW_DEG})")
    print(f"  {'게이트 3 자세 진동':<22}{roll_pp:>7.1f} / {pitch_pp:.1f} °  (< {GATE_TILT_DEG}, 끼이면 튄다)")
    print(f"  {'게이트 4 IMU 사용':<22}{'예' if not gyro.degraded else '아니오(휠 폴백)':>7}")
    print(f"  {'보고  진입 후 옆 밀림':<22}{pass_slide_cm:>7.1f} cm / {pass_len:.1f}m 주행"
          f"  (방위 잔차 × 거리 — 주행 중 카메라 루프 몫)")
    print(f"  {'보고  추정↔GT 표류':<22}{drift_cm:>7.1f} cm  (회전 중 몸통 미끄러짐 — 온보드로 관측 불가)")

    if y_err_cm >= GATE_Y_CM:
        raise Fail(f"게이트 1 실패: 진입 시 두둑 중심에서 {y_err_cm:.1f}cm — 걸터타기 여유를 먹는다")
    if yaw_err_deg >= GATE_YAW_DEG:
        raise Fail(f"게이트 2 실패: 진입 yaw {yaw_err_deg:.1f}° — 재진입에서 낀다")
    if roll_pp >= GATE_TILT_DEG or pitch_pp >= GATE_TILT_DEG:
        raise Fail(f"게이트 3 실패: 자세 진동 {roll_pp:.1f}/{pitch_pp:.1f}° — 바퀴가 두둑을 탔다")
    if gyro.degraded:
        raise Fail("게이트 4 실패: IMU 없이 휠 yaw 로 폴백했습니다 (회전당 26° 오차)")

    print("\n[통과] 순간이동 치트 없이 두둑 사이를 실제로 돌아 재진입했습니다.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fail as e:
        print(f"\n[실패] {e}", file=sys.stderr)
        sys.exit(1)
