#!/usr/bin/env python3
"""헤드랜드 U턴 진단 — 제자리 회전이 휠 오도메트리를 얼마나 틀어놓나.

두둑 사이 이동은 지금 순간이동 치트(036). 진짜로 하려면 U턴을 해야 한다:
  A 두둑을 벗어나 헤드랜드로  →  B 좌회전 90°  →  C 옆으로 pitch(1.2m)  →  D 좌회전 90°
  →  E 다음 두둑에 재진입(반대 방향 주행)

재진입 자체의 허용오차는 이미 쟀다(diag_turn: yaw 8°까지 안 낀다). 남은 미지수는 **회전이
위치추정을 얼마나 깨뜨리나**다. 제자리 회전은 좌우 바퀴가 옆으로 긁혀(scrub) 휠 오도메트리가
가장 크게 틀어지는 동작이고, 코디네이터는 그 odom 으로 world x 를 앵커링한다. 직선 평지에서는
odom↔GT 가 0.5% 였지만(033) 회전은 잰 적이 없다.

규율: **제어는 온보드 센서(odom·IMU), 채점은 지상진실(GT)** — 프로세스로 분리한다. ww_cmd 는 GT 를
의도적으로 구독하지 않는다. 여기서 두 값을 나란히 놓는 건 사후 채점뿐이다.

odom 은 스폰 자세 기준 상대값이므로 world 추정 = odom + 스폰(0, 0.6, 0) 으로 환산해 비교한다.

── 방위 소스를 가른다 (이 진단의 요지) ─────────────────────────────────────
이 로봇은 조향축 없는 4륜 고정 = **스키드 스티어**다. 돌면 바퀴가 옆으로 긁히는데 휠 오도메트리는
그걸 못 본다. 그래서 방위를 무엇으로 재느냐를 바꿔가며 같은 U턴을 시킨다:
  --heading odom : 휠 오도메트리 yaw (지금 코디네이터가 쓰는 것)
  --heading imu  : IMU 자세 + **실물 잔차**(bias 0.8° · 노이즈 0.5°, 025 와 같은 처리)
그리고 회전 방식도 가른다:
  --turn pivot   : 제자리 회전 (긁힘 최대)
  --turn arc     : 반경 R 원호 (바퀴가 구르며 돎 — 긁힘이 줄어드는지)

판정선 (둘 다 이미 측정된 물리에서 나온 값):
  · yaw 오차 < 8°   — 그 이상이면 재진입 때 포드가 두둑에 낀다 (diag_turn)
  · y 오차 < 5cm    — 걸터타기 여유 11cm/쪽(034)의 절반

실행:  ./scripts/env.sh python3 tools/diag_uturn.py [--heading odom|imu] [--turn pivot|arc]
"""
import argparse
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

WORLD = str(WW / "worlds" / "robot_field_multi.sdf")
WORLD_NAME, MODEL = "robot_field_multi", "weedwatch"
GT_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
GT_FILE = "/tmp/ww_uturn_gt.log"
WW_CMD = str(WW / "build" / "ww_cmd")

SPAWN = (0.0, 0.60, 0.0)     # 월드가 로봇을 여기 스폰 (odom 원점)
BED0_Y, BED1_Y = 0.60, 1.80  # 두둑 중심 (pitch 1.2)
RIDGE_X_END = 3.30           # 두둑(ridge) 가 끝나는 x
V = 0.20                     # 주행 속도 [m/s] (무정차 상한, 020)
W = 0.50                     # 회전 각속도 [rad/s]

# 회전 반경: 로봇 대각 반지름 = √((deck_len/2)² + (전폭/2)²) = √(0.65²+0.70²) ≈ 0.955m.
# 그만큼 두둑 끝에서 더 나가야 도는 동안 모서리가 두둑을 안 스친다.
SWING_R = 0.955
X_EXIT = RIDGE_X_END + SWING_R + 0.15    # 4.40 — 여기까지 나가서 돈다
X_REENTRY_END = 1.50                     # 재진입 후 여기까지 두둑 위를 달려본다(끼임 확인)

JAM_YAW_DEG = 8.0            # diag_turn 실측: 이 이상 틀어지면 낀다
JAM_Y_CM = 5.0               # 걸터타기 여유 11cm/쪽의 절반

# 실물 IMU 잔차 (BNO085 동적 오차 ~1°, DECISIONS 025 와 같은 처리). 시뮬 IMU 의 orientation 은
# 참자세에서 계산돼 오차 0 이라, 그대로 쓰면 제어에 지상진실을 주는 셈이 된다. 그래서 얹는다.
IMU_BIAS_DEG, IMU_NOISE_DEG = 0.8, 0.5


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class WwCmd:
    """ww_cmd 상주 프로세스 래퍼 — O(odom) 스트림만 쓴다. 제어는 오직 이 odom."""

    def __init__(self, proc):
        self.proc = proc
        self.lock = threading.Lock()
        self.odom = None                 # (simt, x, y, yaw, vx, wz)
        self.imu = None                  # (simt, roll, pitch, yaw)
        self.ready = threading.Event()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for raw in self.proc.stdout:
            line = raw.rstrip("\n")
            if not line:
                continue
            if line[0] == "R":
                self.ready.set()
            elif line[0] == "O":
                p = line.split()
                try:
                    s = tuple(float(v) for v in p[1:7])
                except (IndexError, ValueError):
                    continue
                with self.lock:
                    self.odom = s
            elif line[0] == "I":
                p = line.split()
                try:
                    s = tuple(float(v) for v in p[1:5])
                except (IndexError, ValueError):
                    continue
                with self.lock:
                    self.imu = s
            elif line[0] == "E":
                print(f"  [ww_cmd] {line}", file=sys.stderr)

    def get(self):
        with self.lock:
            return self.odom

    def get_imu(self):
        with self.lock:
            return self.imu

    def send(self, line):
        try:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    def drive(self, lin, ang=0.0):
        self.send(f"v {lin:.4f} {ang:.4f}")

    def stop(self):
        self.drive(0.0, 0.0)


class Maneuver:
    """온보드 센서 폐루프 마뉴버 프리미티브. yaw 는 누적(unwrap)으로 추적해 ±π 접힘을 피한다.

    위치는 언제나 odom(휠)에서, 방위는 heading 인자가 고르는 소스에서 온다.
    """

    def __init__(self, ww, heading="odom", turn="pivot", pose="odom", seed=7):
        self.ww = ww
        self.heading_src = heading
        self.turn_style = turn
        self.pose_src = pose
        self.yaw_acc = None      # 누적 yaw (unwrapped)
        self._last_raw = None
        # 자이로-오도메트리 누적 (pose=gyro): 바퀴가 **얼마나 갔나**만 쓰고, **어느 쪽으로**는
        # IMU 가 준다. 휠 오도메트리 자체 x,y 는 회전 중 틀어진 yaw 로 적분돼 못 쓴다(실측).
        self.gx = self.gy = 0.0
        self._last_odom_xy = None
        rng = random.Random(seed)
        self._imu_bias = math.radians(IMU_BIAS_DEG) * rng.choice((1, -1))
        self._rng = rng

    def _update_yaw(self, raw):
        if self.yaw_acc is None:
            self.yaw_acc, self._last_raw = raw, raw
        else:
            self.yaw_acc += wrap(raw - self._last_raw)
            self._last_raw = raw
        return self.yaw_acc

    def _raw_yaw(self):
        """제어가 믿는 방위 (선택된 센서). IMU 는 실물 잔차를 얹어 GT 를 그냥 주지 않는다."""
        if self.heading_src == "imu":
            im = self.ww.get_imu()
            if im is None:
                return None
            noise = math.radians(IMU_NOISE_DEG) * self._rng.gauss(0, 1)
            return wrap(im[3] + self._imu_bias + noise)
        o = self.ww.get()
        return None if o is None else o[3]

    def pose(self):
        o = self.ww.get()
        raw = self._raw_yaw()
        if o is None or raw is None:
            return None
        yaw = self._update_yaw(raw)
        # 자이로-오도메트리: 바퀴에서 이동 **거리**만 뽑아 IMU **방위**로 적분한다.
        if self._last_odom_xy is None:
            self._last_odom_xy = (o[1], o[2])
        else:
            dx, dy = o[1] - self._last_odom_xy[0], o[2] - self._last_odom_xy[1]
            self._last_odom_xy = (o[1], o[2])
            ds = math.copysign(math.hypot(dx, dy), o[4] if abs(o[4]) > 1e-6 else 1.0)
            self.gx += ds * math.cos(yaw)
            self.gy += ds * math.sin(yaw)
        if self.pose_src == "gyro":
            return self.gx, self.gy, yaw
        return o[1], o[2], yaw

    def wait_pose(self, timeout=15):
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.pose()
            if p:
                return p
            time.sleep(0.02)
        raise RuntimeError("odom 이 안 옵니다 — ww_cmd/시뮬 실패")

    def drive_until(self, done, timeout):
        """done(x, y, yaw) 가 참이 될 때까지 전진. 제동거리(≈0.40·v²)만큼 미리 세운다."""
        self.ww.drive(V)
        t0 = time.time()
        while time.time() - t0 < timeout:
            x, y, yaw = self.wait_pose()
            if done(x, y, yaw):
                break
            time.sleep(0.01)
        else:
            self.ww.stop()
            raise RuntimeError("주행 구간 시간 초과")
        self.ww.stop()
        time.sleep(1.2)          # 제동 미끄러짐 정착 (바퀴는 서고 몸통은 더 간다)

    def drive_to_x(self, x_target, forward=True):
        lead = 0.40 * V * V      # 제동거리 실측 피팅 (STATUS Stage 4-3)
        if forward:
            self.drive_until(lambda x, y, yw: x >= x_target - lead, timeout=90)
        else:
            self.drive_until(lambda x, y, yw: x <= x_target + lead, timeout=90)

    def drive_dist(self, dist):
        x0, y0, _ = self.wait_pose()
        lead = 0.40 * V * V
        self.drive_until(lambda x, y, yw: math.hypot(x - x0, y - y0) >= dist - lead, timeout=90)

    def arc_turn(self, yaw_target, radius):
        """원호 회전 — 전진하면서 돈다. 바퀴가 구르므로 제자리 회전보다 긁힘이 적어야 한다.

        두둑 pitch 가 반경을 못 박는다: 같은 방향 90° 원호 두 개(=180°)의 옆이동이 2R 이므로
        R = pitch/2 = 0.6m 여야 옆 두둑에 정확히 선다. 더 크면 지나치고, 더 작으면 못 미친다.
        """
        _, _, yaw = self.wait_pose()
        sign = 1.0 if yaw_target > yaw else -1.0
        lead = math.radians(4.0)
        self.ww.drive(V, sign * V / radius)
        t0 = time.time()
        while time.time() - t0 < 60:
            _, _, yaw = self.wait_pose()
            if sign * (yaw_target - yaw) <= lead:
                break
            time.sleep(0.01)
        self.ww.stop()
        time.sleep(1.2)
        self._fine_yaw(yaw_target)

    def _fine_yaw(self, yaw_target, fine_deg=0.5):
        """남은 방위 오차를 느린 제자리 회전으로 마무리."""
        for _ in range(400):
            _, _, yaw = self.wait_pose()
            err = yaw_target - yaw
            if abs(err) <= math.radians(fine_deg):
                break
            self.ww.drive(0.0, math.copysign(0.12, err))
            time.sleep(0.05)
        self.ww.stop()
        time.sleep(1.0)

    def turn_to(self, yaw_target, fine_deg=0.5):
        """제자리 회전. 거친 회전 → 느린 마무리(잔차 fine_deg 이내)."""
        _, _, yaw = self.wait_pose()
        sign = 1.0 if yaw_target > yaw else -1.0
        lead = math.radians(4.0)                      # 회전 관성 선행 정지
        self.ww.drive(0.0, sign * W)
        t0 = time.time()
        while time.time() - t0 < 30:
            _, _, yaw = self.wait_pose()
            if sign * (yaw_target - yaw) <= lead:
                break
            time.sleep(0.01)
        self.ww.stop()
        time.sleep(1.0)
        self._fine_yaw(yaw_target, fine_deg)


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def run(heading="odom", turn="pivot", pose="odom", seed=7):
    """U턴을 실제로 시켜보고, 각 구간 끝에서 (온보드 추정, GT) 를 같이 남긴다."""
    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    marks = []          # [(구간, simt, odom_x, odom_y, odom_yaw)]
    log = open("/tmp/ww_uturn_sim.log", "w")
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
            raise RuntimeError("토픽이 안 떴습니다 — 시뮬 초기화 실패")

        gf = open(GT_FILE, "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", GT_TOPIC],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        wwp = subprocess.Popen([ENV, WW_CMD, "--world", WORLD_NAME, "--model", MODEL],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, text=True, bufsize=1,
                               start_new_session=True)
        ww = WwCmd(wwp)
        if not ww.ready.wait(timeout=20):
            raise RuntimeError("ww_cmd 준비(R) 신호가 안 왔습니다")
        time.sleep(2.0)                                # 스폰 안착

        m = Maneuver(ww, heading=heading, turn=turn, pose=pose, seed=seed)
        if heading == "imu" and ww.get_imu() is None:
            raise RuntimeError("IMU 가 안 옵니다 — 월드에 imu-system 플러그인이 있나 확인")

        def mark(name):
            o = ww.get()
            ex, ey, _ = m.pose()
            marks.append((name, o[0], ex, ey, m.yaw_acc))
            print(f"    · {name}: 추정 x={o[1]:+.3f} y={o[2]:+.3f} yaw={math.degrees(m.yaw_acc):+.1f}°",
                  flush=True)

        m.wait_pose()
        mark("시작")
        m.drive_to_x(X_EXIT - SPAWN[0]);        mark("A 헤드랜드 진출")
        if turn == "arc":
            # 180° 원호 하나로 옆 두둑에 선다 (옆이동 = 2R = pitch).
            m.arc_turn(math.radians(180), (BED1_Y - BED0_Y) / 2);  mark("B 원호 U턴 180°")
        else:
            m.turn_to(math.radians(90));        mark("B 좌회전 90°")
            m.drive_dist(BED1_Y - BED0_Y);      mark("C 옆으로 1.2m")
            m.turn_to(math.radians(180));       mark("D 좌회전 90°")
        m.drive_to_x(X_REENTRY_END - SPAWN[0], forward=False)
        mark("E 재진입 주행")
        ww.stop()
        time.sleep(1.0)
    finally:
        kill(wwp); kill(gtsub)
        time.sleep(0.5)
        kill(sim)
        try:
            sim.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()
    return marks


def nearest_gt(gt, t):
    return min(gt, key=lambda s: abs(s[0] - t)) if gt else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--heading", choices=("odom", "imu"), default="odom",
                    help="제어가 방위를 재는 센서 (imu 는 실물 잔차 얹음)")
    ap.add_argument("--turn", choices=("pivot", "arc"), default="pivot",
                    help="회전 방식: 제자리 / 원호(R=pitch/2)")
    ap.add_argument("--pose", choices=("odom", "gyro"), default="odom",
                    help="위치 추정: 휠 오도메트리 그대로 / 자이로-오도메트리(거리는 바퀴, 방위는 IMU)")
    ap.add_argument("--seed", type=int, default=7, help="IMU 잔차 실현값 시드 (반복성 확인용)")
    a = ap.parse_args()

    print(f"=== 헤드랜드 U턴 — 방위={a.heading} · 회전={a.turn} · 위치={a.pose} ===")
    print(f"    두둑0(y={BED0_Y}) → 헤드랜드 x={X_EXIT:.2f} → U턴(옆 {BED1_Y-BED0_Y:.1f}m) → 두둑1 재진입")
    print(f"    제어=온보드 센서(ww_cmd) · 채점=GT(별도 프로세스) · 판정선 yaw<{JAM_YAW_DEG}° · y<{JAM_Y_CM}cm\n")
    marks = run(heading=a.heading, turn=a.turn, pose=a.pose, seed=a.seed)
    gt = gt_samples(Path(GT_FILE).read_text(errors="ignore"))
    if len(gt) < 10:
        print(f"[실패] GT 샘플 부족({len(gt)})")
        return 2

    print(f"\n    {'구간':<16} {'제어가 믿는 (x, y, yaw)':<30} {'지상진실 (x, y, yaw)':<30} {'Δ위치':>8} {'Δyaw':>8}")
    rows = []
    for name, simt, ox, oy, oyaw in marks:
        s = nearest_gt(gt, simt)
        ex, ey, eyaw = ox + SPAWN[0], oy + SPAWN[1], oyaw + SPAWN[2]   # odom → world 추정
        gx, gy, gyaw = s[1], s[2], s[6]
        dpos = math.hypot(ex - gx, ey - gy)
        dyaw = math.degrees(wrap(eyaw - gyaw))
        rows.append((name, ex, ey, eyaw, gx, gy, gyaw, dpos, dyaw))
        print(f"    {name:<16} ({ex:+6.3f},{ey:+6.3f},{math.degrees(eyaw):+7.1f}°) "
              f"({gx:+6.3f},{gy:+6.3f},{math.degrees(gyaw):+7.1f}°) "
              f"{dpos*100:>6.1f}cm {dyaw:>+7.1f}°")

    last = rows[-1]
    gy, gyaw = last[5], last[6]
    y_err_cm = abs(gy - BED1_Y) * 100
    yaw_err_deg = abs(math.degrees(wrap(gyaw - math.pi)))
    # 재진입 구간의 자세 스파이크(바퀴가 두둑 옆면을 타면 튄다) — 끼임 신호
    t_reentry = marks[-2][1]
    seg = [s for s in gt if s[0] >= t_reentry]
    roll_pp = (max(s[4] for s in seg) - min(s[4] for s in seg)) * 180 / math.pi if seg else float("nan")
    pitch_pp = (max(s[5] for s in seg) - min(s[5] for s in seg)) * 180 / math.pi if seg else float("nan")
    gx_end = last[4]

    print("\n    ── 재진입 결과 (지상진실) ──")
    print(f"    두둑1 중심에서 y 오차 : {y_err_cm:>6.1f} cm   (판정선 {JAM_Y_CM}cm)")
    print(f"    heading yaw 오차      : {yaw_err_deg:>6.1f} °    (판정선 {JAM_YAW_DEG}°)")
    print(f"    재진입 도달 x         : {gx_end:>6.2f} m    (목표 {X_REENTRY_END})")
    print(f"    자세 진동 roll/pitch  : {roll_pp:>6.1f}° / {pitch_pp:.1f}°  (끼임이면 튄다)")
    print(f"    odom↔GT 누적 표류     : {last[7]*100:>6.1f} cm · {last[8]:+.1f}°")

    ok_y, ok_yaw = y_err_cm < JAM_Y_CM, yaw_err_deg < JAM_YAW_DEG
    jam = gx_end > X_REENTRY_END + 0.5 or roll_pp > 8 or pitch_pp > 8
    print(f"\n    판정: y {'OK' if ok_y else 'NG'} · yaw {'OK' if ok_yaw else 'NG'} · "
          f"재진입 {'끼임/미달' if jam else '통과'}")
    if ok_y and ok_yaw and not jam:
        print("    ⟹ odom 만으로 U턴이 성립한다 — 마뉴버 구현으로 진행.")
    else:
        print("    ⟹ odom 만으로는 부족하다 — 재정렬 관측(두둑/검출 기반)이 선행돼야 한다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
