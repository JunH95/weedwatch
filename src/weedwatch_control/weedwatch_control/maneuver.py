#!/usr/bin/env python3
"""주행 마뉴버 — 헤드랜드 U턴과 그 프리미티브 (자율주행, DECISIONS 040).

두둑 사이 이동을 순간이동 치트(`set_pose`, 036)에서 **실제 주행**으로 바꾼다:
  A 두둑 밖 헤드랜드로 진출 → B 좌회전 90° → C 옆으로 pitch → D 좌회전 90° → E 옆 두둑 재진입

── 왜 위치를 휠 오도메트리로 안 잡나 (diag_uturn 실측) ──────────────────────
이 로봇은 조향축 없는 **4륜 고정 = 스키드 스티어**다. 제자리로 돌면 네 바퀴가 옆으로 긁히고
(scrub), DiffDrive 오도메트리는 바퀴 회전만 적분하므로 그걸 못 본다:

  방위 소스        회전 1회 yaw 오차   U턴 뒤 재진입 y 오차
  휠 오도메트리     26°                 170 cm      ← 두둑에 못 들어간다
  IMU + 자이로오도  0.0°                2.5 cm      ← 걸터타기 여유 11cm 안

그래서 이 모듈은 **거리는 바퀴에서, 방위는 IMU 에서** 받아 적분한다(자이로-오도메트리).
휠 오도메트리의 x,y 는 회전 중 틀어진 yaw 로 적분돼 쓸 수 없다.

── 남는 오차: 제자리 회전의 몸통 미끄러짐 ──────────────────────────────────
회전 중 몸통이 실제로 미끄러지는데(GT), 바퀴도 IMU 도 못 본다. 실측은 **계통적**이었다 —
좌회전 90°마다 로봇 기준 앞 0.27m · 오른쪽 0.26m. 재진입 y·yaw 는 판정선 안이고, 남는 건
절대 x 가 U턴당 ~0.5m 밀리는 것이다. 타격 정밀도에는 안 들어간다(검출 시점 앵커와 타격 시점
비교가 같은 오도메트리라 상수 오프셋이 상쇄된다). 두둑 진입/종료 판단에만 영향이 있다.
"""
import math
import time

# 헤드랜드 진출 여유: 로봇 대각 반지름 √((deck_len/2)²+(전폭/2)²) ≈ 0.955m.
# 두둑 끝에서 이만큼은 나가야 도는 동안 모서리가 두둑을 안 스친다.
SWING_RADIUS = 0.955
EXIT_MARGIN = 0.15

TURN_W = 0.50          # 제자리 회전 각속도 [rad/s]
TURN_LEAD = math.radians(4.0)   # 회전 관성 선행 정지 (명령 끊어도 더 돈다)
FINE_YAW = math.radians(0.5)    # 마무리 방위 허용오차
BRAKE_K = 0.40         # 제동거리 ≈ BRAKE_K·v² [m] (실측 피팅, STATUS Stage 4-3)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class GyroOdom:
    """자이로-오도메트리(+시각 오도메트리) — 각 센서를 **강한 데서만** 쓴다.

    실측(DECISIONS 041)이 역할을 정했다:

      방위          IMU        회전당 0.0° (휠 yaw 는 26° 부푼다)
      직진 거리     바퀴        0.5cm/65cm = 0.8%  (VO 는 7~13% 부족)
      회전 미끄러짐  VO         바퀴는 **0.0cm — 원리적으로 못 봄**, VO 는 86% 관측

    그래서 회전 중(|wz| > TURN_WZ)에는 VO 증분을, 그 외에는 휠 증분을 쓴다. VO 가 안 오면
    조용히 옛 동작(휠만)으로 돌아가되 vo_used 로 드러낸다.

    노드가 /odometry 와 /robot/imu 를 받을 때마다 update() 를 부른다. IMU 가 아직 없으면
    (월드에 imu-system 이 없는 경우) 휠 yaw 로 폴백하되 degraded 를 True 로 남긴다 —
    조용히 나쁜 추정으로 돌아가지 않게.
    """

    # 한 샘플 사이에 물리적으로 가능한 최대 이동 [m]. 0.2 m/s · 50Hz 면 4mm 이므로 0.5m 는
    # 아주 넉넉한 상한이다. 이걸 넘는 값은 **로봇의 움직임이 아니다** — 좀비 프로세스가 남아
    # 다른 시뮬의 /odometry 를 같이 발행하면 두 위치 사이를 오가며 매 샘플 수 미터씩 튄다
    # (2026-07-27 사용자 실행에서 추정이 −155m 로 폭발했고, 원인이 이것이었다).
    MAX_STEP = 0.5
    # 이 각속도를 넘으면 "회전 중"으로 보고 VO 를 쓴다. 제자리 회전 명령이 0.5 rad/s 이고
    # 직진 중 흔들림은 0.05 이하라 그 사이면 된다.
    TURN_WZ = 0.15
    # VO 증분의 물리적 상한 [m/샘플]. 제자리 회전 중 몸통 미끄러짐은 프레임당 1cm 안팎이고,
    # 최고 속도 0.2m/s 로 달려도 50Hz 샘플이면 4mm 다. 5cm 를 넘는 값은 **측정이 아니라 실패**다
    # (상관 피크가 엉뚱한 데 꽂힌 것). 안 거르면 추정이 폭주해 로봇이 도착했다고 착각한다 —
    # 실측으로 U턴 뒤 표류가 43.7cm → 221.9cm 로 나빠졌다(2026-07-27).
    VO_MAX_STEP = 0.05

    def __init__(self, x0=0.0, y0=0.0, yaw0=0.0):
        self.x, self.y, self.yaw = x0, y0, yaw0
        self.degraded = False
        self.rejected = 0          # 튄 샘플 수 — 0 이 아니면 환경이 오염됐다는 신호
        self.vo_used = 0           # VO 증분을 실제로 쓴 횟수 (0 이면 융합이 안 붙은 것)
        self.vo_rejected = 0       # 물리적으로 불가능해 버린 VO 증분 (상관 실패 신호)
        self.wheel_used = 0
        self._last_xy = None
        self._last_raw_yaw = None

    def update(self, odom_x, odom_y, odom_yaw, odom_vx, imu_yaw=None,
               vo=None, odom_wz=0.0):
        """센서 한 묶음으로 자세를 전진시킨다.

        vo: (전방, 좌) 이동 증분 [m] — 로봇 기준. 마지막 update 이후 쌓인 값.
        odom_wz: 각속도 [rad/s] — 회전 중인지 판정해 VO/휠 중 무엇을 믿을지 고른다.
        """
        raw = odom_yaw if imu_yaw is None else imu_yaw
        self.degraded = imu_yaw is None
        if self._last_raw_yaw is None:
            self._last_raw_yaw = raw
        else:
            self.yaw += wrap(raw - self._last_raw_yaw)
            self._last_raw_yaw = raw
        if self._last_xy is None:
            self._last_xy = (odom_x, odom_y)
            return self.x, self.y, self.yaw

        dx, dy = odom_x - self._last_xy[0], odom_y - self._last_xy[1]
        self._last_xy = (odom_x, odom_y)
        ds = math.hypot(dx, dy)
        if ds > self.MAX_STEP:
            # 로봇이 순간이동할 리 없다 → 다른 시뮬(좀비 프로세스)의 오도메트리가 섞였다.
            # 적산하면 추정이 통째로 망가지므로 버리고 센다. 조용히 흡수하지 않는다.
            self.rejected += 1
            return self.x, self.y, self.yaw

        turning = abs(odom_wz) > self.TURN_WZ
        if turning and vo is not None and math.hypot(*vo) > self.VO_MAX_STEP:
            self.vo_rejected += 1        # 상관 실패 — 버리고 휠로 간다(회전 중 휠은 0 에 가깝다)
            vo = None
        if turning and vo is not None:
            # 회전 중: 바퀴는 미끄러짐을 못 본다(0.0cm 보고). 카메라가 본 것을 쓴다.
            fwd, left = vo
            self.x += fwd * math.cos(self.yaw) - left * math.sin(self.yaw)
            self.y += fwd * math.sin(self.yaw) + left * math.cos(self.yaw)
            self.vo_used += 1
        else:
            # 직진: 바퀴가 0.8% 로 압도적이다. VO(7~13%)를 쓰면 오히려 나빠진다.
            if odom_vx < 0:
                ds = -ds
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)
            self.wheel_used += 1
        return self.x, self.y, self.yaw


class Maneuver:
    """U턴과 프리미티브. 제어 인터페이스(drive/stop)와 자세 추정을 주입받아 전송에 안 묶인다.

    drive(lin, ang) : 속도 명령 (WwControl.drive 또는 동등물)
    pose()          : (x, y, yaw) 추정 — 자이로-오도메트리. None 이면 아직 센서 없음.
    """

    def __init__(self, drive, pose, v=0.20, sleep=time.sleep, now=time.time):
        self._drive, self._pose = drive, pose
        self.v = v
        self._sleep, self._now = sleep, now

    # ── 프리미티브 ────────────────────────────────────────────────────
    def wait_pose(self, timeout=15.0):
        t0 = self._now()
        while self._now() - t0 < timeout:
            p = self._pose()
            if p is not None:
                return p
            self._sleep(0.02)
        raise RuntimeError("자세 추정이 안 옵니다 (odom/IMU 미수신)")

    def _drive_until(self, done, timeout):
        self._drive(self.v, 0.0)
        t0 = self._now()
        while self._now() - t0 < timeout:
            x, y, yaw = self.wait_pose()
            if done(x, y, yaw):
                break
            self._sleep(0.01)
        else:
            self._drive(0.0, 0.0)
            raise RuntimeError("주행 구간 시간 초과")
        self._drive(0.0, 0.0)
        self._sleep(1.2)              # 제동 미끄러짐 정착 (바퀴는 서고 몸통은 더 간다)

    def drive_to_x(self, x_target, forward=True, timeout=120.0):
        lead = BRAKE_K * self.v * self.v
        if forward:
            self._drive_until(lambda x, y, yw: x >= x_target - lead, timeout)
        else:
            self._drive_until(lambda x, y, yw: x <= x_target + lead, timeout)

    def drive_dist(self, dist, timeout=120.0):
        x0, y0, _ = self.wait_pose()
        lead = BRAKE_K * self.v * self.v
        self._drive_until(lambda x, y, yw: math.hypot(x - x0, y - y0) >= dist - lead, timeout)

    def turn_to(self, yaw_target, timeout=40.0):
        """제자리 회전 — 거친 회전 뒤 느린 마무리. 방위는 IMU 라 긁힘에 안 속는다."""
        _, _, yaw = self.wait_pose()
        sign = 1.0 if yaw_target > yaw else -1.0
        self._drive(0.0, sign * TURN_W)
        t0 = self._now()
        while self._now() - t0 < timeout:
            _, _, yaw = self.wait_pose()
            if sign * (yaw_target - yaw) <= TURN_LEAD:
                break
            self._sleep(0.01)
        self._drive(0.0, 0.0)
        self._sleep(1.0)
        for _ in range(400):
            _, _, yaw = self.wait_pose()
            err = yaw_target - yaw
            if abs(err) <= FINE_YAW:
                break
            self._drive(0.0, math.copysign(0.12, err))
            self._sleep(0.05)
        self._drive(0.0, 0.0)
        self._sleep(1.0)

    # ── U턴 ──────────────────────────────────────────────────────────
    def uturn(self, x_exit, dy, entry_x, log=None):
        """두둑 끝에서 옆 두둑으로. dy>0 이면 좌회전 U턴, dy<0 이면 우회전.

        x_exit : 헤드랜드 진출 x (두둑 끝 + SWING_RADIUS + 여유)
        dy     : 옆 두둑까지 y 이동량 (= 두둑 pitch)
        entry_x: 재진입 후 패스를 시작할 x
        """
        _, _, yaw0 = self.wait_pose()
        forward = math.cos(yaw0) > 0        # 지금 +x 로 가고 있나
        # 회전 방향은 **지금 어디를 보고 있나**에도 달렸다. +x 로 가는 중이면 +y 는 왼쪽이지만,
        # −x 로 가는 중이면 +y 는 **오른쪽**이다. dy 부호만 보면 두둑 3번째부터 반대로 돈다.
        side = math.copysign(1.0, dy) * (1.0 if forward else -1.0)

        def note(step):
            if log:
                x, y, yaw = self.wait_pose()
                log(f"U턴 {step}: 추정 ({x:+.3f}, {y:+.3f}) yaw={math.degrees(yaw):+.1f}°")

        self.drive_to_x(x_exit, forward=forward);            note("A 헤드랜드 진출")
        self.turn_to(yaw0 + side * math.pi / 2);             note("B 90°")
        self.drive_dist(abs(dy));                            note("C 옆 두둑 열")
        self.turn_to(yaw0 + side * math.pi);                 note("D 90°")
        self.drive_to_x(entry_x, forward=not forward);       note("E 재진입")
