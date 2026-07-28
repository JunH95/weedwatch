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

# 직진 중 방위 유지 (DECISIONS 042 2단계). 매끈한 밭에서는 필요 없었다 — 명령만 주면 곧게 갔다.
# 현실적인 밭에서는 다르다: 바퀴가 흙덩이를 타는 순간 좌우 견인이 어긋나 **yaw 가 튄다**.
# 실측(개발 밭, 2m 패스): pitch +2.7° 와 동시에 yaw −11.9°, 패스 끝까지 누적 −12.4° · y 이탈 17.4cm.
# 개루프로 달리면 그 뒤 U턴·재진입이 전부 틀어진 방위 위에서 일어난다.
# IMU 가 방위를 0.0° 로 주므로(040) 비례 보정만으로 잡을 수 있다 — 카메라가 필요한 건 **옆 위치**지
# 방위가 아니다.
# **불감대가 필요하다**: 이 로봇은 조향축이 없어서 각속도 명령 자체가 바퀴를 긁고(scrub) 몸통을
# 옆으로 민다(회전 90°당 옆 0.26m, 040). 그래서 잔오차까지 계속 고치면 방위는 붙들리는데 옆으로
# 스멀스멀 밀린다 — 실측으로 매끈한 밭 U턴 진입 오차가 0.3~2.8cm 에서 5.0cm(임계값)로 나빠졌다.
# 큰 교란(흙덩이에 채임)만 되돌리고 잔오차는 놔둔다.
HEADING_DEADBAND = math.radians(2.0)
HEADING_KP = 1.0        # [rad/s per rad]
HEADING_MAX_WZ = 0.25   # 보정 각속도 상한 — 이보다 크면 주행이 뱀처럼 흔들린다


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def heading_correction(yaw: float, target: float) -> float:
    """방위 오차를 각속도 명령으로 — 직진 중 IMU 로 방위를 붙든다.

    불감대 안이면 **0 을 낸다**. 조향축 없는 로봇에서 각속도 명령은 공짜가 아니다(긁힘 → 옆 밀림).
    """
    err = wrap(target - yaw)
    if abs(err) < HEADING_DEADBAND:
        return 0.0
    err -= math.copysign(HEADING_DEADBAND, err)      # 불감대 경계에서 연속이 되게
    return max(-HEADING_MAX_WZ, min(HEADING_MAX_WZ, HEADING_KP * err))


class GyroOdom:
    """자이로-오도메트리(+시각 오도메트리) — 각 센서를 **강한 데서만** 쓴다.

    실측(DECISIONS 041)이 역할을 정했다:

      방위          IMU        회전당 0.0° (휠 yaw 는 26° 부푼다)
      직진 거리     바퀴        0.5cm/65cm = 0.8%  (VO 는 7~13% 부족)
      회전 미끄러짐  VO         바퀴는 **0.0cm — 원리적으로 못 봄**, VO 는 86% 관측

    **044 이후 규칙이 바뀌었다.** 그 0.8% 는 매끈한 밭 값이고, 현실 밭에서 바퀴는 9.2%(흙덩이를
    넘는 순간엔 국소적으로 25~65%)다. 우위가 사라졌으므로 "직진은 무조건 바퀴"가 아니라
    **불일치를 슬립 신호로 쓴다**: 바퀴가 갔다는 거리와 카메라가 본 거리가 크게 어긋나면
    그 순간 바퀴가 헛돈 것이므로 카메라를 믿는다. 회전 중에는 여전히 무조건 카메라다(바퀴는 0).

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
    # 슬립 판정 임계 — **같은 창(window) 안의 이동을 비교한 뒤** 적용한다.
    # 카메라는 5Hz(창 200ms ≈ 4cm), 바퀴는 50Hz(20ms ≈ 4mm)다. 이 둘을 그대로 비교하면 항상
    # 어긋나 보이고, 그 주기에 카메라 증분(창 전체)까지 더하면 **이중 계산**이 된다 —
    # 실제로 그렇게 만들었다가 VO 47/50 샘플이 슬립으로 잡히고 추정이 부풀었다(2026-07-27).
    # 그래서 카메라 프레임이 올 때까지 바퀴 이동을 모아뒀다가 한 번에 비교한다.
    SLIP_REL = 0.20        # 창 안 이동의 20% 이상 어긋나면 슬립
    SLIP_ABS = 0.010       # 그리고 최소 1cm 는 어긋나야 (작은 창의 잡음 배제)
    # **카메라를 믿는 범위**. 실제 슬립은 10~60% 지 100% 가 아니다 — 바퀴가 0.2m/s 로 도는데
    # 몸이 전혀 안 가는 일은 흙에서 드물다. 카메라가 창 이동의 40% 미만/160% 초과를 말하면
    # 그건 슬립이 아니라 **상관 실패**로 보고 바퀴를 지킨다.
    # 안 그러면 실패한 VO(≈0)가 멀쩡한 바퀴 값을 덮어써 추정이 사실상 멈춘다 — 실측으로
    # 로봇이 밭(x −0.3~1.7m)을 벗어나 **x=21m 까지 달렸다**(추정상 목표에 영영 못 닿아서).
    VO_TRUST_LO, VO_TRUST_HI = 0.4, 1.6

    def __init__(self, x0=0.0, y0=0.0, yaw0=0.0):
        self.x, self.y, self.yaw = x0, y0, yaw0
        self.degraded = False
        self.rejected = 0          # 튄 샘플 수 — 0 이 아니면 환경이 오염됐다는 신호
        self.vo_used = 0           # VO 증분을 실제로 쓴 횟수 (0 이면 융합이 안 붙은 것)
        self.vo_rejected = 0       # 물리적으로 불가능해 버린 VO 증분 (상관 실패 신호)
        self.wheel_used = 0
        self.slip_events = 0       # 직진 중 바퀴-카메라 불일치가 커서 카메라로 보정한 횟수
        self._wheel_win = [0.0, 0.0]   # 카메라 프레임 사이에 바퀴로 간 world 변위(창)
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
        if vo is not None and math.hypot(*vo) > self.VO_MAX_STEP:
            self.vo_rejected += 1        # 상관 실패 — 버리고 휠로 간다
            vo = None

        # 1) 바퀴 증분은 **항상** 적산한다. 그리고 카메라 창을 위해 따로 모아둔다.
        if odom_vx < 0:
            ds = -ds
        wdx = ds * math.cos(self.yaw)
        wdy = ds * math.sin(self.yaw)
        self.x += wdx
        self.y += wdy
        self._wheel_win[0] += wdx
        self._wheel_win[1] += wdy
        self.wheel_used += 1

        # 2) 카메라 프레임이 오면 **같은 창**의 바퀴 변위와 비교해 보정한다.
        if vo is not None:
            fwd, left = vo
            vdx = fwd * math.cos(self.yaw) - left * math.sin(self.yaw)
            vdy = fwd * math.sin(self.yaw) + left * math.cos(self.yaw)
            wdx_w, wdy_w = self._wheel_win
            gap = math.hypot(vdx - wdx_w, vdy - wdy_w)
            win = max(math.hypot(vdx, vdy), math.hypot(wdx_w, wdy_w))
            wheel_d, vo_d = math.hypot(wdx_w, wdy_w), math.hypot(vdx, vdy)
            # 직진에서 카메라 값이 신뢰 범위를 벗어나면 상관 실패로 본다(회전은 바퀴가 0 이라 예외).
            if not turning and wheel_d > 0.005 and not (
                    self.VO_TRUST_LO * wheel_d <= vo_d <= self.VO_TRUST_HI * wheel_d):
                self.vo_rejected += 1
                self._wheel_win = [0.0, 0.0]
                return self.x, self.y, self.yaw
            if turning or (gap > self.SLIP_ABS and gap > self.SLIP_REL * win):
                # 회전이거나 큰 불일치 → 그 창의 이동을 카메라 값으로 **교체**한다(더하지 않는다).
                self.x += vdx - wdx_w
                self.y += vdy - wdy_w
                self.vo_used += 1
                if not turning:
                    self.slip_events += 1
            self._wheel_win = [0.0, 0.0]
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

    # 한 구간에서 이보다 더 갔는데도 목표에 못 닿으면 **센서를 의심한다**. 밭 길이가 수 m 이므로
    # 8m 는 어떤 정상 구간보다 길다. 이게 없으면 추정이 안 늘 때 로봇이 밭 밖으로 달려나간다
    # (실측: x=21m 까지 갔다 — 시간 초과로 겨우 멈췄다).
    MAX_LEG = 8.0

    def _drive_until(self, done, timeout, hold_heading=True):
        """직진 구간. **방위를 붙들고** 간다 — 흙덩이에 채이면 개루프는 그대로 돌아간 채 달린다."""
        x0, y0, yaw0 = self.wait_pose()
        self._drive(self.v, 0.0)
        t0 = self._now()
        while self._now() - t0 < timeout:
            x, y, yaw = self.wait_pose()
            if done(x, y, yaw):
                break
            if math.hypot(x - x0, y - y0) > self.MAX_LEG:
                self._drive(0.0, 0.0)
                raise RuntimeError(
                    f"한 구간에서 {self.MAX_LEG}m 넘게 갔는데 목표에 못 닿았다 — 추정이 이동을 "
                    f"못 따라오고 있다(융합·센서 의심). 로봇을 더 달리게 두지 않는다.")
            self._drive(self.v, heading_correction(yaw, yaw0) if hold_heading else 0.0)
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
