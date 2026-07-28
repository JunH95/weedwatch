"""U턴 마뉴버의 순수 로직 단언 (Tier 1 — 시뮬 없이 산수로).

물리는 `make turn`(Tier 2)이 잰다. 여기서 잡는 건 **시뮬을 돌려도 두둑 2개짜리 밭에서는 안 드러나는**
방향 논리다: 세 번째 두둑으로 넘어갈 때 회전 방향이 뒤집히는가. 실제로 처음 구현이 여기서 틀렸다 —
dy 부호만 보고 돌아서, −x 로 달리던 로봇이 +y 두둑으로 간다며 −y 로 돌 뻔했다.

자이로-오도메트리(거리는 바퀴·방위는 IMU)도 여기서 산수로 검사한다 — IMU 가 없을 때 조용히
휠 yaw 로 폴백하지 않고 degraded 를 드러내는지 포함.
"""
import math
import sys
from pathlib import Path

import pytest

WW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WW / "src" / "weedwatch_control"))
from weedwatch_control.maneuver import GyroOdom, Maneuver, wrap  # noqa: E402


class FakeRobot:
    """명령을 받아 완벽하게 따르는 로봇 — 마뉴버의 **논리**만 시험한다(물리는 make turn)."""

    def __init__(self, x=0.0, y=0.6, yaw=0.0, dt=0.05):
        self.x, self.y, self.yaw, self.dt = x, y, yaw, dt
        self.t = 0.0
        self.lin = self.ang = 0.0
        self.mid_turn_sign = 0.0     # 첫 회전의 방향 부호 (+좌 / −우)

    def drive(self, lin, ang=0.0):
        if ang and not self.mid_turn_sign:
            self.mid_turn_sign = math.copysign(1.0, ang)
        self.lin, self.ang = lin, ang

    def sleep(self, s):              # 시계를 명령대로 전진시킨다 (벽시계 안 씀)
        steps = max(1, int(s / self.dt))
        for _ in range(steps):
            self.yaw = wrap(self.yaw + self.ang * self.dt)
            self.x += self.lin * math.cos(self.yaw) * self.dt
            self.y += self.lin * math.sin(self.yaw) * self.dt
            self.t += self.dt

    def now(self):
        return self.t

    def pose(self):
        return self.x, self.y, self.yaw


def make(robot):
    m = Maneuver(robot.drive, robot.pose, v=0.20, sleep=robot.sleep, now=robot.now)
    # wait_pose 가 now() 로 타임아웃을 재는데 sleep 이 시계를 돌리므로, 폴링 자체는 즉시 성립한다.
    return m


@pytest.mark.parametrize("yaw0,expect_side", [(0.0, +1), (math.pi, -1)])
def test_uturn_turns_toward_next_bed(yaw0, expect_side):
    """+y 두둑으로 갈 때, +x 주행 중이면 왼쪽·−x 주행 중이면 오른쪽으로 돌아야 한다."""
    r = FakeRobot(x=0.0, y=0.6, yaw=yaw0)
    m = make(r)
    x_exit = 4.4 if yaw0 == 0.0 else -1.4
    m.uturn(x_exit, dy=1.2, entry_x=(2.2 if yaw0 == 0.0 else 0.2))
    # 옆 두둑(+1.2m)에 도달했고, 방위가 반대로 뒤집혔다
    assert r.y == pytest.approx(0.6 + 1.2, abs=0.05), f"옆 두둑에 못 감: y={r.y:.3f}"
    assert abs(wrap(r.yaw - (yaw0 + math.pi))) < math.radians(2), "방위가 반대로 안 뒤집힘"
    # 중간에 어느 쪽으로 돌았나 — +x 주행이면 왼쪽(+y 가 왼쪽), −x 주행이면 오른쪽
    assert math.copysign(1, r.mid_turn_sign) == expect_side, \
        f"회전 방향이 반대다 (yaw0={math.degrees(yaw0):.0f}°)"


def test_uturn_reaches_headland_before_turning():
    """두둑 밖으로 충분히 나가서 돌아야 한다 — 안 나가면 회전 중 포드가 두둑에 낀다."""
    r = FakeRobot(x=0.0, y=0.6, yaw=0.0)
    m = make(r)
    reached = []
    m.uturn(4.4, dy=1.2, entry_x=2.2, log=lambda s: reached.append((s, r.x, r.y)))
    step_a = next(s for s in reached if s[0].startswith("U턴 A"))
    assert step_a[1] >= 4.4 - 0.05, f"헤드랜드까지 안 나가고 돌았다: x={step_a[1]:.3f}"


def test_uturn_entry_x_is_reached():
    r = FakeRobot(x=0.0, y=0.6, yaw=0.0)
    m = make(r)
    m.uturn(4.4, dy=1.2, entry_x=2.2)
    assert r.x == pytest.approx(2.2, abs=0.1), f"재진입 지점에 못 섬: x={r.x:.3f}"


def test_gyro_odom_uses_imu_heading_not_wheel_yaw():
    """휠 yaw 가 26° 틀어져도(스키드 스티어 실측) IMU 가 있으면 그쪽 방위로 적분해야 한다.

    한 샘플에 1m 를 보내면 안 된다 — 그건 점프 가드(MAX_STEP)에 걸린다. 실제 오도메트리는
    50Hz 에 4mm 씩 오므로 여기서도 잘게 나눠 보낸다.
    """
    g = GyroOdom(x0=0.0, y0=0.0, yaw0=0.0)
    g.update(0.0, 0.0, math.radians(26), 0.0, imu_yaw=0.0)       # 휠은 26°, IMU 는 0°
    for i in range(1, 11):                                        # 0.1m 씩 10번 = 1m 전진
        g.update(i * 0.1, 0.0, math.radians(26), 0.2, imu_yaw=0.0)
    assert g.x == pytest.approx(1.0, abs=1e-6)
    assert g.y == pytest.approx(0.0, abs=1e-6), "휠 yaw 로 적분했다 — IMU 를 안 씀"
    assert not g.degraded
    assert g.rejected == 0


def test_gyro_odom_rejects_teleport_jumps():
    """물리적으로 불가능한 점프는 버려야 한다 — 좀비 프로세스가 남의 오도메트리를 같이 발행하면
    두 위치 사이를 오가며 샘플당 수 미터가 들어오고, 적산하면 추정이 통째로 폭발한다
    (2026-07-27 실행에서 −155m). 버리고 **세어서** 알린다."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.02, 0.0, 0.0, 0.2, imu_yaw=0.0)      # 정상 2cm
    g.update(4.50, 0.0, 0.0, 0.2, imu_yaw=0.0)      # 4.5m 점프 = 다른 시뮬의 값
    assert g.rejected == 1
    assert g.x == pytest.approx(0.02, abs=1e-6), "점프를 적산했다 — 추정이 오염된다"


def test_gyro_odom_flags_fallback_when_imu_missing():
    """IMU 가 없으면 조용히 나빠지지 말고 degraded 로 드러내야 한다 (폴백이면 회전당 26° 오차)."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=None)
    assert g.degraded


def test_gyro_odom_distance_is_signed_by_direction():
    """후진하면 거리도 음수로 — 부호를 놓치면 뒤로 간 만큼 앞으로 갔다고 적분한다."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(-0.5, 0.0, 0.0, -0.2, imu_yaw=0.0)
    assert g.x == pytest.approx(-0.5, abs=1e-6)


# ── 융합 게이팅 (DECISIONS 041) ────────────────────────────────────────────
# 실측이 역할을 정했다: 직진 거리는 바퀴가 0.8%, 회전 미끄러짐은 바퀴가 **0.0cm**(못 봄)이고
# 카메라가 86% 본다. 그래서 회전 중에만 VO 를 쓴다. 여기서는 그 규칙이 코드에 맞게 들어갔는지
# 산수로 확인한다 — 시뮬을 돌리지 않고.

def test_fusion_uses_vo_while_turning():
    """회전 중에는 카메라가 본 이동으로 **교체**한다 — 바퀴는 미끄러짐을 원리적으로 못 본다."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0, vo=(0.02, -0.01), odom_wz=0.5)
    assert g.vo_used == 1
    assert g.x == pytest.approx(0.02, abs=1e-9)
    assert g.y == pytest.approx(-0.01, abs=1e-9)


def test_fusion_uses_wheels_when_they_agree():
    """직진에서 바퀴와 카메라가 일치하면 바퀴를 쓴다(잡음이 더 적다)."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.010, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=(0.0105, 0.0), odom_wz=0.0)
    assert g.wheel_used == 1 and g.vo_used == 0 and g.slip_events == 0
    assert g.x == pytest.approx(0.010, abs=1e-9)


def test_fusion_detects_slip_when_they_disagree():
    """바퀴가 헛돌면(카메라와 크게 어긋나면) 그 창의 이동을 카메라 값으로 교체한다 — 044 의 수리.

    Step B 실측: 흙덩이를 넘는 순간 바퀴가 1m 에 20~31cm 를 부풀렸고, 그 과대가 그대로
    타격 오차(전후 −28~−36cm)가 됐다.
    """
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    for k in range(1, 11):                       # 바퀴로 4cm (10 × 4mm)
        g.update(0.004 * k, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=None, odom_wz=0.0)
    # 카메라는 같은 창에서 2.4cm 만 갔다고 본다(60% — 신뢰 범위 안의 진짜 슬립)
    g.update(0.040, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=(0.024, 0.0), odom_wz=0.0)
    assert g.slip_events == 1 and g.vo_used == 1
    assert g.x == pytest.approx(0.024, abs=1e-9), "헛도는 바퀴를 그대로 적산했다"


def test_fusion_does_not_double_count_across_the_camera_window():
    """**이중 계산 금지** — 카메라는 200ms 치, 바퀴는 20ms 치다.

    카메라 프레임이 올 때까지 바퀴 이동을 모아뒀다가 그 창끼리 비교해야 한다. 그냥 더하면
    같은 이동을 두 번 세서 추정이 부푼다(실제로 그렇게 만들었다가 VO 47/50 샘플이 슬립으로
    잡혔다, 2026-07-27).
    """
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    for k in range(1, 6):                       # 바퀴로 5회 × 4mm = 2cm
        g.update(0.004 * k, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=None, odom_wz=0.0)
    # 카메라가 같은 창(2cm)을 봤다고 말한다 → 보정 없음, 위치는 2cm 그대로여야 한다
    g.update(0.020, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=(0.020, 0.0), odom_wz=0.0)
    assert g.slip_events == 0, "일치하는데 슬립으로 판정했다"
    assert g.x == pytest.approx(0.020, abs=1e-9), f"이중 계산 — x={g.x:.4f} (기대 0.020)"


def test_fusion_falls_back_to_wheels_without_vo():
    """VO 노드가 없으면 회전 중이라도 휠로 돈다 — 조용히 멈추지 않고 이전 동작 유지."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.01, 0.0, 0.0, 0.0, imu_yaw=0.0, vo=None, odom_wz=0.5)
    assert g.wheel_used == 1 and g.vo_used == 0


def test_fusion_rotates_vo_into_world_frame():
    """VO 는 로봇 기준(전방·좌)이라 방위로 회전시켜 누적해야 한다."""
    g = GyroOdom(yaw0=math.pi / 2)                 # +y 를 보고 있음
    g.update(0.0, 0.0, math.pi / 2, 0.0, imu_yaw=math.pi / 2)
    g.update(0.0, 0.0, math.pi / 2, 0.0, imu_yaw=math.pi / 2, vo=(0.03, 0.0), odom_wz=0.5)
    assert g.x == pytest.approx(0.0, abs=1e-9)
    assert g.y == pytest.approx(0.03, abs=1e-9), "로봇 전방(+y)이 world +y 로 안 갔다"


def test_fusion_rejects_impossible_vo_steps():
    """상관이 실패해 말도 안 되는 증분이 오면 버려야 한다.

    안 거르면 추정이 폭주하고 로봇이 "도착했다"고 착각해 일찍 멈춘다 — 실측으로 U턴 뒤 표류가
    43.7cm → 221.9cm 로 나빠졌다(2026-07-27). 회전 중 몸통 미끄러짐은 프레임당 1cm 안팎이다.
    """
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0, vo=(0.50, 0.0), odom_wz=0.5)
    assert g.vo_rejected == 1 and g.vo_used == 0
    assert g.x == pytest.approx(0.0, abs=1e-9), "불가능한 VO 를 적산했다"


# ── 방위 유지 (DECISIONS 042 2단계) ────────────────────────────────────────

def test_heading_correction_has_deadband():
    """잔오차에는 명령을 안 낸다 — 조향축 없는 로봇에서 각속도는 옆 밀림을 부른다."""
    from weedwatch_control.maneuver import heading_correction, HEADING_DEADBAND
    assert heading_correction(0.0, math.radians(1.0)) == 0.0
    assert heading_correction(0.0, -math.radians(1.0)) == 0.0
    assert heading_correction(0.0, math.radians(10.0)) > 0.0


def test_heading_correction_is_continuous_at_deadband():
    """불감대 경계에서 명령이 툭 튀면 주행이 덜컥거린다."""
    from weedwatch_control.maneuver import heading_correction, HEADING_DEADBAND
    just_out = heading_correction(0.0, HEADING_DEADBAND + math.radians(0.01))
    assert abs(just_out) < math.radians(1.0), "불감대를 막 벗어날 때 명령이 튄다"


def test_heading_correction_saturates():
    """큰 오차에도 상한을 넘지 않는다 — 뱀처럼 흔들리지 않게."""
    from weedwatch_control.maneuver import heading_correction, HEADING_MAX_WZ
    assert heading_correction(0.0, math.pi / 2) == pytest.approx(HEADING_MAX_WZ)
    assert heading_correction(0.0, -math.pi / 2) == pytest.approx(-HEADING_MAX_WZ)


def test_fusion_ignores_missing_vo_rather_than_reading_zero():
    """카메라 증분이 **안 온** 주기를 '정지를 봤다'로 읽으면 안 된다.

    odom 50Hz · 카메라 5Hz 라 대부분의 주기에는 새 증분이 없다. 그때 (0,0) 을 넘기면 융합이
    매번 슬립으로 오판한다(실측: 매끈한 밭에서 VO 사용 534회). 없으면 None 이어야 한다.
    """
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.004, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=None, odom_wz=0.0)   # 증분 없음
    assert g.slip_events == 0 and g.wheel_used == 1
    assert g.x == pytest.approx(0.004, abs=1e-9)


def test_fusion_rejects_vo_that_says_almost_nothing():
    """카메라가 "거의 안 갔다"고 하면 그건 슬립이 아니라 **상관 실패**로 본다.

    실제 슬립은 10~60% 지 100% 가 아니다. 이 가드가 없으면 실패한 VO(≈0)가 멀쩡한 바퀴 값을
    덮어써 추정이 사실상 멈추고, 로봇은 "아직 도착 안 했다"며 밭 밖으로 달린다
    (실측: x −0.3~1.7m 밭에서 **x=21m 까지 갔다**).
    """
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    for k in range(1, 6):
        g.update(0.004 * k, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=None, odom_wz=0.0)   # 바퀴 2cm
    g.update(0.020, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=(0.001, 0.0), odom_wz=0.0)   # 카메라 0.1cm(5%)
    assert g.vo_rejected == 1 and g.slip_events == 0
    assert g.x == pytest.approx(0.020, abs=1e-9), "실패한 VO 가 바퀴를 덮어썼다"


def test_fusion_accepts_plausible_slip_within_trust_band():
    """신뢰 범위(40~160%) 안의 불일치는 진짜 슬립으로 받아들인다."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    for k in range(1, 11):
        g.update(0.004 * k, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=None, odom_wz=0.0)   # 바퀴 4cm
    g.update(0.040, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=(0.024, 0.0), odom_wz=0.0)   # 카메라 2.4cm(60%)
    assert g.slip_events == 1 and g.vo_rejected == 0
    assert g.x == pytest.approx(0.024, abs=1e-9)
