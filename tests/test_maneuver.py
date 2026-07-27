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
    """회전 중에는 VO 증분을 쓴다 — 바퀴는 미끄러짐을 못 보므로."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0, vo=(0.02, -0.01), odom_wz=0.5)
    assert g.vo_used == 1 and g.wheel_used == 0
    assert g.x == pytest.approx(0.02, abs=1e-9)
    assert g.y == pytest.approx(-0.01, abs=1e-9)


def test_fusion_uses_wheels_while_driving_straight():
    """직진에서는 바퀴가 압도적이다 — VO 가 와도 안 쓴다."""
    g = GyroOdom()
    g.update(0.0, 0.0, 0.0, 0.0, imu_yaw=0.0)
    g.update(0.10, 0.0, 0.0, 0.2, imu_yaw=0.0, vo=(0.30, 0.0), odom_wz=0.0)
    assert g.wheel_used == 1 and g.vo_used == 0
    assert g.x == pytest.approx(0.10, abs=1e-9), "VO 를 직진에 썼다 — 7~13% 오차를 들여온다"


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
