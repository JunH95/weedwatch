"""두 진입점이 같은 환경을 만드는지 단언 (Tier 1 — 시뮬 없이).

이 저장소엔 진입점이 둘이다:
  · `./scripts/env.sh <cmd>`        에이전트용 래퍼 (Bash 호출 간 상태가 안 남아서 매번 자기완결)
  · `source scripts/ros_env.sh`     사람용 — 셸을 ROS 작업 상태로 만들고 평범하게 ros2 를 쓴다

둘이 어긋나면 "에이전트는 되는데 사람은 안 되는"(또는 그 반대) 상태가 생기고, 그건 교차검증
(에이전트=수치, 사람=화면)의 전제를 깬다. 그래서 환경 설정은 ros_env.sh 한 곳에만 두고,
여기서 **두 경로가 같은 값을 내는지** 기계로 확인한다.

느린 것 같지만 bash 두 번 = 수십 ms 다. 시뮬도 GPU 도 안 쓴다.
"""
import subprocess
import sys
from pathlib import Path

import pytest

WW = Path(__file__).resolve().parents[1]
KEYS = ("PYTHONNOUSERSITE", "__EGL_VENDOR_LIBRARY_FILENAMES", "IGN_GAZEBO_RENDER_ENGINE",
        "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "IGN_GAZEBO_RESOURCE_PATH", "AMENT_PREFIX_PATH")
PRINT = "; ".join(f'echo "{k}=${{{k}}}"' for k in KEYS)


def _env_via_wrapper() -> dict:
    r = subprocess.run([str(WW / "scripts" / "env.sh"), "bash", "-c", PRINT],
                       capture_output=True, text=True, cwd=WW, timeout=60)
    return dict(line.split("=", 1) for line in r.stdout.strip().splitlines() if "=" in line)


def _env_via_source() -> dict:
    r = subprocess.run(["bash", "-c", f"source {WW}/scripts/ros_env.sh && {PRINT}"],
                       capture_output=True, text=True, cwd=WW, timeout=60)
    return dict(line.split("=", 1) for line in r.stdout.strip().splitlines() if "=" in line)


@pytest.fixture(scope="module")
def envs():
    return _env_via_wrapper(), _env_via_source()


@pytest.mark.parametrize("key", KEYS)
def test_both_entrypoints_agree(key, envs):
    wrapper, sourced = envs
    assert wrapper.get(key) == sourced.get(key), (
        f"{key} 가 다르다 — 에이전트 경로 {wrapper.get(key)!r} vs 사람 경로 {sourced.get(key)!r}")


def test_egl_pinned_to_nvidia(envs):
    """EGL 을 안 고정하면 인텔 내장 그래픽을 잡아 카메라가 검게 나오거나 100배 느려진다."""
    for e in envs:
        assert "10_nvidia.json" in e.get("__EGL_VENDOR_LIBRARY_FILENAMES", "")


def test_resource_path_has_project_assets(envs):
    """worlds/·models/ 가 안 잡히면 Gazebo 가 두둑·로봇 모델을 못 찾는다."""
    for e in envs:
        rp = e.get("IGN_GAZEBO_RESOURCE_PATH", "")
        assert "/worlds" in rp and "/models" in rp


def test_python_is_ros_python():
    """사람 경로에서도 python3 가 3.10(ROS 용)이어야 한다 — conda 3.13 이면 rclpy 가 깨진다."""
    r = subprocess.run(["bash", "-c",
                        f"source {WW}/scripts/ros_env.sh && python3 -c "
                        f"'import sys; print(sys.version_info[:2])'"],
                       capture_output=True, text=True, cwd=WW, timeout=60)
    assert "(3, 10)" in r.stdout, f"사람 경로 파이썬이 3.10 이 아니다: {r.stdout.strip()!r}"
