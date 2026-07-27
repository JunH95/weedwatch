"""사람이 `ros2 launch` 를 직접 써도 되는지 단언 (Tier 1 — 시뮬 없이).

**이 테스트가 없어서 사용자가 대신 밟았다**(2026-07-27). `make` 는 `WW_ROOT=<repo>` 를 넣어주는데
사람이 `ros2 launch weedwatch_bringup skeleton.launch.py` 를 직접 치면 그게 없어서 노드가 죽었다:

    ModuleNotFoundError: No module named 'garden_geometry'

앞서 만든 환경 테스트(test_env_entrypoints)는 **환경변수 값만** 비교해서 이걸 못 잡았다.
값이 같은 것과 **실제로 import 가 되는 것**은 다른 주장이다. 그래서 여기서는 설치 트리의 모듈을
**환경변수 없이, 저장소 밖 디렉토리에서** 실제로 import 하고 런치 파일도 실제로 만들어 본다.
"""
import os
import subprocess
import sys
from pathlib import Path

import pytest

WW = Path(__file__).resolve().parents[1]
INSTALL = WW / "install" / "setup.bash"

pytestmark = pytest.mark.skipif(not INSTALL.exists(),
                                reason="colcon 빌드 전 (make ros-build)")

def _run(code: str):
    """WW_ROOT 를 빼고, 저장소 밖(/tmp)에서 실행 — 사람이 아무 데서나 치는 상황."""
    script = Path("/tmp/ww_native_check.py")
    script.write_text(code)
    env = dict(os.environ)
    env.pop("WW_ROOT", None)
    return subprocess.run(
        ["bash", "-c", f"source {WW}/scripts/ros_env.sh && source {INSTALL} && "
                       f"unset WW_ROOT && cd /tmp && python3 {script}"],
        capture_output=True, text=True, timeout=120, env=env)


@pytest.mark.parametrize("mod", [
    "weedwatch_control.params",
    "weedwatch_control.maneuver",
    "weedwatch_control.control_node",
    "weedwatch_sim.field",
    "weedwatch_coordinator.coordinator_node",
])
def test_module_imports_without_env(mod):
    """WW_ROOT 없이, 저장소 밖에서도 import 돼야 한다 (tools/ 를 스스로 찾는다)."""
    r = _run(f"import {mod}; print(\"ok\")")
    assert "ok" in r.stdout, f"{mod} import 실패:\n{r.stdout[-500:]}\n{r.stderr[-800:]}"


def test_launch_description_builds_without_env():
    """설치된 런치 파일이 월드 경로를 환경변수 없이 풀어내고, 그 파일이 실제로 있어야 한다."""
    code = (
        "import importlib.util, os\n"
        "from ament_index_python.packages import get_package_share_directory\n"
        "share = get_package_share_directory('weedwatch_bringup')\n"
        "path = os.path.join(share, 'launch', 'skeleton.launch.py')\n"
        "spec = importlib.util.spec_from_file_location('skel', path)\n"
        "m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)\n"
        "ld = m.generate_launch_description()\n"
        "assert os.path.exists(m.WORLD), m.WORLD\n"
        "print('ok', len(ld.entities), m.WORLD)\n")
    r = _run(code)
    assert "ok" in r.stdout, f"{r.stdout[-400:]}\n{r.stderr[-800:]}"


def test_repo_root_found_from_outside():
    """find_repo_root 가 저장소 밖 cwd 에서도 설치 트리 위치로 루트를 찾아야 한다."""
    r = _run("from weedwatch_control.ww_paths import find_repo_root; "
             "print(find_repo_root())")
    assert str(WW) in r.stdout, f"루트를 잘못 찾음: {r.stdout.strip()!r}"
