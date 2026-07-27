#!/usr/bin/env python3
"""저장소 루트 찾기 — 환경변수에 기대지 않는다.

ROS 패키지들은 `tools/garden_geometry.py`(로봇 치수 단일 출처)와 `tools/oracle.py`(채점 정답)를
읽어야 한다. 예전엔 `WW_ROOT` 환경변수로 그 위치를 받았는데, 그걸 넣어주는 건 `make` 뿐이라
**사람이 `ros2 launch` 를 직접 쓰면 노드가 import 에서 죽었다**(2026-07-27 사용자 실행에서 발생):

    ModuleNotFoundError: No module named 'garden_geometry'

에이전트 경로(make)에서만 되고 사람 경로에서는 안 되는 상태 — 교차검증의 전제를 깨는 종류의
버그다. 그래서 환경변수는 **선택적 재정의**로만 두고, 없으면 파일 위치에서 위로 올라가며 찾는다.
설치 트리(`install/...`)도 저장소 안에 있으므로 같은 방식으로 루트에 닿는다.
"""
import os
from pathlib import Path

MARKER = Path("tools") / "garden_geometry.py"


def find_repo_root(start: Path | None = None) -> Path:
    """저장소 루트. 우선순위: WW_ROOT 환경변수 → 파일 위치에서 위로 → 현재 작업 디렉토리에서 위로."""
    env = os.environ.get("WW_ROOT")
    if env and (Path(env) / MARKER).exists():
        return Path(env)

    candidates = []
    here = (start or Path(__file__)).resolve()
    candidates += list(here.parents)
    candidates += [Path.cwd().resolve(), *Path.cwd().resolve().parents]
    for d in candidates:
        if (d / MARKER).exists():
            return d

    raise RuntimeError(
        f"weedwatch 저장소 루트를 못 찾았습니다({MARKER} 기준). "
        f"저장소 밖에서 실행했다면 WW_ROOT=<저장소 경로> 를 주세요.")


def tools_dir() -> str:
    """`tools/`(순수 config·기하) 경로. sys.path 에 넣어 garden_geometry·oracle 을 import 한다."""
    return str(find_repo_root() / "tools")
