#!/usr/bin/env python3
"""로봇 제어 기하 파라미터 — garden_geometry(단일 출처)에서 직접 뽑는다.

예전엔 coordinator 가 tools/assert_row_stamp 에서 이 상수들을 가져왔는데, 그게 WwCmd(ign-transport
직결)를 딸려와 직결 코드 삭제를 막았다. 여기서 garden_geometry(순수 기하 config)만 import 해서
그 끈을 끊는다. garden_geometry 는 ROS 패키지·물리 테스트가 공유하는 config 라 tools/ 에 남긴다.
"""
import sys

from weedwatch_control.ww_paths import tools_dir

sys.path.insert(0, tools_dir())
from garden_geometry import Garden, Portal  # noqa: E402  (순수 config — ww_cmd 안 끌어옴)

_G, _P = Garden(), Portal()

N = _P.n_tools                          # 툴 개수 (3)
TOOL_XS = _P.tool_xs()                  # 툴별 X 엇갈림 [-0.09,-0.27,-0.45]
BAND_CENTERS = _P.tool_band_centers(_G)  # 툴별 Y 밴드 중심 [-0.30,0,+0.30]
BASE_Y = 0.60                           # 로봇 spawn y (두둑 중심)

V = 0.20            # 무정차 주행 속도 [m/s] (020: ≤0.2 라야 ±2cm 창 200ms > Z 180ms)
STRIKE = -0.15      # 도구 하강 명령 [m] (두둑 충돌로 멈춤)
RAISE = 0.0         # 도구 접힘 (주행 중 두둑 안 긁게)
Z_SETTLE = 0.180    # Z 하강 정착 시간 [s] (020 실측) → 이만큼 앞서 하강 건다


def weed_tool(wy: float) -> int:
    """잡초 world y → 담당 툴 인덱스 (밴드 배정). assert_row_stamp 와 동일."""
    return _P.band_of(_G, wy - BASE_Y)
