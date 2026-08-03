"""저장소 코드에 **정의되지 않은 이름**이 없어야 한다 (Tier 1).

실제로 당했다: `tools/assert_step_b.py` 가 `USE_VO` 를 쓰는데 어디서도 대입하지 않아
`make step-b` 가 헤더를 찍다가 NameError 로 죽었다. 커밋 4867f3c 부터 그 상태였고,
그동안 "Step B 가 23cm 로 실패" 라는 숫자를 근거로 얘기하고 있었다 — 돌아가지도 않는 게이트를.

문법 검사(compile)로는 안 잡힌다. import 만으로도 안 잡힌다 — 함수 안이라 호출해야 터진다.
그래서 이름 해석을 정적으로 하는 pyflakes 를 쓰되, **F821(undefined name) 하나만** 본다.
미사용 import 같은 취향 문제로 게이트를 막지 않기 위해서다.
"""
import subprocess
import sys
from pathlib import Path

import pytest

WW = Path(__file__).resolve().parents[1]
TARGETS = ["tools", "src", "perception", "tests"]
# 남의 코드는 우리 책임이 아니다 — venv 안 표준 라이브러리는 star import 때문에 오탐이 난다.
SKIP = ("perception/condaenv/", "/build/", "/install/", "/log/", "__pycache__")


def test_no_undefined_names():
    pytest.importorskip("pyflakes")
    paths = [str(WW / t) for t in TARGETS if (WW / t).exists()]
    r = subprocess.run([sys.executable, "-m", "pyflakes", *paths],
                       capture_output=True, text=True)
    bad = [ln for ln in r.stdout.splitlines()
           if "undefined name" in ln and not any(s in ln for s in SKIP)]
    assert not bad, "정의되지 않은 이름:\n  " + "\n  ".join(bad)
