"""docs/spec.html 이 살아있는 문서인지 단언 — 사람이 읽는 종합본의 부패를 기계로 막는다.

사용자는 이 프로젝트를 spec.html 로만 본다. 그런데 코드가 바뀌어도 문서는 조용히 낡는다 —
실제로 ROS 이관에서 `make field-run`·`make field-multi` 를 지웠는데 문서엔 그대로 남아,
문서만 보는 사람이 **없는 명령을 치게** 돼 있었다(2026-07-27 발견).

여기서 단언하는 것 (문장 표현이 아니라 검증 가능한 사실만):
  1. spec.html 이 적은 모든 `make X` 가 Makefile 에 실존하는가
  2. spec.html 이 적은 모든 `pytest <경로>` 가 실존하는가
  3. 테스트 개수·결정 기록 건수 표기가 실제와 맞는가

문체·구조 규율(지금 상태만 쓰기, 200자 넘는 산문 쪼개기)은 기계로 못 재므로 CLAUDE.md 규율로 남긴다.
"""
import re
import subprocess
import sys
from pathlib import Path

import pytest

WW = Path(__file__).resolve().parents[1]
SPEC = WW / "docs" / "spec.html"
MAKEFILE = WW / "Makefile"
DECISIONS = WW / "docs" / "DECISIONS.md"

_spec = SPEC.read_text(encoding="utf-8")


def _make_targets() -> set[str]:
    return set(re.findall(r"^([a-zA-Z0-9_-]+):", MAKEFILE.read_text(encoding="utf-8"), re.M))


@pytest.mark.parametrize("cmd", sorted(set(re.findall(r"<code>make ([a-z0-9-]+)</code>", _spec))))
def test_spec_make_command_exists(cmd):
    """문서가 안내하는 make 명령이 실제로 있어야 한다 (죽은 명령 = 독자가 막힌다)."""
    assert cmd in _make_targets(), f"spec.html 이 안내하는 `make {cmd}` 가 Makefile 에 없다"


@pytest.mark.parametrize("path", sorted(set(re.findall(r"<code>pytest ([^<\s]+)", _spec))))
def test_spec_pytest_path_exists(path):
    assert (WW / path).exists(), f"spec.html 이 안내하는 `pytest {path}` 가 없다"


def test_spec_test_count_matches():
    """"make test · N개" 표기가 실제 수집 개수와 같아야 한다."""
    m = re.search(r"make test · (\d+)개", _spec)
    assert m, "spec.html 검증 섹션에 테스트 개수 표기가 없다"
    out = subprocess.run([sys.executable, "-m", "pytest", "tests", "--collect-only", "-q",
                          "-p", "no:cacheprovider"], cwd=WW, capture_output=True, text=True).stdout
    got = re.search(r"(\d+) tests? collected", out)
    assert got, f"수집 개수를 못 읽음:\n{out[-400:]}"
    assert int(m.group(1)) == int(got.group(1)), \
        f"spec.html 은 {m.group(1)}개라 적었는데 실제 {got.group(1)}개"


def test_spec_decision_count_matches():
    n = len(re.findall(r"^## \d+\.", DECISIONS.read_text(encoding="utf-8"), re.M))
    m = re.search(r"DECISIONS\.md</code>\((\d+)건\)", _spec)
    assert m, "spec.html 부록에 결정 기록 건수 표기가 없다"
    assert int(m.group(1)) == n, f"spec.html 은 {m.group(1)}건이라 적었는데 실제 {n}건"
