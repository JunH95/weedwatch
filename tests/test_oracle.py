"""오라클이 정답을 제대로 읽는지 검사한다. 시뮬 없이, 밀리초 안에.

오라클은 이 프로젝트의 심장이다. "잡초 위에 정확히 선다"를 채점하려면 잡초가
**진짜로** 어디 있는지 알아야 하고, 그게 없으면 로봇에게 "너 잘했니?"를 묻는 셈이다.

가장 중요한 테스트는 `test_잡초를_weeds_블록에_선언하면_좌표가_없다` 다.
그게 이 프로젝트가 밟을 뻔한 함정이고, 조용히 실패하기 때문이다 —
weeds: 에 선언하면 밭은 멀쩡히 생성되고 잡초도 화면에 보이는데, 채점만 못 한다.

실행:  ./scripts/env.sh python3 -m pytest tests/ -v
"""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from oracle import TARGET_PREFIX, Garden, Plant, load  # noqa: E402


def make_description(beds: list[dict], weeds: list[dict] | None = None, seed: int = 42) -> dict:
    """CropCraft 의 field_description 출력을 흉내낸다.

    실물 구조 그대로다 (models/oracle_test.json 에서 확인):
      config.beds[i].name  ← 설정에서의 이름이 살아남는다
      field.beds[i]        ← 같은 순서로 좌표가 들어있다
    """
    return {
        "config": {
            "seed": seed,
            "beds": [{"name": b["name"], "plant_type": b["type"]} for b in beds],
            "weeds": weeds or [],
        },
        "field": {
            "beds": [
                {
                    "rows": [
                        {
                            "crops": [
                                {
                                    "type": b["type"],
                                    "x": p[0],
                                    "y": p[1],
                                    "z": 0.0,
                                    "height": p[2],
                                    "width": 0.1,
                                }
                                for p in b["plants"]
                            ]
                        }
                    ]
                }
                for b in beds
            ],
            "leaf_area": 1.0,
        },
    }


@pytest.fixture
def garden_file(tmp_path) -> Path:
    d = make_description(
        beds=[
            {"name": "lettuce", "type": "bean", "plants": [(0.0, 0.0, 0.21), (0.2, 0.0, 0.20)]},
            {"name": "target_polygonum", "type": "polygonum", "plants": [(0.1, 0.1, 0.07)]},
            {"name": "target_portulaca", "type": "portulaca", "plants": [(0.3, 0.3, 0.02)]},
        ]
    )
    p = tmp_path / "g.json"
    p.write_text(json.dumps(d))
    return p


# ── 기본 동작 ────────────────────────────────────────────────────────────


def test_작물과_잡초를_가른다(garden_file):
    g = load(garden_file)
    assert len(g.crops) == 2
    assert len(g.weeds) == 2
    assert {p.species for p in g.crops} == {"bean"}
    assert {p.species for p in g.weeds} == {"polygonum", "portulaca"}


def test_시드를_읽는다(garden_file):
    assert load(garden_file).seed == 42


def test_좌표가_나온다(garden_file):
    """이게 프로젝트의 성립 조건이다. 좌표가 없으면 채점을 못 한다."""
    g = load(garden_file)
    poly = next(p for p in g.weeds if p.species == "polygonum")
    assert poly.xy == (0.1, 0.1)
    assert poly.height == 0.07


# ── 함정 방지 (여기가 핵심) ──────────────────────────────────────────────


def test_잡초를_weeds_블록에_선언하면_좌표가_없다(tmp_path):
    """이 프로젝트가 밟을 뻔한 함정을 코드로 고정한다.

    CropCraft 의 weeds: 블록은 Blender 지오메트리 노드로 잡초를 뿌린다.
    좌표가 파이썬에 존재한 적이 없어서 description.json 에 안 나온다.
    밭은 멀쩡히 생성되고 화면에도 잡초가 보인다 — 채점만 조용히 못 한다.

    그래서 채점 대상은 반드시 beds: 에 target_ 이름으로 선언해야 한다.
    """
    d = make_description(
        beds=[{"name": "lettuce", "type": "bean", "plants": [(0.0, 0.0, 0.21)]}],
        weeds=[{"name": "clutter", "plant_type": "polygonum", "density": 5.0}],
    )
    p = tmp_path / "g.json"
    p.write_text(json.dumps(d))

    g = load(p)
    assert len(g.crops) == 1
    assert len(g.weeds) == 0, (
        "weeds: 블록의 잡초는 좌표가 없어야 정상이다. "
        "만약 여기 잡히면 CropCraft 가 바뀐 것이고, 그건 좋은 소식이다."
    )


def test_target_접두사가_없으면_작물로_센다(tmp_path):
    """종이 아니라 이름으로 판별한다.

    '포르툴라카는 잡초'라고 박아버리면 '작물/잡초는 설정이 정한다'는 발견 자체를 버린다.
    같은 종이 herb garden 에서는 작물일 수도 있다 — 잡초는 '내가 안 심은 식물'이지
    특정 종이 아니다.
    """
    d = make_description(
        beds=[{"name": "portulaca_as_crop", "type": "portulaca", "plants": [(0.0, 0.0, 0.02)]}]
    )
    p = tmp_path / "g.json"
    p.write_text(json.dumps(d))
    g = load(p)
    assert len(g.weeds) == 0
    assert len(g.crops) == 1
    assert g.crops[0].species == "portulaca"


def test_순서가_어긋나면_시끄럽게_실패한다(tmp_path):
    """config.beds 와 field.beds 를 zip 으로 묶으므로 순서가 생명이다.

    CropCraft 가 순서를 바꾸면 잡초를 작물로, 작물을 잡초로 채점하게 된다.
    그건 조용히 틀린 결과를 내는 최악의 실패라서, 시끄럽게 죽어야 한다.
    """
    d = make_description(
        beds=[{"name": "lettuce", "type": "bean", "plants": [(0.0, 0.0, 0.2)]}]
    )
    d["config"]["beds"][0]["plant_type"] = "maize"  # 좌표는 bean 인데 설정은 maize
    p = tmp_path / "g.json"
    p.write_text(json.dumps(d))
    with pytest.raises(ValueError, match="순서가 안 맞습니다"):
        load(p)


def test_개수가_다르면_시끄럽게_실패한다(tmp_path):
    d = make_description(
        beds=[{"name": "lettuce", "type": "bean", "plants": [(0.0, 0.0, 0.2)]}]
    )
    d["config"]["beds"].append({"name": "ghost", "plant_type": "maize"})
    p = tmp_path / "g.json"
    p.write_text(json.dumps(d))
    with pytest.raises(ValueError, match="개수가 다릅니다"):
        load(p)


# ── 채점에 쓰는 것들 ─────────────────────────────────────────────────────


def test_작물까지의_거리(garden_file):
    """잡초가 작물에 붙어 있을수록 어렵다 — 잘못 건드리면 작물이 죽는다.

    거리별 recall 곡선이 키 규칙이 어디서 죽는지를 보여준다.
    """
    g = load(garden_file)
    poly = next(p for p in g.weeds if p.species == "polygonum")
    # (0.1, 0.1) 에서 가장 가까운 작물은 (0.0, 0.0) → 거리 sqrt(0.02) ≈ 0.1414
    assert g.nearest_crop_distance(poly) == pytest.approx(0.1414, abs=1e-3)


def test_작물이_없으면_거리가_무한대(tmp_path):
    d = make_description(
        beds=[{"name": "target_x", "type": "polygonum", "plants": [(0.0, 0.0, 0.07)]}]
    )
    p = tmp_path / "g.json"
    p.write_text(json.dumps(d))
    g = load(p)
    assert g.nearest_crop_distance(g.weeds[0]) == float("inf")


# ── 실험이 실제로 성립하는지 ─────────────────────────────────────────────


def test_Tertill_키규칙이_직립형에_지고_포복형에_이긴다(garden_file):
    """실험 설계가 성립하는지를 코드로 고정한다.

    Tertill 특허 US10888045: 잡초 센서 1인치(2.54cm), 작물 센서 1.5인치(3.81cm).
    규칙 원문: "tall objects are considered crops, short objects are weeds"

    마디풀(6~8cm)은 규칙을 이기고 살아남는다 → 규칙 실패
    쇠비름(1~2.6cm)은 제대로 잡힌다        → 규칙 성공 (대조군)

    **대조군이 없으면 실험이 조작으로 보인다.** 모든 잡초가 규칙을 이기는 표는
    아무도 안 믿는다. 지는 칸이 있어야 이기는 칸을 믿는다.
    """
    TERTILL_WEED_SENSOR = 0.0254  # 1 inch, 특허 명시
    g = load(garden_file)

    poly = next(p for p in g.weeds if p.species == "polygonum")
    portu = next(p for p in g.weeds if p.species == "portulaca")

    assert poly.height > TERTILL_WEED_SENSOR, "마디풀이 규칙을 이겨야 실험이 성립한다"
    assert portu.height < TERTILL_WEED_SENSOR, "쇠비름은 규칙이 이겨야 대조군이 된다"


def test_작물은_규칙에_안_걸린다(garden_file):
    """키 규칙의 공정한 평가: 성숙한 작물에는 오검출이 없다.

    이걸 인정해야 표를 믿는다. Tertill 은 '성숙한 작물 + 어린 잡초' 전제에서
    설계됐고(특허가 그렇게 말한다), 그 전제 안에서는 실제로 작동한다.
    """
    g = load(garden_file)
    for c in g.crops:
        assert c.height > 0.0254
