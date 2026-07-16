"""머티리얼 변환이 검은 렌더를 실제로 고치는지 검사한다. 시뮬 없이, 밀리초 안에.

이 파일이 존재하는 이유: CropCraft 출력을 그대로 Fortress 에 띄우면 정원 전체가
새까맣게 렌더링된다. 에러 메시지는 한 줄도 없다. 실측했다 — 평균 밝기 33.4, 하늘만 회색.

그리고 그걸 모르고 진행하면 **검은 실루엣으로 YOLO 를 학습시키게 된다.**
프로젝트를 죽이는 종류의 버그이고, 조용해서 더 위험하다.

진짜 원인은 <script> 가 아니라 **색의 기본값**이었다:
    sdformat 스펙에서 ambient/diffuse 의 기본값이 "0 0 0 1" = 검정이다.
    CropCraft 는 <material> 안에 <script> 만 넣고 색을 안 준다.
    gz-sim 은 script 를 못 알아듣지만 <material> 이 있으니 머티리얼을 만들고,
    색이 검정으로 잡히고, 그게 메시 자체 머티리얼을 덮는다.

그래서 <pbr> 로 바꾸는 것만으로는 부족했다 (실제로 이 실수를 했다 — 평균 33.4 → 33.3,
아무 변화 없음). 색을 명시해야 33.4 → 72.8 로 살아난다.

실행:  ./scripts/env.sh python3 -m pytest tests/ -v
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from fix_materials import convert, texture_of  # noqa: E402

# CropCraft 가 실제로 내보내는 것을 그대로 축약한 것 (models/test1/model.sdf 에서 발췌)
CROPCRAFT_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="g">
    <static>true</static>
    <link name="bed1">
      <visual name="bed1">
        <geometry><mesh><uri>model://g/meshes/bed1.obj</uri></mesh></geometry>
        <material>
          <script>
            <uri>model://g/materials/bed1.material</uri>
            <name>bed1</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# CropCraft 가 내보내는 Ogre1 머티리얼 스크립트 (Gazebo Classic 시절 형식)
OGRE_MATERIAL = """material bed1
{
  technique
  {
    pass
    {
      cull_hardware none
      cull_software none

      alpha_rejection greater 128

      texture_unit
      {
        texture maize_leaf.jpg
      }
    }
  }
}
"""


@pytest.fixture
def model(tmp_path: Path) -> Path:
    """CropCraft 출력을 흉내낸 가짜 모델 디렉토리."""
    d = tmp_path / "g"
    (d / "materials").mkdir(parents=True)
    (d / "meshes").mkdir()
    (d / "model.sdf").write_text(CROPCRAFT_SDF)
    (d / "materials" / "bed1.material").write_text(OGRE_MATERIAL)
    (d / "materials" / "maize_leaf.jpg").write_bytes(b"fake")
    return d / "model.sdf"


def test_ogre_스크립트에서_텍스처_이름을_읽는다(tmp_path):
    f = tmp_path / "x.material"
    f.write_text(OGRE_MATERIAL)
    assert texture_of(f) == "maize_leaf.jpg"


def test_변환_전에는_check_가_실패한다(model, capsys):
    """고쳐야 할 게 남아 있으면 알려줘야 한다."""
    assert convert(model, check_only=True) == 1


def test_변환_후에는_check_가_통과한다(model):
    assert convert(model) == 0
    assert convert(model, check_only=True) == 0


def test_script_가_사라진다(model):
    """gz-sim 이 <script> 를 무시하므로 남겨두면 안 된다."""
    convert(model)
    root = ET.parse(model).getroot()
    assert root.find(".//material/script") is None


def test_diffuse_가_검정이_아니다(model):
    """이게 이 파일에서 가장 중요한 테스트다.

    SDF 는 diffuse 를 안 주면 "0 0 0 1"(검정)로 기본값을 잡고, 그 검정이
    텍스처에 곱해져서 화면이 새까맣게 된다. 색을 명시하지 않으면 pbr 을 아무리
    제대로 써도 소용없다 — 실제로 그 실수를 했고 평균 밝기가 33.4 → 33.3 이었다.
    """
    convert(model)
    mat = ET.parse(model).getroot().find(".//material")
    diffuse = mat.find("diffuse")
    assert diffuse is not None, "diffuse 가 없으면 SDF 기본값 검정이 적용된다"
    assert [float(x) for x in diffuse.text.split()][:3] != [0.0, 0.0, 0.0]

    ambient = mat.find("ambient")
    assert ambient is not None, "ambient 가 없으면 그늘진 면이 완전히 검게 된다"
    assert [float(x) for x in ambient.text.split()][:3] != [0.0, 0.0, 0.0]


def test_텍스처가_albedo_map_으로_연결된다(model):
    convert(model)
    root = ET.parse(model).getroot()
    albedo = root.find(".//material/pbr/metal/albedo_map")
    assert albedo is not None
    assert albedo.text.endswith("maize_leaf.jpg")
    # model:// URI 여야 Gazebo 가 IGN_GAZEBO_RESOURCE_PATH 로 찾을 수 있다
    assert albedo.text.startswith("model://")


def test_잎이_양면으로_렌더된다(model):
    """원본 Ogre1 스크립트의 cull_hardware none 을 대신하는 것.

    없으면 잎이 한쪽에서만 보이고 반대편에서는 사라진다 — 카메라 각도에 따라
    식물이 없어지는 셈이라, 비전이 전부인 이 프로젝트에서는 치명적이다.
    """
    convert(model)
    ds = ET.parse(model).getroot().find(".//material/double_sided")
    assert ds is not None and ds.text == "true"


def test_잎은_금속이_아니다(model):
    """metalness 가 높으면 잎이 쇳덩이처럼 보인다."""
    convert(model)
    root = ET.parse(model).getroot()
    assert float(root.find(".//material/pbr/metal/metalness").text) == 0.0
    assert float(root.find(".//material/pbr/metal/roughness").text) > 0.5


def test_고칠게_없으면_조용히_통과(tmp_path):
    """이미 변환된 파일에 두 번 돌려도 안전해야 한다 (멱등성)."""
    d = tmp_path / "g"
    d.mkdir()
    (d / "model.sdf").write_text(
        '<?xml version="1.0"?><sdf version="1.7"><model name="g">'
        "<link name='l'><visual name='v'><material><diffuse>1 1 1 1</diffuse>"
        "</material></visual></link></model></sdf>"
    )
    assert convert(d / "model.sdf", check_only=True) == 0
    assert convert(d / "model.sdf") == 0
