"""포탈형 로봇 몸체를 Blender 로 생성한다. GUI 없이, 파라메트릭하게.

── 이 스크립트가 하는 일 ────────────────────────────────────────────────
tools/garden_geometry.py 의 Portal/Garden 치수를 읽어서 로봇 3D 모델을 만든다.
숫자 하나 바꾸면 로봇 전체가 다시 생성된다 — CAD 커널 없이 파라메트릭 엄밀함.
(docs/DECISIONS.md 012)

실루엣은 AVO/Aigen 인용 (docs/DECISIONS.md 010):
"바퀴 달린 테이블 + 그 아래 매달린 도구." 태양광 데크가 상판이고, 다리·바퀴·
캐리지·도구·배터리가 전부 그 아래 매달린다. 상판 위로 튀어나오는 건 없다.

── 실행 ────────────────────────────────────────────────────────────────
    blender --background --python tools/robot_body.py -- render
        → artifacts/robot/*.png (여러 각도, Claude 가 보고 판단)
    blender --background --python tools/robot_body.py -- export
        → models/weedwatch_robot/*.obj (Gazebo 용, PBR)

── 왜 Blender 인가 (다른 CAD 대신) ─────────────────────────────────────
헤드리스에서 "코드→렌더→보고→수정" 루프를 혼자 완결하는 유일한 도구라서.
Cycles OPTIX 로 GPU 렌더가 이미 되고(tools/blender_gpu.py), OBJ+MTL(Fortress 가
PBR 을 받는 유일한 포맷)을 직접 뽑는다. FreeCAD/build123d 는 렌더러가 없어서
어차피 Blender 로 넘겨야 한다.
"""

import sys
from math import cos, pi, sin
from pathlib import Path

import bpy
from mathutils import Vector

# tools/ 를 path 에 넣어 치수 단일 출처를 읽는다.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from garden_geometry import Garden, Portal  # noqa: E402

G = Garden()
P = Portal()

# ── 색 팔레트 (docs/DECISIONS.md 010, 012) ───────────────────────────────
# 움직이는 부품(캐리지·도구)만 고채도 주황 → 데모 스크린샷에서 초점이 거기로 간다.
# 나머지는 차분하게: 짙은 남색 패널 + 알루미늄 프레임 + 검정 타이어.
PALETTE = {
    "cell": (0.04, 0.06, 0.16),      # 태양광 셀 — 짙은 남색 (약간 광택)
    "body": (0.82, 0.83, 0.85),      # 몸통 케이스 — 밝은 흰회색 (AVO 본체)
    "frame": (0.62, 0.64, 0.67),     # 알루미늄 프레임
    "glass": (0.7, 0.8, 0.95),       # 유리 커버 (반투명)
    "wheel": (0.06, 0.06, 0.07),     # 고무 타이어
    "hub": (0.5, 0.52, 0.55),        # 바퀴 허브 (금속)
    "carriage": (0.85, 0.35, 0.05),  # 주황 — 움직이는 부품 (초점)
    "tool": (0.80, 0.30, 0.04),      # 주황 — 도구
    "battery": (0.15, 0.16, 0.18),   # 짙은 회색
    "camera": (0.03, 0.03, 0.03),    # 검정 렌즈
    "led": (1.0, 0.95, 0.7),         # 발광 (emission)
}


def reset_scene():
    """빈 씬에서 시작. --factory-startup 안 써도 확실하게 비운다."""
    bpy.ops.wm.read_factory_settings(use_empty=True)


def material(name: str, rgb, *, metal=0.0, rough=0.5, emit=False, transmit=0.0, coat=0.0):
    """PBR 머티리얼 하나. emit=발광(LED), transmit=반투명(유리), coat=클리어코트(도장)."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    if emit:
        bsdf.inputs["Emission Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 4.0
    if transmit > 0:
        bsdf.inputs["Transmission Weight"].default_value = transmit
        bsdf.inputs["IOR"].default_value = 1.45  # 유리
    if coat > 0:
        # 클리어코트 — 자동차 도장 느낌. 흰 케이스가 "플라스틱 장난감"에서
        # "도장된 기계"로 바뀐다.
        bsdf.inputs["Coat Weight"].default_value = coat
        bsdf.inputs["Coat Roughness"].default_value = 0.1
    return m


def box(name, size, loc, mat, bevel=0.006):
    """모서리를 살짝 깎은 상자. bevel 이 "기계처럼" 보이게 하는 핵심."""
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.object
    o.name = name
    o.scale = Vector(size)  # size = (x,y,z) 전체 길이
    bpy.ops.object.transform_apply(scale=True)
    if bevel > 0:
        mod = o.modifiers.new("bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 3
    o.data.materials.append(mat)
    _shade_smooth(o)
    return o


def cyl(name, radius, length, loc, mat, axis="z", verts=48):
    """실린더 (바퀴·막대). axis 는 길이 방향."""
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=length, location=loc, vertices=verts)
    o = bpy.context.object
    o.name = name
    if axis == "y":
        o.rotation_euler[0] = 1.5708
    elif axis == "x":
        o.rotation_euler[1] = 1.5708
    bpy.ops.object.transform_apply(rotation=True)
    o.data.materials.append(mat)
    _shade_smooth(o)
    return o


def make_ag_wheel(name, radius, width, loc, mat_tire, mat_hub, outboard,
                  n_lugs=18, n_bolts=6, chevron=True):
    """농업용 러그 타이어. 책상 캐스터가 아니라 흙 파는 바퀴.

    ── 왜 지오메트리로 러그를 만드나 (범프맵이 아니라) ──────────────────
    책상 캐스터의 정체는 "완벽한 원형 실루엣"이다. 범프/노멀 맵은 셰이딩만
    건드리지 실루엣(윤곽선)을 못 바꾼다. side/front 뷰에서 바퀴 윤곽이 매끈한
    원이면 여전히 캐스터다. 그래서 러그를 실제 상자로 만들어 윤곽을 깬다.
    상자 18개 × 4바퀴 = 72개지만 Cycles 128spp 엔 아무것도 아니다.

    축(axle)은 +Y. outboard(±1)는 림이 어느 쪽 고랑 벽을 보는가.
    """
    cx, cy, cz = loc
    parts = []

    # 1) 타이어 카커스 — 최종 지름보다 좁게. 러그가 나머지를 채운다.
    r_car = radius * 0.86
    tire = cyl(f"{name}_tire", r_car, width, loc, mat_tire, axis="y", verts=64)
    bv = tire.modifiers.new("shoulder", "BEVEL")   # 양쪽 숄더를 둥글게 (타이어 크라운)
    bv.width = min(width * 0.32, r_car * 0.4)
    bv.segments = 6
    bv.profile = 0.5  # 0.5 = 둥근
    parts.append(tire)

    # 2) 러그 — 삼각함수 루프로 방사형 상자. 겹치는 솔리드는 Cycles 에서 합쳐진다.
    # 러그 바깥 끝이 정확히 radius 까지만 닿게 (넘으면 바퀴가 뜬다). 카커스에 겹쳐 붙는다.
    lug_h = radius - r_car + 0.03      # 카커스에 겹치는 만큼 + 밖으로 나가는 만큼
    r_lug = radius - lug_h / 2         # 러그 중심 원 = 바깥 끝이 radius 가 되게
    for k in range(n_lugs):
        a = 2 * pi * k / n_lugs
        p = (cx + r_lug * cos(a), cy, cz + r_lug * sin(a))
        bpy.ops.mesh.primitive_cube_add(size=1, location=p)
        lug = bpy.context.object
        lug.name = f"{name}_lug{k}"
        lug.scale = (0.020, width * 0.70, lug_h)   # 접선, 축, 반지름 방향
        roll = ((-1) ** k) * 0.22 if chevron else 0.0  # R-1 셰브런 패턴
        lug.rotation_euler = (roll, pi / 2 - a, 0)     # 로컬 +Z → 방사 바깥
        bpy.ops.object.transform_apply(scale=True, rotation=True)
        lug.data.materials.append(mat_tire)
        _shade_smooth(lug)
        parts.append(lug)

    # 3) 림(오목) + 허브모터 캡. 림이 타이어보다 좁아 고무 사이드월이 테두리를 만든다.
    parts.append(cyl(f"{name}_rim", r_car * 0.60, width * 0.72, loc, mat_hub, axis="y", verts=48))
    face_y = cy + outboard * (width / 2 + 0.008)
    parts.append(cyl(f"{name}_cap", r_car * 0.22, 0.05,
                     (cx, face_y, cz), mat_hub, axis="y", verts=24))

    # 4) 볼트 서클 (바깥쪽 면). 축이 Y 라 볼트는 X-Z 평면에.
    for b in range(n_bolts):
        a = 2 * pi * b / n_bolts
        parts.append(cyl(f"{name}_bolt{b}", 0.006, 0.02,
                         (cx + r_car * 0.40 * cos(a), face_y, cz + r_car * 0.40 * sin(a)),
                         mat_hub, axis="y", verts=8))
    return parts


def wedge_body(name, size, loc, mat, front_slope=0.35, seam=True):
    """쐐기형 몸통 — 직육면체가 아니라 앞뒤가 경사진 동체. "책상 탈출"의 핵심.

    직육면체 상자는 아무리 커도 "책상"으로 읽힌다. AVO/Aigen 이 "기계"로 보이는 건
    앞으로 기울어진 경사면과 패널 분할선 때문이다. bmesh 로 윗면 앞뒤 변을 안쪽으로
    당겨 사다리꼴 단면(위가 좁은 쐐기)을 만들고, 옆면에 얕은 패널 홈을 판다.

    front_slope: 윗변이 아랫변보다 앞뒤로 얼마나 안쪽인가 (0~1, 길이 대비).
    """
    import bmesh

    lx, ly, lz = size
    x, y, z = loc
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.object
    o.name = name
    o.scale = (lx, ly, lz)
    bpy.ops.object.transform_apply(scale=True)

    # bmesh 로 윗면 4정점을 안쪽으로 당겨 쐐기 단면을 만든다.
    me = o.data
    bm = bmesh.new()
    bm.from_mesh(me)
    inset_x = lx * front_slope / 2
    for v in bm.verts:
        if v.co.z > z:  # 윗면 정점만
            # 앞(+x)·뒤(-x) 로 당겨 위를 좁힌다
            v.co.x -= (1 if v.co.x > x else -1) * inset_x
    bm.to_mesh(me)
    bm.free()

    # 모서리 살짝 깎기
    bv = o.modifiers.new("bevel", "BEVEL")
    bv.width = 0.012
    bv.segments = 3
    o.data.materials.append(mat)
    _shade_smooth(o)

    parts = [o]

    # 패널 분할선: 옆면에 얕게 파인 홈(살짝 안쪽·짙은 색 상자)으로 표현.
    if seam:
        seam_mat = material(f"{name}_seam", (0.3, 0.31, 0.33), metal=0.5, rough=0.6)
        for sy in (-1, 1):
            groove = box(f"{name}_seam_{sy}", (lx * 0.7, 0.004, lz * 0.5),
                         (x, y + sy * (ly / 2 - 0.001), z), seam_mat, bevel=0)
            parts.append(groove)
    return parts


def solar_deck(name, size, loc, frame_mat, cell_mat, glass_mat, bevel=0.01):
    """태양광 데크 = 프레임 테두리 + 셀 격자. 밋밋한 판 대신 '진짜 패널'.

    구조: (1) 알루미늄 프레임(살짝 큰 상자) (2) 그 위에 셀 격자(약간 안쪽·위)
    셀은 개별 상자로 깔면 무거우니, 큰 판 하나에 격자 무늬를 노멀/색으로 넣지 않고
    실제 얕은 홈(inset)으로 나눈 셀 타일 몇 개만 얹어 격자감을 준다.
    """
    lx, ly, lz = size
    x, y, z = loc
    parts = []

    # 프레임 (데크 전체 크기, 살짝 두껍게)
    fr = box(f"{name}_frame", (lx, ly, lz), (x, y, z), frame_mat, bevel=bevel)
    parts.append(fr)

    # 셀 타일: 프레임 안쪽에 격자로. gap 만큼 프레임이 사이로 보인다.
    margin = 0.03      # 프레임 테두리 폭
    gap = 0.008        # 셀 사이 틈
    ncol, nrow = 6, 3  # 셀 격자 (가로 x 세로)
    inner_x = lx - 2 * margin
    inner_y = ly - 2 * margin
    cell_x = (inner_x - (ncol - 1) * gap) / ncol
    cell_y = (inner_y - (nrow - 1) * gap) / nrow
    top_z = z + lz / 2 + 0.002  # 프레임 윗면 살짝 위
    for i in range(ncol):
        for j in range(nrow):
            cx = x - inner_x / 2 + cell_x / 2 + i * (cell_x + gap)
            cy = y - inner_y / 2 + cell_y / 2 + j * (cell_y + gap)
            c = box(f"{name}_cell_{i}_{j}", (cell_x, cell_y, 0.004),
                    (cx, cy, top_z), cell_mat, bevel=0.002)
            parts.append(c)

    # 반투명 유리 커버 (셀 위에 얇게) — 패널 특유의 유리 반사를 준다.
    glass = box(f"{name}_glass", (lx - 2 * margin + 0.01, ly - 2 * margin + 0.01, 0.003),
                (x, y, top_z + 0.004), glass_mat, bevel=0.002)
    parts.append(glass)
    return parts


def _shade_smooth(o):
    """부드러운 셰이딩 + 각진 데는 각지게 (weighted normal). 렌더가 매끈해진다."""
    for poly in o.data.polygons:
        poly.use_smooth = True
    mod = o.modifiers.new("wn", "WEIGHTED_NORMAL")
    mod.keep_sharp = True


def build():
    """로봇을 조립한다. 모든 좌표는 고랑 바닥 z=0 기준.

    좌표계: x=주행방향(앞뒤), y=좌우(두둑을 가로지름), z=위.
    로봇은 두둑 중심(y=0)에 서고, 바퀴는 양쪽 고랑(y=±track/2)에.
    """
    reset_scene()
    mats = {k: material(k, v, **_mat_opts(k)) for k, v in PALETTE.items()}

    deck_w = P.deck_width(G)
    deck_z = P.deck_top_z() - P.deck_thickness / 2  # 데크 중심 높이
    half_track = P.track(G) / 2

    parts = []

    # ── 1. 태양광 데크 (상판) — 로봇의 지붕이자 실루엣의 핵심 ──────────
    # 밋밋한 판 대신 프레임 테두리 + 셀 격자 + 유리 커버 → 진짜 패널.
    parts += solar_deck("deck", (P.deck_length, deck_w, P.deck_thickness),
                        (0, 0, deck_z), mats["frame"], mats["cell"], mats["glass"])

    # ── 2. 몸통 케이스 (데크 바로 아래, 꽉 찬 본체) — AVO 의 핵심 ──────
    # 핵심: 몸통을 위에 작게 두고 다리를 길게 뽑으면 "책상"이 된다. 대신 몸통을
    # 바퀴 바로 위까지 크게 내린 "사이드 포드" 두 덩어리로 만들고 가운데(두둑 위)는
    # 비워 터널을 만든다. 다리가 거의 안 보이고 바퀴가 포드에 직접 붙은 "덩어리 기계".
    body_l = P.deck_length - 2 * P.body_inset
    deck_bottom = P.deck_top_z() - P.deck_thickness
    pod_top = deck_bottom
    pod_bottom = pod_top - P.pod_drop
    pod_z = (pod_top + pod_bottom) / 2

    # 얇은 상판 데크 스킨 (두 포드를 잇는 얕은 판, 데크 바로 아래)
    skin_h = 0.06
    parts += wedge_body("body_skin", (body_l, P.deck_width(G) - 2 * P.body_inset, skin_h),
                        (0, 0, pod_top - skin_h / 2), mats["body"])

    # 양쪽 사이드 포드 (바퀴 위로 크게 내려온 덩어리) — 쐐기형
    for sy in (-1, 1):
        pod_y = sy * half_track
        parts += wedge_body(f"pod_{sy}", (body_l, P.pod_width, P.pod_drop),
                            (0, pod_y, pod_z), mats["body"], front_slope=0.28)

    body_bottom = pod_bottom  # 캐리지 빔이 여기(상판 스킨 아래)에 붙는다

    # ── 3. 짧은 다리 (포드 아랫면 → 바퀴 축) + 러그 바퀴 ──────────────
    # 다리가 이제 짧다 (포드가 바퀴 가까이까지 내려왔으니). 거의 안 보인다.
    stub_len = pod_bottom - P.wheel_dia / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            wx = sx * (body_l / 2 - 0.10)   # 포드 앞뒤 끝쯤
            wy = sy * half_track
            if stub_len > 0.01:
                parts.append(cyl(f"leg_{sx}_{sy}", P.leg_width / 2, stub_len,
                                 (wx, wy, P.wheel_dia / 2 + stub_len / 2),
                                 mats["frame"], axis="z", verts=12))
            parts += make_ag_wheel(f"wheel_{sx}_{sy}", P.wheel_dia / 2, P.wheel_width,
                                   (wx, wy, P.wheel_dia / 2), mats["wheel"], mats["hub"],
                                   outboard=sy)

    # ── 4. 빔 (몸통 아랫면에 붙어 좌우로 뻗음, 캐리지 레일) ────────────
    # 몸통 폭보다 넓게 뻗어 두둑 전체를 캐리지가 훑을 수 있게.
    beam_z = body_bottom - P.beam_height / 2
    parts.append(box("beam", (0.10, P.track(G) * 0.85, P.beam_height),
                     (0, 0, beam_z), mats["frame"], bevel=0.01))
    beam_bottom = beam_z - P.beam_height / 2

    # ── 5. Y 캐리지 (빔에 매달려 좌우로) — 주황, 움직이는 부품 ─────────
    # 빔 아랫면에 딱 붙는다. 실제 주행 시 Y축으로 ±carriage_travel 움직임.
    carriage_y = 0.0
    carriage_top = beam_bottom
    carriage_z = carriage_top - P.carriage_size / 2
    parts.append(box("carriage", (P.carriage_size * 1.4, P.carriage_size, P.carriage_size),
                     (0, carriage_y, carriage_z), mats["carriage"], bevel=0.008))
    carriage_bottom = carriage_z - P.carriage_size / 2

    # ── 6. 카메라 마운트 + 카메라 (캐리지 아래 앞쪽) ───────────────────
    # 캐리지에서 아래로 내려오는 마운트에 카메라가 붙는다. 허공에 안 뜬다.
    cam_x = 0.09
    mount_len = 0.05
    parts.append(box("cam_mount", (0.03, 0.03, mount_len),
                     (cam_x, carriage_y, carriage_bottom - mount_len / 2), mats["frame"]))
    cam_z = carriage_bottom - mount_len - 0.02
    parts.append(box("camera", (0.05, 0.05, 0.04),
                     (cam_x, carriage_y, cam_z), mats["camera"], bevel=0.008))
    # LED 링 (카메라 둘레) — 조명 터널 (010)
    parts.append(cyl("led", 0.04, 0.008, (cam_x, carriage_y, cam_z - 0.026),
                     mats["led"], axis="z"))

    # ── 7. Z 막대 (캐리지 아래 뒤쪽 = 점 타격 도구) — 주황 ────────────
    # 캐리지 아랫면에서 시작해 아래로. "앞에서 보고(카메라) 뒤에서 친다(도구)".
    tool_x = -0.09
    rod_len = P.z_travel * 0.55
    parts.append(cyl("tool_rod", P.tool_rod_dia / 2, rod_len,
                     (tool_x, carriage_y, carriage_bottom - rod_len / 2),
                     mats["tool"], axis="z"))

    # ── 8. 배터리 (몸통 케이스 안 — 이제 뜰 데가 없다) ────────────────
    # 몸통 내부에 들어간다. 앞쪽·낮게 (무게중심). 케이스에 반쯤 박힌 걸로 표현.
    batt_l = P.battery_size * 1.3
    parts.append(box("battery", (batt_l, P.track(G) * 0.45, P.battery_size),
                     (body_l / 2 - batt_l / 2 - 0.03, 0, body_bottom + P.battery_size / 2 + 0.005),
                     mats["battery"], bevel=0.006))

    # 전부 하나로 합쳐 이동/렌더 편하게
    for p in parts:
        p.select_set(True)
    bpy.context.view_layer.objects.active = parts[0]
    return parts


def _mat_opts(key):
    """머티리얼별 metal/rough/emit/transmit."""
    if key == "cell":
        return dict(metal=0.4, rough=0.2)    # 태양광 셀 — 광택
    if key == "glass":
        return dict(metal=0.0, rough=0.05, transmit=0.85)  # 유리 커버 — 반투명
    if key == "body":
        return dict(metal=0.1, rough=0.4, coat=1.0)  # 몸통 — 도장 + 클리어코트
    if key in ("frame", "hub"):
        return dict(metal=0.8, rough=0.4)    # 알루미늄
    if key == "wheel":
        return dict(metal=0.0, rough=0.9)    # 고무 — 거침
    if key == "led":
        return dict(emit=True)
    if key in ("carriage", "tool"):
        return dict(metal=0.1, rough=0.5)
    return dict(metal=0.3, rough=0.6)


def setup_render(w=1200, h=900):
    """Cycles OPTIX GPU 렌더 + 조명 + 바닥."""
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "GPU"
    scene.cycles.samples = 128
    scene.cycles.transmission_bounces = 8  # 유리 커버가 제대로 보이게
    scene.render.resolution_x = w
    scene.render.resolution_y = h
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "AgX"  # 톤 매핑 — 하이라이트가 덜 타서 사진 같음

    # GPU 켜기 (안 켜면 조용히 CPU — tools/blender_gpu.py 와 같은 함정)
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for backend in ("OPTIX", "CUDA"):
        try:
            prefs.compute_device_type = backend
            prefs.refresh_devices()
            for d in prefs.devices:
                d.use = d.type == backend
            if prefs.has_active_device():
                break
        except TypeError:
            continue

    # 흙색 바닥 (밭 느낌)
    bpy.ops.mesh.primitive_plane_add(size=20, location=(0, 0, 0))
    ground = bpy.context.object
    gm = material("ground", (0.20, 0.15, 0.10), rough=0.95)
    ground.data.materials.append(gm)

    # 3점 조명 — 점토 렌더에서 제품 사진으로 가는 가장 큰 차이.
    # 키(주광) + 필(그림자 완화) + 림(바닥에서 로봇 분리 = "프로" 느낌의 핵심).
    bpy.ops.object.light_add(type="SUN", location=(3, -2, 6))  # 키
    key = bpy.context.object
    key.data.energy = 3.5
    key.data.angle = 0.12
    key.rotation_euler = (0.5, 0.2, 0.4)

    bpy.ops.object.light_add(type="AREA", location=(-2.5, 1.5, 1.2))  # 필
    fill = bpy.context.object
    fill.data.energy = 120
    fill.data.size = 3.0
    fill.rotation_euler = (1.2, 0, -2.2)

    bpy.ops.object.light_add(type="AREA", location=(-1.0, 2.5, 1.7))  # 림/백
    rim = bpy.context.object
    rim.data.energy = 220
    rim.data.size = 1.5
    rim.rotation_euler = (1.0, 0, 3.3)

    # use_empty=True 로 씬을 비우면 World 도 없어진다. 다시 만든다.
    world = bpy.data.worlds.get("World") or bpy.data.worlds.new("World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.6, 0.7, 0.85, 1.0)  # 하늘색
    bg.inputs["Strength"].default_value = 0.5


def render_views(outdir: Path):
    """여러 각도에서 렌더. Claude 가 이걸 보고 형태를 판단한다."""
    outdir.mkdir(parents=True, exist_ok=True)
    scene = bpy.context.scene

    # 카메라 — 긴 렌즈(85mm)가 제품 사진처럼 납작하게 잡는다. 넓은 렌즈는 왜곡돼서
    # 아마추어 스냅샷처럼 보인다. 그래서 카메라를 멀리 두고 렌즈를 길게.
    bpy.ops.object.camera_add()
    cam = bpy.context.object
    cam.data.lens = 85  # mm — 제품 사진용 망원
    scene.camera = cam

    # 로봇 실제 중심 높이에 겨눈다. 데크가 z≈0.7 이라 이전의 0.35 는 너무 낮아
    # 바퀴가 원근으로 떠 보였다. 데크 절반 높이쯤을 본다.
    target = Vector((0, 0, P.deck_top_z() / 2))
    # 렌즈를 길게 했으니 카메라를 더 멀리 (거리 ~2배).
    views = {
        "hero": Vector((3.8, -3.4, 2.4)),      # 3/4 앞, 약간 위에서
        "side": Vector((0.05, -4.8, 1.2)),     # 옆 (걸터탄 실루엣)
        "front": Vector((5.2, 0.05, 1.1)),     # 앞 (포탈 아치)
        "top": Vector((0.8, -0.8, 5.2)),       # 위 (데크)
    }
    for name, pos in views.items():
        cam.location = pos
        direction = target - pos
        cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = str(outdir / f"robot_{name}.png")
        bpy.ops.render.render(write_still=True)
        print(f"RENDERED {name}")


def export_obj(outdir: Path):
    """Gazebo 용 OBJ+MTL export (Fortress 가 PBR 을 받는 유일한 포맷)."""
    outdir.mkdir(parents=True, exist_ok=True)
    # 로봇 파트만 선택 (바닥·조명·카메라 제외)
    bpy.ops.object.select_all(action="DESELECT")
    for o in bpy.data.objects:
        if o.type == "MESH" and o.name != "Plane":
            o.select_set(True)
    path = outdir / "weedwatch_robot.obj"
    bpy.ops.wm.obj_export(
        filepath=str(path),
        export_selected_objects=True,
        export_materials=True,
        export_pbr_extensions=True,
        path_mode="COPY",
        up_axis="Z",
        forward_axis="Y",
    )
    print(f"EXPORTED {path}")


def dump_bboxes() -> dict:
    """각 부품의 실제 월드 bounding box를 뽑는다.

    ⚠️ o.location 을 쓰면 안 된다 — box()가 transform_apply(scale) 를 하므로
    location 은 항상 (0,0,0)에 남고 지오메트리만 옮겨진다. bounding box 를 봐야 한다.

    이게 "눈이 원근에 속는" 문제의 해법이다: 렌더만 보면 배터리가 로봇 밖에
    있는 것처럼 보이지만(원근 겹침), 실제 좌표는 데크 아래에 있다. 산수로 확인한다.
    """
    from mathutils import Vector

    out = {}
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        ws = [o.matrix_world @ Vector(c) for c in o.bound_box]
        out[o.name] = {
            "cx": sum(v.x for v in ws) / 8,
            "cy": sum(v.y for v in ws) / 8,
            "cz": sum(v.z for v in ws) / 8,
            "zmin": min(v.z for v in ws),
            "zmax": max(v.z for v in ws),
            "xmin": min(v.x for v in ws), "xmax": max(v.x for v in ws),
            "ymin": min(v.y for v in ws), "ymax": max(v.y for v in ws),
        }
    return out


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["render"]
    mode = argv[0]

    build()
    root = Path(__file__).resolve().parents[1]

    if mode == "render":
        setup_render()
        render_views(root / "artifacts" / "robot")
    elif mode == "export":
        export_obj(root / "models" / "weedwatch_robot")
    elif mode == "bboxes":
        # 테스트가 파싱할 JSON 을 stdout 에 찍는다.
        import json
        print("BBOXES_JSON", json.dumps(dump_bboxes()))
    else:
        print(f"모르는 모드: {mode} (render | export | bboxes)", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
