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
    "accent": (0.22, 0.52, 0.20),    # 초록 악센트 (ecoRobotix/AVO 흰+초록 투톤)
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


def ring(name, major_r, minor_r, loc, mat, verts=32):
    """토러스(도넛) — LED 링. 평평한 z 링, 가운데 구멍으로 카메라가 내려다본다."""
    bpy.ops.mesh.primitive_torus_add(major_radius=major_r, minor_radius=minor_r, location=loc,
                                     major_segments=verts, minor_segments=8)
    o = bpy.context.object
    o.name = name
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


def fairing(name, size, loc, mat, top_taper=0.14, bot_taper=0.30, outer_slant=0.30, outer_sign=1):
    """각진 사이드 바디(모노코크 페어링) — "책상 다리" 대신 갠트리 섀시.

    측면(x-z)을 8각형처럼 깎는다: 윗변 앞뒤로 좁히고(top_taper), 아랫변 앞뒤 더 좁혀
    (bot_taper) 쐐기 실루엣. 바깥(outer) 아랫면을 안으로 당겨(outer_slant) 바퀴 위로
    비스듬히 덮는 펜더면. Naio Dino/farm-ng 처럼 각진 기계로 읽히게.
    """
    import bmesh

    lx, ly, lz = size
    x, y, z = loc
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.object
    o.name = name
    o.scale = (lx, ly, lz)
    bpy.ops.object.transform_apply(scale=True)

    me = o.data
    bm = bmesh.new()
    bm.from_mesh(me)
    for v in bm.verts:
        top = v.co.z > z
        # 앞뒤(x) 모서리 깎기 → 8각 실루엣
        dx = -(1 if v.co.x > x else -1) * lx * (top_taper if top else bot_taper) / 2
        v.co.x += dx
        # 바깥 아랫면을 안으로 → 바퀴 덮는 비스듬한 펜더면
        if (not top) and (v.co.y - y) * outer_sign > 0:
            v.co.y -= outer_sign * ly * outer_slant
    bm.to_mesh(me)
    bm.free()

    bv = o.modifiers.new("bevel", "BEVEL")
    bv.width = 0.02
    bv.segments = 3
    o.data.materials.append(mat)
    _shade_smooth(o)
    return o


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

    # 양쪽 사이드 페어링 (각진 모노코크 — 바퀴를 몸통 아래로 tuck, "책상 다리" 제거).
    # 바퀴 바로 위(0.24)부터 데크 아래(pod_top)까지 크게 덮고, 바깥으로 데크 끝까지 내밀어
    # 바퀴를 overhang. 안쪽 면은 캐리지 행정(±0.45+0.05) 밖(y≥0.52)에 둔다.
    fair_bot = P.wheel_dia + 0.02        # ≈0.24, 바퀴 top(0.22) 바로 위
    fair_top = pod_top                    # 0.68
    fair_z = (fair_bot + fair_top) / 2
    fair_h = fair_top - fair_bot
    fair_w = 0.18
    fair_y = 0.61                         # 안쪽 0.52(캐리지 clear) / 바깥 0.70(데크 끝·바퀴 overhang)
    for sy in (-1, 1):
        parts.append(fairing(f"pod_{sy}", (body_l, fair_w, fair_h),
                             (0, sy * fair_y, fair_z), mats["body"], outer_sign=sy))
        # 초록 하부 밴드 (흰-상/초록-하 투톤, 페어링 면을 따라감)
        band_h = fair_h * 0.4
        parts.append(fairing(f"skirt_{sy}", (body_l * 0.99, fair_w + 0.004, band_h),
                             (0, sy * fair_y, fair_bot + band_h / 2), mats["accent"], outer_sign=sy))

    # 데크 앞 초록 트림 (제품 느낌 + 방향성)
    parts.append(box("deck_trim", (0.03, deck_w * 0.98, P.deck_thickness * 1.5),
                     (P.deck_length / 2 - 0.01, 0, deck_z), mats["accent"], bevel=0.006))

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

    # ── 4. 빔 (터널 천장 = clearance 높이, 멀티툴 갠트리 레일) ──────────────
    # 빔은 clearance(고랑 바닥~빔 아랫면=0.60) 높이의 **터널 천장**. 예전엔 pod 아랫면 밑에
    # 뒀다가 캐리지·도구가 바퀴 밑으로 매달려 로봇이 캐리지로 서는 diff-drive 버그를 냈다
    # (회귀 가드: test_urdf 비-바퀴 충돌 z>0). deck_top_z() 도 빔 아랫면=clearance 를 전제한다.
    # 멀티툴: 3개 툴이 X 로 엇갈려 매달리므로 빔을 툴 X 범위만큼 넓혀 갠트리 platen 처럼 만든다.
    beam_z = P.clearance + P.beam_height / 2
    txs = P.tool_xs()
    band_centers = P.tool_band_centers(G)
    beam_cx = (min(txs) + max(txs)) / 2
    beam_lx = (max(txs) - min(txs)) + P.carriage_size * 1.4 + 0.06   # 툴 X 범위 + 캐리지 + 여유
    parts.append(box("beam", (beam_lx, P.track(G) * 0.85, P.beam_height),
                     (beam_cx, 0, beam_z), mats["frame"], bevel=0.01))
    beam_bottom = beam_z - P.beam_height / 2

    # ── 5. 하방 카메라 (base 전방 팔에 고정 — 멀티툴이라 캐리지에 못 붙임) ──────
    # 캐리지가 3개라 "카메라를 캐리지에 붙여 툴이 항상 같은 픽셀"(DECISIONS 006)이 불가능하다.
    # base 전방 팔에 올려 두둑 폭 전체를 내려다본다(고정 카메라 + 다중 툴 = 실제 다중툴 기계,
    # Andela/ecoRobotix). 툴 팁은 FK(base GT + carriage_i + tool_i)로 구하므로 픽셀 고정 없이도
    # 단언이 성립한다. 높이·X 는 garden_geometry 단일 출처(camera_x, camera_z). LED 는 렌즈 둘레 링.
    cam_x = P.camera_x
    cam_z = P.camera_z()                 # ≈0.58 = 빔 바로 아래, 두둑(0.25) 위 ~0.33m
    arm_x0 = beam_cx + beam_lx / 2       # 빔 앞면에서 팔이 앞으로 뻗어나감
    parts.append(box("cam_arm", (cam_x - arm_x0, 0.03, 0.03),
                     ((arm_x0 + cam_x) / 2, 0.0, cam_z + 0.012), mats["frame"]))
    parts.append(box("camera", (0.05, 0.05, 0.035), (cam_x, 0.0, cam_z),
                     mats["camera"], bevel=0.006))
    parts.append(ring("led", 0.045, 0.007, (cam_x, 0.0, cam_z - 0.022), mats["led"]))

    # ── 6. N개 Y 캐리지 + Z 점타격 툴 (독립 액추에이터. DECISIONS 020) ─────────
    # 각 툴: 자기 밴드(90/N cm) 중심에 서서 짧게 ±tool_band_half Y 훑고, Z 리드스크류로 내려찍는다.
    # X 로 엇갈려(txs) 있어 인접 툴 Y 범위가 겹치는 순간에도 캐리지끼리 안 부딪힌다.
    # "앞에서 보고(카메라) 뒤에서 친다(툴)" — 툴이 전부 카메라 뒤(음수 X)에 있다.
    carriage_top = beam_bottom
    carriage_z = carriage_top - P.carriage_size / 2
    carriage_bottom = carriage_z - P.carriage_size / 2
    rod_len = P.tool_rod_len  # 단일 출처 — make_urdf 충돌·관성도 같은 값을 읽는다
    for i, (cx, cy) in enumerate(zip(txs, band_centers)):
        parts.append(box(f"carriage{i}", (P.carriage_size * 1.4, P.carriage_size, P.carriage_size),
                         (cx, cy, carriage_z), mats["carriage"], bevel=0.008))
        parts.append(box(f"z_motor{i}", (0.05, 0.05, 0.06), (cx, cy, carriage_top + 0.03),
                         mats["frame"]))                     # NEMA23 스텝 (캐리지 위)
        parts.append(cyl(f"z_screw{i}", 0.006, 0.15, (cx + 0.02, cy, carriage_bottom - 0.06),
                         mats["hub"], axis="z"))              # Tr8×8 리드스크류
        parts.append(cyl(f"tool_rod{i}", P.tool_rod_dia / 2, rod_len,
                         (cx, cy, carriage_bottom - rod_len / 2), mats["tool"], axis="z"))

    # ── 8. 배터리 + 전장함 (사이드 포드 안, 앞쪽·낮게 — 무게중심) ──────────
    # 예전엔 가운데 터널에 떠 있었다(물리 영향 0이나 이상함). 실제 배치: 배터리는 한 포드,
    # 전장(Jetson·드라이버 IP66 함)은 반대 포드 → 좌우 균형 + 무게중심 낮게 (DESIGN.md).
    batt_l, batt_w, batt_h = P.battery_size * 1.6, P.battery_size, P.battery_size * 1.2
    batt_x = body_l / 2 - batt_l / 2 - 0.05
    batt_z = pod_bottom + batt_h / 2 + 0.02
    parts.append(box("battery", (batt_l, batt_w, batt_h),
                     (batt_x, -half_track, batt_z), mats["battery"], bevel=0.006))
    parts.append(box("ecase", (batt_l, batt_w, batt_h),
                     (batt_x, +half_track, batt_z), mats["frame"], bevel=0.006))

    # ── 9. Y 스텝모터 (빔 끝 — 캐리지 벨트 구동, DESIGN.md) ────────────────
    parts.append(box("y_motor", (0.06, 0.06, 0.06),
                     (0, P.track(G) * 0.42, beam_z), mats["frame"]))

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
    if key in ("body", "accent"):
        return dict(metal=0.1, rough=0.4, coat=1.0)  # 도장 + 클리어코트
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


# URDF 링크별로 부품을 묶는 규칙. 각 링크는 별도 OBJ 가 되고 URDF 가 조인트로 잇는다.
# 이름 접두사로 판별. 여기 안 걸리는 건 전부 base(고정 몸통) — cam_arm/camera/led 가 이제 base.
# 멀티툴: carriage{i}/z_motor{i}/z_screw{i} → carriage{i}, tool_rod{i} → tool{i} (i=0..n_tools-1).
def _link_rules():
    rules = [
        ("wheel_-1_-1", "wheel_rl"),  # 뒤좌
        ("wheel_-1_1", "wheel_rr"),   # 뒤우
        ("wheel_1_-1", "wheel_fl"),   # 앞좌
        ("wheel_1_1", "wheel_fr"),    # 앞우
    ]
    for i in range(P.n_tools):
        rules += [
            (f"tool_rod{i}", f"tool{i}"),       # Z 프리즘 (막대)
            (f"carriage{i}", f"carriage{i}"),   # Y 프리즘
            (f"z_motor{i}", f"carriage{i}"),    # Z 스텝모터 (캐리지에 고정)
            (f"z_screw{i}", f"carriage{i}"),    # Z 리드스크류 (캐리지에 고정)
        ]
    return rules


LINK_OF = _link_rules()


def link_of(name: str) -> str:
    """부품 이름 → URDF 링크 이름. 안 걸리면 base."""
    for prefix, link in LINK_OF:
        if name.startswith(prefix):
            return link
    return "base"


def export_obj(outdir: Path):
    """Gazebo 용 OBJ+MTL export — URDF 링크별로 분리.

    관절별 메시 분리 (사용자 결정): 바퀴가 실제로 굴러가고 캐리지가 움직이려면
    각 링크가 별도 메시여야 한다. 그리고 각 메시는 그 링크의 조인트 원점 기준으로
    내보낸다 — 예: 바퀴 메시는 바퀴 축이 (0,0,0)에 와야 회전이 맞다. URDF 가 다시
    제 위치에 배치한다. 원점(조인트 위치)은 링크별로 여기서 계산해 links.json 에 남긴다.
    """
    import json

    from mathutils import Vector

    outdir.mkdir(parents=True, exist_ok=True)

    # 부품을 링크별로 분류
    groups: dict[str, list] = {}
    for o in bpy.data.objects:
        if o.type != "MESH" or o.name == "Plane":
            continue
        groups.setdefault(link_of(o.name), []).append(o)

    # 각 링크의 조인트 원점 = 그 링크 부품들의 월드 bbox 중심.
    # (바퀴는 축, 캐리지는 중심 등. URDF joint origin 이 여기에 온다.)
    origins = {}
    for link, objs in groups.items():
        pts = [o.matrix_world @ Vector(c) for ob in objs for c in ob.bound_box]
        origins[link] = [
            sum(p.x for p in pts) / len(pts),
            sum(p.y for p in pts) / len(pts),
            sum(p.z for p in pts) / len(pts),
        ]
    # base 원점은 (0,0,0) 고정 — 로봇 기준 프레임.
    origins["base"] = [0.0, 0.0, 0.0]

    # 카메라 시각 박스의 월드 위치 → make_urdf 가 down_cam 센서를 여기 정확히 배치한다
    # (센서와 시각 카메라가 어긋나지 않게). export 루프가 위치를 바꾸기 전에 잡는다.
    cam_obj = next((o for o in bpy.data.objects if o.name == "camera"), None)
    if cam_obj is not None:
        cpts = [cam_obj.matrix_world @ Vector(c) for c in cam_obj.bound_box]
        origins["camera_world"] = [sum(p.x for p in cpts) / 8, sum(p.y for p in cpts) / 8,
                                   sum(p.z for p in cpts) / 8]

    # 링크별로 export. 메시를 그 링크 원점만큼 빼서 원점 기준으로 만든다.
    for link, objs in groups.items():
        ox, oy, oz = origins[link]
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.location.x -= ox
            o.location.y -= oy
            o.location.z -= oz
            o.select_set(True)
        # location 변경을 지오메트리에 반영 (transform_apply 로 origin 을 옮겼던 것 보정)
        bpy.ops.object.transform_apply(location=True)
        path = outdir / f"{link}.obj"
        bpy.ops.wm.obj_export(
            filepath=str(path),
            export_selected_objects=True,
            export_materials=True,
            export_pbr_extensions=True,
            path_mode="COPY",
            up_axis="Z",
            forward_axis="Y",
        )
        print(f"EXPORTED {link}: {len(objs)} parts")
        # 다시 원위치 (다음 링크 계산이 안 틀리게)
        for o in objs:
            o.location.x += ox
            o.location.y += oy
            o.location.z += oz
        bpy.ops.object.transform_apply(location=True)

    (outdir / "links.json").write_text(json.dumps(origins, indent=1))
    print(f"EXPORTED links.json ({len(origins)} links)")


def dump_bboxes() -> dict:
    """각 부품의 실제 월드 bounding box를 뽑는다.

    주의: o.location 을 쓰면 안 된다 — box()가 transform_apply(scale) 를 하므로
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
