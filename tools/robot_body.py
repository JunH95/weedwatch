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


def material(name: str, rgb, *, metal=0.0, rough=0.5, emit=False, transmit=0.0):
    """PBR 머티리얼 하나. emit=True 면 발광(LED), transmit>0 이면 반투명(유리)."""
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
    # 데크 다음으로 큰 덩어리. 배터리·컴퓨터가 여기 들어가고 다리·캐리지가 여기 붙는다.
    # 데크 아래가 뻥 뚫린 "테이블"이 아니라 몸통이 있는 "기계"가 되는 지점.
    body_w = P.deck_width(G) - 2 * P.body_inset
    body_l = P.deck_length - 2 * P.body_inset
    body_top = P.deck_top_z() - P.deck_thickness  # 데크 아랫면
    body_z = body_top - P.body_height / 2
    parts.append(box("body", (body_l, body_w, P.body_height),
                     (0, 0, body_z), mats["body"], bevel=0.015))
    body_bottom = body_top - P.body_height

    # ── 3. 다리 4개 (몸통 케이스 아래 → 고랑의 바퀴) ──────────────────
    # 다리는 몸통 아랫면에서 시작해 바퀴 축까지. 몸통에 붙어 있다.
    leg_len = body_bottom - P.wheel_dia / 2
    leg_z = P.wheel_dia / 2 + leg_len / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            lx = sx * (body_l / 2 - P.leg_width / 2)
            ly = sy * half_track
            parts.append(box(f"leg_{sx}_{sy}", (P.leg_width, P.leg_width, leg_len),
                             (lx, ly, leg_z), mats["frame"]))
            # 바퀴 (고랑 안, y축 방향으로 누움) — 타이어 + 허브
            parts.append(cyl(f"wheel_{sx}_{sy}", P.wheel_dia / 2, P.wheel_width,
                             (lx, ly, P.wheel_dia / 2), mats["wheel"], axis="y"))
            hub_y = ly - sy * (P.wheel_width / 2 + 0.005)
            parts.append(cyl(f"hub_{sx}_{sy}", P.wheel_dia / 4, 0.02,
                             (lx, hub_y, P.wheel_dia / 2), mats["hub"], axis="y", verts=24))

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
        return dict(metal=0.2, rough=0.35)   # 몸통 케이스 — 도장된 흰 케이스
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

    # 3점 조명 대신 HDRI 스타일: 태양 + 하늘
    bpy.ops.object.light_add(type="SUN", location=(3, -2, 6))
    sun = bpy.context.object
    sun.data.energy = 3.0
    sun.data.angle = 0.15  # 부드러운 그림자
    sun.rotation_euler = (0.5, 0.2, 0.3)

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

    # 카메라
    bpy.ops.object.camera_add()
    cam = bpy.context.object
    scene.camera = cam

    # 로봇 실제 중심 높이에 겨눈다. 데크가 z≈0.7 이라 이전의 0.35 는 너무 낮아
    # 바퀴가 원근으로 떠 보였다. 데크 절반 높이쯤을 본다.
    target = Vector((0, 0, P.deck_top_z() / 2))
    views = {
        "hero": Vector((1.9, -1.7, 1.3)),      # 3/4 앞, 약간 위에서
        "side": Vector((0.05, -2.4, 0.7)),     # 옆 (걸터탄 실루엣)
        "front": Vector((2.6, 0.05, 0.65)),    # 앞 (포탈 아치)
        "top": Vector((0.5, -0.5, 2.8)),       # 위 (데크)
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
