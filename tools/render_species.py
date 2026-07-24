#!/usr/bin/env python3
"""종별 표본 렌더 — 콩·옥수수·잡초 3종을 사람이 눈으로 구분하게 (사용자 요청).

CropCraft 원본 에셋(third_party/cropcraft/assets/plants/<종>/)의 대표 메시를 **정하방**(로봇 카메라와
같은 각도)으로 흙 위에 렌더한다. 학습 마스크는 잡초 3종을 다 빨강으로 뭉뚱그려 종 구분이 안 되므로,
종별로 하나씩 렌더해야 "이게 마디풀, 이게 쇠비름, 이게 민들레"가 보인다.

각 종을 개별 PNG 로 렌더 → tools/species_montage.py 가 라벨 붙여 한 장으로 합친다.
실행:  blender --background --python tools/render_species.py    (make species 가 부른다)
"""
import os
import sys

import bpy
from mathutils import Vector

WW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(WW, "artifacts", "species")
ASSETS = os.path.join(WW, "third_party", "cropcraft", "assets", "plants")

# (종, 대표 메시 파일, 분류)  — 우리 4클래스에서 콩·옥수수=작물, 나머지 3종=잡초.
SPECIES = [
    ("bean", "bean_big.obj", "작물 (콩)"),
    ("maize", "maize_big_1.obj", "작물 (옥수수)"),
    ("polygonum", "polygonum_04.obj", "잡초 (마디풀)"),
    ("portulaca", "portulaca_04.obj", "잡초 (쇠비름)"),
    ("taraxacum", "taraxacum_04.obj", "잡초 (민들레)"),
]


def clear():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def setup_scene():
    clear()   # 기본 큐브를 지운다 — 안 지우면 카메라(z=0.6)가 2m 큐브 안에 들어가 검게 찍힌다(실측)
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    try:
        scene.cycles.device = "GPU"
    except Exception:
        pass
    scene.cycles.samples = 48
    scene.render.resolution_x = 512
    scene.render.resolution_y = 512
    scene.render.film_transparent = False
    # 월드 배경광 (이게 없으면 렌더가 통째로 검게 나온다 — 실측)
    world = scene.world or bpy.data.worlds.new("W")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Color"].default_value = (0.6, 0.65, 0.75, 1)
        bg.inputs["Strength"].default_value = 1.2
    # 흙 바닥
    bpy.ops.mesh.primitive_plane_add(size=1.2, location=(0, 0, 0))
    soil = bpy.context.active_object
    m = bpy.data.materials.new("soil"); m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (0.32, 0.24, 0.17, 1)
    bsdf.inputs["Roughness"].default_value = 1.0
    soil.data.materials.append(m)
    # 정하방 카메라 (로봇 down_cam 과 같은 각도)
    bpy.ops.object.camera_add(location=(0, 0, 0.6), rotation=(0, 0, 0))
    scene.camera = bpy.context.active_object
    # 자연광 (학습 자연광 정합)
    bpy.ops.object.light_add(type="SUN", location=(0.5, 0.5, 2))
    bpy.context.active_object.data.energy = 3.0
    bpy.ops.object.light_add(type="AREA", location=(0, 0, 1.5))
    bpy.context.active_object.data.energy = 40


def load_and_frame(objpath):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=objpath)
    new = [o for o in bpy.data.objects if o not in before and o.type == "MESH"]
    if not new:
        return None
    # 원점으로 모으고 지면에 앉히기
    zmin = min((o.matrix_world @ Vector(c)).z for o in new for c in o.bound_box)
    cx = sum((o.matrix_world @ Vector(c)).x for o in new for c in o.bound_box) / (8 * len(new))
    cy = sum((o.matrix_world @ Vector(c)).y for o in new for c in o.bound_box) / (8 * len(new))
    for o in new:
        o.location.x -= cx; o.location.y -= cy; o.location.z -= zmin
    return new


def main():
    os.makedirs(OUT, exist_ok=True)
    setup_scene()
    keep = set(bpy.data.objects)
    for sp, mesh, _label in SPECIES:
        for o in list(bpy.data.objects):
            if o not in keep:
                bpy.data.objects.remove(o, do_unlink=True)
        path = os.path.join(ASSETS, sp, mesh)
        if not os.path.exists(path):
            print(f"W 메시 없음: {path}", file=sys.stderr); continue
        load_and_frame(path)
        bpy.context.scene.render.filepath = os.path.join(OUT, f"{sp}.png")
        bpy.ops.render.render(write_still=True)
        print(f"렌더: {sp}.png")


if __name__ == "__main__":
    main()
