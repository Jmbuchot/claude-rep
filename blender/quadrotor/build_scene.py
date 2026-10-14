"""Scène Blender : quadrirotor en X rejouant la simulation de simulate.py.

Le drone (classe F450, bras avant rouges), ses hélices qui tournent à la
vitesse calculée, la consigne (sphère verte), le vecteur erreur, la
trajectoire de consigne (gris) et la trace réelle (bleu) qui se dessine au fil
du temps, des flèches de vent pendant la rafale.

Les hélices tournent à leur vraie vitesse (~550 rad/s) ; c'est le flou de
mouvement de Cycles qui les transforme en disque, comme à l'œil.

Depuis le MCP (execute_blender_code) :
    import sys; sys.path.insert(0, "<dossier>")
    import build_scene; build_scene.build("sim.npz", "quadrirotor.blend")
En ligne de commande :
    python build_scene.py sim.npz quadrirotor.blend
"""

import math
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

Z_OFF = 0.115            # m, hauteur du centre de gravité drone posé
ARM = 0.225
PROP_R = 0.127           # hélice 10 pouces
MOTOR_TOP = 0.040        # m, plan des hélices au-dessus du centre de gravité
LINEAR = 1               # valeur de l'énum interpolation pour foreach_set


# --- Utilitaires --------------------------------------------------------------

def srgb(hex_color, alpha=1.0):
    c = [int(hex_color[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    lin = [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
    return (*lin, alpha)


def material(name, color, metallic=0.0, roughness=0.5, emission=0.0, alpha=1.0,
             coat=0.0):
    mat = bpy.data.materials.new(name)
    bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = srgb(color)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Coat Weight"].default_value = coat
    if emission:
        bsdf.inputs["Emission Color"].default_value = srgb(color)
        bsdf.inputs["Emission Strength"].default_value = emission
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
    return mat


def mesh_object(name, build, mat, parent=None, location=(0, 0, 0), smooth=True):
    """Crée un objet maillé ; build(bm) remplit le bmesh."""
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    build(bm)
    bm.to_mesh(me)
    bm.free()
    if smooth:
        me.shade_smooth()
        me.set_sharp_from_angle(angle=math.radians(35))
    me.materials.append(mat)
    obj = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = parent
    obj.location = location
    return obj


def empty(name, parent=None, location=(0, 0, 0), size=0.05):
    obj = bpy.data.objects.new(name, None)
    obj.empty_display_size = size
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = parent
    obj.location = location
    return obj


def cylinder(radius, depth, segments=32, z0=None):
    """Cylindre d'axe z ; z0 donne la base (sinon centré)."""
    def build(bm):
        bmesh.ops.create_cone(bm, cap_ends=True, segments=segments,
                              radius1=radius, radius2=radius, depth=depth)
        if z0 is not None:
            bmesh.ops.translate(bm, verts=bm.verts, vec=(0, 0, z0 + depth / 2))
    return build


def box(sx, sy, sz, offset=(0, 0, 0)):
    def build(bm):
        bmesh.ops.create_cube(bm, size=1.0)
        bmesh.ops.scale(bm, verts=bm.verts, vec=(sx, sy, sz))
        bmesh.ops.translate(bm, verts=bm.verts, vec=offset)
    return build


def sphere(radius, scale=(1, 1, 1)):
    def build(bm):
        bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=radius)
        bmesh.ops.scale(bm, verts=bm.verts, vec=scale)
    return build


def bake(datablock, path, values, frames):
    """Pose une image clé par image, en bloc (foreach_set)."""
    values = np.asarray(values, dtype=float).reshape(len(frames), -1)
    datablock.keyframe_insert(path, frame=float(frames[0]))
    action = datablock.animation_data.action
    for i in range(values.shape[1]):
        fc = action.fcurve_ensure_for_datablock(datablock, path, index=i)
        pts = fc.keyframe_points
        pts.clear()
        pts.add(len(frames))
        co = np.empty(2 * len(frames))
        co[0::2], co[1::2] = frames, values[:, i]
        pts.foreach_set("co", co)
        pts.foreach_set("interpolation", [LINEAR] * len(frames))
        fc.update()


def continuous_quats(q):
    """Supprime les sauts q -> -q pour une interpolation correcte."""
    q = q.copy()
    for k in range(1, len(q)):
        if np.dot(q[k], q[k - 1]) < 0:
            q[k] = -q[k]
    return q


def poly_curve(name, points, radius, mat):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = radius
    cu.bevel_resolution = 3
    cu.use_fill_caps = True
    sp = cu.splines.new("POLY")
    sp.points.add(len(points) - 1)
    co = np.hstack([points, np.ones((len(points), 1))]).ravel()
    sp.points.foreach_set("co", co)
    cu.materials.append(mat)
    obj = bpy.data.objects.new(name, cu)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def zero_phase_smooth(x, fps, tau):
    """Lissage exponentiel aller-retour (sans retard) pour la caméra."""
    a = 1 / (1 + tau * fps)
    y = x.copy()
    for k in range(1, len(y)):
        y[k] = y[k - 1] + a * (y[k] - y[k - 1])
    for k in range(len(y) - 2, -1, -1):
        y[k] = y[k + 1] + a * (y[k] - y[k + 1])
    return y


# --- Modèle du drone ----------------------------------------------------------

def build_drone(rotor_xy, spin):
    mats = {
        "carbon": material("Carbone", "#232323", roughness=0.35, coat=0.6),
        "front": material("Bras avant", "#d63b2f", roughness=0.4, coat=0.3),
        "rear": material("Bras arrière", "#e8e6e1", roughness=0.4, coat=0.3),
        "motor": material("Moteur", "#9aa0a6", metallic=1.0, roughness=0.3),
        "bell": material("Cloche", "#1f5fae", metallic=1.0, roughness=0.35),
        "prop": material("Hélice", "#121212", roughness=0.25),
        "canopy": material("Capot", "#f2f1ee", roughness=0.3, coat=0.8),
        "battery": material("Batterie", "#3a3f46", roughness=0.6),
        "led_front": material("LED avant", "#2bd96b", emission=12.0),
        "led_rear": material("LED arrière", "#ff3b30", emission=12.0),
    }
    root = empty("Drone", size=0.3)
    root.rotation_mode = "QUATERNION"

    mesh_object("Plaque", box(0.15, 0.15, 0.012), mats["carbon"], root, smooth=False)
    mesh_object("Plaque basse", box(0.12, 0.12, 0.008, (0, 0, -0.03)),
                mats["carbon"], root, smooth=False)
    mesh_object("Capot", sphere(0.065, (1.25, 1.0, 0.45)), mats["canopy"], root,
                location=(0.0, 0, 0.012))
    mesh_object("Batterie", box(0.13, 0.045, 0.03, (0, 0, -0.05)),
                mats["battery"], root, smooth=False)

    props = []
    for i, ((x, y), s) in enumerate(zip(rotor_xy, spin)):
        ang = math.atan2(y, x)
        front = x > 0
        arm = mesh_object(f"Bras {i + 1}", cylinder(0.012, ARM - 0.05, 24),
                          mats["front" if front else "rear"], root)
        arm.rotation_euler = (0, math.pi / 2, ang)
        arm.location = ((ARM + 0.05) / 2 * math.cos(ang), (ARM + 0.05) / 2 * math.sin(ang), 0)
        mesh_object(f"Moteur {i + 1}", cylinder(0.022, 0.022, 32, z0=0.0),
                    mats["motor"], root, location=(x, y, 0.004))
        mesh_object(f"Cloche {i + 1}", cylinder(0.023, 0.012, 32, z0=0.0),
                    mats["bell"], root, location=(x, y, 0.026))
        # Pied du train d'atterrissage sous chaque bras.
        r = 0.13
        leg_xy = (r * math.cos(ang), r * math.sin(ang))
        mesh_object(f"Pied {i + 1}", cylinder(0.005, 0.105, 16, z0=-Z_OFF + 0.006),
                    mats["carbon"], root, location=(*leg_xy, 0))
        mesh_object(f"Patin {i + 1}", sphere(0.009), mats["rear"], root,
                    location=(*leg_xy, -Z_OFF + 0.006))
        mesh_object(f"LED {i + 1}", sphere(0.007),
                    mats["led_front" if front else "led_rear"], root,
                    location=(x, y, -0.006))

        hub = empty(f"Hélice {i + 1}", root, (x, y, MOTOR_TOP), size=PROP_R)
        hub.rotation_mode = "XYZ"
        mesh_object(f"Moyeu {i + 1}", cylinder(0.008, 0.008, 16, z0=-0.004),
                    mats["motor"], hub)
        for k in range(2):
            blade = mesh_object(
                f"Pale {i + 1}.{k + 1}",
                box(PROP_R - 0.01, 0.02, 0.0035, ((PROP_R + 0.01) / 2, 0, 0)),
                mats["prop"], hub, smooth=False)
            blade.rotation_euler = (s * math.radians(12), 0, k * math.pi)
            mod = blade.modifiers.new("Arrondi", "BEVEL")
            mod.width, mod.segments = 0.0015, 2
        hub.cycles.motion_steps = 6      # 32 poses par image : rotation rapide
        props.append(hub)
    return root, props


# --- Décor --------------------------------------------------------------------

def build_ground():
    mat = bpy.data.materials.new("Sol")
    nt = mat.node_tree
    bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Roughness"].default_value = 0.8
    coord = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    nt.links.new(coord.outputs["Object"], sep.inputs[0])
    lines = []
    for axis in ("X", "Y"):
        fr = nt.nodes.new("ShaderNodeMath"); fr.operation = "FRACT"
        sub = nt.nodes.new("ShaderNodeMath"); sub.operation = "SUBTRACT"
        sub.inputs[1].default_value = 0.5
        ab = nt.nodes.new("ShaderNodeMath"); ab.operation = "ABSOLUTE"
        gt = nt.nodes.new("ShaderNodeMath"); gt.operation = "GREATER_THAN"
        gt.inputs[1].default_value = 0.49
        nt.links.new(sep.outputs[axis], fr.inputs[0])
        nt.links.new(fr.outputs[0], sub.inputs[0])
        nt.links.new(sub.outputs[0], ab.inputs[0])
        nt.links.new(ab.outputs[0], gt.inputs[0])
        lines.append(gt)
    mx = nt.nodes.new("ShaderNodeMath"); mx.operation = "MAXIMUM"
    nt.links.new(lines[0].outputs[0], mx.inputs[0])
    nt.links.new(lines[1].outputs[0], mx.inputs[1])
    mix = nt.nodes.new("ShaderNodeMix"); mix.data_type = "RGBA"
    mix.inputs["A"].default_value = srgb("#dcdbd6")
    mix.inputs["B"].default_value = srgb("#a9a8a2")
    nt.links.new(mx.outputs[0], mix.inputs["Factor"])
    nt.links.new(mix.outputs["Result"], bsdf.inputs["Base Color"])
    def plane(bm):
        bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=200.0)
    mesh_object("Sol", plane, mat, smooth=False)


def build_wind_arrows(mat):
    """Trois flèches selon +x, à gauche du point de stationnaire."""
    root = empty("Vent", location=(-1.0, 0, Z_OFF + 1.5))
    for k, (dx, dz) in enumerate([(0.15, 0.3), (0.0, 0.0), (0.15, -0.3)]):
        a = empty(f"Flèche {k + 1}", root, (dx - 0.35, 0, dz))
        a.rotation_euler = (0, math.pi / 2, 0)       # axe z de la flèche -> +x
        mesh_object(f"Tige {k + 1}", cylinder(0.015, 0.5, 16, z0=0.0), mat, a)
        def cone(bm):
            bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=0.05,
                                  radius2=0.0, depth=0.14)
            bmesh.ops.translate(bm, verts=bm.verts, vec=(0, 0, 0.57))
        mesh_object(f"Pointe {k + 1}", cone, mat, a)
    return root


# --- Assemblage ---------------------------------------------------------------

def build(npz_path, blend_path=None, resolution=(960, 540), samples=20):
    d = np.load(npz_path)
    fps = int(d["fps"])
    t = d["frame_t"]
    n = len(t)
    frames = np.arange(1, n + 1, dtype=float)
    off = np.array([0.0, 0.0, Z_OFF])
    pos = d["frame_pos"] + off
    ref = d["frame_pos_ref"] + off

    # Scène vide.
    for coll in (bpy.data.objects, bpy.data.meshes, bpy.data.curves,
                 bpy.data.materials, bpy.data.actions, bpy.data.lights,
                 bpy.data.cameras):
        for item in list(coll):
            coll.remove(item)
    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = 1, n
    scene.render.fps = fps

    root, props = build_drone(d["rotor_xy"], d["spin"])
    bake(root, "location", pos, frames)
    bake(root, "rotation_quaternion", continuous_quats(d["frame_quat"]), frames)
    for hub, angle in zip(props, d["frame_prop_angle"].T):
        bake(hub, "rotation_euler", np.column_stack([np.zeros(n), np.zeros(n), angle]),
             frames)

    build_ground()

    # Consigne, erreur, trajectoires.
    green = material("Consigne", "#1baf7a", emission=3.0)
    target = mesh_object("Consigne", sphere(0.035), green)
    bake(target, "location", ref, frames)

    err_mat = material("Erreur", "#eb6834", emission=2.0)
    err = mesh_object("Vecteur erreur", cylinder(0.011, 1.0, 12, z0=0.0), err_mat)
    err.rotation_mode = "QUATERNION"
    vec = ref - pos
    length = np.linalg.norm(vec, axis=1)
    quats = []
    for v in vec:
        q = Vector((0, 0, 1)).rotation_difference(Vector(v)) if np.linalg.norm(v) > 1e-6 \
            else Vector((0, 0, 1)).rotation_difference(Vector((0, 0, 1)))
        quats.append(tuple(q))
    bake(err, "location", pos, frames)
    bake(err, "rotation_quaternion", continuous_quats(np.array(quats)), frames)
    bake(err, "scale", np.column_stack([np.ones(n), np.ones(n), np.maximum(length, 1e-4)]),
         frames)

    full_t = d["full_t"]
    flying = full_t >= 1.0
    ref_full = d["full_pos_ref"][flying] + off
    ref_full[:, 2] = np.maximum(ref_full[:, 2], Z_OFF)
    poly_curve("Trajectoire de consigne", ref_full, 0.006,
               material("Trait consigne", "#8a8984", roughness=0.6))
    # La fraction visible de la trace est rapportée à la longueur d'arc
    # (mapping SPLINE) : abscisse curviligne parcourue / longueur totale,
    # image par image. Les autres mappings dépendent de la façon dont Blender
    # découpe la courbe et fusionne les points confondus.
    full_pos = d["full_pos"] + off
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(full_pos, axis=0), axis=1))])
    trail = poly_curve("Trace", full_pos, 0.012,
                       material("Trait mesure", "#2a78d6", emission=0.6))
    trail.data.bevel_factor_mapping_end = "SPLINE"
    bake(trail.data, "bevel_factor_end", np.interp(t, full_t, arc) / arc[-1], frames)

    wind_mat = material("Vent", "#5598e7", emission=2.5)
    wind = build_wind_arrows(wind_mat)
    gust = np.linalg.norm(d["frame_wind"], axis=1)
    g = gust / max(gust.max(), 1e-9)
    # Échelle nulle hors rafale : hide_render ne se propage pas aux enfants.
    bake(wind, "scale", np.column_stack([g, g, g]) + 1e-4, frames)

    # Éclairage et monde.
    sun = bpy.data.objects.new("Soleil", bpy.data.lights.new("Soleil", "SUN"))
    scene.collection.objects.link(sun)
    sun.data.energy = 5.0
    sun.data.angle = math.radians(2)
    sun.rotation_euler = (math.radians(35), 0, math.radians(-25))
    world = scene.world or bpy.data.worlds.new("Monde")
    scene.world = world
    bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
    bg.inputs["Color"].default_value = srgb("#b9c7d8")
    bg.inputs["Strength"].default_value = 0.45

    # Caméra de poursuite : distance constante, visée lissée sans retard,
    # ramenée un peu vers le centre pour garder la trajectoire dans le cadre.
    aim = zero_phase_smooth(0.75 * pos + 0.25 * np.array([0, 0, 1.2]), fps, 0.8)
    cam_target = empty("Visée caméra")
    bake(cam_target, "location", aim, frames)
    cam = bpy.data.objects.new("Caméra", bpy.data.cameras.new("Caméra"))
    scene.collection.objects.link(cam)
    cam.data.lens = 50
    offset = np.array([1.6, -5.0, 1.6])
    cam_pos = aim * np.array([0.6, 0.6, 0.5]) + offset
    cam_pos[:, 2] = np.maximum(cam_pos[:, 2], 1.6)
    bake(cam, "location", cam_pos, frames)
    track = cam.constraints.new("TRACK_TO")
    track.target = cam_target
    scene.camera = cam

    # Rendu.
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = resolution
    scene.render.resolution_percentage = 100
    scene.render.use_motion_blur = True
    scene.render.motion_blur_shutter = 0.5
    scene.render.image_settings.file_format = "PNG"
    scene.frame_set(1)

    if blend_path:
        bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    return {"images": n, "fps": fps, "objets": len(bpy.data.objects)}


if __name__ == "__main__":
    print(build(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None))
