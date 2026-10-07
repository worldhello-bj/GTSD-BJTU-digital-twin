"""B10 source replay plus a separate, explicitly inferred auxiliary bench."""
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
BASE = REPO/"local-replay-base/GTSD_BJTU_B8"
DISPLAY = ROOT/"replay"
SCRIPTS = REPO/"source/GTSD_BJTU_B8/scripts"
SCALE = 20.0


def material(name, color, metallic=0):
    mat = bpy.data.materials.new(name); mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = .32
    return mat


def cube(name, xyz, size, mat, parent=None):
    bpy.ops.mesh.primitive_cube_add(size=1, location=xyz)
    obj = bpy.context.object; obj.name = name; obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if parent:
        obj.parent = parent
    return obj


def cylinder(name, xyz, radius, depth, mat, parent=None):
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=radius, depth=depth, location=xyz)
    obj = bpy.context.object; obj.name = name; obj.data.materials.append(mat)
    if parent:
        obj.parent = parent
    return obj


def empty(name, xyz=(0,0,0)):
    obj = bpy.data.objects.new(name, None); bpy.context.scene.collection.objects.link(obj)
    obj.location = xyz
    return obj


def text(name, body, xyz, size, mat):
    data = bpy.data.curves.new(name, "FONT"); data.body = body; data.size = size
    obj = bpy.data.objects.new(name, data); bpy.context.scene.collection.objects.link(obj)
    obj.location = xyz; obj.rotation_euler = (math.pi/2, 0, 0); data.materials.append(mat)
    return obj


def bench_scene():
    identity = json.loads((DISPLAY/"inputs/auxiliary_startup_identity.json").read_text(encoding="utf-8"))
    path = DISPLAY/"inputs/auxiliary_startup.csv"
    if hashlib.sha256(path.read_bytes()).hexdigest() != identity["output_sha256"]:
        raise RuntimeError("auxiliary pose identity mismatch")
    for name, expected in identity["source_sha256"].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError("auxiliary probe source changed")
    with path.open(encoding="utf-8", newline="") as f:
        rows = [{k:(float(v) if v not in ("True", "False") else float(v == "True"))
                 for k,v in row.items()} for row in csv.DictReader(f)]
    scene = bpy.data.scenes.new("B10_INFERRED_AUXILIARY_BENCH")
    bpy.context.window.scene = scene
    scene.render.engine = "CYCLES"; scene.cycles.samples = 24
    scene.render.resolution_x = 1600; scene.render.resolution_y = 1000; scene.render.resolution_percentage = 100
    scene.render.fps = 30; scene.frame_start = 1; scene.frame_end = len(rows)
    scene.world = bpy.data.worlds.new("B10_BenchWorld"); scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs[0].default_value = (.08,.1,.13,1)
    scene.world.node_tree.nodes["Background"].inputs[1].default_value = .35
    scene["provenance"] = "INFERRED equivalent only, not reconstructed source compressor/fan internals"
    scene["physical_duration_s"] = identity["physical_duration_s"]
    scene["display_time_dilation"] = identity["time_dilation"]
    scene["pump_geometry_display_scale"] = SCALE
    scene["probe_sha256"] = identity["output_sha256"]
    scene["scope"] = "Fixed external bench; solver samples at integer frames; interpolated frames are display only"
    steel = material("B10_Steel", (.46,.54,.61), .8)
    teal = material("B10_Teal", (.025,.47,.52), .5)
    amber = material("B10_Amber", (.92,.48,.08), .5)
    dark = material("B10_Dark", (.035,.05,.075), .3)
    light = material("B10_Text", (.84,.9,.94))
    cube("B10_BenchBoard", (0,.15,.6), (3.5,.08,2.2), dark)
    cube("B10_BenchBase", (0,0,-.3), (3.5,.65,.1), steel)
    text("B10_BenchTitle", "B10 / FORCE-DRIVEN AUXILIARIES", (-1.58,-.01,1.55), .115, light)
    text("B10_BenchScope", "INFERRED EQUIVALENT  |  FIXED BENCH  |  UNCALIBRATED", (-1.58,-.012,1.36), .065, amber)
    text("B10_BenchTime", "0.5 s solved startup / 33.3x slower display / no prescribed speed", (-1.58,-.013,1.22), .058, light)
    text("B10_PumpLabel", "PISTON + FLYWHEEL", (-.44,-.11,-.12), .055, light)
    text("B10_LeftFanLabel", "PUMP FAN", (-1.44,-.11,-.12), .055, light)
    text("B10_RightFanLabel", "BAY FAN", (.9,-.11,-.12), .055, light)
    crank = empty("B10_SOLVED_CRANK")
    disc = cylinder("B10_InferredFlywheel", (0,.015,0), .18, .04, teal, crank)
    disc.rotation_euler[0] = math.pi/2
    for sign in (-1,1):
        spoke = cube("B10_FlywheelSpoke_"+str(sign), (0,-.018,0), (.30,.015,.023), steel, crank)
        spoke.rotation_euler[1] = sign*math.pi/4
    pin = cylinder("B10_SOLVED_CRANK_PIN", (0,-.05,0), .025, .10, amber)
    pin.rotation_euler[0] = math.pi/2
    yoke = cube("B10_SOLVED_YOKE", (0,-.05,.1), (.34,.028,.035), steel)
    piston = cylinder("B10_SOLVED_PISTON", (0,0,.5), .108, .06, amber)
    rod = cylinder("B10_SOLVED_PISTON_ROD", (0,0,.2), .022, .4, steel)
    for sign in (-1,1):
        cube("B10_CylinderGuide_"+str(sign), (sign*.14,.05,.43), (.035,.10,.4), teal)
    params = identity["configuration"]["auxiliary_model"]["pump"]
    clearance_m = 2*params["crank_radius_m"]*params["clearance_fraction"]
    head_z = .4+SCALE*params["crank_radius_m"]+.03+SCALE*clearance_m+.045/2
    cylinder("B10_CylinderHead", (0,0,head_z), .16, .045, teal)
    fan_roots = {}
    for name, x in (("compressor_fan", -1.12), ("bay_fan", 1.12)):
        fan = empty("B10_SOLVED_"+name.upper(), (x,0,.43)); fan_roots[name] = fan
        hub = cylinder("B10_InferredHub_"+name, (0,0,0), .07, .055, steel, fan)
        hub.rotation_euler[0] = math.pi/2
        for j in range(5):
            angle = j*math.tau/5
            blade = cube("B10_InferredBlade_"+name+"_"+str(j), (.20*math.sin(angle),0,.20*math.cos(angle)), (.12,.025,.27), teal, fan)
            blade.rotation_euler[1] = angle
        for side in (-1,1):
            cube("B10_FanSupport_"+name+"_"+str(side), (x+side*.36,.07,.42), (.025,.10,.85), steel)
        cube("B10_FanSupportTop_"+name, (x,.07,.845), (.75,.10,.025), steel)
    radius = identity["configuration"]["auxiliary_model"]["pump"]["crank_radius_m"]
    for frame, row in enumerate(rows, 1):
        theta = row["pump_phase_rad"]; extension = row["pump_piston_displacement_m"]
        crank.rotation_euler[1] = row["pump_angle_rad"]
        crank.keyframe_insert("rotation_euler", frame=frame)
        pin.location = (SCALE*radius*math.sin(theta), -.05, SCALE*radius*math.cos(theta))
        pin.keyframe_insert("location", frame=frame)
        yoke.location.z = SCALE*(radius-extension)
        yoke.keyframe_insert("location", frame=frame)
        piston.location.z = .4+SCALE*(radius-extension)
        piston.keyframe_insert("location", frame=frame)
        rod.location.z = .2+SCALE*(radius-extension)
        rod.keyframe_insert("location", frame=frame)
        for name, fan in fan_roots.items():
            fan.rotation_euler[1] = row[name+"_angle_rad"]
            fan.keyframe_insert("rotation_euler", frame=frame)
    for obj in [crank, pin, yoke, piston, rod, *fan_roots.values()]:
        obj["provenance"] = "B10 inferred equivalent display; baked solved auxiliary trace"
        for slot in obj.animation_data.action.slots:
            for layer in obj.animation_data.action.layers:
                for strip in layer.strips:
                    bag = strip.channelbag(slot)
                    if bag:
                        for fc in bag.fcurves:
                            for key in fc.keyframe_points:
                                key.interpolation = "LINEAR"
    camera_data = bpy.data.cameras.new("B10_BenchCamera"); camera = bpy.data.objects.new("B10_BenchCamera", camera_data)
    scene.collection.objects.link(camera); camera.location = (1.0,-5.0,2.5)
    camera.rotation_euler = (Vector((0,0,.65))-camera.location).to_track_quat("-Z", "Y").to_euler()
    camera_data.type = "ORTHO"; camera_data.ortho_scale = 3.9; scene.camera = camera
    for name, loc, power, size in (("Key", (-2,-3,4), 600, 3), ("Fill", (3,-1,2), 400, 3)):
        data = bpy.data.lights.new("B10_Bench"+name, "AREA"); data.energy = power; data.shape = "DISK"; data.size = size
        obj = bpy.data.objects.new(data.name, data); scene.collection.objects.link(obj); obj.location = loc
        obj.rotation_euler = (Vector((0,0,.4))-obj.location).to_track_quat("-Z", "Y").to_euler()
    scene.frame_set(251)
    return scene


def main():
    acceptance = json.loads((ROOT/"results/verification.json").read_text(encoding="utf-8"))
    if not acceptance["passed"]:
        raise RuntimeError("B10 physics acceptance must pass before replay publication")
    (DISPLAY/"reports").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(BASE/"reports/source_door_closed_rest.json", DISPLAY/"reports/source_door_closed_rest.json")
    spec = importlib.util.spec_from_file_location("b10_b8_display_adapter", SCRIPTS/"import_b8_replay.py")
    adapter = importlib.util.module_from_spec(spec); spec.loader.exec_module(adapter)
    adapter.ROOT = DISPLAY; adapter.SOURCE = BASE/"source/GTSD_BJTU_B7_detailed.blend"
    source_hash = hashlib.sha256(adapter.SOURCE.read_bytes()).hexdigest()
    adapter.run(ROOT/"results/service_brake_replay.json", ROOT/"results/service_brake.xml", True)
    main_scene = bpy.data.scenes["B8_SOURCE_TOPOLOGY_REPLAY"]; main_scene.name = "B10_MECHANICAL_REPLAY"
    bpy.data.scenes["B8_PANTOGRAPH_CLOSEUP"].name = "B10_PANTOGRAPH_CLOSEUP"
    for scene in bpy.data.scenes:
        scene["version"] = "B10 vehicle + separate inferred auxiliary bench"
        scene["acceptance_status"] = acceptance["status"]
        scene["internal_source_scope"] = "Source pump/fan shells stay fixed; inferred internals in separate labelled bench"
    for name, body in (("B8_Header", "GTSD BJTU  |  B10 FORCE-DRIVEN REPLAY"),
                       ("B8_Subheader", "B8 vehicle + B9 valve motion + B10 pneumatic auxiliaries"),
                       ("B8_Caution", "UNCALIBRATED  |  Separate scene: inferred compressor and fan bench")):
        if name in bpy.data.objects:
            bpy.data.objects[name].data.body = body
    bench_scene()
    bpy.data.texts.new("B10_ACCEPTANCE.json").write(json.dumps(acceptance, indent=2))
    bpy.data.texts.new("B10_AUXILIARY_CONFIGURATION.json").write(json.dumps(json.loads((ROOT/"results/service_brake_metrics.json").read_text())["pneumatic_configuration"]["auxiliary_model"], indent=2))
    bpy.context.window.scene = main_scene; main_scene.frame_set(121)
    dest = DISPLAY/"assets/GTSD_BJTU_B10_source_dynamics.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(dest), compress=True)
    if hashlib.sha256(adapter.SOURCE.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError("frozen source modified")
    report = dict(source_sha256=source_hash, blend_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
                  scenes=[s.name for s in bpy.data.scenes], auxiliary_display="inferred independent fixed bench, 0.5 s startup at 33.3x slowed time")
    (DISPLAY/"reports/b10_export.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
