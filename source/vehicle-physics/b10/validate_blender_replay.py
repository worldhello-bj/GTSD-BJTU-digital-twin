"""Read back every vehicle and inferred auxiliary sample from saved B10 file."""
import csv
import hashlib
import json
import math
from pathlib import Path
import bpy
from mathutils import Matrix, Quaternion

ROOT = Path(__file__).resolve().parent/"replay"
asset = ROOT/"assets/GTSD_BJTU_B10_source_dynamics.blend"
before = hashlib.sha256(asset.read_bytes()).hexdigest()
bpy.ops.wm.open_mainfile(filepath=str(asset), load_ui=False, use_scripts=False)
j = json.loads((ROOT/"inputs/final_replay.json").read_text(encoding="utf-8"))
scene = bpy.data.scenes["B10_MECHANICAL_REPLAY"]; bpy.context.window.scene = scene
xml_hash = hashlib.sha256((ROOT/"inputs/final_model.xml").read_bytes()).hexdigest()
assert xml_hash == j["model_xml_sha256"] == scene["model_xml_sha256"]
nodes = {n:bpy.data.objects["B8_BODY_"+n] for n in j["reference_zero_qpos_body_transforms"]}
inventory = json.loads((ROOT/"reports/source_door_closed_rest.json").read_text(encoding="utf-8"))
records = {r["name"]:r for g in inventory["groups"] for r in g["objects"]}
parts = {o["source_object"]:o for o in scene.objects if o.get("source_object") in records}
assert len(nodes) == 44 and len(parts) == 84
def error(a, b):
    return max(abs(a[r][c]-b[r][c]) for r in range(4) for c in range(4))
body_error = part_error = 0.0
for frame in j["frames"]:
    f = 1+frame["time_s"]*scene.render.fps
    scene.frame_set(int(f), subframe=f-int(f)); bpy.context.view_layer.update()
    transforms = {n:Matrix.Translation(t["position"]) @ Quaternion(t["quaternion_wxyz"]).to_matrix().to_4x4()
                  for n,t in frame["body_transforms"].items()}
    for n, transform in transforms.items():
        body_error = max(body_error, error(nodes[n].matrix_world, transform))
    for n, obj in parts.items():
        expected = transforms[obj["solver_body"]] @ Matrix(records[n]["source_hinge_local_matrix"])
        part_error = max(part_error, error(obj.matrix_world, expected))
assert max(body_error, part_error) < 2e-5
bench = bpy.data.scenes["B10_INFERRED_AUXILIARY_BENCH"]; bpy.context.window.scene = bench
identity = json.loads((ROOT/"inputs/auxiliary_startup_identity.json").read_text(encoding="utf-8"))
path = ROOT/"inputs/auxiliary_startup.csv"
assert hashlib.sha256(path.read_bytes()).hexdigest() == identity["output_sha256"] == bench["probe_sha256"]
with path.open(encoding="utf-8", newline="") as f:
    rows = list(csv.DictReader(f))
assert len(rows) == 501 and bench.frame_end == len(rows)
aux_error = 0.0
scale = bench["pump_geometry_display_scale"]
radius = identity["configuration"]["auxiliary_model"]["pump"]["crank_radius_m"]
for frame, row in enumerate(rows, 1):
    bench.frame_set(frame); bpy.context.view_layer.update()
    theta, extension = float(row["pump_phase_rad"]), float(row["pump_piston_displacement_m"])
    pin = bpy.data.objects["B10_SOLVED_CRANK_PIN"]
    expected = (scale*radius*math.sin(theta), -.05, scale*radius*math.cos(theta))
    aux_error = max(aux_error, max(abs(pin.location[i]-expected[i]) for i in range(3)))
    for name, offset in (("B10_SOLVED_YOKE",0), ("B10_SOLVED_PISTON",.4), ("B10_SOLVED_PISTON_ROD",.2)):
        aux_error = max(aux_error, abs(bpy.data.objects[name].location.z-offset-scale*(radius-extension)))
    for name, field in (("B10_SOLVED_CRANK", "pump_angle_rad"),
                        ("B10_SOLVED_COMPRESSOR_FAN", "compressor_fan_angle_rad"),
                        ("B10_SOLVED_BAY_FAN", "bay_fan_angle_rad")):
        aux_error = max(aux_error, abs(bpy.data.objects[name].rotation_euler[1]-float(row[field])))
assert aux_error < 2e-5
drivers = [o.name for o in bpy.data.objects if o.animation_data and len(o.animation_data.drivers)]
assert not drivers
assert hashlib.sha256(asset.read_bytes()).hexdigest() == before
report = dict(passed=True, vehicle_frames=len(j["frames"]), bodies=len(nodes), source_door_parts=len(parts),
              auxiliary_frames=len(rows), auxiliary_moving_nodes=7, display_time_dilation=bench["display_time_dilation"],
              maximum_body_matrix_error=body_error, maximum_source_part_matrix_error=part_error,
              maximum_auxiliary_position_or_angle_error=aux_error, error_limit=2e-5,
              xml_sha256=xml_hash, blend_sha256=before, auxiliary_csv_sha256=identity["output_sha256"],
              expression_drivers=len(drivers), scope="saved display identity/poses; auxiliary geometry is explicitly inferred")
(ROOT/"reports/b10_readback.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2), flush=True)
