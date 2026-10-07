"""Reopen B9 and verify all 301 poses, 44 bodies and 84 source door members."""
import hashlib
import json
from pathlib import Path
import bpy
from mathutils import Matrix, Quaternion

ROOT = Path(__file__).resolve().parent/"replay"
asset = ROOT/"assets/GTSD_BJTU_B9_source_dynamics.blend"
before = hashlib.sha256(asset.read_bytes()).hexdigest()
bpy.ops.wm.open_mainfile(filepath=str(asset), load_ui=False, use_scripts=False)
j = json.loads((ROOT/"inputs/final_replay.json").read_text(encoding="utf-8"))
scene = bpy.data.scenes["B9_MECHANICAL_REPLAY"]
bpy.context.window.scene = scene
xml_hash = hashlib.sha256((ROOT/"inputs/final_model.xml").read_bytes()).hexdigest()
assert xml_hash == j["model_xml_sha256"] == scene["model_xml_sha256"]
nodes = {n: bpy.data.objects["B8_BODY_"+n] for n in j["reference_zero_qpos_body_transforms"]}
inventory = json.loads((ROOT/"reports/source_door_closed_rest.json").read_text(encoding="utf-8"))
records = {r["name"]: r for g in inventory["groups"] for r in g["objects"]}
parts = {o["source_object"]: o for o in scene.objects if o.get("source_object") in records}
assert len(nodes) == 44 and len(parts) == 84
def error(a, b):
    return max(abs(a[r][c]-b[r][c]) for r in range(4) for c in range(4))
body_error = part_error = 0.0
for frame in j["frames"]:
    f = 1+frame["time_s"]*scene.render.fps
    scene.frame_set(int(f), subframe=f-int(f)); bpy.context.view_layer.update()
    transforms = {n: Matrix.Translation(t["position"]) @ Quaternion(t["quaternion_wxyz"]).to_matrix().to_4x4()
                  for n,t in frame["body_transforms"].items()}
    for n, transform in transforms.items():
        body_error = max(body_error, error(nodes[n].matrix_world, transform))
    for n, obj in parts.items():
        expected = transforms[obj["solver_body"]] @ Matrix(records[n]["source_hinge_local_matrix"])
        part_error = max(part_error, error(obj.matrix_world, expected))
drivers = [o.name for o in bpy.data.objects if o.animation_data and len(o.animation_data.drivers)]
assert not drivers
assert max(body_error, part_error) < 2e-5
assert hashlib.sha256(asset.read_bytes()).hexdigest() == before
report = dict(passed=True, frames=len(j["frames"]), bodies=len(nodes), door_parts=len(parts),
              maximum_body_matrix_error=body_error, maximum_source_part_matrix_error=part_error,
              unchanged_B8_matrix_error_limit=2e-5, xml_sha256=xml_hash, blend_sha256=before,
              expression_drivers=len(drivers), scope="Saved display representation and identity, not additional physics validation")
(ROOT/"reports/b9_readback.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2), flush=True)
