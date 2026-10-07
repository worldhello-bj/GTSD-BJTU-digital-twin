"""Read source centerlines and real attachment evidence without saving source."""
import hashlib
import json
from pathlib import Path
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
SOURCE = REPO/"local-replay-base/GTSD_BJTU_B8/source/GTSD_BJTU_B7_detailed.blend"


def main():
    before = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(SOURCE), load_ui=False, use_scripts=False)
    bpy.context.scene.frame_set(1)
    for obj in bpy.data.objects:
        if "cabinet_open_deg" in obj:
            obj["cabinet_open_deg"] = 0
            obj.update_tag()
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    names = ["B4_MovingLoop_"+str(i) for i in range(3)]
    names += ["B4_CarServiceLead_"+str(i) for i in range(3)]
    names += ["B5_PWR 01_BondingLoop", "B5_DAQ 01_BondingLoop"]
    records = []
    for name in names:
        obj = bpy.data.objects[name]; evaluated = obj.evaluated_get(depsgraph)
        points = [list(evaluated.matrix_world @ Vector(p.co[:3])) for p in evaluated.data.splines[0].points]
        length = sum((Vector(b)-Vector(a)).length for a,b in zip(points, points[1:]))
        props = {k:obj[k].to_list() if hasattr(obj[k], "to_list") else obj[k] for k in obj.keys()}
        props = json.loads(json.dumps(props, default=str))
        record = dict(name=name, points_world_m=points, polyline_length_m=length,
                      radius_m=obj.data.bevel_depth, source_properties=props,
                      driver_count=len(obj.data.animation_data.drivers) if obj.data.animation_data else 0)
        if name.endswith("BondingLoop"):
            pivot = bpy.data.objects[obj["door_pivot"]]
            record.update(pivot_name=pivot.name, pivot_world_matrix=[list(r) for r in pivot.matrix_world],
                          fixed_endpoint_world_m=props["fixed_end"], moving_endpoint_local_m=props["door_end_local"])
            assert abs(pivot.rotation_euler.z) < 1e-6, "reference cabinet door must be closed"
            assert (Vector(points[-1])-pivot.matrix_world @ Vector(props["door_end_local"])).length < 1e-6
        records.append(record)
    assert before == hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    report = dict(source_sha256=before, reference_frame=1, cabinet_open_degrees=0,
                  cable_count=len(records), records=records,
                  source_scope="centerlines/attachment evidence only; masses, stiffness, damping and guide friction not measured")
    path = ROOT/"source_cables.json"; path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(dict(source_sha256=before, cables=[dict(name=r["name"], points=len(r["points_world_m"]),
                       length_m=r["polyline_length_m"], radius_m=r["radius_m"]) for r in records]), indent=2), flush=True)


if __name__ == "__main__":
    main()
