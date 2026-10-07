"""Import B9 solved rigid-body poses using the unchanged B8 source adapter.

Run in background Blender. No invented valve-internal mesh or keyframe motion.
Source is the byte-verified original A-package B7 model; output is a new B9 file.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import bpy

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
BASE = REPO/"local-replay-base/GTSD_BJTU_B8"
DISPLAY = ROOT/"replay"
SCRIPTS = REPO/"source/GTSD_BJTU_B8/scripts"

def main():
    acceptance = json.loads((ROOT/"results/verification.json").read_text(encoding="utf-8"))
    if not acceptance["passed"]:
        raise RuntimeError("B9 acceptance must pass before publishing a replay")
    (DISPLAY/"reports").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(BASE/"reports/source_door_closed_rest.json", DISPLAY/"reports/source_door_closed_rest.json")
    spec = importlib.util.spec_from_file_location("b9_b8_display_adapter", SCRIPTS/"import_b8_replay.py")
    adapter = importlib.util.module_from_spec(spec); spec.loader.exec_module(adapter)
    adapter.ROOT = DISPLAY
    adapter.SOURCE = BASE/"source/GTSD_BJTU_B7_detailed.blend"
    source_hash = hashlib.sha256(adapter.SOURCE.read_bytes()).hexdigest()
    intermediate = adapter.run(ROOT/"results/service_brake_replay.json", ROOT/"results/service_brake.xml", True)
    main_scene = bpy.data.scenes["B8_SOURCE_TOPOLOGY_REPLAY"]
    main_scene.name = "B9_MECHANICAL_REPLAY"
    bpy.data.scenes["B8_PANTOGRAPH_CLOSEUP"].name = "B9_PANTOGRAPH_CLOSEUP"
    for scene in bpy.data.scenes:
        scene["version"] = "B9 balanced dynamic metering-port prototype"
        scene["spool_port_count"] = 51
        scene["extra_spool_state_count"] = 102
        scene["valve_mesh_scope"] = "No source evidence for moving valve internals; states are in solver traces"
        scene["acceptance_status"] = acceptance["status"]
    for name, text in (("B8_Header", "GTSD BJTU  |  B9 FORCE-DRIVEN REPLAY"),
                       ("B8_Subheader", "B8 source mechanics + B9 dynamic metering ports"),
                       ("B8_Caution", "UNCALIBRATED  |  Valve internals: numerical proxy, no source mesh motion")):
        if name in bpy.data.objects:
            bpy.data.objects[name].data.body = text
    metrics = json.loads((ROOT/"results/service_brake_metrics.json").read_text(encoding="utf-8"))
    bpy.data.texts.new("B9_DYNAMIC_METERING_PORTS.json").write(json.dumps(metrics["pneumatic_configuration"]["dynamic_metering_ports"], indent=2))
    bpy.data.texts.new("B9_ACCEPTANCE.json").write(json.dumps(acceptance, indent=2))
    bpy.context.window.scene = main_scene
    main_scene.frame_set(121)
    dest = DISPLAY/"assets/GTSD_BJTU_B9_source_dynamics.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(dest), compress=True)
    if hashlib.sha256(adapter.SOURCE.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError("Frozen source model changed")
    report = dict(output=dest.relative_to(REPO).as_posix(), source_sha256=source_hash, blend_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
                  intermediate=intermediate.relative_to(REPO).as_posix(), scope="B9 solved vehicle replay; dynamic port states in CSV and text metadata, no fabricated internal mesh animation")
    (DISPLAY/"reports/b9_export.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)

if __name__ == "__main__":
    main()
