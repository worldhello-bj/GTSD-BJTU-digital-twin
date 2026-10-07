"""Render B10 review views without modifying the saved asset."""
from pathlib import Path
import bpy

ROOT = Path(__file__).resolve().parent/"replay"
bpy.ops.wm.open_mainfile(filepath=str(ROOT/"assets/GTSD_BJTU_B10_source_dynamics.blend"), load_ui=False, use_scripts=False)
(ROOT/"renders").mkdir(exist_ok=True)
for name, scene_name, frame in (("b10_overview", "B10_MECHANICAL_REPLAY",121),
                                ("b10_auxiliary_bench", "B10_INFERRED_AUXILIARY_BENCH",251)):
    scene = bpy.data.scenes[scene_name]; bpy.context.window.scene = scene
    scene.cycles.samples = 16; scene.render.threads_mode = "FIXED"; scene.render.threads = 6
    scene.render.resolution_percentage = 75; scene.frame_set(frame)
    scene.render.filepath = str(ROOT/"renders"/(name+".png"))
    bpy.ops.render.render(write_still=True)
    print("RENDERED", name, flush=True)
