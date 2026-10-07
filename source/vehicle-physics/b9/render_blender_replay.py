"""Render review views without changing the saved B9 physical display file."""
from pathlib import Path
import bpy

ROOT = Path(__file__).resolve().parent/"replay"
bpy.ops.wm.open_mainfile(filepath=str(ROOT/"assets/GTSD_BJTU_B9_source_dynamics.blend"), load_ui=False, use_scripts=False)
(ROOT/"renders").mkdir(exist_ok=True)
for name, scene_name in (("b9_overview", "B9_MECHANICAL_REPLAY"),
                         ("b9_pantograph", "B9_PANTOGRAPH_CLOSEUP")):
    scene = bpy.data.scenes[scene_name]
    bpy.context.window.scene = scene
    scene.cycles.samples = 16
    scene.render.threads_mode = "FIXED"
    scene.render.threads = 6
    scene.render.resolution_percentage = 75
    scene.frame_set(121)
    scene.render.filepath = str(ROOT/"renders"/(name+".png"))
    bpy.ops.render.render(write_still=True)
    print("RENDERED", name, flush=True)
