"""Render saved inferred cover handling and landing."""
from pathlib import Path
import bpy
ROOT=Path(__file__).resolve().parent/'replay'
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'assets/GTSD_BJTU_B12_cover_maintenance_bench.blend'),load_ui=False,use_scripts=False)
for name,scene,frame in [('b12_cover_lift','B12_COVER_HANDLING',28),('b12_cover_landed','B12_COVER_DROP',121)]:
    sc=bpy.data.scenes[scene]; bpy.context.window.scene=sc; sc.frame_set(frame)
    sc.render.threads_mode='FIXED'; sc.render.threads=4; sc.cycles.samples=16; sc.render.resolution_percentage=75
    sc.render.image_settings.file_format='PNG'; sc.render.filepath=str(ROOT/'renders'/(name+'.png'))
    bpy.ops.render.render(write_still=True)
    print('RENDERED '+name,flush=True)
