"""Render saved B11 overview and both solved cable closeups."""
from pathlib import Path
import bpy

ROOT=Path(__file__).resolve().parent/'replay'
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'assets/GTSD_BJTU_B11_source_cable_dynamics.blend'),load_ui=False,use_scripts=False)
for name,scene,frame in [('b11_overview','B11_MECHANICAL_REPLAY',121),('b11_supply_cables','B11_SUPPLY_CABLE_CLOSEUP',121),('b11_bonding_cable','B11_BONDING_CABLE_CLOSEUP',61)]:
    sc=bpy.data.scenes[scene]; bpy.context.window.scene=sc; sc.frame_set(frame)
    sc.render.threads_mode='FIXED'; sc.render.threads=6; sc.cycles.samples=16; sc.render.resolution_percentage=75
    sc.render.image_settings.file_format='PNG'; sc.render.filepath=str(ROOT/'renders'/(name+'.png'))
    bpy.ops.render.render(write_still=True)
    print('RENDERED '+name,flush=True)
