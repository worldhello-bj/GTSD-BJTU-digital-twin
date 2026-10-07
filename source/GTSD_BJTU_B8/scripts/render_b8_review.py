import bpy,sys,os,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
args=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else sys.argv[1:]
ap=argparse.ArgumentParser();ap.add_argument('--video-frames',action='store_true');ap.add_argument('--step',type=int,default=5);ap.add_argument('--package-previews',action='store_true');a=ap.parse_args(args)
bpy.context.preferences.filepaths.use_scripts_auto_execute=False;bpy.ops.wm.open_mainfile(filepath=str(ROOT/'assets/GTSD_BJTU_B8_source_dynamics.blend'),load_ui=False,use_scripts=False)
if a.video_frames:
    sc=bpy.data.scenes['B8_SOURCE_TOPOLOGY_REPLAY'];bpy.context.window.scene=sc;sc.cycles.samples=8;sc.render.threads=4;sc.render.resolution_x=960;sc.render.resolution_y=600
    out=ROOT/'renders/video_frames';out.mkdir(exist_ok=True)
    for i,f in enumerate(range(sc.frame_start,sc.frame_end+1,a.step)):
        sc.frame_set(f);sc.render.filepath=str(out/f'{i:04d}.png');bpy.ops.render.render(write_still=True);print('FRAME',i,f,flush=True)
elif a.package_previews:
    out=ROOT/'renders/package';out.mkdir(exist_ok=True)
    for name,frame,sn in [('b8_overview',121,'B8_SOURCE_TOPOLOGY_REPLAY'),('b8_access_cabinet_open',91,'B8_ACCESS_CABINET_CLOSEUP')]:
        sc=bpy.data.scenes[sn];bpy.context.window.scene=sc;sc.cycles.samples=24;sc.render.threads=6;sc.render.resolution_percentage=80;sc.frame_set(frame);sc.render.filepath=str(out/f'{name}.png');bpy.ops.render.render(write_still=True);print('DONE',name,flush=True)
else:
    for name,frame,sn in [('b8_overview',121,'B8_SOURCE_TOPOLOGY_REPLAY'),('b8_pantograph_contact',121,'B8_PANTOGRAPH_CLOSEUP'),('b8_pantograph_vented',291,'B8_PANTOGRAPH_CLOSEUP'),('b8_access_overview',91,'B8_ACCESS_INSPECTION_REPLAY'),('b8_access_cabinet_open',91,'B8_ACCESS_CABINET_CLOSEUP'),('b8_access_cabinet_closed',271,'B8_ACCESS_CABINET_CLOSEUP')]:
        sc=bpy.data.scenes[sn];bpy.context.window.scene=sc;sc.cycles.samples=24;sc.render.threads=6;sc.frame_set(frame);sc.render.filepath=str(ROOT/'renders'/f'{name}.png');bpy.ops.render.render(write_still=True);print('DONE',name,flush=True)
sys.stdout.flush();os._exit(0)
