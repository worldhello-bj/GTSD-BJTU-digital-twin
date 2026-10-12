"""Split native engineering scenes from separate CAD library review assets."""
import bpy,json,hashlib,os
from pathlib import Path
R=Path(__file__).resolve().parents[1];O=R/'outputs'/os.environ.get('GTSD_OUTPUT_PASS','pass3')
source=O/'GTSD_CAD_full_review_working.blend'
bpy.ops.wm.read_factory_settings(use_empty=True)
names=['01_CAD_STATIC_ASSEMBLY_WORKING','09_EXPLODED_DISPLAY_OFFSETS_NOT_SOURCE_POSE']
with bpy.data.libraries.load(str(source),link=False)as(src,loaded):loaded.scenes=list(names)
main=bpy.data.scenes[names[0]];bpy.context.window.scene=main
for sc in list(bpy.data.scenes):
 if sc.name not in names:bpy.data.scenes.remove(sc)
bpy.data.orphans_purge(do_local_ids=True,do_linked_ids=True,do_recursive=True)
main['delivery_scope']='Static vehicle and complete bench, with separately labelled display-only exploded scene. Independent unused source libraries supplied separately.'
for o in main.objects:
 if o.type=='CAMERA'and o.name=='CAM_vehicle':main.camera=o
for c in main.collection.children:
 if c.name=='04_BENCH_CAD_AND_PHOTO':c.hide_render=True
bpy.ops.wm.save_as_mainfile(filepath=str(O/'GTSD_CAD_vehicle_and_bench.blend'),compress=True)
for ob in list(bpy.data.objects):
 if ob.type=='MESH'and ob.get('group')=='environment':bpy.data.objects.remove(ob,do_unlink=True)
bpy.data.orphans_purge(do_local_ids=True,do_linked_ids=True,do_recursive=True)
main['delivery_scope']='Static vehicle only and display-only exploded scene; no bench or independent uninstalled library geometry.'
bpy.ops.wm.save_as_mainfile(filepath=str(O/'GTSD_CAD_vehicle.blend'),compress=True)
rows=[]
for n in ['GTSD_CAD_vehicle_and_bench.blend','GTSD_CAD_vehicle.blend']:
 p=O/n;rows.append({'name':n,'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(R/'evidence/delivery_native_models.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps(rows),flush=True)
