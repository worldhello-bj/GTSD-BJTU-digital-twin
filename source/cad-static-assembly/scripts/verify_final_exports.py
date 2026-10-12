"""Read exported artifacts back into Blender; compare both point directions."""
import bpy,gzip,json,hashlib,os
from pathlib import Path
import numpy as np
from mathutils.kdtree import KDTree
R=Path(__file__).resolve().parents[1];O=R/'outputs'/os.environ.get('GTSD_OUTPUT_PASS','pass3')
src=R/'inputs/reference_assembly.json.gz';data=json.load(gzip.open(src,'rt'))
expected={m['name']:m for m in data['meshes']};reports=[]
def check(objects,names):
 actual={o['source_name']:o for o in objects if o.get('current_scene_role')=='STATIC_SOURCE_GEOMETRY_REVIEW'}
 assert set(actual)==set(names),(set(names)-set(actual),set(actual)-set(names))
 rows=[]
 for name,ob in actual.items():
  m=expected[name];v=np.asarray(m['vertices']).reshape(-1,3)
  target=KDTree(len(v));got=KDTree(len(ob.data.vertices))
  for i,p in enumerate(v):target.insert(p,i)
  target.balance();error=0
  for i,p in enumerate(ob.data.vertices):
   w=ob.matrix_world@p.co;got.insert(w,i);error=max(error,target.find(w)[2])
  got.balance()
  for p in v:error=max(error,got.find(p)[2])
  assert error<4e-6,(name,error)
  assert len(ob.data.polygons)==len(m['indices'])//3,(name,'triangle count')
  assert ob.get('native_pose_bound_in_this_scene') is False
  rows.append({'name':name,'bidirectional_error_m':error,'triangles':len(ob.data.polygons)})
 return {'source_meshes':len(actual),'max_bidirectional_error_m':max(r['bidirectional_error_m']for r in rows),'triangles':sum(r['triangles']for r in rows),'all_static_and_explicitly_unbound':True,'objects':rows}
for file,selected in [('CAD_reference_vehicle_working.glb',[m['name']for m in data['meshes']if m['group']!='environment']),('CAD_reference_complete_bench_working.glb',list(expected))]:
 bpy.ops.wm.read_factory_settings(use_empty=True)
 bpy.ops.import_scene.gltf(filepath=str(O/file));bpy.context.view_layer.update()
 report=check(list(bpy.context.scene.objects),selected)
 report.update(file=file,sha256=hashlib.sha256((O/file).read_bytes()).hexdigest(),bytes=(O/file).stat().st_size)
 reports.append(report);print('VERIFIED_GLB',file,report['source_meshes'],report['max_bidirectional_error_m'],flush=True)
bpy.ops.wm.open_mainfile(filepath=str(O/'GTSD_CAD_full_review_working.blend'))
main=bpy.data.scenes['01_CAD_STATIC_ASSEMBLY_WORKING'];bpy.context.window.scene=main;bpy.context.view_layer.update()
report=check(list(main.objects),list(expected));report['file']='GTSD_CAD_full_review_working.blend';reports.append(report)
assert '09_EXPLODED_DISPLAY_OFFSETS_NOT_SOURCE_POSE'in bpy.data.scenes
assert all(o.get('engineering_placement_changed') is False for o in bpy.data.scenes['09_EXPLODED_DISPLAY_OFFSETS_NOT_SOURCE_POSE'].objects if o.type=='MESH')
(R/'evidence'/('export_validation_'+O.name+'.json')).write_text(json.dumps({'passed':True,'assembly_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'reports':reports,'actual_saved_scene_count':len(bpy.data.scenes),'exploded_display_separate':True},ensure_ascii=False,indent=2)+'\n')
print('ALL_SAVED_EXPORTS_VERIFIED',flush=True)
