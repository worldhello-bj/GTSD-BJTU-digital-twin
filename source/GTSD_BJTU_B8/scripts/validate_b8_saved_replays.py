"""Reopen the saved B8 and check every recorded body and source-door part."""
import bpy,json,hashlib,math,sys,os
from pathlib import Path
from mathutils import Matrix,Quaternion
ROOT=Path(__file__).resolve().parents[1];asset=ROOT/'assets/GTSD_BJTU_B8_source_dynamics.blend'
sha=hashlib.sha256(asset.read_bytes()).hexdigest()
bpy.context.preferences.filepaths.use_scripts_auto_execute=False;bpy.ops.wm.open_mainfile(filepath=str(asset),load_ui=False,use_scripts=False)
def mat(t):return Matrix.Translation(t['position'])@Quaternion(t['quaternion_wxyz']).to_matrix().to_4x4()
def err(a,b):return max(abs(a[r][c]-b[r][c])for r in range(4)for c in range(4))
inv=json.loads((ROOT/'reports/source_door_closed_rest.json').read_text());records={r['name']:r for g in inv['groups']for r in g['objects']}
out={'blend_sha256':sha,'passed':False,'cases':{},'source_input_sha256':hashlib.sha256((ROOT/'source/GTSD_BJTU_B7_detailed.blend').read_bytes()).hexdigest()}
for tag,scene_name,prefix in [('final','B8_SOURCE_TOPOLOGY_REPLAY','B8_BODY_'),('access','B8_ACCESS_INSPECTION_REPLAY','B8_ACCESS_B8_BODY_')]:
 j=json.loads((ROOT/'inputs'/f'{tag}_replay.json').read_text());sc=bpy.data.scenes[scene_name];bpy.context.window.scene=sc
 xhash=hashlib.sha256((ROOT/'inputs'/f'{tag}_model.xml').read_bytes()).hexdigest()
 assert j['model_xml_sha256']==xhash==sc['model_xml_sha256']
 nodes={n:bpy.data.objects[prefix+n]for n in j['reference_zero_qpos_body_transforms']}
 doors={o['source_object']:o for o in sc.objects if o.get('source_object')in records}
 assert len(doors)==84
 worst_body=worst_part=0.;worst_time=None
 for rec in j['frames']:
  f=1+rec['time_s']*sc.render.fps;sc.frame_set(int(f),subframe=f-int(f));bpy.context.view_layer.update();mats={n:mat(t)for n,t in rec['body_transforms'].items()}
  for n,m in mats.items():
   e=err(nodes[n].matrix_world,m)
   if e>worst_body:worst_body=e;worst_time=rec['time_s']
  for n,o in doors.items():worst_part=max(worst_part,err(o.matrix_world,mats[o['solver_body']]@Matrix(records[n]['source_hinge_local_matrix'])))
 assert max(worst_body,worst_part)<2e-5
 out['cases'][tag]={'frames':len(j['frames']),'bodies':len(nodes),'door_parts':len(doors),'maximum_body_matrix_error':worst_body,'maximum_door_part_matrix_error':worst_part,'worst_body_time_s':worst_time,'xml_sha256':xhash,'trajectory_sha256':hashlib.sha256((ROOT/'inputs'/f'{tag}_replay.json').read_bytes()).hexdigest()}
drivers=[o.name for o in bpy.data.objects if o.animation_data and len(o.animation_data.drivers)]
assert not drivers,'Unintended expression drivers in replay: '+str(drivers)
out['expression_drivers']=0;out['shared_mesh_datablocks']=sum(1 for m in bpy.data.meshes if m.users>1);out['scene_names']=[s.name for s in bpy.data.scenes];out['passed']=True;out['scope']='Saved-file representation and identity only; physics acceptance belongs to matched solver reports.'
assert hashlib.sha256(asset.read_bytes()).hexdigest()==sha
(ROOT/'reports/final_readback.json').write_text(json.dumps(out,ensure_ascii=False,indent=2));print(json.dumps(out,indent=2),flush=True);sys.stdout.flush();os._exit(0)
