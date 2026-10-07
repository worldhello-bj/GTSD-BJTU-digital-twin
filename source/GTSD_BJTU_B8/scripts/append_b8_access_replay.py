"""Append a frozen integrated access-door case using shared source geometry.

All moving coordinates are read from the force-solved replay. This script does
not prescribe door angles or synthesize train dynamics. Run the main adapter
first so an existing B8 asset is not appended to repeatedly.
"""
import bpy,sys,os,json,math,hashlib,importlib.util,argparse
from pathlib import Path
from mathutils import Matrix,Vector,Quaternion
ROOT=Path(__file__).resolve().parents[1]

def run(trajectory,model_xml):
 spec=importlib.util.spec_from_file_location('replay_contract',Path(__file__).with_name('replay_contract.py'));contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)
 j,doc,sites,world_geoms,check,jb,xb=contract.inspect(trajectory,model_xml,True)
 base=json.loads((ROOT/'inputs/final_replay.json').read_text())
 if set(base['reference_zero_qpos_body_transforms']) != set(j['reference_zero_qpos_body_transforms']):raise ValueError('Integrated cases have different body sets')
 for n,t in base['reference_zero_qpos_body_transforms'].items():
  if abs(contract.matrix(t)-contract.matrix(j['reference_zero_qpos_body_transforms'][n])).max()>2e-6:raise ValueError('Integrated cases have different source zero frames: '+n)
 if not all(n in j['reference_zero_qpos_body_transforms'] for n in ('access_door_bay_L','access_door_bay_R','access_door_bay_L_secondary','access_door_bay_R_secondary','access_door_cabinet_PWR','access_door_cabinet_DAQ')):raise ValueError('Integrated six-door case required')
 dest=ROOT/'assets/GTSD_BJTU_B8_source_dynamics.blend'
 bpy.context.preferences.filepaths.use_scripts_auto_execute=False;bpy.context.preferences.edit.keyframe_new_interpolation_type='LINEAR';bpy.ops.wm.open_mainfile(filepath=str(dest),load_ui=False,use_scripts=False)
 source=bpy.data.scenes['B8_SOURCE_TOPOLOGY_REPLAY']
 if source['model_xml_sha256']!=base['model_xml_sha256']:raise ValueError('Loaded B8 asset is not the pinned base case')
 if 'B8_ACCESS_INSPECTION_REPLAY' in bpy.data.scenes:raise ValueError('Access replay already appended; rebuild the base first')
 manifest=json.loads(bpy.data.texts['B8_DISPLAY_MAPPING.json'].as_string())
 source.frame_set(1);bpy.context.window.scene=source
 def new_scene(name):
  sc=bpy.data.scenes.new(name);sc.world=source.world;sc.render.engine='CYCLES';sc.cycles.device='CPU';sc.cycles.samples=24;sc.cycles.use_denoising=True;sc.render.threads_mode='FIXED';sc.render.threads=6;sc.render.fps=30;sc.render.resolution_x=1600;sc.render.resolution_y=1000;sc.render.resolution_percentage=100;sc.frame_start=1;sc.frame_end=math.ceil(1+j['frames'][-1]['time_s']*30)
  for key,val in {'mode':'RECORDED_FORWARD_DYNAMICS_DISPLAY_ONLY','scope':j['scope'],'solver':source['solver'],'model_xml_sha256':check['xml_sha256'],'trajectory_sha256':check['trajectory_sha256']}.items():sc[key]=val
  return sc
 sc=new_scene('B8_ACCESS_INSPECTION_REPLAY');cp={}
 for ob in source.objects:
  c=ob.copy();c.name='B8_ACCESS_'+ob.name;c.animation_data_clear();sc.collection.objects.link(c);cp[ob]=c
 for ob,c in cp.items():
  if ob.parent:c.parent=cp.get(ob.parent,ob.parent)
  for md in c.modifiers:
   if md.type=='HOOK' and md.object in cp:md.object=cp[md.object]
 sc.camera=cp[source.camera]
 for old_name,text in [('B8_Header','B8 ACCESS INSPECTION / FORWARD DYNAMICS'),('B8_Subheader','Six source-bound hinge leaves; train reactions remain in the coupled force solve'),('B8_Caution','Assumed mass, springs, latches and joint stops; door obstacle contact is not solved')]:
  c=cp[bpy.data.objects[old_name]];c.data=c.data.copy();c.data.body=text
 nodes={o['physics_body']:o for o in sc.objects if o.get('physics_body')}
 tracker=cp[bpy.data.objects['B8_TranslationOnlyCameraTrack']];ptrack=cp.get(bpy.data.objects.get('B8_PantoTranslationOnlyTrack'))
 spans=[(cp[bpy.data.objects[n]],a,b,L)for n,a,b,L in manifest['derived_endpoint_spans']]
 def mat(t):return Matrix.Translation(t['position'])@Quaternion(t['quaternion_wxyz']).to_matrix().to_4x4()
 def site(name,mats):
  s=sites[name];return (mats[s['body']] if s['body'] else Matrix.Identity(4))@Vector(s['position'])
 def spanquat(d):
  z=d.normalized();h=Vector((1,0,0)) if abs(z.x)<.9 else Vector((0,1,0));x=(h-z*h.dot(z)).normalized();y=z.cross(x).normalized();return Matrix(((x.x,y.x,z.x),(x.y,y.y,z.y),(x.z,y.z,z.z))).to_quaternion()
 def mats_for(rec):
  mats={n:mat(t)for n,t in rec['body_transforms'].items()};a=site('panto_cylinder_base',mats);b=site('panto_cylinder_rod',mats);x=(b-a).normalized();t=mats['panto_mount'].to_3x3()@Vector((0,1,0));y=(t-x*x.dot(t)).normalized();z=x.cross(y).normalized();m=Matrix.Identity(4)
  for row in range(3):m[row][0]=x[row];m[row][1]=y[row];m[row][2]=z[row];m[row][3]=a[row]
  rod=m.copy();rod.translation=b;mats.update(panto_barrel_visual=m,panto_rod_visual=rod);return mats
 for rec in j['frames']:
  f=1+rec['time_s']*30;mats=mats_for(rec)
  for n,m in mats.items():
   o=nodes[n];o.matrix_world=m;o.keyframe_insert('location',frame=f);o.keyframe_insert('rotation_quaternion',frame=f)
  for o,a,b,L0 in spans:
   va=site(a,mats);d=site(b,mats)-va
   if d.length<=0:raise ValueError('Zero display span')
   o.location=va;o.rotation_quaternion=spanquat(d);o.scale=(1,1,d.length)
   for key in ('location','rotation_quaternion','scale'):o.keyframe_insert(key,frame=f)
  av=mats['A_carbody'].translation;bv=mats['B_carbody'].translation;tracker.location=((av.x+bv.x)/2,(av.y+bv.y)/2,0);tracker.keyframe_insert('location',frame=f)
  if ptrack:ptrack.location=(av.x,av.y,0);ptrack.keyframe_insert('location',frame=f)
 # The overview camera follows translation only. Cabinet closeup is world-fixed.
 close=new_scene('B8_ACCESS_CABINET_CLOSEUP')
 for o in sc.objects:
  if o in nodes.values() or o.type=='LIGHT' or (o.get('solver_body','').startswith('access_door_cabinet_')):close.collection.objects.link(o)
 data=bpy.data.cameras.new('B8_AccessCabinetCamera');cam=bpy.data.objects.new(data.name,data);close.collection.objects.link(cam);cam.location=(8,-2,3.6);cam.rotation_euler=(Vector((6,2.6,1.2))-cam.location).to_track_quat('-Z','Y').to_euler();data.type='ORTHO';data.ortho_scale=4.35;data.clip_start=.01;close.camera=cam
 # Independent annotation data, sharing only the established materials.
 for old_name,title,pos,size in [('B8_Header','B8 CABINET DOORS / COUPLED SOLVER REPLAY',(-2.04,1.23),.071),('B8_Caution','Force-driven hinges; hybrid coupled solve; assumed parameters; no obstacle contact',(-2.04,1.10),.042)]:
  src=bpy.data.objects[old_name];o=src.copy();o.data=src.data.copy();o.data.body=title;o.data.size=size;o.animation_data_clear();o.parent=cam;o.location=(*pos,-.2);close.collection.objects.link(o)
 src=bpy.data.objects['B8_MainCaptionBand'];band=src.copy();band.data=src.data.copy();band.parent=cam;band.location=(0,1.20,-.25);band.scale=(4.35/7.2,.30/.90,1);close.collection.objects.link(band)
 bpy.context.window.scene=sc;stats={'body_matrix_error':0.,'span_endpoint_error_m':0.,'door_part_matrix_error':0.,'frames':len(j['frames'])}
 inventory=json.loads((ROOT/'reports/source_door_closed_rest.json').read_text());door_records={r['name']:r for g in inventory['groups'] for r in g['objects']};door_meshes={o.get('source_object'):o for o in sc.objects if o.get('source_object')in door_records}
 if len(door_meshes)!=84:raise ValueError('Not all source door meshes reached access case')
 for rec in j['frames']:
  f=1+rec['time_s']*30;sc.frame_set(int(f),subframe=f-int(f));bpy.context.view_layer.update();mats=mats_for(rec)
  for n,m in mats.items():stats['body_matrix_error']=max(stats['body_matrix_error'],max(abs(nodes[n].matrix_world[a][b]-m[a][b])for a in range(4)for b in range(4)))
  for o,a,b,L in spans:stats['span_endpoint_error_m']=max(stats['span_endpoint_error_m'],(o.matrix_world@Vector((0,0,0))-site(a,mats)).length,(o.matrix_world@Vector((0,0,1))-site(b,mats)).length)
  for n,o in door_meshes.items():
   expected=mats[o['solver_body']]@Matrix(door_records[n]['source_hinge_local_matrix']);stats['door_part_matrix_error']=max(stats['door_part_matrix_error'],max(abs(o.matrix_world[a][b]-expected[a][b])for a in range(4)for b in range(4)))
 if max(stats[k]for k in ('body_matrix_error','span_endpoint_error_m','door_part_matrix_error'))>2e-5:raise ValueError('Access display reconstruction failed: '+str(stats))
 shared_meshes=sum(1 for ob,c in cp.items()if ob.type=='MESH'and c.data==ob.data)
 report=check|{'checks':stats,'shared_mesh_objects':shared_meshes,'door_leaf_parts':84,'display_role':'Recorded coupled solver trajectory; no pose synthesis.','source_file':'GTSD_BJTU_B7_detailed.blend'}
 (ROOT/'inputs/access_replay.json').write_bytes(jb);(ROOT/'inputs/access_model.xml').write_bytes(xb);(ROOT/'reports/access_display_check.json').write_text(json.dumps(report,indent=2));bpy.data.texts.new('B8_ACCESS_REPLAY_CHECK.json').write(json.dumps(report,indent=2))
 sc.frame_set(1);close.frame_set(1);bpy.context.window.scene=source;source.frame_set(1)
 bpy.ops.wm.save_as_mainfile(filepath=str(dest),compress=True)
 print(json.dumps({'output':str(dest),'checks':stats,'shared_mesh_objects':shared_meshes,'scenes':[sc.name,close.name]},indent=2),flush=True)

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--trajectory',required=True);ap.add_argument('--model-xml',required=True);a=ap.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else sys.argv[1:]);run(a.trajectory,a.model_xml);sys.stdout.flush();os._exit(0)
