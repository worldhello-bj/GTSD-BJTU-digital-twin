"""Render one native solver snapshot with exact contract rigid poses, including its wheel.
Usage: blender -b -t 1 --python render_native_state.py -- --contract contract.json --snapshot snapshot.json --output DIR
This script reads the completed solver state and does not advance the solver.
"""
import argparse,datetime,hashlib,json,sys
from pathlib import Path
import bpy
import numpy as np
from mathutils import Quaternion,Vector
H=Path(__file__).resolve().parent;ROOT=H.parents[1];sys.path.insert(0,str(ROOT/'operational_rig'))
from build_rig import load_geometry
ap=argparse.ArgumentParser();ap.add_argument('--contract',type=Path,required=True);ap.add_argument('--snapshot',type=Path,required=True);ap.add_argument('--output',type=Path,default=H);ap.add_argument('--summary',type=Path);a=ap.parse_args(sys.argv[sys.argv.index('--')+1:]);a.output.mkdir(parents=True,exist_ok=True)
contract=json.loads(a.contract.read_text());state=json.loads(a.snapshot.read_text())
for key in ('snapshot','state'):
 if key in state and 'body_poses'not in state:state=state[key]
summary_path=a.summary or a.snapshot.parent/'summary.json';summary=json.loads(summary_path.read_text())
assert state['identity_sha256']==summary['identity_sha256'],'Snapshot/runtime identity mismatch'
assert contract['identity_sha256']==summary['assembly_identity_sha256'],'Contract/assembly identity mismatch'
assert state['steps']>0 and state['time_s']>0,'Require an advanced native solver state'
geometry=load_geometry(root=ROOT,contract=contract);assert geometry['source_sha256']==contract['profile_sha256'],'Geometry/contract identity mismatch'
bindings={b['mesh_name']:b for b in contract['bindings']};assert len(bindings)==len(geometry['meshes']);poses=state['body_poses'];transformed={};materials={}
bpy.context.preferences.filepaths.save_version=0
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
def material(role):
 if role in materials:return materials[role]
 color={'vehicle':(.08,.15,.23),'frog':(.57,.40,.18),'wing':(.24,.40,.54),'long_plate':(.18,.26,.23),'pad':(.44,.29,.13)}.get(role,(.46,.5,.54));m=bpy.data.materials.new(role);m.diffuse_color=(*color,1);m.use_nodes=True;n=m.node_tree.nodes.get('Principled BSDF');n.inputs['Base Color'].default_value=(*color,1);n.inputs['Metallic'].default_value=.48;n.inputs['Roughness'].default_value=.4;materials[role]=m;return m
for row in geometry['meshes']:
 name=row['name'];b=bindings[name];pose=poses[b['body_name']];v=np.array(row['vertices']).reshape(-1,3);q0=Quaternion(b['body_reference_quaternion_wxyz']);q=Quaternion(pose['quaternion_wxyz']);R=np.array(q.to_matrix())@np.array(q0.to_matrix()).T;out=(v-np.array(b['body_reference_position_m']))@R.T+np.array(pose['position']);origin=(out.min(0)+out.max(0))/2;ff=np.array(row['indices']).reshape(-1,3);me=bpy.data.meshes.new(name);me.from_pydata(((out-origin)*1000).tolist(),[],ff.tolist());me.update();obj=bpy.data.objects.new(name,me);bpy.context.collection.objects.link(obj);obj.location=origin;obj.scale=(.001,)*3;obj['source_mesh_name']=name;obj['native_body_name']=b['body_name'];obj['solver_steps']=state['steps'];obj['native_solver_time_s']=state['time_s'];role='vehicle'if b['branch']=='vehicle'else row.get('metadata',{}).get('role',b['branch']);me.materials.append(material(role));transformed[name]=(out,row['indices'],b['body_name'])
assert 'OP_guided_wheel'in transformed
wheel=np.array(state['wheel_position_world_m']);joint=np.array(json.loads((H/'parameters.json').read_text())['joint']['reference_point_mm'])/1000
route=json.loads((H/'wheel_route_geometry.json').read_text());focus_gaps=route['tread_width_interior_gaps'];nearest=min(focus_gaps,key=lambda g:abs((g['start_x_m']+g['end_x_m'])/2-wheel[0]));joint[0]=(nearest['start_x_m']+nearest['end_x_m'])/2
target=np.array([(wheel[0]+joint[0])/2,wheel[1],.025]);scale=max(.22,abs(wheel[0]-joint[0])*1.2+.22)
scene=bpy.context.scene;scene.unit_settings.system='METRIC';scene.world.color=(.22,.22,.22);scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=24;scene.cycles.use_denoising=False;scene.render.threads_mode='FIXED';scene.render.threads=1;scene.render.resolution_x=1400;scene.render.resolution_y=900;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.filepath=str(a.output/'native_solver_state.png')
bpy.ops.object.camera_add(location=target+[-.025,-.85*scale,.65*scale]);camera=bpy.context.object;camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler();camera.data.type='ORTHO';camera.data.ortho_scale=scale;scene.camera=camera
for offset,power in [((-.3,-.3,.8),70),((.3,.3,.5),45)]:
 bpy.ops.object.light_add(type='AREA',location=target+np.array(offset));light=bpy.context.object;light.data.energy=power;light.data.size=1.;light.rotation_euler=(Vector(target)-light.location).to_track_quat('-Z','Y').to_euler()
scene['source']='native solver snapshot';scene['snapshot_identity_sha256']=state['identity_sha256'];scene['native_solver_time_s']=state['time_s'];scene['native_solver_steps']=state['steps'];scene['displacement_scale']=1.;scene['snapshot_sha256']=hashlib.sha256(a.snapshot.read_bytes()).hexdigest()
blend=a.output/'GTSD_Native_Solver_State.blend';bpy.ops.wm.save_as_mainfile(filepath=str(blend),compress=True);bpy.ops.render.render(write_still=True);bpy.ops.wm.open_mainfile(filepath=str(blend))
maxerr=0.;count=0;names=[]
for obj in bpy.data.objects:
 if obj.type!='MESH':continue
 name=obj['source_mesh_name'];vv,indices,body=transformed[name];actual=np.array([obj.matrix_world@p.co for p in obj.data.vertices]);assert actual.shape==vv.shape;assert [i for f in obj.data.polygons for i in f.vertices]==indices;maxerr=max(maxerr,float(np.max(abs(actual-vv))*1000));count+=1;names.append(name)
assert count==len(transformed) and maxerr<.002
now=datetime.datetime.now(datetime.timezone.utc);report={'schema':'gtsd-native-operational-state-render-v1','completed_utc':now.isoformat(),'completed_beijing':now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'snapshot_file':a.snapshot.name,'protocol_name':a.snapshot.parent.parent.name,'case_name':a.snapshot.parent.name,'render_validation_scope':'source mesh identity, applied native poses, exact triangle indices and native reopen; numerical acceptance is recorded by the source protocol','snapshot_sha256':hashlib.sha256(a.snapshot.read_bytes()).hexdigest(),'contract_sha256':hashlib.sha256(a.contract.read_bytes()).hexdigest(),'runtime_identity_sha256':state['identity_sha256'],'assembly_identity_sha256':contract['identity_sha256'],'case_summary_sha256':hashlib.sha256(summary_path.read_bytes()).hexdigest(),'geometry_source_sha256':geometry['source_sha256'],'native_solver_steps':state['steps'],'native_solver_time_s':state['time_s'],'wheel_position_world_m':wheel.tolist(),'camera_focus_joint':nearest,'camera_target_world_m':target.tolist(),'native_contact_count':state['native_contact_count'],'contact_interfaces':state['contact_interfaces'],'displacement_scale':1.,'transform':'q_actual * inverse(q_reference) * (source_world - reference_position) + actual_position','mesh_count':count,'native_reopened':True,'all_indices_and_membership_verified':True,'maximum_world_vertex_error_mm':maxerr,'tolerance_mm':.002,'render_engine':'CYCLES','threads':1,'samples':24,'render':'native_solver_state.png','blend':blend.name,'solver_advanced_by_renderer':False,'mesh_names':names};(a.output/'native_state_render_validation.json').write_text(json.dumps(report,indent=2)+'\n');print('NATIVE_STATE_RENDER_PASS',count,state['steps'],maxerr)
