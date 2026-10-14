"""Render and reopen the regional native assembly, using frozen actual mesh data."""
import bpy,datetime,gzip,json,sys
from pathlib import Path
from mathutils import Vector
H=Path(sys.argv[sys.argv.index('--')+1]) if '--' in sys.argv else Path(__file__).resolve().parent
meshes=json.load(gzip.open(H/'regional_meshes.json.gz','rt'))['meshes'];materials={};origin=Vector((-8.9,.27,0))
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
for m in list(bpy.data.materials):bpy.data.materials.remove(m)
def material(key,color):
 if key in materials:return materials[key]
 m=bpy.data.materials.new(key);m.diffuse_color=(*color,1);m.use_nodes=True;n=m.node_tree.nodes.get('Principled BSDF');n.inputs['Base Color'].default_value=(*color,1);n.inputs['Metallic'].default_value=.5 if key!='pad'else .05;n.inputs['Roughness'].default_value=.36 if key!='pad'else .7;materials[key]=m;return m
for m in meshes:
 name=m['name'];vv=[[(m['vertices'][i+k]-origin[k])*1000 for k in range(3)]for i in range(0,len(m['vertices']),3)];ff=[m['indices'][i:i+3]for i in range(0,len(m['indices']),3)];data=bpy.data.meshes.new(name);data.from_pydata(vv,[],ff);data.update();obj=bpy.data.objects.new(name,data);bpy.context.collection.objects.link(obj);obj.location=origin;obj.scale=(.001,)*3;obj['source_mesh_name']=name
 role=m.get('metadata',{}).get('role','source');colors={'frog':(.61,.45,.21),'wing':(.26,.42,.54),'long_plate':(.22,.28,.25),'pad':(.47,.32,.15),'toe_clamp':(.55,.61,.68),'washer':(.65,.7,.73),'bolt_head':(.38,.43,.49)};data.materials.append(material(role,colors.get(role,(.46,.49,.52))))
scene=bpy.context.scene;scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1;scene.world.color=(.3,.3,.3);scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=20;scene.cycles.use_denoising=False;scene.render.threads_mode='FIXED';scene.render.threads=1;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
bpy.ops.object.camera_add(location=(-8.76,-.75,.61));camera=bpy.context.object;camera.data.type='ORTHO';scene.camera=camera
target=Vector((-8.89,.28,-.045))
for p,power,size in [((-8.8,-.4,1.5),95,1.6),((-9.5,.8,.8),65,1.2)]:
 bpy.ops.object.light_add(type='AREA',location=p);o=bpy.context.object;o.data.energy=power;o.data.shape='DISK';o.data.size=size;o.rotation_euler=(target-o.location).to_track_quat('-Z','Y').to_euler()
def render(name,pos,target,scale,res):
 camera.location=pos;camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler();camera.data.ortho_scale=scale;scene.render.resolution_x,scene.render.resolution_y=res;scene.render.filepath=str(H/name);bpy.ops.render.render(write_still=True)
scene['profile_id']='operational_assembly_v1';scene['parameters_file']='parameters.json';scene['geometry_coordinate_unit']='m';scene['regional_mesh_count']=len(meshes)
camera.rotation_euler=(target-camera.location).to_track_quat('-Z','Y').to_euler();camera.data.ortho_scale=1.66
bpy.ops.wm.save_as_mainfile(filepath=str(H/'GTSD_Operational_Turnout.blend'),compress=True)
render('operational_turnout.png',(-8.76,-.75,.61),target,1.66,(1200,780))
render('finite_joint_detail.png',(-9.495,.185,.12),(-9.4700,.26,-.02),.12,(1100,760))
bpy.ops.wm.open_mainfile(filepath=str(H/'GTSD_Operational_Turnout.blend'))
scene=bpy.context.scene
expected={m['name']:m for m in meshes};records=[];maxerr=0
for o in bpy.data.objects:
 if o.type!='MESH':continue
 m=expected[o['source_mesh_name']];assert o.name==m['name'];actual=[o.matrix_world@v.co for v in o.data.vertices];assert len(actual)*3==len(m['vertices']);idx=[i for f in o.data.polygons for i in f.vertices];assert idx==m['indices'];err=max(abs(v[k]-m['vertices'][i*3+k])*1000 for i,v in enumerate(actual)for k in range(3));maxerr=max(maxerr,err);records.append({'name':o.name,'vertices':len(actual),'triangles':len(idx)//3,'world_coordinate_error_mm':err})
assert len(records)==205 and maxerr<.0015
now=datetime.datetime.now(datetime.timezone.utc);report={'schema':'gtsd-operational-native-validation-v1','completed_utc':now.isoformat(),'completed_beijing':now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'native_reopened':True,'mesh_membership_and_triangle_indices_verified':True,'mesh_count':len(records),'maximum_world_vertex_error_mm':maxerr,'tolerance_mm':.0015,'render_engine':scene.render.engine,'threads':1,'samples':20,'rendered_files':['operational_turnout.png','finite_joint_detail.png'],'external_libraries':len(bpy.data.libraries),'embedded_images':len(bpy.data.images),'records':records};(H/'native_render_validation.json').write_text(json.dumps(report,indent=2)+'\n');print('NATIVE_REOPEN_PASS',len(records),maxerr)
