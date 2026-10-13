"""One local-geometry Blender review render; no original photo or external assets."""
import bpy,json,gzip,sys,math
from mathutils import Vector
from pathlib import Path
args=sys.argv[sys.argv.index('--')+1:];root=Path(args[0]);out=root/(args[1] if len(args)>1 else 'build')
source=json.load(gzip.open(root/'source_seed.json.gz','rt'));candidate=json.load(gzip.open(out/'candidate_meshes.json.gz','rt'));manifest=json.loads((out/'change_manifest.json').read_text());audit=json.loads((out/'material_audit.json').read_text())
remove={r['name']for r in manifest['remove']};conflicts={r['pair'][1]for r in audit['positive_material_pairs']};meshes=[m for m in source['meshes']if m['name']not in remove]+candidate['meshes']
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
for datablock in list(bpy.data.materials):bpy.data.materials.remove(datablock)
origin=Vector((-8.9,.27,0.));records=[]
materials={}
def material(role,c):
 if role in materials:return materials[role]
 m=bpy.data.materials.new(role);m.diffuse_color=(*c,1);m.use_nodes=True;b=m.node_tree.nodes.get('Principled BSDF');b.inputs['Base Color'].default_value=(*c,1);b.inputs['Metallic'].default_value=.45 if role in ['frog','wing']else.12;b.inputs['Roughness'].default_value=.32 if role in ['frog','wing']else.55;materials[role]=m;return m
for row in meshes:
 name=row['name'];v=[[(row['vertices'][i+k]-origin[k])*1000 for k in range(3)]for i in range(0,len(row['vertices']),3)];f=[row['indices'][i:i+3]for i in range(0,len(row['indices']),3)];data=bpy.data.meshes.new(name);data.from_pydata(v,[],f);data.update();obj=bpy.data.objects.new(name,data);bpy.context.collection.objects.link(obj);obj.location=origin;obj.scale=(.001,)*3;obj['as_built_confirmed']=False;obj['new_physics']=False
 role=row.get('metadata',{}).get('role','source_context');color={'frog':(.60,.44,.19),'wing':(.27,.42,.52),'long_plate':(.21,.28,.25),'pad':(.49,.33,.16),'source_context':(.48,.50,.51),'unchanged_source_remainder':(.48,.50,.51)}.get(role,(.5,.5,.5))
 if name in conflicts:role='retained_interference';color=(.70,.12,.08)
 data.materials.append(material(role,color));records.append({'name':name,'vertices':len(v),'triangles':len(f),'highlighted_retained_interference':name in conflicts})
# Camera and simple neutral lighting; no additional opaque geometry masks parts.
scene=bpy.context.scene;scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1
bpy.ops.object.camera_add(location=(-8.76,-.75,.61));camera=bpy.context.object;target=Vector((-8.89,.28,-.045));camera.rotation_euler=(target-camera.location).to_track_quat('-Z','Y').to_euler();camera.data.type='ORTHO';camera.data.ortho_scale=1.66;scene.camera=camera
for location,power,size in [((-8.8,-.4,1.5),95,1.6),((-9.5,.8,.8),65,1.2)]:
 bpy.ops.object.light_add(type='AREA',location=location);light=bpy.context.object;light.data.energy=power;light.data.shape='DISK';light.data.size=size;light.rotation_euler=(target-light.location).to_track_quat('-Z','Y').to_euler()
scene.world.color=(.28,.28,.28);scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=20;scene.cycles.use_denoising=False;scene.render.threads_mode='FIXED';scene.render.threads=1;scene.render.resolution_x=1200;scene.render.resolution_y=780;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.filepath=str(out/'photo_turnout_candidate.png');scene.render.film_transparent=False
scene['review_status']='STATIC_ENGINEERING_RESEARCH_UNRESOLVED_FIELD_FIXATION';scene['original_photos_included']=False;scene['context_scope']='127-object public-source ROI seed with explicit profile replacements; not a whole-scene native model';scene['candidate_id']=manifest['candidate_id']
bpy.ops.wm.save_as_mainfile(filepath=str(out/'GTSD_Photo_Turnout_Engineering_Candidate.blend'),compress=True)
bpy.ops.render.render(write_still=True)
# Reopen and verify that every expected mesh, triangle index and vertex survived.
bpy.ops.wm.open_mainfile(filepath=str(out/'GTSD_Photo_Turnout_Engineering_Candidate.blend'))
byname={m['name']:m for m in meshes};maximum_error=0.;matched=0
for obj in bpy.data.objects:
 if obj.type!='MESH':continue
 row=byname[obj.name];expected=[row['vertices'][i:i+3]for i in range(0,len(row['vertices']),3)];actual=[obj.matrix_world@v.co for v in obj.data.vertices];assert len(expected)==len(actual)
 for p,q in zip(expected,actual):maximum_error=max(maximum_error,max(abs(p[k]-q[k])*1000 for k in range(3)))
 indices=[i for p in obj.data.polygons for i in p.vertices];assert indices==row['indices'],obj.name;matched+=1
assert matched==len(meshes)and maximum_error<.0015,(matched,len(meshes),maximum_error)
report={'schema':'gtsd-photo-turnout-native-render-check-v1','native_reopened':True,'mesh_membership_and_indices_verified':True,'mesh_count':matched,'candidate_count':len(candidate['meshes']),'maximum_world_vertex_error_mm':maximum_error,'tolerance_mm':.0015,'single_small_render':True,'resolution':[1200,780],'samples':20,'threads':1,'rendered_conflicting_retained_objects_red':True,'original_images_embedded':False,'external_libraries':len(bpy.data.libraries),'text_blocks':len(bpy.data.texts),'rigid_bodies':sum(o.rigid_body is not None for o in bpy.data.objects),'records':records}
import datetime
now=datetime.datetime.now(datetime.timezone.utc);report['completed_utc']=now.isoformat();report['completed_beijing']=now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat()
(out/'native_render_validation.json').write_text(json.dumps(report,indent=2)+'\n');print('NATIVE_REOPEN_VERIFIED',matched,'MAX_ERROR_MM',maximum_error)
