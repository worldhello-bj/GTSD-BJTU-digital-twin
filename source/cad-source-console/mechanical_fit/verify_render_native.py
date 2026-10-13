"""Reopen every native scene, verify exact mesh bytes, then render actual geometry."""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import struct
import sys
import bpy
from mathutils import Vector

ap=argparse.ArgumentParser()
ap.add_argument('--input', type=Path, required=True)
ap.add_argument('--native', type=Path, required=True)
ap.add_argument('--out', type=Path, required=True)
ap.add_argument('--verify-only', action='store_true')
args=ap.parse_args(sys.argv[sys.argv.index('--')+1:])
args.out.mkdir(parents=True,exist_ok=True)
before=hashlib.sha256(args.native.read_bytes()).hexdigest()
bpy.ops.wm.open_mainfile(filepath=str(args.native),load_ui=False)
checked={};scenes=[]
for item in json.loads((args.input/'scenes.json').read_text())['scenes']:
    data=json.load(gzip.open(args.input/item['file'],'rt'))
    scene=bpy.data.scenes['SOURCE_REVIEW' if item['profile']=='source' else 'FIT_'+item['profile']]
    expected=data.get('mesh_refs') or [{'name':m['name'],'geometry_witness_sha256':m['geometry_witness_sha256']} for m in data['meshes']]
    objects={o['source_name']:o for o in scene.objects if o.type=='MESH'}
    assert set(objects)=={m['name'] for m in expected}
    for row in expected:
        ob=objects[row['name']]
        assert not ob.rigid_body and not ob.constraints and not ob.modifiers and not ob.animation_data
        key=ob.data.as_pointer()
        if key not in checked:
            v=[0.]*(len(ob.data.vertices)*3);ob.data.vertices.foreach_get('co',v)
            assert all(len(p.vertices)==3 for p in ob.data.polygons)
            ii=[x for p in ob.data.polygons for x in p.vertices]
            checked[key]=hashlib.sha256(struct.pack('<'+'f'*len(v),*[0. if x==0 else x for x in v])+struct.pack('<'+'I'*len(ii),*ii)).hexdigest()
        assert checked[key]==row['geometry_witness_sha256'],row['name']
    scenes.append({'profile':item['profile'],'mesh_count':len(objects),'all_float32_mesh_streams_equal':True})
assert len(bpy.data.images)==len(bpy.data.sounds)==len(bpy.data.movieclips)==len(bpy.data.libraries)==len(bpy.data.texts)==0
private_paths=[]
for ob in bpy.data.objects:
    if any(x in str(ob.get('source_metadata_json','')) for x in ['/workspace/', '/tmp/', '/home/', 'libfile_']):
        private_paths.append(ob.name)
assert not private_paths,private_paths
views=[]
targets=[('track_support',(-8.65,-.65,.85),(-8.65,.26,-.03),.95),
         ('balise',(-6.9,-.35,.60),(-7.1,.15,-.03),.80),
         ('lower_clamp',(-1.90,.46,1.72),(-2.0725,.2453,1.626),.15),
         ('upper_clamp',(-1.85,-.45,2.30),(-2.05,.008,2.08),.18)]
jobs=[('whole_candidate','FIT_combined',(-7,-12,8),(-4,.3,.55),13.)]
for name,eye,target,scale in targets:
    for label,scene in [('before','SOURCE_REVIEW'),('after','FIT_combined')]:
        jobs.append((name+'_'+label,scene,eye,target,scale))
if args.verify_only:
    jobs=[]
for name,scene_name,eye,target,scale in jobs:
    scene=bpy.data.scenes[scene_name];bpy.context.window.scene=scene
    scene.world=bpy.data.worlds.new('fit_review_background')
    scene.world.color=(.90,.93,.95)
    scene.render.engine='BLENDER_WORKBENCH'
    scene.render.threads_mode='FIXED';scene.render.threads=1
    shade=scene.display.shading;shade.light='STUDIO';shade.color_type='OBJECT'
    shade.show_shadows=True;shade.show_cavity=False;shade.background_type='WORLD'
    scene.display.render_aa='8'
    for ob in scene.objects:
        if ob.type=='MESH':
            ob.color=(1.,.48,.08,1.) if ob.get('mechanical_fit_candidate') else tuple(ob.data.materials[0].diffuse_color)
    camera_data=bpy.data.cameras.new(name);camera=bpy.data.objects.new(name,camera_data)
    scene.collection.objects.link(camera);scene.camera=camera
    camera_data.type='ORTHO';camera_data.clip_start=.001;camera_data.clip_end=100
    camera.location=eye;camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
    camera_data.ortho_scale=scale
    scene.render.resolution_x=1400;scene.render.resolution_y=900;scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG';scene.render.filepath=str(args.out/(name+'.png'))
    bpy.ops.render.render(write_still=True)
    views.append({'file':name+'.png','scene':scene_name,'eye':eye,'target':target,'ortho_scale_m':scale,'actual_triangle_render':True,'candidate_color':'orange','camera_only_no_geometry_exaggeration':True})
assert hashlib.sha256(args.native.read_bytes()).hexdigest()==before
report={'ok':True,'native_sha256':before,'blender':bpy.app.version_string,'scenes':scenes,'unique_mesh_streams_verified':len(checked),'images_embedded_in_native':0,'external_libraries':0,'embedded_texts':0,'private_machine_paths':private_paths,'new_physics':False,'native_file_unchanged_after_review':True,'views':views,'render_sha256':{v['file']:hashlib.sha256((args.out/v['file']).read_bytes()).hexdigest() for v in views}}
(args.out/'native_render_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'ok':True,'scenes':len(scenes),'images':len(views),'native_sha256':before}))
