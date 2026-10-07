"""Display actual B11 rigid link poses, replacing the eight legacy cable curves."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
DISPLAY = ROOT/'replay'
BASE = REPO/'local-replay-base/GTSD_BJTU_B8'
SCRIPTS = REPO/'source/GTSD_BJTU_B8/scripts'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
    return result


def material(name, color, metal=0):
    mat = bpy.data.materials.new(name); mat.use_nodes = True
    node = next(n for n in mat.node_tree.nodes if n.type=='BSDF_PRINCIPLED')
    node.inputs['Base Color'].default_value = (*color,1)
    node.inputs['Metallic'].default_value = metal
    node.inputs['Roughness'].default_value = .5
    return mat


def add_mesh(name, vertices, faces, mat, scenes, parent=None):
    data = bpy.data.meshes.new(name); data.from_pydata(vertices,[],faces); data.update()
    data.materials.append(mat)
    obj = bpy.data.objects.new(name,data)
    for scene in scenes:
        scene.collection.objects.link(obj)
    if parent:
        obj.parent = parent; obj.matrix_parent_inverse = Matrix.Identity(4)
    return obj


def capsule(name, delta, radius, mat, scenes, parent):
    direction = Vector(delta); length = direction.length
    # One closed mesh, local +Z axis; hemispherical ends reproduce the solver
    # capsule. No animation or changing scale on geometry itself.
    profile = [(radius*math.cos(a),radius*math.sin(a)) for a in [-math.pi/2+i*math.pi/12 for i in range(7)]]
    profile += [(radius*math.cos(a),length+radius*math.sin(a)) for a in [i*math.pi/12 for i in range(7)]]
    count = 12
    vertices = [(r*math.cos(k*math.tau/count),r*math.sin(k*math.tau/count),z) for r,z in profile for k in range(count)]
    faces = [(j*count+k,j*count+(k+1)%count,(j+1)*count+(k+1)%count,(j+1)*count+k)
             for j in range(len(profile)-1) for k in range(count)]
    obj = add_mesh(name,vertices,faces,mat,scenes,parent)
    obj.rotation_mode = 'QUATERNION'; obj.rotation_quaternion = direction.to_track_quat('Z','Y')
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    obj['solver_capsule_length_m'] = length; obj['solver_radius_m'] = radius
    obj['solver_body'] = parent['physics_body']
    obj['display_role'] = 'rigid equivalent cable capsule; parent pose is recorded dynamics'
    return obj


def box(name, pos, size, mat, scenes):
    vertices = [tuple(pos[i]+sign[i]*size[i] for i in range(3)) for sign in
                [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
    return add_mesh(name,vertices,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],mat,scenes)


def decorate(scenes, replay, xml, tag):
    spec = replay['cable_configuration']
    legacy = {name for route in spec['routes'] for name in route['source_objects']}
    removed = []
    for obj in list(scenes[0].objects):
        if obj.get('source_object') in legacy:
            removed.append(obj['source_object']); bpy.data.objects.remove(obj,do_unlink=True)
    nodes = {obj['physics_body']:obj for obj in scenes[0].objects if obj.get('physics_body')}
    mats = [material('B11_'+tag+'_'+str(i), color) for i,color in enumerate([(.03,.05,.06),(.025,.21,.48),(.055,.38,.13)])]
    for route in spec['routes']:
        points = [Vector(v) for v in route['reference_centerline_world_m']]
        mat = mats[int(route['key'][-1])] if route['key'].startswith('supply_') else mats[2]
        for i, name in enumerate(route['body_names']):
            capsule('B11_'+tag+'_'+name,points[i+1]-points[i],route['radius_m'],mat,scenes,nodes[name])
    support = material('B11_'+tag+'_support',(.18,.22,.24),.45)
    for geom in ET.fromstring(xml).find('worldbody').findall('geom'):
        name = geom.get('name','')
        if not name.startswith('cable_'):
            continue
        pos = list(map(float,geom.get('pos','0 0 0').split()))
        if geom.get('type')=='box':
            box('B11_'+tag+'_'+name,pos,list(map(float,geom.attrib['size'].split())),support,scenes)
        elif geom.get('type')=='plane':
            box('B11_'+tag+'_'+name,[pos[0],pos[1],pos[2]-.005],[20,5,.005],support,scenes)
    for scene in scenes:
        scene['version'] = 'B11 source-anchored cable equivalent'
        scene['cable_specification_json'] = json.dumps(spec)
        scene['legacy_cable_scope'] = 'eight source centerlines replaced by force-solved rigid capsule chains'
        scene['calibration'] = 'UNCALIBRATED source geometry with inferred material/contact parameters'
    for obj in scenes[0].objects:
        if obj.type=='FONT':
            if obj.name.startswith('B8_Header'):
                obj.data.body = 'GTSD BJTU  |  B11 FORCE-DRIVEN CABLE REPLAY'
            elif obj.name.startswith('B8_Subheader'):
                obj.data.body = 'Vehicle reactions / three supply routes / two cabinet bonding leads'
            elif obj.name.startswith('B8_Caution'):
                obj.data.body = 'UNCALIBRATED  |  source centerlines; inferred rigid-link bending/contact equivalents'
    return dict(case=tag, links=spec['total_links'], physical_bodies=len(nodes)-2, removed_legacy_clones=removed,
                source_cable_objects=sorted(legacy), capsule_geometry='rigid, no scale animation', configuration=spec)


def closeup(source, name, location, target, scale):
    scene=bpy.data.scenes.new(name); scene.world=source.world
    scene.render.engine='CYCLES'; scene.cycles.samples=24; scene.cycles.use_denoising=True
    scene.render.resolution_x=1600; scene.render.resolution_y=1000; scene.render.resolution_percentage=100
    scene.render.fps=source.render.fps; scene.frame_start=source.frame_start; scene.frame_end=source.frame_end
    for obj in source.objects:
        if obj.name.startswith(('B8_MainCaptionBand','B8_Header','B8_Subheader','B8_Caution')):
            continue
        scene.collection.objects.link(obj)
    data=bpy.data.cameras.new(name+'_Camera'); camera=bpy.data.objects.new(data.name,data)
    scene.collection.objects.link(camera); camera.location=location
    camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
    data.type='ORTHO'; data.ortho_scale=scale; scene.camera=camera
    font=bpy.data.curves.new(name+'_ScopeLabel','FONT')
    font.body='B11 SOURCE CABLE EQUIVALENT\nUNCALIBRATED | inferred material properties'
    font.size=scale*.018
    label=bpy.data.objects.new(font.name,font); scene.collection.objects.link(label)
    label.rotation_euler=camera.rotation_euler
    label.location=camera.location+camera.rotation_euler.to_quaternion()@Vector((-.46*scale,.26*scale,-.2))
    caption=material(name+'_ScopeMaterial',(.95,.97,1.))
    node=next(n for n in caption.node_tree.nodes if n.type=='BSDF_PRINCIPLED')
    node.inputs['Emission Color'].default_value=(.95,.97,1.,1.)
    node.inputs['Emission Strength'].default_value=1.
    font.materials.append(caption)
    label['display_only_annotation']=True
    scene['scope']='B11 recorded rigid-link cable equivalent; inferred material, uncalibrated'
    return scene


def main():
    # The frozen adapter names newly created shader nodes in English. This
    # background-process preference is never saved to the user's preferences.
    bpy.context.preferences.view.language='en_US'
    if hasattr(bpy.context.preferences.view,'use_translate_new_dataname'):
        bpy.context.preferences.view.use_translate_new_dataname=False
    acceptance = json.loads((ROOT/'results/verification.json').read_text(encoding='utf-8'))
    if not acceptance['passed']:
        raise RuntimeError('B11 complete physics acceptance required before final Blender export')
    for name in ('reports','inputs','assets','renders'):
        (DISPLAY/name).mkdir(parents=True,exist_ok=True)
    shutil.copyfile(BASE/'reports/source_door_closed_rest.json',DISPLAY/'reports/source_door_closed_rest.json')
    adapter = module('b11_frozen_display_adapter',SCRIPTS/'import_b8_replay.py')
    adapter.ROOT = DISPLAY; adapter.SOURCE = BASE/'source/GTSD_BJTU_B7_detailed.blend'
    source_sha = hashlib.sha256(adapter.SOURCE.read_bytes()).hexdigest()
    records = []
    for case in ('doors_open_close','service_brake'):
        adapter.run(ROOT/'results'/(case+'_replay.json'),ROOT/'results'/(case+'.xml'),True)
        scenes = [bpy.data.scenes['B8_SOURCE_TOPOLOGY_REPLAY'],bpy.data.scenes['B8_PANTOGRAPH_CLOSEUP']]
        replay = json.loads((ROOT/'results'/(case+'_replay.json')).read_text(encoding='utf-8'))
        records.append(decorate(scenes,replay,(ROOT/'results'/(case+'.xml')).read_text(encoding='utf-8'),case))
        if case=='doors_open_close':
            scenes[0].name='B11_CABINET_CABLE_REPLAY'; bpy.data.scenes.remove(scenes[1])
            scenes[0].frame_set(61)
            bpy.ops.wm.save_as_mainfile(filepath=str(DISPLAY/'assets/B11_door_intermediate.blend'),compress=True)
            for suffix in ('_replay.json','.xml'):
                shutil.copyfile(ROOT/'results'/(case+suffix),DISPLAY/'inputs'/(case+suffix))
        else:
            scenes[0].name='B11_MECHANICAL_REPLAY'; scenes[1].name='B11_PANTOGRAPH_CLOSEUP'
    with bpy.data.libraries.load(str(DISPLAY/'assets/B11_door_intermediate.blend'),link=False) as (available,loaded):
        loaded.scenes=['B11_CABINET_CABLE_REPLAY']
    closeup(bpy.data.scenes['B11_MECHANICAL_REPLAY'],'B11_SUPPLY_CABLE_CLOSEUP',(.9928527,2.6850805,1.5360739),(4.8565559,1.6498044,-.0639261),2.5).frame_set(121)
    closeup(bpy.data.scenes['B11_CABINET_CABLE_REPLAY'],'B11_BONDING_CABLE_CLOSEUP',(5.8886,2.12294,.39233),(5.28756,2.72398,.24233),.6).frame_set(61)
    # Retain the accepted B10 standalone startup, with its original timing and
    # explicit inference label. It is a separate bench rather than B11 time.
    bench_asset=ROOT.parent/'b10/replay/assets/GTSD_BJTU_B10_source_dynamics.blend'
    bench_check=json.loads((ROOT.parent/'b10/replay/reports/b10_readback.json').read_text(encoding='utf-8'))
    if hashlib.sha256(bench_asset.read_bytes()).hexdigest()!=bench_check['blend_sha256']:
        raise RuntimeError('accepted B10 bench source asset changed')
    with bpy.data.libraries.load(str(bench_asset),link=False) as (available,loaded):
        loaded.scenes=['B10_INFERRED_AUXILIARY_BENCH']
    bpy.data.texts.new('B11_ACCEPTANCE.json').write(json.dumps(acceptance,indent=2))
    bpy.data.texts.new('B11_CABLE_DISPLAY_MAPPING.json').write(json.dumps(records,indent=2))
    for text in bpy.data.texts:
        if text.name.startswith('B8_DISPLAY_MAPPING.json'):
            j=json.loads(text.as_string()); j['unresolved_geometry']='B11 source cable capsules are force-solved; primary trailing-arm literals and unmeasured internals remain reduced.'
            text.clear(); text.write(json.dumps(j,indent=2))
    main_scene=bpy.data.scenes['B11_MECHANICAL_REPLAY']; bpy.context.window.scene=main_scene; main_scene.frame_set(121)
    destination=DISPLAY/'assets/GTSD_BJTU_B11_source_cable_dynamics.blend'
    bpy.ops.wm.save_as_mainfile(filepath=str(destination),compress=True)
    if hashlib.sha256(adapter.SOURCE.read_bytes()).hexdigest()!=source_sha:
        raise RuntimeError('frozen source changed')
    report=dict(source_sha256=source_sha,blend_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
                scenes=[s.name for s in bpy.data.scenes],cases=records,scope='recorded B11 source-cable equivalents plus the separate accepted B10 inferred startup bench')
    (DISPLAY/'reports/b11_export.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2),flush=True)


if __name__=='__main__':
    main()
