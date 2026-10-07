"""Bake actual solved cover poses onto audited source meshes on a separate bench."""
import hashlib
import json
from pathlib import Path
import bpy
from mathutils import Matrix,Quaternion,Vector

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
DISPLAY=ROOT/'replay'
SOURCE=REPO/'local-replay-base/GTSD_BJTU_B8/source/GTSD_BJTU_B7_detailed.blend'


def material(name,color):
    mat=bpy.data.materials.new(name); mat.use_nodes=True
    node=next(n for n in mat.node_tree.nodes if n.type=='BSDF_PRINCIPLED'); node.inputs['Base Color'].default_value=(*color,1)
    node.inputs['Roughness'].default_value=.45
    return mat


def box(scene,name,pos,size,mat):
    bpy.ops.mesh.primitive_cube_add(size=1,location=pos)
    obj=bpy.context.object; obj.name=name; obj.dimensions=size; obj.data.materials.append(mat)
    return obj


def build_scene(name,replay,source_objects,source_records):
    scene=bpy.data.scenes.new(name); bpy.context.window.scene=scene
    scene.render.engine='CYCLES'; scene.cycles.samples=24; scene.cycles.use_denoising=True
    scene.render.resolution_x=1600; scene.render.resolution_y=1000; scene.render.resolution_percentage=100; scene.render.fps=60
    scene.world=bpy.data.worlds.new(name+'_world'); scene.world.use_nodes=True
    nodes=scene.world.node_tree.nodes; nodes.clear()
    background=nodes.new('ShaderNodeBackground'); output=nodes.new('ShaderNodeOutputWorld')
    background.inputs[0].default_value=(.06,.085,.11,1); background.inputs[1].default_value=.4
    scene.world.node_tree.links.new(background.outputs[0],output.inputs[0])
    steel=material(name+'_steel',(.27,.35,.39)); dark=material(name+'_dark',(.03,.06,.08)); label=material(name+'_label',(.9,.95,.98))
    nodes={}; mapping=[]
    for record in replay['configuration']['covers']:
        key=record['key']; mesh,world=source_objects[record['source_object']]
        node=bpy.data.objects.new(name+'_'+key,None); scene.collection.objects.link(node); node.rotation_mode='QUATERNION'; node['physics_body']=key; nodes[key]=node
        obj=bpy.data.objects.new(name+'_SOURCE_'+record['source_object'],mesh.copy()); scene.collection.objects.link(obj)
        obj.parent=node; obj.matrix_parent_inverse=Matrix.Identity(4)
        obj.matrix_basis=Matrix.Translation(-Vector(record['source_geometry_center_world_m']))@world
        obj['source_object']=record['source_object']; obj['solver_body']=key
        obj['geometry_is_not_collision_definition']=True
        mapping.append(dict(source_object=record['source_object'],solver_body=key,mesh_bind_matrix=[list(row) for row in obj.matrix_basis]))
        # Inferred fixture stands are annotation geometry, not source fasteners
        # or additional contacts; the solver uses explicitly declared welds.
        x=record['initial_bench_center_world_m'][0]
        stand=box(scene,name+'_FIXTURE_'+key,(x,.25,.2),(.035,.035,.4),steel)
        stand['display_role']='inferred fixture visualization only; no source fastener or contact claim'
    box(scene,name+'_floor',(0,0,-.02),(3.0,1.1,.04),dark)
    def text(name,body,pos,size):
        curve=bpy.data.curves.new(name,'FONT'); curve.body=body; curve.size=size; curve.materials.append(label)
        obj=bpy.data.objects.new(name,curve); scene.collection.objects.link(obj); obj.location=pos; obj.rotation_euler=(1.5707963267948966,0,0)
    text(name+'_title','B12 / FORCE-DRIVEN REMOVABLE COVERS',(-1.35,.42,.85),.08)
    text(name+'_scope','INFERRED FIXED BENCH | 0.25 kg proxies | box contacts | uncalibrated',(-1.35,.42,.70),.039)
    text(name+'_pose','Source cover shapes only; two original mounting poses remain unresolved',(-1.35,.42,.62),.035)
    for frame in replay['frames']:
        f=1+60*frame['time_s']
        for key,state in frame['body_transforms'].items():
            node=nodes[key]; node.location=state['position']; node.rotation_quaternion=state['quaternion_wxyz']
            node.keyframe_insert('location',frame=f); node.keyframe_insert('rotation_quaternion',frame=f)
    data=bpy.data.cameras.new(name+'_camera'); camera=bpy.data.objects.new(data.name,data); scene.collection.objects.link(camera)
    camera.location=(1.6,-3,1.7); camera.rotation_euler=(Vector((0,0,.35))-camera.location).to_track_quat('-Z','Y').to_euler()
    data.type='ORTHO'; data.ortho_scale=3.0; scene.camera=camera
    for tag,location,power in [('key',(-1,-2,3),700),('fill',(2,1,2),500)]:
        light=bpy.data.lights.new(name+'_'+tag,'AREA'); light.energy=power; light.size=3
        obj=bpy.data.objects.new(light.name,light); scene.collection.objects.link(obj); obj.location=location
        obj.rotation_euler=(Vector((0,0,.25))-obj.location).to_track_quat('-Z','Y').to_euler()
    scene.frame_start=1; scene.frame_end=121; scene.frame_set(28)
    scene['model_xml_sha256']=replay['model_xml_sha256']; scene['source_sha256']=replay['source_sha256']
    scene['scope']=replay['scope']; scene['display_mapping_json']=json.dumps(mapping)
    return scene


def main():
    acceptance=json.loads((ROOT/'results/verification.json').read_text(encoding='utf-8'))
    if not acceptance['passed']:
        raise RuntimeError('cover physics acceptance required')
    for name in ('assets','reports','inputs','renders'):
        (DISPLAY/name).mkdir(parents=True,exist_ok=True)
    digest=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(SOURCE),load_ui=False,use_scripts=False)
    old=bpy.data.scenes['B7_MAIN_ASSET_KINEMATIC']; bpy.context.window.scene=old
    collection=bpy.data.collections['27_B5_COMPONENT_COVERS']; collection.hide_viewport=False; collection.hide_render=False
    def enable(layer):
        if layer.name=='27_B5_COMPONENT_COVERS':layer.exclude=False; layer.hide_viewport=False
        for child in layer.children:enable(child)
    enable(old.view_layers[0].layer_collection)
    source=json.loads((ROOT/'source_covers.json').read_text(encoding='utf-8'))
    for record in source['covers']:
        obj=bpy.data.objects[record['name']]; obj.hide_viewport=False; obj.hide_set(False); obj.update_tag()
    bpy.data.objects['GTSD_Rig'].update_tag(); old.frame_set(2); old.frame_set(1); bpy.context.view_layer.update()
    dg=bpy.context.evaluated_depsgraph_get(); objects={}
    for record in source['covers']:
        ev=bpy.data.objects[record['name']].evaluated_get(dg)
        world=ev.matrix_world.copy(); expected=Matrix(record['world_matrix'])
        assert max(abs(world[i][j]-expected[i][j]) for i in range(4) for j in range(4))<2e-7
        mesh=bpy.data.meshes.new_from_object(ev,preserve_all_data_layers=True,depsgraph=dg)
        objects[record['name']]=(mesh,world)
    scenes=[]
    for case,name in [('handle_remove','B12_COVER_HANDLING'),('drop','B12_COVER_DROP')]:
        path=ROOT/'results'/(case+'_replay.json'); replay=json.loads(path.read_text(encoding='utf-8'))
        scenes.append(build_scene(name,replay,objects,source))
        (DISPLAY/'inputs'/(case+'_replay.json')).write_bytes(path.read_bytes())
        (DISPLAY/'inputs'/(case+'.xml')).write_bytes((ROOT/'results'/(case+'.xml')).read_bytes())
    for scene in list(bpy.data.scenes):
        if scene not in scenes:bpy.data.scenes.remove(scene)
    bpy.data.orphans_purge(do_recursive=True)
    for obj in bpy.data.objects:
        if obj.animation_data and obj.animation_data.action:
            for slot in obj.animation_data.action.slots:
                for layer in obj.animation_data.action.layers:
                    for strip in layer.strips:
                        bag=strip.channelbag(slot)
                        if bag:
                            for curve in bag.fcurves:
                                for key in curve.keyframe_points:key.interpolation='LINEAR'
    bpy.data.texts.new('B12_ACCEPTANCE.json').write(json.dumps(acceptance,indent=2))
    bpy.data.texts.new('B12_SOURCE_COVER_AUDIT.json').write(json.dumps(source,indent=2))
    bpy.context.window.scene=scenes[0]
    destination=DISPLAY/'assets/GTSD_BJTU_B12_cover_maintenance_bench.blend'
    bpy.ops.wm.save_as_mainfile(filepath=str(destination),compress=True)
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==digest
    report=dict(source_sha256=digest,blend_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),scenes=[s.name for s in scenes],
                scope='source meshes on inferred separate fixtures; solved release/free-body/contact poses; no actual mounting pose claim')
    (DISPLAY/'reports/b12_export.json').write_text(json.dumps(report,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
