"""Build a static source review scene plus independently framed CAD libraries.

This is geometry reconstruction, not a continuation of a prior physics state.
"""
import bpy, gzip, hashlib, json, math, os
from pathlib import Path
import numpy as np
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs' / os.environ.get('GTSD_OUTPUT_PASS','pass3')
OUT.mkdir(exist_ok=True,parents=True)
source_path = ROOT / 'inputs/reference_assembly.json.gz'
data = json.load(gzip.open(source_path, 'rt'))
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.name = '01_CAD_STATIC_ASSEMBLY_WORKING'
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1
scene['scope'] = 'Static CAD/source reconstruction. No dynamics, no hardware calibration.'
scene['assembly_geometry_sha256'] = hashlib.sha256(source_path.read_bytes()).hexdigest()
scene['source_reference_pose'] = True
scene['all_CAD_consumed'] = False
scene['known_review_items'] = 'Native source suspension plates with fitted photo coil; B31 transverse placement is a candidate; floor39780 verified unchanged with source24.5mm offset already applied; wheel/brake libraries remain separate.'

def collection(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or scene.collection).children.link(c)
    return c

groups = {g:collection(g) for g in ('01_VEHICLE_BODY', '02_BOGIE_CAD_AND_PHOTO',
                                   '03_PANTOGRAPH_SOURCE_AND_PHOTO', '04_BENCH_CAD_AND_PHOTO')}
presentation = collection('99_PRESENTATION_NOT_ENGINEERING_GEOMETRY')
layers = {}
materials = {}
objects = []
manifest = []

def material(color):
    key = tuple(round(float(v), 5) for v in color[:3])
    if key not in materials:
        mat = bpy.data.materials.new('source_color_' + str(len(materials)))
        mat.diffuse_color = (*key, 1)
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes['Principled BSDF']
        bsdf.inputs['Base Color'].default_value = (*key, 1)
        bsdf.inputs['Roughness'].default_value = .42
        bsdf.inputs['Metallic'].default_value = .3
        materials[key] = mat
    return materials[key]

for i,m in enumerate(data['meshes']):
    name = m['name']
    root = {'body':'01_VEHICLE_BODY', 'bogie':'02_BOGIE_CAD_AND_PHOTO',
            'pantograph':'03_PANTOGRAPH_SOURCE_AND_PHOTO', 'environment':'04_BENCH_CAD_AND_PHOTO'}[m['group']]
    layer = (root, m.get('assembly_layer', 'SOURCE_REFERENCE'))
    if layer not in layers:
        layers[layer] = collection(layer[1], groups[root])
    verts = np.asarray(m['vertices'], dtype=float).reshape(-1,3)
    triangles = np.asarray(m['indices'], dtype=int).reshape(-1,3)
    mesh = bpy.data.meshes.new(name + '_source_geometry')
    mesh.from_pydata(verts.tolist(), [], triangles.tolist())
    mesh.update()
    ob = bpy.data.objects.new(name, mesh)
    layers[layer].objects.link(ob)
    ob.data.materials.append(material(m.get('color', [.48,.53,.6])))
    for key,value in m.items():
        if key in ('vertices', 'indices', 'name', 'color'):
            continue
        ob[key] = value if isinstance(value,(str,int,float,bool)) else json.dumps(value,ensure_ascii=False)
    ob['current_scene_role'] = 'STATIC_SOURCE_GEOMETRY_REVIEW'
    ob['source_name'] = name
    ob['as_built_confirmed'] = False
    ob['native_pose_bound_in_this_scene'] = False
    objects.append(ob)
    manifest.append({'object_name':name, 'group':m['group'],
                     'assembly_layer':layer[1], 'source_refs':m.get('source_refs'),
                     'vertices':len(verts), 'triangles':len(triangles),
                     'bounds_min_m':verts.min(0).tolist(), 'bounds_max_m':verts.max(0).tolist(),
                     'geometry_status':m.get('geometry_status'),
                     'registry_id':m.get('registered_component_id')})
    if i % 500 == 0:
        print('MESH', i, flush=True)

# Preserve the 109 registration rows as semantic empties, not as 109 physical items.
reg = collection('05_REGISTRY_109_SEMANTIC_ROWS_NOT_PHYSICAL_COUNTS')
regobjects = {}
for r in data['registry']:
    key = r.get('id', r.get('key'))
    ob = bpy.data.objects.new(str(key) + ' | ' + str(r.get('name','')), None)
    reg.objects.link(ob)
    for k,v in r.items():
        ob[k] = v if isinstance(v,(str,int,float,bool)) else json.dumps(v,ensure_ascii=False)
    ob.empty_display_size = .008
    ob.hide_render = True
    regobjects[key] = ob
for r in data['registry']:
    key = r.get('id',r.get('key'))
    if r.get('parent') in regobjects:
        regobjects[key].parent = regobjects[r['parent']]

def aim(ob,target):
    ob.rotation_euler = (Vector(target)-ob.location).to_track_quat('-Z','Y').to_euler()

def area(name,location,power,size,target):
    d = bpy.data.lights.new(name,'AREA'); d.energy=power; d.size=size
    o = bpy.data.objects.new(name,d); presentation.objects.link(o); o.location=location; aim(o,target)

world = bpy.data.worlds.new('CAD review studio')
scene.world = world; world.use_nodes=True
world.node_tree.nodes['Background'].inputs[0].default_value=(.12,.15,.20,1)
world.node_tree.nodes['Background'].inputs[1].default_value=.5
area('vehicle_key',(-2,-3,4),700,4,(0,0,.65))
area('vehicle_fill',(2,1,3),700,3,(0,0,.65))
area('bench_key',(-5,-3,8),4000,9,(-4,0,.5))

def camera(name,location,target,scale):
    d=bpy.data.cameras.new(name);d.type='ORTHO';d.ortho_scale=scale
    o=bpy.data.objects.new(name,d);presentation.objects.link(o);o.location=location;aim(o,target)
    return o

cameras = {
    'vehicle':camera('CAM_vehicle',(-2.5,-3.4,2.2),(0,0,.72),2.55),
    'bench':camera('CAM_complete_bench',(-13,-12,8),(-4.1,0,.6),13.5),
    'bogie':camera('CAM_bogie_source_stack',(-1.7,-2.4,1.1),(0,0,.21),1.5),
    'panto':camera('CAM_pantograph_multiview',(-1.8,-2.6,2.8),(.13,0,1.46),1.9),
    'interior':camera('CAM_interior_source_review',(-2,-3,1.85),(0,0,.62),2.1),
}
scene.camera=cameras['vehicle']
scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=12
scene.cycles.use_denoising=True;scene.cycles.max_bounces=4
scene.render.resolution_x=1400;scene.render.resolution_y=1000;scene.render.resolution_percentage=100
scene.render.threads_mode='FIXED';scene.render.threads=1
scene.render.image_settings.file_format='PNG';scene.view_settings.view_transform='AgX'

# Exploded view is an assembly illustration with explicit display offsets. It
# is a linked-mesh scene, never a new source placement or a solved pose.
exploded=bpy.data.scenes.new('09_EXPLODED_DISPLAY_OFFSETS_NOT_SOURCE_POSE')
exploded.world=world;exploded.unit_settings.system='METRIC'
exploded['source_pose']=False
exploded['display_offsets_m']=json.dumps({'body':[0,0,.38],'pantograph':[0,0,.82],'bogie':[0,0,0]})
for ob in objects:
    if ob.get('group')=='environment':continue
    dup=ob.copy();dup.data=ob.data;exploded.collection.objects.link(dup)
    offset={'body':.38,'pantograph':.82,'bogie':0}[ob['group']]
    dup.location.z+=offset
    dup['display_Z_offset_m']=offset;dup['engineering_placement_changed']=False
for ob in presentation.objects:
    if ob.type=='LIGHT':exploded.collection.objects.link(ob)
cd=bpy.data.cameras.new('CAM_exploded');cd.type='ORTHO';cd.ortho_scale=3.6
co=bpy.data.objects.new('CAM_exploded',cd);exploded.collection.objects.link(co)
co.location=(-3.5,-4.2,2.8);aim(co,(0,0,1.22));exploded.camera=co
exploded.render.engine='CYCLES';exploded.cycles.device='CPU';exploded.cycles.samples=12
exploded.cycles.use_denoising=True;exploded.cycles.max_bounces=4
exploded.render.resolution_x=1400;exploded.render.resolution_y=1000;exploded.render.resolution_percentage=100
exploded.render.threads_mode='FIXED';exploded.render.threads=1
exploded.render.image_settings.file_format='PNG';exploded.view_settings.view_transform='AgX'

def export(name, selected):
    bpy.ops.object.select_all(action='DESELECT')
    for ob in selected: ob.select_set(True)
    bpy.context.view_layer.objects.active=selected[0]
    bpy.ops.export_scene.gltf(filepath=str(OUT/name),export_format='GLB',
        use_selection=True,export_extras=True,export_animations=False,
        export_cameras=False,export_lights=False)

export('CAD_reference_vehicle_working.glb',[o for o in objects if o.get('group')!='environment'])
export('CAD_reference_complete_bench_working.glb',objects)

# Separate uninstalled candidate libraries are intentionally excluded from this stage.
library_receipts=[]

# Displaced photo approximations are preserved in their own scene rather than
# coexisting as duplicate installed supports in the default assembly.
if data.get('alternative_meshes'):
    alt = bpy.data.scenes.new('08_PRIOR_GEOMETRY_EXCLUSIVE_REFERENCES')
    alt.unit_settings.system='METRIC'
    alt['assembly_role']='PRIOR_GEOMETRY_NOT_CO_INSTALLED_WITH_NEW_SOURCE_DEFAULTS'
    alt['source_quantity_not_proven']=True
    for m in data['alternative_meshes']:
        me=bpy.data.meshes.new(m['name']+'_alternative')
        me.from_pydata(np.asarray(m['vertices']).reshape(-1,3).tolist(),[],
                      np.asarray(m['indices']).reshape(-1,3).tolist())
        ob=bpy.data.objects.new(m['name']+'_ALT',me);alt.collection.objects.link(ob)
        me.materials.append(material(m.get('color',[.5,.5,.5])))
        ob['original_name']=m['name'];ob['default_installed']=False
        ob['geometry_status']=m.get('geometry_status','PHOTO_INFERRED')

bpy.context.window.scene=scene
groups['04_BENCH_CAD_AND_PHOTO'].hide_render=True
scene.camera=cameras['vehicle']
bpy.ops.object.select_all(action='DESELECT')
OUT.mkdir(exist_ok=True)
(ROOT/'evidence/blender_composition.json').write_text(json.dumps({
    'main_scene_meshes':len(objects),'registry_rows':len(regobjects),
    'library_scenes':library_receipts,'static_scene_not_physical_result':True,
    'all_CAD_consumption_complete':False,'geometry_manifest':manifest,
},ensure_ascii=False,indent=2))
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'GTSD_CAD_full_review_working.blend'),compress=True)
print('SAVED_GEOMETRY_REVIEW',flush=True)
if os.environ.get('GTSD_RENDER', '0') != '1':
    raise SystemExit(0)
scene.render.filepath=str(OUT/'01_vehicle_CAD_additions_working.png')
bpy.ops.render.render(write_still=True)
scene.camera=cameras['bogie']
groups['01_VEHICLE_BODY'].hide_render=True
groups['03_PANTOGRAPH_SOURCE_AND_PHOTO'].hide_render=True
scene.render.filepath=str(OUT/'02_bogie_CAD_stack_working.png')
bpy.ops.render.render(write_still=True)
print('REVIEW_RENDER_DONE',flush=True)
groups['01_VEHICLE_BODY'].hide_render=False
groups['03_PANTOGRAPH_SOURCE_AND_PHOTO'].hide_render=False
groups['04_BENCH_CAD_AND_PHOTO'].hide_render=False
scene.camera=cameras['bench'];scene.render.filepath=str(OUT/'03_complete_bench_CAD_additions_working.png')
bpy.ops.render.render(write_still=True)
print('FULL_BENCH_RENDER_DONE',flush=True)
groups['04_BENCH_CAD_AND_PHOTO'].hide_render=True
groups['01_VEHICLE_BODY'].hide_render=True
groups['02_BOGIE_CAD_AND_PHOTO'].hide_render=True
scene.camera=cameras['panto'];scene.render.filepath=str(OUT/'04_pantograph_multiview_working.png')
bpy.ops.render.render(write_still=True)
groups['01_VEHICLE_BODY'].hide_render=False
groups['02_BOGIE_CAD_AND_PHOTO'].hide_render=False
groups['03_PANTOGRAPH_SOURCE_AND_PHOTO'].hide_render=True
skins=[o for o in objects if o.name.startswith('CAD_body_skin_')]
for o in skins:o.hide_render=True
scene.camera=cameras['interior'];scene.render.filepath=str(OUT/'05_interior_cutaway_working.png')
bpy.ops.render.render(write_still=True)
for o in skins:o.hide_render=False
groups['03_PANTOGRAPH_SOURCE_AND_PHOTO'].hide_render=False
exploded.render.filepath=str(OUT/'06_exploded_display_working.png')
bpy.context.window.scene=exploded
bpy.ops.render.render(write_still=True)
bpy.context.window.scene=scene
scene.camera=cameras['vehicle']
print('ALL_SIX_REVIEW_RENDERS_DONE',flush=True)
