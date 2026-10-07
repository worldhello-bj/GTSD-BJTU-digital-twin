"""Reopen saved B12 and validate every solved pose and original mesh binding."""
import hashlib
import json
from pathlib import Path
import bpy
from mathutils import Matrix,Quaternion

ROOT=Path(__file__).resolve().parent/'replay'
asset=ROOT/'assets/GTSD_BJTU_B12_cover_maintenance_bench.blend'
before=hashlib.sha256(asset.read_bytes()).hexdigest()
bpy.ops.wm.open_mainfile(filepath=str(asset),load_ui=False,use_scripts=False)
records=[]
for case,name in [('handle_remove','B12_COVER_HANDLING'),('drop','B12_COVER_DROP')]:
    scene=bpy.data.scenes[name]; bpy.context.window.scene=scene
    replay=json.loads((ROOT/'inputs'/(case+'_replay.json')).read_text(encoding='utf-8'))
    digest=hashlib.sha256((ROOT/'inputs'/(case+'.xml')).read_bytes()).hexdigest()
    assert digest==replay['model_xml_sha256']==scene['model_xml_sha256']
    nodes={o['physics_body']:o for o in scene.objects if o.get('physics_body')}
    meshes={o['source_object']:o for o in scene.objects if o.get('source_object')}
    mapping=json.loads(scene['display_mapping_json']); pose_error=mesh_error=0.
    assert len(nodes)==len(meshes)==4
    def error(a,b):return max(abs(a[i][j]-b[i][j]) for i in range(4) for j in range(4))
    for frame in replay['frames']:
        f=1+scene.render.fps*frame['time_s']; scene.frame_set(int(f),subframe=f-int(f)); bpy.context.view_layer.update()
        transforms={key:Matrix.Translation(s['position'])@Quaternion(s['quaternion_wxyz']).to_matrix().to_4x4() for key,s in frame['body_transforms'].items()}
        for key,matrix in transforms.items():pose_error=max(pose_error,error(nodes[key].matrix_world,matrix))
        for record in mapping:
            obj=meshes[record['source_object']]; expected=transforms[record['solver_body']]@Matrix(record['mesh_bind_matrix'])
            mesh_error=max(mesh_error,error(obj.matrix_world,expected)); assert obj.animation_data is None
    assert max(pose_error,mesh_error)<2e-5
    records.append(dict(case=case,frames=len(replay['frames']),bodies=4,source_meshes=4,
                        maximum_body_matrix_error=pose_error,maximum_source_mesh_matrix_error=mesh_error,xml_sha256=digest))
drivers=[o.name for o in bpy.data.objects if o.animation_data and len(o.animation_data.drivers)]
assert not drivers and hashlib.sha256(asset.read_bytes()).hexdigest()==before
record=dict(passed=True,blend_sha256=before,cases=records,error_limit=2e-5,expression_drivers=0,
            scope='saved solver pose and source-shape binding; independent inferred maintenance-bench layout')
(ROOT/'reports/b12_readback.json').write_text(json.dumps(record,indent=2),encoding='utf-8',newline='\n')
print(json.dumps(record,indent=2),flush=True)
