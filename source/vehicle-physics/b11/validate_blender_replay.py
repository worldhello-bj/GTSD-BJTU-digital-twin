"""Read every saved vehicle/cable pose and capsule binding in both B11 cases."""
import csv
import hashlib
import json
import math
from pathlib import Path
import bpy
from mathutils import Matrix, Quaternion, Vector

ROOT=Path(__file__).resolve().parent
DISPLAY=ROOT/'replay'
asset=DISPLAY/'assets/GTSD_BJTU_B11_source_cable_dynamics.blend'


def error(a,b):
    return max(abs(a[i][j]-b[i][j]) for i in range(4) for j in range(4))


def validate_cases(display=DISPLAY, pairs=None):
    inventory=json.loads((display/'reports/source_door_closed_rest.json').read_text(encoding='utf-8'))
    records={r['name']:r for group in inventory['groups'] for r in group['objects']}
    reports=[]
    pairs = pairs or [('service_brake','B11_MECHANICAL_REPLAY'),('doors_open_close','B11_CABINET_CABLE_REPLAY')]
    for case,scene_name in pairs:
        scene=bpy.data.scenes[scene_name]; bpy.context.window.scene=scene
        path=display/'inputs'/('final_replay.json' if case=='service_brake' else case+'_replay.json')
        xml=display/'inputs'/('final_model.xml' if case=='service_brake' else case+'.xml')
        replay=json.loads(path.read_text(encoding='utf-8'))
        digest=hashlib.sha256(xml.read_bytes()).hexdigest()
        assert scene['model_xml_sha256']==replay['model_xml_sha256']==digest
        nodes={o['physics_body']:o for o in scene.objects if o.get('physics_body') and not o.get('display_only_derived')}
        parts={o['source_object']:o for o in scene.objects if o.get('source_object') in records}
        spec=replay['cable_configuration']; legacy={n for r in spec['routes'] for n in r['source_objects']}
        assert not any(o.get('source_object') in legacy for o in scene.objects)
        assert len(nodes)==44+spec['total_links'] and len(parts)==84
        capsules={o['solver_body']:o for o in scene.objects if o.get('solver_capsule_length_m')}
        assert len(capsules)==spec['total_links']
        body_error=part_error=length_error=binding_error=mesh_error=0.
        # Inspect saved mesh coordinates, not only the object's dimension tags.
        # Every vertex lies on the capsule surface in its fixed local frame.
        for route in spec['routes']:
            points=[Vector(p) for p in route['reference_centerline_world_m']]
            radius=route['radius_m']
            for i,name in enumerate(route['body_names']):
                obj=capsules[name]
                expected_length=(points[i+1]-points[i]).length
                assert obj.type=='MESH' and not obj.modifiers and obj.data.shape_keys is None
                assert abs(obj['solver_radius_m']-radius)<1e-12
                mesh_error=max(mesh_error,abs(obj['solver_capsule_length_m']-expected_length))
                vertices=[vertex.co for vertex in obj.data.vertices]
                assert vertices
                mesh_error=max(mesh_error,abs(min(p.z for p in vertices)+radius),
                               abs(max(p.z for p in vertices)-expected_length-radius))
                for p in vertices:
                    axial_offset=p.z-min(expected_length,max(0.,p.z))
                    surface_radius=math.sqrt(p.x*p.x+p.y*p.y+axial_offset*axial_offset)
                    mesh_error=max(mesh_error,abs(surface_radius-radius))
        for frame in replay['frames']:
            f=1+frame['time_s']*scene.render.fps
            scene.frame_set(int(f),subframe=f-int(f)); bpy.context.view_layer.update()
            matrices={n:Matrix.Translation(t['position'])@Quaternion(t['quaternion_wxyz']).to_matrix().to_4x4()
                      for n,t in frame['body_transforms'].items()}
            for name,matrix in matrices.items():
                body_error=max(body_error,error(nodes[name].matrix_world,matrix))
            for name,obj in parts.items():
                part_error=max(part_error,error(obj.matrix_world,matrices[obj['solver_body']]@Matrix(records[name]['source_hinge_local_matrix'])))
            for route in spec['routes']:
                points=[Vector(p) for p in route['reference_centerline_world_m']]
                for i,name in enumerate(route['body_names']):
                    obj=capsules[name]; length=obj['solver_capsule_length_m']
                    a=obj.matrix_world@Vector((0,0,0)); b=obj.matrix_world@Vector((0,0,length))
                    expected_a=matrices[name]@Vector((0,0,0)); expected_b=matrices[name]@(points[i+1]-points[i])
                    binding_error=max(binding_error,(a-expected_a).length,(b-expected_b).length)
                    length_error=max(length_error,abs((b-a).length-length))
                    assert obj.parent==nodes[name] and obj.animation_data is None
        assert max(body_error,part_error,binding_error,length_error,mesh_error)<2e-5
        reports.append(dict(case=case,frames=len(replay['frames']),bodies=len(nodes),source_door_parts=len(parts),
                            cable_capsules=len(capsules),maximum_body_matrix_error=body_error,
                            maximum_source_part_matrix_error=part_error,maximum_capsule_binding_error_m=binding_error,
                            maximum_capsule_length_error_m=length_error,maximum_local_capsule_surface_error_m=mesh_error,
                            xml_sha256=digest))
    return reports


def main():
    before=hashlib.sha256(asset.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(asset),load_ui=False,use_scripts=False)
    reports=validate_cases()
    bench=bpy.data.scenes['B10_INFERRED_AUXILIARY_BENCH']; bpy.context.window.scene=bench
    path=ROOT.parent/'b10/replay/inputs/auxiliary_startup.csv'
    identity=json.loads((path.with_name('auxiliary_startup_identity.json')).read_text(encoding='utf-8'))
    assert hashlib.sha256(path.read_bytes()).hexdigest()==identity['output_sha256']==bench['probe_sha256']
    with path.open(encoding='utf-8',newline='') as stream:
        rows=list(csv.DictReader(stream))
    aux_error=0.; scale=bench['pump_geometry_display_scale']; radius=identity['configuration']['auxiliary_model']['pump']['crank_radius_m']
    for frame,row in enumerate(rows,1):
        bench.frame_set(frame); bpy.context.view_layer.update()
        theta,extension=float(row['pump_phase_rad']),float(row['pump_piston_displacement_m'])
        pin=bpy.data.objects['B10_SOLVED_CRANK_PIN']; expected=(scale*radius*math.sin(theta),-.05,scale*radius*math.cos(theta))
        aux_error=max(aux_error,max(abs(pin.location[i]-expected[i]) for i in range(3)))
        for name,offset in [('B10_SOLVED_YOKE',0),('B10_SOLVED_PISTON',.4),('B10_SOLVED_PISTON_ROD',.2)]:
            aux_error=max(aux_error,abs(bpy.data.objects[name].location.z-offset-scale*(radius-extension)))
        for name,field in [('B10_SOLVED_CRANK','pump_angle_rad'),('B10_SOLVED_COMPRESSOR_FAN','compressor_fan_angle_rad'),('B10_SOLVED_BAY_FAN','bay_fan_angle_rad')]:
            aux_error=max(aux_error,abs(bpy.data.objects[name].rotation_euler[1]-float(row[field])))
    assert len(rows)==501 and aux_error<2e-5
    drivers=[o.name for o in bpy.data.objects if o.animation_data and len(o.animation_data.drivers)]
    assert not drivers and hashlib.sha256(asset.read_bytes()).hexdigest()==before
    report=dict(passed=True,blend_sha256=before,cases=reports,auxiliary_frames=len(rows),maximum_auxiliary_error=aux_error,
                error_limit=2e-5,expression_drivers=0,scope='saved pose and geometry identity; physical acceptance is separate')
    (DISPLAY/'reports/b11_readback.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
