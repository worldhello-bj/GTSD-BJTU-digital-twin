import bpy,json,hashlib,gzip
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];O=R/'outputs/pass3';source=json.load(gzip.open(R/'inputs/reference_assembly.json.gz','rt'));expected={m['name']:{'group':m['group'],'float32_vertices_sha256':hashlib.sha256(np.asarray(m['vertices'],dtype=np.float32).tobytes()).hexdigest(),'int32_indices_sha256':hashlib.sha256(np.asarray(m['indices'],dtype=np.int32).tobytes()).hexdigest()}for m in source['meshes']};reports=[]
for name,vehicle in [('GTSD_CAD_vehicle_and_bench.blend',False),('GTSD_CAD_vehicle.blend',True)]:
 bpy.ops.wm.open_mainfile(filepath=str(O/name));main=bpy.data.scenes['01_CAD_STATIC_ASSEMBLY_WORKING'];bpy.context.window.scene=main
 objs=[o for o in main.objects if o.type=='MESH'];exp={k:v for k,v in expected.items()if not vehicle or v['group']!='environment'}
 assert {o['source_name']for o in objs}==set(exp)
 for o in objs:
  e=exp[o['source_name']];v=np.array([tuple(p.co)for p in o.data.vertices],dtype=np.float32)
  f=np.array([tuple(p.vertices)for p in o.data.polygons],dtype=np.int32)
  assert hashlib.sha256(v.tobytes()).hexdigest()==e['float32_vertices_sha256'],o.name
  assert hashlib.sha256(f.tobytes()).hexdigest()==e['int32_indices_sha256'],o.name
  assert not o.get('native_pose_bound_in_this_scene') and o.rigid_body is None
 assert len(bpy.data.scenes)==2
 assert len([o for o in main.objects if o.type=='EMPTY'])==109
 reports.append({'file':name,'source_meshes':len(objs),'triangles':sum(len(o.data.polygons)for o in objs),'all_vertices_and_indices_exact':True,'scene_count':2,'registry_empties':109,'physical_binding':False,'sha256':hashlib.sha256((O/name).read_bytes()).hexdigest()})
(R/'evidence/trimmed_native_validation.json').write_text(json.dumps({'passed':True,'reports':reports},indent=2)+'\n');print(json.dumps(reports),flush=True)
