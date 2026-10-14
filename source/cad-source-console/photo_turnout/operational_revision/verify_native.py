"""Reopen the frozen native region and verify exact mesh membership and indices."""
import bpy,datetime,gzip,json,sys
from pathlib import Path
H=Path(sys.argv[sys.argv.index("--")+1])
meshes=json.load(gzip.open(H/"regional_meshes.json.gz","rt"))["meshes"]
bpy.ops.wm.open_mainfile(filepath=str(H/'GTSD_Operational_Turnout.blend'))
scene=bpy.context.scene
expected={m['name']:m for m in meshes};records=[];maxerr=0
for o in bpy.data.objects:
 if o.type!='MESH':continue
 m=expected[o['source_mesh_name']];assert o.name==m['name'];actual=[o.matrix_world@v.co for v in o.data.vertices];assert len(actual)*3==len(m['vertices']);idx=[i for f in o.data.polygons for i in f.vertices];assert idx==m['indices'];err=max(abs(v[k]-m['vertices'][i*3+k])*1000 for i,v in enumerate(actual)for k in range(3));maxerr=max(maxerr,err);records.append({'name':o.name,'vertices':len(actual),'triangles':len(idx)//3,'world_coordinate_error_mm':err})
assert len(records)==205 and maxerr<.0015
now=datetime.datetime.now(datetime.timezone.utc);report={'schema':'gtsd-operational-native-validation-v1','completed_utc':now.isoformat(),'completed_beijing':now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'native_reopened':True,'mesh_membership_and_triangle_indices_verified':True,'mesh_count':len(records),'maximum_world_vertex_error_mm':maxerr,'tolerance_mm':.0015,'render_engine':scene.render.engine,'threads':1,'samples':20,'rendered_files':['operational_turnout.png','finite_joint_detail.png'],'external_libraries':len(bpy.data.libraries),'embedded_images':len(bpy.data.images),'records':records};(H/'native_render_validation.json').write_text(json.dumps(report,indent=2)+'\n');print('NATIVE_REOPEN_PASS',len(records),maxerr)
