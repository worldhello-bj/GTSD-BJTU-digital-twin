"""Single-threaded native exact CSG for explicit inherited I-section candidate.
Coordinates are local millimetres during CSG to avoid large world-offset loss.
No renderer, physics, simulation, network, images, or original photo is used.
"""
import bpy,bmesh,json,sys
from pathlib import Path
args=sys.argv[sys.argv.index('--')+1:];src,out=map(Path,args);request=json.loads(src.read_text());origin=request['origin_m']
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
def obj(name,m):
 data=bpy.data.meshes.new(name);v=[[(m['vertices'][i+k]-origin[k])*1000 for k in range(3)]for i in range(0,len(m['vertices']),3)];f=[m['indices'][i:i+3]for i in range(0,len(m['indices']),3)];data.from_pydata(v,[],f);data.update();bm=bmesh.new();bm.from_mesh(data);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(data);bm.free();o=bpy.data.objects.new(name,data);bpy.context.collection.objects.link(o);return o
objects={n:obj(n,m)for n,m in request['inputs'].items()}
def boolean(target,tool,operation):
 bpy.context.view_layer.objects.active=target;mod=target.modifiers.new('explicit_'+operation,'BOOLEAN');mod.operation=operation;mod.solver='EXACT';mod.object=tool;bpy.ops.object.modifier_apply(modifier=mod.name)
for spec in request.get('keep_masks',[]):
 target=objects[spec['top']]
 if spec['bottom']:boolean(target,objects[spec['bottom']],'UNION')
 objects[spec['key']+'_keep']=target
result={}
for spec in request['operations']:
 target=objects[spec['base']].copy();target.data=target.data.copy();bpy.context.collection.objects.link(target)
 for operation,operand in spec['steps']:boolean(target,objects[operand],operation)
 bm=bmesh.new();bm.from_mesh(target.data);bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-4);bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=1e-5);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bmesh.ops.triangulate(bm,faces=list(bm.faces));bm.to_mesh(target.data);bm.free();target.data.update()
 result[spec['name']]={'name':spec['name'],'vertices':[float(v.co[k])/1000+origin[k]for v in target.data.vertices for k in range(3)],'indices':[i for p in target.data.polygons for i in p.vertices]}
out.write_text(json.dumps({'schema':'gtsd-inherited-section-csg-result-v1','blender_version':bpy.app.version_string,'local_units':'mm','numerical_weld_tolerance_mm':1e-4,'meshes':result},sort_keys=True,separators=(',',':'))+'\n')
print('SECTION_CSG_COMPLETE',len(result))
