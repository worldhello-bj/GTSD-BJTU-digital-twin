"""Independent actual-triangle material checks, never broad-phase-only clearance."""
from pathlib import Path
import gzip,json,itertools,hashlib,sys
import numpy as np
import cap_geometry as C
import geos_kernel as K
from build_candidate import write,meshhash
HERE=Path(__file__).resolve().parent

def bounds(m):
 v=C.verts(m);return v.min(0),v.max(0)
def positive_aabb(a,b,tol=1e-9):
 al,ah=bounds(a);bl,bh=bounds(b);return bool(np.all(np.minimum(ah,bh)-np.maximum(al,bl)>tol))
def main():
 out=HERE/'build';source=json.load(gzip.open(HERE/'source_seed.json.gz','rt'));seed={m['name']:m for m in source['meshes']};manifest=json.loads((out/'change_manifest.json').read_text());payload=json.load(gzip.open(out/'candidate_meshes.json.gz','rt'));allnew={m['name']:m for m in payload['meshes']};removed={r['name']for r in manifest['remove']};retained={n:m for n,m in seed.items()if n not in removed};retained.update({n:m for n,m in allnew.items()if m['metadata']['role']=='unchanged_source_remainder'});core={n:m for n,m in allnew.items()if m['metadata']['role']!='unchanged_source_remainder'}
 # Full-scene inventory proves that no object outside extracted ROI is skipped.
 misses=[];inventory_count=0
 for name,a in core.items():
  al,ah=bounds(a)
  for r in source['full_inventory']:
   if r['name']in removed:continue
   if np.all(np.minimum(ah,np.array(r['hi_m']))-np.maximum(al,np.array(r['lo_m']))>1e-9):
    inventory_count+=1
    if r['name']not in retained:misses.append([name,r['name']])
 assert not misses,misses
 topology={n:C.topology(m)for n,m in allnew.items()};assert all(t['closed']and t['positive_volume']for t in topology.values())
 before=C.sliced_overlap(seed['DETAIL_BENCH_turnout_inner_left_segment'],seed['ENV_main_rail_1'])
 rows=[];candidates=list(itertools.combinations(core,2))+[(a,b)for a in core for b in retained];allmeshes={**retained,**core}
 for i,(a,b) in enumerate(candidates):
  if not positive_aabb(allmeshes[a],allmeshes[b]):continue
  result=C.sliced_overlap(allmeshes[a],allmeshes[b]);v=result['volume_mm3'];status='POSITIVE_MATERIAL_OVERLAP'if v>1e-6 else 'NO_POSITIVE_VOLUME_FROM_ACTUAL_TRIANGLE_INTEGRATION'
  rows.append({'pair':[a,b],'status':status,**result});print(a,b,status,round(v,6),flush=True)
 # Exact projected triangle coverage provides a conservative 3D volume bound
 # for nonuniform I-sections too. No prism-volume assumption is used here.
 def projection(m):
  tri=C.verts(m)[C.faces(m)][:,:,:2]*1000;polygons=[]
  for q in tri:
   if abs(np.cross(q[1]-q[0],q[2]-q[0]))>1e-10:polygons.append('(('+K.coords(q.tolist()+[q[0].tolist()])+'))')
  return K.G.GEOSUnaryUnion(K.read('MULTIPOLYGON('+','.join(polygons)+')'))
 projection_cache={};coreproof=[]
 for a,b in itertools.combinations(core,2):
  al,ah=bounds(core[a]);bl,bh=bounds(core[b]);depth=max(0.,min(ah[2],bh[2])-max(al[2],bl[2]))*1000
  if depth<=1e-6:x={'method':'disjoint actual Z intervals or numerical boundary contact','common_z_depth_mm':depth,'volume_mm3_upper_bound':0.}
  else:
   for n in [a,b]:
    if n not in projection_cache:projection_cache[n]=projection(core[n])
   area=K.area(K.G.GEOSIntersection(projection_cache[a],projection_cache[b]));x={'method':'union of every actual projected surface triangle; intersection area times common Z depth is a conservative material-volume upper bound','projected_overlap_area_mm2':area,'common_z_depth_mm':depth,'volume_mm3_upper_bound':area*depth}
  coreproof.append({'pair':[a,b],**x})
 negative=[r for r in coreproof if r['volume_mm3_upper_bound']>1e-6]
 positives=[r for r in rows if r['status']=='POSITIVE_MATERIAL_OVERLAP']
 retained_source=source['scene_mesh_count']-len(removed)
 report={'schema':'gtsd-photo-turnout-material-audit-v1','geometry_validation_pass':True,'all_candidate_pairs_material_separated':not negative,'all_candidate_vs_retained_clearance_pass':not positives,'strict_whole_scene_clearance_pass':False,'no_solver_executed':True,'source_mesh_count':source['scene_mesh_count'],'unchanged_meshes':retained_source,'new_core_mesh_count':len(core),'source_remainder_count':len(allnew)-len(core),'all_scene_aabb_checked_pairs':len(core)*retained_source,'positive_aabb_core_vs_original_retained_count':inventory_count,'missing_from_material_check':misses,'candidate_mesh_topology':topology,'baseline_rail_crossing_actual_material':before,'core_pairs_full_triangle_projection_bound':coreproof,'positive_aabb_actual_triangle_checks':rows,'positive_material_pairs':positives,'scope':'All nine newly designed core meshes versus every retained combined-scene object, with full-scene broadphase and actual triangle integration on positive AABBs. Five unchanged rail remainders are checked against core meshes only. Unchanged-versus-unchanged pairs, wheel sweep, source guard conflict and whole-scene completeness are not certified.','caution':'The inherited source I-section and explicit full-depth slot are engineering choices, not as-built measurements. Existing fasteners are deliberately retained; positive material conflicts block assembly acceptance and must not be removed merely to make this audit green.'}
 write(out/'material_audit.json',report);print(json.dumps({'baseline_overlap_mm3':before['volume_mm3'],'new_core_overlap_pairs':len(negative),'retained_interference_pairs':len(positives),'actual_positive_aabb_checks':len(rows)}))
if __name__=='__main__':main()
