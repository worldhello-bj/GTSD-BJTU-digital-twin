"""Full-scene broadphase, actual cap material / actual triangle integration."""
import sys,json,gzip,itertools,datetime
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import cap_geometry as C,geos_kernel as K
from build_candidate import write
H=Path(__file__).resolve().parent
s=json.load(gzip.open(ROOT/'source_seed.json.gz','rt'));seed={m['name']:m for m in s['meshes']};new={m['name']:m for m in json.load(gzip.open(H/'candidate_meshes.json.gz','rt'))['meshes']};manifest=json.loads((H/'change_manifest.json').read_text());remove={r['name']for r in manifest['remove']};retained={n:m for n,m in seed.items()if n not in remove};allm={**retained,**new};bounds={n:(C.verts(m).min(0),C.verts(m).max(0))for n,m in allm.items()};missing=[]
for a in new:
 if new[a].get("metadata",{}).get("role")=="unchanged_source_remainder":continue
 al,ah=bounds[a]
 for r in s['full_inventory']:
  if r['name']in remove:continue
  if np.all(np.minimum(ah,r['hi_m'])-np.maximum(al,r['lo_m'])>1e-9)and r['name']not in retained:missing.append([a,r['name']])
assert not missing,missing
rows=[];caps={};candidates=list(itertools.combinations(new,2))+[(a,b)for a in new for b in retained]
for a,b in candidates:
 al,ah=bounds[a];bl,bh=bounds[b]
 if not np.all(np.minimum(ah,bh)-np.maximum(al,bl)>1e-9):continue
 ma,mb=allm[a],allm[b]
 # Source-preserving rail remainders are long: unchanged pairs are outside scope.
 if ma.get('metadata',{}).get('role')=='unchanged_source_remainder' and (b in retained or mb.get('metadata',{}).get('role')=='unchanged_source_remainder'):continue
 if all(m.get('metadata',{}).get('uniform_z_extrusion')for m in [ma,mb]):
  for n in [a,b]:
   if n not in caps:caps[n]=C.cap(allm[n],2,np.zeros(3),'min')
  p=K.G.GEOSIntersection(caps[a],caps[b]);area=K.area(p);K.G.GEOSGeom_destroy(p);r={'method':'actual triangulated cap material intersection times common Z, certified uniform extrusions','volume_mm3':area*(min(ah[2],bh[2])-max(al[2],bl[2]))*1000}
 else:r=C.sliced_overlap(ma,mb)
 rows.append({'pair':[a,b],**r,'positive':bool(r['volume_mm3']>1e-6)});print(a,b,round(r['volume_mm3'],8),flush=True)
pos=[r for r in rows if r['positive']];now=datetime.datetime.now(datetime.timezone.utc)
report={'schema':'gtsd-full-hold-down-actual-material-audit-v1','completed_utc':now.isoformat(),'completed_beijing':now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'full_scene_original_count':s['scene_mesh_count'],'result_scene_count':manifest['result_mesh_count'],'source_retained_count':s['scene_mesh_count']-len(remove),'added_mesh_count':len(new),'full_scene_broadphase_pairs':sum(m.get('metadata',{}).get('role')!='unchanged_source_remainder'for m in new.values())*(s['scene_mesh_count']-len(remove)),'missing_from_exact_check':missing,'actual_positive_aabb_checks':len(rows),'positive_material_pairs':pos,'actual_mesh_checks':rows,'all_new_hardware_meshes_closed_positive':all(C.topology(m)['closed']and C.topology(m)['positive_volume']for m in new.values()),'strict_whole_scene_clearance_pass':False,'no_solver_executed':True,'caution':'Actual-model material proof only. Outer-envelope assembly count, cross sections, fixation and thread abstraction remain research choices. Historical 17 proxy overlaps are preserved in the separate v1 review; source guards and unchanged-versus-unchanged pairs are not certified.'}
write(H/'material_audit.json',report);print('COMPLETE',len(rows),'checks',len(pos),'positive pairs',flush=True)
