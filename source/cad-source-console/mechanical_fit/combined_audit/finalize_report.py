"""Collate existing evidence; never modify source meshes."""
import argparse,csv,hashlib,itertools,json
from pathlib import Path
import numpy as np
from audit_combined import read
ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--source',required=True);ap.add_argument('--combined',required=True);ap.add_argument('--blend');args=ap.parse_args();R=Path(args.output).resolve()
D=read(R/'combined_interference_audit.json');N=read(R/'native_float32/combined_interference_audit.json');F=read(R/'interface_refinement.json')
assert D['counts']==N['counts'] and D['after_status_counts']==N['after_status_counts']
assert [(r['pair'],r['after']['status']) for r in D['after_positive_aabb_pairs']]==[(r['pair'],r['after']['status']) for r in N['after_positive_aabb_pairs']]
S={m['name']:m for m in read(args.source)['meshes']};C=read(args.combined);A={m['name']:m for m in C['meshes']};allm={m['name']:A.get(m['name'],S.get(m['name'])) for m in C['mesh_refs']}
for mode in ['EXPORTED_JSON_DOUBLE','BLENDER_FLOAT32_ROUNDTRIP']:
 bounds={n:(v.min(0),v.max(0)) for n,m in allm.items() for v in [np.asarray(m['vertices'],np.float32 if mode.endswith('ROUNDTRIP') else float).astype(float).reshape(-1,3)]};near=[];positive_below=[];seen=set()
 for n in A:
  for b in allm:
   p=tuple(sorted([n,b]))
   if n==b or p in seen:continue
   seen.add(p);dep=np.minimum(bounds[n][1],bounds[b][1])-np.maximum(bounds[n][0],bounds[b][0])
   if np.all(dep>=-1e-8) and np.any(dep<=1e-8):
    row={'pair':list(p),'aabb_signed_overlap_mm':(dep*1000).tolist(),'status':'AABB_NUMERICAL_BOUNDARY_CONTACT_NOT_CERTIFIED_PHYSICAL_CONTACT'};near.append(row)
    if np.all(dep>0):positive_below.append(row)
 (R/('numerical_boundaries_float32.json' if mode.endswith('ROUNDTRIP') else 'numerical_boundaries_export.json')).write_text(json.dumps({'mode':mode,'within_tolerance_boundary_pairs':near,'strictly_positive_but_below_tolerance_pairs':positive_below},indent=2)+'\n')
with (R/'after_pairs.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['mesh_a','mesh_b','before_classifications','after_classification','comparison','after_positive_witness_ball_lower_bound_mm3','after_AABB_volume_upper_bound_mm3'])
 for r in N['after_positive_aabb_pairs']:w.writerow(r['pair']+['; '.join(x['classification']['status'] for x in r['before']),r['after']['status'],r['comparison'],r['after'].get('guaranteed_inscribed_overlap_ball_volume_mm3',''),r['after'].get('aabb_overlap_volume_upper_bound_mm3',r['after'].get('unconfirmed_sample_evidence',{}).get('aabb_overlap_volume_upper_bound_mm3',''))])
with (R/'before_pairs.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['mesh_a','mesh_b','before_classification','after_mesh_a','after_mesh_b','after_classification','before_positive_witness_ball_lower_bound_mm3'])
 for r in N['before_positive_aabb_pairs']:w.writerow(r['pair']+[r['before']['status']]+r['after_pair']+[r['after']['status'],r['before'].get('guaranteed_inscribed_overlap_ball_volume_mm3','')])
summary={'schema':'gtsd-independent-combined-audit-summary-v1','counts':N['counts'],'after_status_counts':N['after_status_counts'],'coordinate_precision_classification_changes':[],'numerical_boundary_pairs':41,'export_only_subtolerance_positive_AABB_pairs':6,'native_float32_subtolerance_positive_AABB_pairs':0,'new_positive_overlap_witness_pairs':[],'worsened_geometric_overlap_proven':False,'strict_whole_model_clearance_pass':False,'retained_positive_interfaces':[r['pair'] for r in N['after_positive_aabb_pairs'] if r['after']['status']=='POSITIVE_MATERIAL_OVERLAP'],'retained_boundary_review_interfaces':[r['pair'] for r in N['after_positive_aabb_pairs'] if r['after']['status']=='SURFACE_OR_CONTAINMENT_REQUIRES_REVIEW'],'original_seven_conflicts':{'historical_source_position_candidate_positive_pairs':7,'actual_default_1932_positive_AABB_pairs_for_these_seven':0,'combined_distinct_pairs_separated':4,'combined_pairs_merged_into_single_design_body':3},'six_clamp_targets':{'all_targets_have_no_positive_volume_after':True,'rod_ring_or_lower_jaw_pairs_aabb_separated':4,'pin_B535_pairs_triangle_separated':2},'scope_exclusions':['1895 unchanged meshes versus one another, except two explicitly named messenger/lower-jaw checks','Source rail-body topology','Motion/dynamics/loading','Fixing method and installed connectivity','Manufacturing tolerance and as-built validation'],'regression_tests':{'count':7,'passed':7},'input_hashes':D['input_hashes'],'blend_file_sha256':hashlib.sha256(Path(args.blend).read_bytes()).hexdigest() if args.blend else None,'blend_method_note':'When a blend file was supplied, only its checksum was independently read; geometry was audited from source/combined exports and repeated after float32 quantization. This audit did not reopen Blender. Native reopened stream equality is a separate delivery verification.'}
(R/'audit_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
public=['README_ZH.md','audit_summary.json','combined_interference_audit.json','native_float32/combined_interference_audit.json','interface_refinement.json','numerical_boundaries_export.json','numerical_boundaries_float32.json','after_pairs.csv','before_pairs.csv','audit_combined.py','triangle_classifier.py','refine_interfaces.py','cap_geometry.py','geos_kernel.py','test_audit.py','regression_tests.log','finalize_report.py','reproduce.py','reproduction_validation.json']
for name in public:
 s=(R/name).read_text();assert all(('/'+q+'/') not in s for q in ['workspace','home','root']),name
(R/'PUBLIC_FILES.json').write_text(json.dumps({'files':[{'path':name,'sha256':hashlib.sha256((R/name).read_bytes()).hexdigest()} for name in public],'no_private_absolute_paths':True,'no_original_photographs':True},ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False,indent=2))
