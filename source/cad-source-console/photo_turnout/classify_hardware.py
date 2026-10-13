"""Classify modeled intersections separately from physical hardware evidence."""
from pathlib import Path
import gzip,json,collections,csv
import numpy as np
from build_candidate import write
H=Path(__file__).resolve().parent;out=H/'build';source=json.load(gzip.open(H/'source_seed.json.gz','rt'));meshes={m['name']:m for m in source['meshes']};audit=json.loads((out/'material_audit.json').read_text());rows=[]
for pair in audit['positive_material_pairs']:
 name=pair['pair'][1]
 if name.startswith('PHOTO_RETAIN'):continue
 m=meshes[name];d=m['metadata'];v=np.asarray(m['vertices']).reshape(-1,3);center=(v.min(0)+v.max(0))/2
 if name.startswith('ENV_fastener'):
  category='PHOTO_INFERRED_GENERIC_BOLT_HEAD_PROXY';num=int(name.rsplit('.',1)[1]);chair='ENV_rail_chair.'+f'{num//2:03d}';seat=next((x for x in source['meshes']if x['metadata'].get('replaces_mesh_name')==chair),None);handles=seat['metadata'].get('source_handles',[])if seat else[];parent={'original_chair':chair,'source_association_method':'numbered two-fastener/one-chair hypothesis, verified by original chair inverse-transform X where available','revised_chair':seat['name']if seat else('FIT_TRACK_station03_common_support'if chair in ['ENV_rail_chair.007','ENV_rail_chair.009']else None)}
  if seat:
   matrix=np.asarray(seat['metadata']['old_baked_world_to_new_baked_world_matrix']);sv=np.asarray(seat['vertices']).reshape(-1,3);sc=(sv.min(0)+sv.max(0))/2;old=sc-matrix[:3,3];parent.update(original_centerline_X_difference_mm=float((center[0]-old[0])*1000),old_pair_side_offset_Y_mm=float((center[1]-old[1])*1000),chair_source_relocation_delta_mm=(matrix[:3,3]*1000).tolist());assert abs(parent['original_centerline_X_difference_mm'])<.001
  sourceposition='Legacy photo-proxy pose. Source-station chair relocation is separate; the fastener itself has no native fastening datum.'
 elif name.startswith('CAD_TRACK_REV_branch_clip'):
  category='NATIVE_STATION_XY_WITH_INHERITED_GUESSED_BOLT_HEAD';old=d['replaces_mesh_name'];chair=old.replace('fastener','chair').rsplit('_',1)[0];parent={'original_chair':chair,'source_station_index':d['source_station_index'],'revised_chair':'CAD_TRACK_REV_'+chair.removeprefix('DETAIL_BENCH_'),'source_association_method':'explicit full three-mesh group in original source metadata'};handles=d['source_handles'];sourceposition='Native sleeper-axis / rail-band intersection is sourced; individual clip offset, section, yaw and Z are inherited guessed geometry.'
 else:
  category='ENGINEERING_INSTALLATION_OF_INHERITED_BOLT_HEAD_PROXY';parent={'original_chair':'DETAIL_BENCH_branch_chair_03_1','revised_chair':'FIT_TRACK_station03_common_support','source_association_method':'explicit station03 fit group and retained proxy shape'};handles=d.get('source_refs',[]);sourceposition='Declared 42 mm exterior support-normal mounting offsets; station03 XY uses native rail/station data, final individual fastener pose is an engineering design.'
 rows.append({'name':name,'intersects_candidate':pair['pair'][0],'modeled_material_overlap_mm3':pair['volume_mm3'],'physical_hardware_conflict_confirmed':False,'category':category,'mesh_shape':'eight-sided vertical circular-head proxy; no spring clip, shank, toe, nut/washer stack or seating detail','dimensions_mm':(np.ptp(v,axis=0)*1000).tolist(),'unique_vertices':len(np.unique(v,axis=0)),'triangle_count':len(m['indices'])//3,'source_exact_clip_solid':False,'source_position_evidence':sourceposition,'source_handles':handles,'parent_clip_seat_assembly':parent,'source_geometry_status':d.get('geometry_status'),'source_metadata_witness':d,'next_action':'Reconstruct complete clip/seat/head/shank/foot-contact family and register its mounting datum as one assembly. Do not translate this head proxy alone, shrink it, or use its overlap as proof of a real collision.'})
counts=dict(collections.Counter(r['category']for r in rows));assert len(rows)==17 and counts=={'PHOTO_INFERRED_GENERIC_BOLT_HEAD_PROXY':9,'NATIVE_STATION_XY_WITH_INHERITED_GUESSED_BOLT_HEAD':6,'ENGINEERING_INSTALLATION_OF_INHERITED_BOLT_HEAD_PROXY':2}
reference=[]
for m in source['meshes']:
 if m['name'].startswith('CAD_track_fastener_4EDF8'):
  reference.append({'name':m['name'],'metadata':m['metadata'],'use_limit':'Exact CAD plan reference with explicit extrusion/Z hypothesis, not a verified 3D spring clip. Its INSERT belongs to the other rail context; do not transplant or relabel as one of these 17 without mounting association.'})
report={'schema':'gtsd-photo-turnout-hardware-provenance-v1','hardware_mesh_pairs':17,'exact_native_3D_clip_solids_among_conflicts':0,'counts':counts,'physical_hardware_conflicts_confirmed':0,'model_intersections_are_still_real_for_the_supplied_meshes':True,'rows':rows,'separate_native_plan_references':reference,'photo7_qualitative_guidance':['Foreground outer-rail fasteners show looped/spring-like shapes and separate seats; a solid head alone is not that assembly.','The frog long-plate area shows distinct small rectangular seating/clamping pieces and bolt heads. Their exact identity and Z/axis cannot be assigned to all proxies from this single oblique photo.','Rail-web-side heads and vertical seat fasteners must be distinguished before choosing an axis.','No photo-derived dimensions or uniquely identified station/serial numbers are available.']}
write(out/'hardware_provenance.json',report)
with (out/'hardware_provenance.csv').open('w',newline='')as f:
 w=csv.writer(f);w.writerow(['mesh','category','modeled_overlap_mm3','physical_conflict_confirmed','source_handles','parent_chair'])
 for r in rows:w.writerow([r['name'],r['category'],r['modeled_material_overlap_mm3'],False,' '.join(r['source_handles']),r['parent_clip_seat_assembly']['original_chair']])
print(json.dumps(counts))
