from pathlib import Path
import json,numpy as np
HERE=Path(__file__).resolve().parent;G=json.loads((HERE/'native_edge_graph.json').read_text());rows=G['source_curves'];by={r['handle']:r for r in rows};details=['4ECE9','4ECEE','4ED02','4ED2C','4ED5C','4ED5D'];connections=[]
for h in details:
 q=np.array(by[h]['points'])
 for endpoint,p in enumerate([q[0],q[-1]]):
  hits=[]
  for r in rows:
   if r['handle']==h:continue
   vs=np.array(r['points'])
   for i,(a,b)in enumerate(zip(vs[:-1],vs[1:])):
    t=np.dot(p-a,b-a)/np.dot(b-a,b-a)
    if -1e-10<=t<=1+1e-10:
     residual=np.linalg.norm(p-(a+t*(b-a)))
     if residual<1e-5:hits.append({'handle':r['handle'],'polyline_segment':i,'parameter':float(t),'relation':'ENDPOINT'if abs(t)<1e-9 or abs(t-1)<1e-9 else'INTERIOR_OF_FINITE_SOURCE_EDGE','residual_mm':float(residual)})
  connections.append({'handle':h,'endpoint':endpoint,'world_XY_mm':p.tolist(),'exact_finite_edge_connections':hits})
# Paired horizontal head boundaries are finite plan evidence only.
intervals=[]
for h in ['4ECED','4ECEC','4ECEB','4ECEA','4ED15']:
 q=np.array(by[h]['points']);xs=sorted([float(q[0,0]),float(q[-1,0])]);width=abs(float(np.array(by['4ED17']['points'])[0,1]-q[0,1]));intervals.append({'lower_head_handle':h,'upper_head_handle':'4ED17','world_X_interval_mm':xs,'plan_head_band_width_mm':width,'z_section_confirmed':False,'existing_complete_main_head_rectangle_already_matches_this_local_plan_band':True})
intervals.sort(key=lambda r:r['world_X_interval_mm'][0]);gaps=[]
for a,b in zip(intervals[:-1],intervals[1:]):gaps.append({'after':a['lower_head_handle'],'before':b['lower_head_handle'],'gap_X_interval_mm':[a['world_X_interval_mm'][1],b['world_X_interval_mm'][0]],'gap_length_mm':b['world_X_interval_mm'][0]-a['world_X_interval_mm'][1],'not_permission_to_delete_material':True})
report={'schema':'gtsd-complete-track-connection-review-v1','reviewed_finite_curves':len(rows),'detail_endpoints':connections,'paired_head_plan_intervals':intervals,'gaps_between_horizontal_head_segments':gaps,'key_correction_to_single_handle_view':'4ED15 alone is not the complete main inner head edge. 4ECED provides the edge at the X≈-8970 mm head overlap, so missing-4ED15 extrapolation does not explain that overlap.','source_curve_deduplication':'These native records are distinct finite top-level Model records in the common track layout; they are not repeated transformed block instances. Their different geometries do not match as duplicate projections. This does not establish whether the drawing superimposes operating states.','topology_limit':'An endpoint on a finite rail edge establishes a planar junction, not automatically a manufacturing joint or a single connected material region. The graph contains open chains and crossing material-band interpretations. Flangeway and switch-route semantics remain undetermined.','source_determined_mesh_patch':'Already-correct paired straight plan bands do not warrant a change. No unique complete replacement head solid follows from these connections alone. Provide all relevant native curves and piecewise boundary constraints; retain unknown lengths as explicitly unresolved, never silently delete them for collision clearance.','unknown_physical_boundaries':['Closed top rail/frog contours through crossing band intersections','Flangeway profile/depth and active switch route','Meaning of 4ECE9/4ED2C open trace beyond straight main segment','Rail/guard shared-base or separately fixed relationship']}
(HERE/'complete_connection_review.json').write_text(json.dumps(report,indent=2)+'\n')
# Exact source-curve overlay: not a triangulated material or installed part.
overlay={'schema':'gtsd-native-track-source-curves-v1','units':'m','frame':'same baked world XY as console; Z=0 is source drafting plane','default_enabled':False,'candidate_material':False,'installed_parts_added':0,'physics_binding':False,'collision_enabled':False,'replace_names':[],'source_curves':[{'name':'NATIVE_TRACK_EDGE_'+r['handle'],'handle':r['handle'],'native_record_id':r['native_record_id'],'raw_line_range':r['source_line_range'],'role':r['class'],'world_points_m':[[v[0]/1000,v[1]/1000,0]for v in r['points']],'material_width_m':None,'material_thickness_m':None}for r in rows],'paired_head_interval_constraints':intervals,'unknown_regions':gaps}
(HERE/'native_track_curve_overlay.json').write_text(json.dumps(overlay,indent=2)+'\n')
print('CONNECTIONS')
for r in connections:print(r['handle'],r['endpoint'],[(x['handle'],x['relation'])for x in r['exact_finite_edge_connections']])
print('GAPS',gaps)
