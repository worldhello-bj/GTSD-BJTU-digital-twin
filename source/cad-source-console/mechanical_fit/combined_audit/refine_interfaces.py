"""Refine nominal interfaces on exact exported and native-float32 meshes.
GEOS is used only for 2D sections of the supplied triangles; no proxy booleans.
"""
import argparse,copy,itertools,json,math
from pathlib import Path
import numpy as np
import cap_geometry as C
from audit_combined import read,sha,winding,surface_distance
from triangle_classifier import classify

def geom(m):return np.array(m['vertices']).reshape(-1,3),np.array(m['indices']).reshape(-1,3)
def verify_uniform(m,axis,lo,hi):
 v=C.verts(m);lo=max(lo,v[:,axis].min());hi=min(hi,v[:,axis].max());o=(v.min(0)+v.max(0))/2;p=C.cap(m,axis,o);xs=sorted(set([float(lo),float(hi)]+[float(x) for x in v[:,axis] if lo<x<hi]));checks=[]
 for a,b in zip(xs,xs[1:]):
  if b-a<1e-12:continue
  q=C.actual_section(m,axis,(a+b)/2,o);delta=C.K.area(C.K.G.GEOSDifference(p,q))+C.K.area(C.K.G.GEOSDifference(q,p));h=C.K.D();C.K.G.GEOSHausdorffDistance(p,q,C.K.C.byref(h));lp=C.K.D();lq=C.K.D();C.K.G.GEOSLength(p,C.K.C.byref(lp));C.K.G.GEOSLength(q,C.K.C.byref(lq));bound=(lp.value+lq.value)*1e-5+math.pi*1e-10+1e-8;checks.append({'axis_pos_world_m':(a+b)/2,'cap_section_symmetric_difference_mm2':delta,'hausdorff_mm':h.value,'perimeter_scaled_area_bound_mm2':bound,'equivalent_at_1e_minus5_mm':bool(h.value<=1e-5 and delta<=bound)});C.K.G.GEOSGeom_destroy(q)
 C.K.G.GEOSGeom_destroy(p)
 return {'axis':'XYZ'[axis],'clipped_interval_m':[lo,hi],'number_of_piecewise_section_checks':len(checks),'maximum_symmetric_difference_mm2':max((x['cap_section_symmetric_difference_mm2'] for x in checks),default=0),'uniform_within_cap_sampling_tolerance':all(x['equivalent_at_1e_minus5_mm'] for x in checks),'checks':checks}
def cross(a,aa,b,ab):
 av,bv=C.verts(a),C.verts(b)
 if np.any(np.minimum(av.max(0),bv.max(0))-np.maximum(av.min(0),bv.min(0))<=0):return {'section_integral':{'method':'Exact separating world-space AABB','volume_mm3':0.0}}
 ua=verify_uniform(a,aa,bv[:,aa].min(),bv[:,aa].max());ub=verify_uniform(b,ab,av[:,ab].min(),av[:,ab].max());r={'uniform_slab_validation':[ua,ub]}
 if ua['uniform_within_cap_sampling_tolerance'] and ub['uniform_within_cap_sampling_tolerance']:r['section_integral']=C.cross_extrusion_overlap(a,aa,b,ab)
 else:r['status']='CAP_MODEL_NOT_VALID_ON_REQUIRED_INTERVAL'
 return r

def main():
 ap=argparse.ArgumentParser();
 for x in ['source','combined','audit','source-library','output']:ap.add_argument('--'+x,required=True)
 a=ap.parse_args();o=Path(a.output);src={m['name']:m for m in read(a.source)['meshes']};add={m['name']:m for m in read(a.combined)['meshes']};aud=read(a.audit);succ={n:k for k,v in aud['predecessors'].items() for n in v};allm=src|add
 original=[]
 for i in range(1,8):
  path=Path(a.source_library)/f'chair_conflict_{i:02d}_source_candidate.json';d=read(path);pair=d['note_zh'].split('：')[1].split('。')[0].split('|');mesh={m['name']:m for m in d['meshes']};pre=classify(*[geom(mesh[n]) for n in pair]);current=classify(*[geom(src[n]) for n in pair]);afterpair=[succ.get(n,n) for n in pair];post={'status':'MERGED_INTO_SINGLE_DESIGN_CANDIDATE','mesh':afterpair[0]} if afterpair[0]==afterpair[1] else classify(*[geom(allm[n]) for n in afterpair]);
  if pre['status']=='POSITIVE_MATERIAL_OVERLAP':
   p=np.array(pre['witness_world_m']);pre['independent_winding_numbers']=[winding(geom(mesh[n]),p) for n in pair];pre['independent_surface_distances_mm']=[surface_distance(geom(mesh[n]),p)*1000 for n in pair]
  original.append({'target_id':f'original_{i:02d}','pair':pair,'historical_source_candidate_sha256':sha(path),'historical_source_candidate_before':pre,'actual_1932_default_before':current,'combined_after_pair':afterpair,'combined_after':post})
 modes={}
 for float32 in [False,True]:
  mm={k:{'name':k,'vertices':np.array(v['vertices'],dtype=np.float32 if float32 else float).astype(float).tolist(),'indices':v['indices']} for k,v in allm.items()};rows=[]
  for side,front,plate in [('518DF','50DF4','REPAIR_518DD_88AC_1_source-plan6.5'),('518E0','51409','REPAIR_518DE_88AC_1_source-plan6.5_FRONT_ROUTE_REFLECTED_CANDIDATE')]:
   rod=f'CAD_{side}_4079_vertical_link';nr=succ[rod];bar=f'CAD_{front}_F643_profile_2';pin=succ[bar];strip=f'REPAIR_{front}_F643_profile_1_B535';ns=succ[strip];p0=f'CAD_{front}_F643_profile_0'
   rows.append({'pair':[pin,p0],'before_pair':[bar,p0],'type':'NOMINAL_ZERO_MARGIN_PIN_CHANNEL','before':C.cap_parallel_overlap(mm[bar],mm[p0],0),'after':cross(mm[pin],1,mm[p0],0)})
   rows.append({'pair':[ns,plate],'before_pair':[strip,plate],'type':'NOMINAL_TONGUE_SLOT_CONTACT','before':cross(mm[strip],1,mm[plate],2),'after':cross(mm[ns],1,mm[plate],2)})
   rows.append({'pair':[nr,'ENV_contact_wire'],'before_pair':[rod,'ENV_contact_wire'],'type':'INHERITED_UNRESOLVED_END_FIXING','before':cross(mm['ENV_contact_wire'],0,mm[rod],2),'after':cross(mm['ENV_contact_wire'],0,mm[nr],2)})
   rows.append({'pair':[pin,ns],'before_pair':[bar,strip],'type':'TARGET_CLEARANCE_PIN_B535','before':C.cap_parallel_overlap(mm[bar],mm[strip],1),'after':C.cap_parallel_overlap(mm[pin],mm[ns],1)})
   for suffix,ax in [('ring',1),('jaw_lower',0)]:
    target=f'CAD_SOURCE_{side}_C4D1_4AEC_{suffix}_square_fit_candidate';rows.append({'pair':[nr,target],'before_pair':[rod,target],'type':'TARGET_CLEARANCE_ROD_CLAMP','before':cross(mm[rod],2,mm[target],ax),'after':cross(mm[nr],2,mm[target],ax)})
   jaw=f'CAD_SOURCE_{side}_C4D1_4AEC_jaw_lower_square_fit_candidate';rows.append({'pair':['ENV_messenger_wire',jaw],'before_pair':['ENV_messenger_wire',jaw],'type':'UNCHANGED_PAIR_SUPPLEMENT_NOT_GLOBAL_SCAN','before':C.cap_parallel_overlap(mm['ENV_messenger_wire'],mm[jaw],0),'after':C.cap_parallel_overlap(mm['ENV_messenger_wire'],mm[jaw],0)})
  # The one nanometre-scale float boundary intersection has orthogonal uniform sections.
  p3='CAD_50DF4_F643_profile_3';strip='REPAIR_50DF4_F643_profile_1_B535';ns=succ[strip]
  rows.append({'pair':[p3,ns],'before_pair':[p3,strip],'type':'INHERITED_FLOAT32_BOUNDARY_MICRO_OVERLAP','before':C.cap_parallel_overlap(mm[p3],mm[strip],1),'after':C.cap_parallel_overlap(mm[p3],mm[ns],1)})
  modes['BLENDER_FLOAT32_ROUNDTRIP' if float32 else 'EXPORTED_JSON_DOUBLE']=rows
  print('Mode done',float32,flush=True)
 report={'schema':'gtsd-independent-interface-refinement-v1','original_seven_targets':original,'interface_refinement_by_coordinate_mode':modes,'method_limit':'Uniform-cap integrity is independently compared with solid-angle-selected actual triangle sections in each axial vertex interval inside the required overlap slab. Section equivalence uses 1e-5 mm Hausdorff tolerance with a perimeter-scaled area bound, equal to the primary 1e-8 m collision tolerance. Section areas use 1e-9 mm snapping; uncertainty at sub-grid boundaries remains numerical. Zero volume is not clearance, fastening confirmation or a manufacturing tolerance claim.'};(o/'interface_refinement.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':main()
