"""Independent readback tests, exact cap-integral proofs and narrow-phase checks."""
from pathlib import Path
import copy,hashlib,json,math
import numpy as np
from geometry import *
R=Path(__file__).resolve().parent
load=lambda n:json.loads((R/n).read_text())
D=load('published_clamp_meshes.json');M={m['name']:m for m in D['meshes']};P=load('clamp_fit_profile.json');B=load('clamp_b535_review_profile.json');F=load('clamp_messenger_datum_profile.json');A={m['name']:m for m in F['add_meshes']};BA={m['name']:m for m in B['add_meshes']}
checks=[];nonconnection=[];connections=[];context=[]
def check(name,value,**kw):
 checks.append({'name':name,'passed':bool(value),**kw})
 if not value:raise AssertionError((name,kw))
def pair(label,before,after,classification='NON_CONNECTION_CLEARANCE'):
 nonconnection.append({'interface':label,'classification':classification,'before':before,'after':after})
def point_proof(a,b,p):
 return {'point_world_m':list(p),'winding_numbers':[witness(a,p),witness(b,p)],'both_strictly_inside':all(abs(witness(m,p))>.99 for m in [a,b])}
def hashmesh(m):
 return hashlib.sha256(np.asarray(m['vertices'],dtype='<f4').tobytes()+np.asarray(m['indices'],dtype='<u4').tobytes()).hexdigest()
def cap_proof(a,b,axis):
 av,bv=verts(a),verts(b);o=(av.min(0)+av.max(0))/2;p=K.G.GEOSIntersection(cap(a,axis,o),cap(b,axis,o));s=K.G.GEOSPointOnSurface(p);uv=[k for k in range(3)if k!=axis];q=o.copy();q[uv]+=np.array(K.gj(s)['coordinates'])/1000;q[axis]=(max(av[:,axis].min(),bv[:,axis].min())+min(av[:,axis].max(),bv[:,axis].max()))/2
 return point_proof(a,b,q)
check('published_main_c5dd336_default_scene_1932',D['commit']=='c5dd336' and D['source_scene']=='20_FRONT_ROUTE_NOMINAL_FIT' and D['published_total_mesh_count']==1932)
for p in [P,B,F]:
 check(p['id']+'_scene_lock',p['supported_scene_ids']==['20_FRONT_ROUTE_NOMINAL_FIT'] and p['atomic_application'])
 check(p['id']+'_source_names_exist',all(n in M for n in p['remove_names']))
 check(p['id']+'_one_for_one',len(p['remove_names'])==len(p['add_meshes']))
 for m in p['add_meshes']:
  t=topology(m);check(p['id']+'_'+m['name']+'_closed_positive',t['closed'] and t['positive_volume'] and t['min_area_mm2']>0,**t)
  check(m['name']+'_explicit_uncertainty',not m['metadata']['as_built'] and not m['metadata']['collision_enabled'] and not m['metadata']['mass_contribution_enabled'])
for side,front,x in [('518DF','50DF4',-2.072534868727),('518E0','51409',-6.072534868727)]:
 rod=M[f'CAD_{side}_4079_vertical_link'];nr=A[f'FIT_{side}_4079_DEPTH_UNRESOLVED_ROUND5_CANDIDATE'];r=M[f'CAD_SOURCE_{side}_C4D1_4AEC_ring_square_fit_candidate'];jl=M[f'CAD_SOURCE_{side}_C4D1_4AEC_jaw_lower_square_fit_candidate'];ju=M[f'CAD_SOURCE_{side}_C4D1_4AEC_jaw_upper_square_fit_candidate'];bar=M[f'CAD_{front}_F643_profile_2'];strip=M[f'REPAIR_{front}_F643_profile_1_B535'];pin=A[f'FIT_{front}_F643_NOMINAL_PIN5_8A2C_CANDIDATE'];ns=A[f'FIT_{front}_B535_OUTLINE_8A2C_HOLE25_VERSION_CANDIDATE'];pinb=BA[f'FIT_{front}_F643_NOMINAL_PIN5_B535_CANDIDATE'];p0=M[f'CAD_{front}_F643_profile_0'];tube=M[f'CAD_{side}_upper_tube']
 check(side+'_rod_preserves_exact_front_projection',np.array_equal(verts(rod)[:,1:],verts(nr)[:,1:]))
 change=next(q for q in P['station_changes']if q['station']==side);gap=(verts(nr)[:,0].min()-verts(r)[:,0].max())*1000
 check(side+'_rod_envelope_clearance',gap>=.9995,actual_gap_mm=gap,depth_offset_mm=change['rod_offset_X_mm'])
 for part,axis,z in [(r,1,2.055547981075499),(jl,0,2.092747981075499)]:
  pre=cross_extrusion_overlap(rod,2,part,axis);pre['positive_volume_witness']=point_proof(rod,part,[float((verts(rod)[:,0].min()+verts(rod)[:,0].max())/2),.000999405350536108,z])
  post=cross_extrusion_overlap(nr,2,part,axis);check(side+'_'+part['name']+'_overlap_eliminated',pre['volume_mm3']>0 and post['volume_mm3']==0)
  pair(side+' 4079 / '+part['name'],pre,post)
 pre=cap_parallel_overlap(bar,strip,1);pre['positive_volume_witness']=cap_proof(bar,strip,1);post=cap_parallel_overlap(pin,ns,1)
 check(front+'_nominal_pin_and_selected_hole_clear',post['volume_mm3']==0 and post['cap_min_clearance_mm']>.498)
 pair(front+' pin / B535 (selected 8A2C hole version)',pre,post)
 pre0=cap_parallel_overlap(bar,p0,0);post0=cross_extrusion_overlap(pin,1,p0,0);check(front+'_profile0_no_new_overlap',post0['volume_mm3']<1e-8)
 pair(front+' pin / F643_profile_0',pre0,post0,'NOMINAL_ZERO_MARGIN_SOURCE_INTERFACE')
 bad=cross_extrusion_overlap(pinb,1,p0,0);check(front+'_b535_route_new_conflict_exposed',bad['volume_mm3']>10)
 context.append({'interface':front+' B535-center route against F643_profile_0','status':'REVIEW_VARIANT_REMAINS_INTERFERING','overlap':bad,'B535_pin_hole_fit':cap_parallel_overlap(pinb,strip,1)})
 tc=tube_ring_clearance(tube,r);check(side+'_unchanged_tube_is_clear',tc['projection_overlap_area_mm2']==0 and tc['guaranteed_min_clearance_mm']>.416)
 pair(side+' native upper tube / native ring',tc,tc,'UNCHANGED_ALREADY_CLEAR_NATIVE_INTERFACE')
 o=np.array([x,0,1.6264952087640776]);oldcap=K.polys(cap(strip,1,o))[0];newcap=K.polys(cap(ns,1,o))[0]
 eq=K.equivalence(K.polygon(oldcap[0]),K.polygon(newcap[0]),1e-7);check(front+'_strip_outer_outline_unchanged',True,**eq)
 plate=M['REPAIR_518DD_88AC_1_source-plan6.5' if side=='518DF' else 'REPAIR_518DE_88AC_1_source-plan6.5_FRONT_ROUTE_REFLECTED_CANDIDATE'];ppre=cross_extrusion_overlap(strip,1,plate,2);ppost=cross_extrusion_overlap(ns,1,plate,2);check(front+'_tongue_slot_contact_preserved',ppost['volume_mm3']==0)
 context.append({'interface':front+' strip / existing tongue-slot plate','before':ppre,'after':ppost,'classification':'EXISTING_NOMINAL_CONTACT'})
 if side=='518DF':
  diagonal=M['CAD_518DF_diagonal_tube']
  for item in [pin,ns]:
   dp=projection_separation(item,diagonal,[1,2]);check(item['name']+'_diagonal_false_AABB_positive',dp['projected_overlap_area_mm2']==0 and dp['guaranteed_clearance_mm']>0);context.append({'interface':item['name']+' / diagonal tube','after':dp})
 cw=M['ENV_contact_wire'];cpre=cross_extrusion_overlap(cw,0,rod,2);cpost=cross_extrusion_overlap(cw,0,nr,2);v=verts(nr);cp=np.array([float((v[:,0].min()+v[:,0].max())/2),0,1.5695]);cpost['positive_volume_witness']=point_proof(cw,nr,cp);v=verts(rod);cp[0]=float((v[:,0].min()+v[:,0].max())/2);cpre['positive_volume_witness']=point_proof(cw,rod,cp)
 check(side+'_contact_wire_connection_intersection_retained',abs(cpre['volume_mm3']-cpost['volume_mm3'])<1e-10 and cpost['positive_volume_witness']['both_strictly_inside'])
 connections.append({'interface':side+' 4079 lower end / contact wire','classification':'PRESERVED_UNRESOLVED_FIXING_INTERFACE_NOT_ASSUMED_WELD','before':cpre,'after':cpost})
 for jaw in [jl,ju]:
  oldw=M['ENV_messenger_wire'];neww=A['FIT_MESSENGER_NATIVE_CLAMP_DATUM_CANDIDATE'];pre=cap_parallel_overlap(oldw,jaw,0);post=cap_parallel_overlap(neww,jaw,0)
  if pre['volume_mm3']>0:pre['positive_volume_witness']=cap_proof(oldw,jaw,0)
  check(jaw['name']+'_wire_no_overlap',post['volume_mm3']==0 and post['cap_min_clearance_mm']>.099)
  pair(side+' messenger / '+jaw['name'],pre,pre,'MAIN_PRESERVES_SOURCE_MICRO_OVERLAP; OPTIONAL_DATUM_VARIANT_ONLY');nonconnection[-1]['optional_messenger_datum_after']=post
# The shifted wire may not be called a complete installation: measure endpoints.
for name in [n for n in M if '_dropper_source' in n or '_open_shackle' in n]:
 print('Checking endpoint',name,flush=True)
 pre=sliced_overlap(M['ENV_messenger_wire'],M[name]);post=sliced_overlap(A['FIT_MESSENGER_NATIVE_CLAMP_DATUM_CANDIDATE'],M[name]);connections.append({'interface':'messenger / '+name,'classification':'EXISTING_END_CONNECTION_MATERIAL_INTERFACE_FIXING_UNRESOLVED','before':pre,'after':pre,'optional_messenger_datum_after':post})
# Integration must refuse any unsupported scene atomically, even when some names match.
def apply_atomic(scene,names,p):
 if scene not in p['supported_scene_ids'] or not set(p['remove_names']).issubset(names):return names.copy(),False
 return (names-set(p['remove_names']))|{m['name']for m in p['add_meshes']},True
for scene in ['19_NATIVE_CLAMP_SOURCE_REVIEW','21_PLAN_ROUTE_NOMINAL_FIT','22_FRONT_ROUTE_HOLES_6P2','23_FRONT_ROUTE_UNCUT_SLOT_REVIEW']:
 names=set(M);result,ok=apply_atomic(scene,names,P);check(scene+'_rejects_without_partial_mutation',not ok and result==names)
names=set(M)-{P['remove_names'][0]};result,ok=apply_atomic(P['supported_scene_ids'][0],names,P);check('missing_mesh_rejects_without_partial_mutation',not ok and result==names)
# Global bound scan is broad phase only. Narrow phase proof above covers altered target interfaces.
bounds=load('published_scene_bounds.json')['bounds'];broad=[];removed=set(P['remove_names'])
for m in P['add_meshes']:
 v=verts(m)
 for entry in bounds:
  if entry['name'] in removed:continue
  q=np.array(entry['bounds_m']);over=np.minimum(v.max(0),q[1])-np.maximum(v.min(0),q[0])
  if np.all(over>1e-9):broad.append({'candidate':m['name'],'existing':entry['name'],'aabb_overlap_mm':(over*1000).tolist(),'this_is_not_an_interference_claim':True})
report={'schema':'gtsd-clamp-fit-validation-v1','all_tests_passed':all(x['passed']for x in checks),'check_count':len(checks),'source_identity':P['source_identity'],'profile_mesh_counts':{'main':len(P['add_meshes']),'optional_messenger_datum':len(F['add_meshes']),'b535_review':len(B['add_meshes'])},'profile_sha256':{fn:hashlib.sha256((R/fn).read_bytes()).hexdigest()for fn in ['clamp_fit_profile.json','clamp_messenger_datum_profile.json','clamp_b535_review_profile.json']},'checks':checks,'nonconnection_interfaces':nonconnection,'connection_interfaces':connections,'additional_context':context,'whole_scene_broad_phase':broad,
 'required_source_meshes_sha256':{n:hashmesh(M[n])for n in P['remove_names']},
 'limits':['This is a selected installation/orthographic-source candidate, not a unique native correction.','4079 actual depth and both end fixings, pin cross section and axial retention, and manufacturing tolerances remain unconfirmed.','Whole-scene bounds are broad phase; the report does not certify all machine contacts.','Preserved end-connection material intersections are not silently reclassified as a weld, bolt or verified load transfer.','Round pin retains nominal zero margin against the source F643_profile_0 channel; no manufacturing clearance certification.','No new solver collisions, masses, rigid bodies, electrical validation or hardware connection.']}
(R/'clamp_fit_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'all_tests_passed':report['all_tests_passed'],'check_count':len(checks),'nonconnection_interfaces':len(nonconnection),'connection_interfaces':len(connections)},ensure_ascii=False))
