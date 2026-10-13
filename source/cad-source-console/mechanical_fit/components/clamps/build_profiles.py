"""Rebuild reviewable clamp-interface profiles from published c5dd336 triangles.
No Blender, repository writes, removed source evidence, or hardware connections.
"""
from pathlib import Path
import copy,hashlib,json,numpy as np
from geometry import *
R=Path(__file__).resolve().parent
load=lambda n:json.loads((R/n).read_text())
D=load('published_clamp_meshes.json');M={m['name']:m for m in D['meshes']};config=load('fit_config.json')
SOURCE_DXF='f45c517fa9a2e5059742f855b94f8eca842c97abd9b54c6b027a3ce8d18cb5e2'
STATIONS=[('518DF','50DF4',-2.072534868727),('518E0','51409',-6.072534868727)]
def save(name,data): (R/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
def finish(m,group,metadata,color=[0.9,0.52,0.16,1]):
 m['vertices']=np.asarray(m['vertices'],np.float32).astype(float).ravel().tolist();m['indices']=np.asarray(m['indices'],int).ravel().tolist()
 for key in list(m):
  if key not in ['name','vertices','indices']:del m[key]
 m.update(group=group,color=color,metadata={
  'candidate':True,'as_built':False,'as_built_confirmed':False,'native_pose_bound':False,
  'physics_enabled':False,'collision_enabled':False,'mass_contribution_enabled':False,
  'geometry_status':'EXPLICIT_SOURCE_VERSION_AND_INSTALLATION_REVIEW_CANDIDATE',
  'source_dxf_sha256':SOURCE_DXF,**metadata})
 return m

def make_profile(hole_choice):
 pid='catenary_8a2c_pin5_depth_candidate' if hole_choice=='8A2C' else 'catenary_b535_pin5_depth_review'
 cfg=copy.deepcopy(config);cfg['lower_pin']['source_hole_location_choice']=hole_choice
 remove=[];add=[];changes=[];stationdata=[]
 for side,front,x in STATIONS:
  old=M[f'CAD_{side}_4079_vertical_link'];ov=verts(old);obstacles=[]
  for other in M.values():
   if other['name']==old['name'] or other['name'] in ['ENV_messenger_wire','ENV_contact_wire'] or '_dropper_source' in other['name'] or '_open_shackle' in other['name']:continue
   q=verts(other)
   if not (q[:,0].min()<x+.15 and q[:,0].max()>x-.15):continue
   if min(q[:,1].max(),ov[:,1].max())<=max(q[:,1].min(),ov[:,1].min()):continue
   if min(q[:,2].max(),ov[:,2].max())<=max(q[:,2].min(),ov[:,2].min()):continue
   obstacles.append({'name':other['name'],'maximum_X_m':float(q[:,0].max())})
  wall=max(obstacles,key=lambda q:q['maximum_X_m']);minshift=wall['maximum_X_m']+cfg['long_link']['minimum_geometric_gap_mm']/1000-ov[:,0].min()
  shift=cfg['long_link']['explicit_station_depth_offsets_mm'].get(side,minshift*1000)/1000
  if shift<minshift-1e-10:raise ValueError('Specified depth does not meet envelope gap: '+side)
  name=f'FIT_{side}_4079_DEPTH_UNRESOLVED_ROUND5_CANDIDATE';new=finish(translate(old,[shift,0,0],name),'upper',{
   'source_mesh':old['name'],'source_paths':[('50DF8' if side=='518DF' else '5140D')+'/4079/3BD17,3BD18,3BD19'],
   'changed':'Only previously assumed world-X depth; original triangle shape and source front Y/Z are retained.',
   'world_translation_m':[shift,0,0],'depth_is_source_confirmed':False,'round_section_is_source_confirmed':False,
   'fixing_method':None,'installation_complete':False,
   'depth_selection':'Minimum positive-X separating envelope plus explicit configurable 1 mm engineering review gap; not a measured installation.',
   'engineering_gap_mm':cfg['long_link']['minimum_geometric_gap_mm']})
  remove.append(old['name']);add.append(new)
  stationdata.append({'station':side,'front':front,'rod_offset_X_mm':shift*1000,'limiting_obstacle':wall,'obstacles':obstacles,'before_world_center_X_m':float((ov[:,0].min()+ov[:,0].max())/2),'after_world_center_X_m':float((verts(new)[:,0].min()+verts(new)[:,0].max())/2),'fixed_endpoints_confirmed':False})
  bar=M[f'CAD_{front}_F643_profile_2'];bv=verts(bar);zfront=1.6006579994617423+.025;zb535=1.6264952087640776;z=zfront if hole_choice=='8A2C' else zb535
  pin=finish(cylinder(f'FIT_{front}_F643_NOMINAL_PIN5_{hole_choice}_CANDIDATE',1,[x,0,z],float(bv[:,1].min()),float(bv[:,1].max()),cfg['lower_pin']['diameter_mm']/2000,cfg['lower_pin']['circle_segments']),'lower',{
   'source_mesh':bar['name'],'source_paths':[front+'/F643/3A9C5/A3AD/3A9BA'],
   'native_shape':'40 by 5 mm front-view rectangle; no depth or circular-shank proof.',
   'changed':'Inherited unproved 4 by 5 mm rectangular extrusion replaced by an explicit nominal 5 mm round pin. No head, nut, thread or axial retention invented.',
   'nominal_diameter_mm':cfg['lower_pin']['diameter_mm'],'nominal_length_mm':40,
   'selected_hole_center_source':hole_choice,'source_versions_conflict_mm':25.8372093023354-25,
   'center_world_m':[x,float((bv[:,1].min()+bv[:,1].max())/2),z],
   'round_section_is_source_confirmed':False,'bolt_identity_is_source_confirmed':False,'axial_retention_method':None,
   'native_front_bar_center_shift_Z_mm':0 if hole_choice=='8A2C' else .8372093023354})
  remove.append(bar['name']);add.append(pin)
  if hole_choice=='8A2C':
   strip=M[f'REPAIR_{front}_F643_profile_1_B535'];sv=verts(strip);o=np.array([x,0,zb535]);p=cap(strip,1,o);loops=K.polys(p);assert len(loops)==1 and len(loops[0])==2
   out,hole=loops[0];delta_mm=-.8372093023354;hole=[[a,b+delta_mm]for a,b in hole];newp=K.polygon(out,[hole]);sm=extrude_poly(f'FIT_{front}_B535_OUTLINE_8A2C_HOLE25_VERSION_CANDIDATE',newp,1,o,float(sv[:,1].min()),float(sv[:,1].max()))
   sm=finish(sm,'lower',{'source_mesh':strip['name'],'source_paths':[front+'/F643/3A9C3/8A2C/3A99B','E778/3C772/B535/3A9A2,3A9A3'],
    'changed':'Native B535 outer outline and diameter 6 mm retained; hole center lowered 0.8372093023354 mm to choose the incompatible 8A2C hidden front-view 25 mm version.',
    'native_B535_circle_center_from_bottom_mm':25.8372093023354,'selected_8A2C_hole_center_from_bottom_mm':25,
    'hole_diameter_mm':6,'hole_center_shift_Z_mm':delta_mm,'outer_outline_changed':False,'native_B535_hole_unchanged':False,
    'source_version_choice_is_unique':False,'hole_enlarged':False})
   remove.append(strip['name']);add.append(sm)
 wire=M['ENV_messenger_wire'];wv=verts(wire);center=(wv.min(0)+wv.max(0))/2;delta=np.array([0,-5.946246746102446e-7-center[1],2.099547981075499-center[2]])
 newwire=finish(translate(wire,delta,'FIT_MESSENGER_NATIVE_CLAMP_DATUM_CANDIDATE'),'environment',{
  'source_mesh':'ENV_messenger_wire','source_paths':['Model/4EE17,4EE18','C4D1/4AEC/9389/3A92C'],
  'changed':'Same actual wire triangles translated transversely to the native clamp groove datum at both stations; explicit datum choice, not a unique source correction.',
  'world_translation_m':delta.tolist(),'translation_mm':(delta*1000).tolist(),'diameter_or_length_changed':False,
  'source_native_wire_version_preserved_in_source_review':True,'sag_tension_dropper_ends_revalidated':False},[.77,.39,.16,1])
 remove.append(wire['name']);add.append(newwire)
 label='接触网接口候选：8A2C前视孔心＋名义Ø5销＋待定深度' if hole_choice=='8A2C' else '接触网B535孔心审阅：名义销仍与F643轮廓干涉'
 profile={'schema':'gtsd-mechanical-fit-profile-v1','id':pid,'label':label,'default_enabled':False,'as_built_confirmed':False,
  'supported_source_scenes':['20_FRONT_ROUTE_NOMINAL_FIT'],'supported_scene_ids':['20_FRONT_ROUTE_NOMINAL_FIT'],'require_exact_scene_and_all_remove_names':True,'atomic_application':True,
  'on_mismatch':'DO_NOT_APPLY_ANY_MESH; report unsupported source scene or missing exact source name',
  'configuration':cfg,'remove_names':remove,'add_meshes':add,'additional_mutually_exclusive_names':[],
  'source_identity':{k:D[k]for k in ['commit','source_scene','base_glb_sha256','source_replacements_sha256','published_total_mesh_count']},
  'station_changes':stationdata,'source_review_preserved':True,'source_review_reference':'published_clamp_meshes.json',
  'source_evidence':['native_source_records.json','INTERFACE_CORRECTION_CONTRACT.json','source_context_summary.json'],
  'review_warning':'4079深度及固定方式仍待实测；销的圆截面与轴向固定仍是工程候选。8A2C/B535是相互冲突的原生版本，不得称唯一原图修复。',
  'known_new_interference':hole_choice=='B535','fit_scope':'Only enumerated clamp interfaces and neighboring F643 profile_0. No whole-machine or manufactured fit certification.',
  'physics_new_collision':False,'physics_added_mass_kg':0,'physics_added_bodies':0,'validation_file':'clamp_fit_validation.json'}
 return profile

full=make_profile('8A2C')
full['id']='clamp_messenger_datum'
full['label']='夹具接口候选＋承力索槽心基准附加候选（端连接待定）'
full['review_warning']+=' 全长承力索也作0.100971 mm基准改位，卸扣与吊弦端连接尚未闭合，不计入主夹具方案已处理数。'
full['includes_optional_messenger_datum']=True
save('clamp_messenger_datum_profile.json',full)
main=copy.deepcopy(full);main['id']='catenary_8a2c_pin5_depth_candidate';main['label']='夹具接口候选：8A2C孔心＋名义Ø5销＋4079待定深度'
main['remove_names'].remove('ENV_messenger_wire');main['add_meshes']=[m for m in main['add_meshes']if m['name']!='FIT_MESSENGER_NATIVE_CLAMP_DATUM_CANDIDATE']
main['configuration']['messenger']['datum_choice']='retain_existing_wire_source_datum';main['includes_optional_messenger_datum']=False
main['review_warning']='4079深度及固定方式、销圆截面与轴向固定仍是工程候选；8A2C/B535原生版本冲突显式保留。主方案不移动承力索，索槽微穿料与端连接未确认为已解决。'
main['unresolved_messenger_groove_overlap_retained']=True
save('clamp_fit_profile.json',main)
bad=make_profile('B535');bad['remove_names'].remove('ENV_messenger_wire');bad['add_meshes']=[m for m in bad['add_meshes']if m['name']!='FIT_MESSENGER_NATIVE_CLAMP_DATUM_CANDIDATE'];bad['configuration']['messenger']['datum_choice']='retain_existing_wire_source_datum'
save('clamp_b535_review_profile.json',bad)
profiles=[{'id':main['id'],'file':'clamp_fit_profile.json','mesh_count':6,'recommended_for_geometric_review':True,'has_known_interference':False},
 {'id':full['id'],'file':'clamp_messenger_datum_profile.json','mesh_count':7,'recommended_for_geometric_review':False,'optional_full_wire_datum_change':True},
 {'id':bad['id'],'file':'clamp_b535_review_profile.json','mesh_count':4,'recommended_for_geometric_review':False,'has_known_interference':True}]
save('profiles_manifest.json',{'schema':'gtsd-catenary-profile-manifest-v1','profiles':profiles,'default':'source_review_unchanged','mutually_exclusive':True,'as_built':False})
print(json.dumps({'profiles':profiles},ensure_ascii=False))
