"""Build the independent, adjustable operational turnout assembly from frozen source meshes."""
from pathlib import Path
import argparse,copy,datetime,gzip,hashlib,json,sys
import numpy as np
H=Path(__file__).resolve().parent; P=H.parent; sys.path.insert(0,str(P))
import cap_geometry as C
from build_candidate import clip_mesh_plane, meshhash
from extract_scene import meshes as read_meshes

def sha(p):
 h=hashlib.sha256()
 with open(p,'rb') as f:
  while b:=f.read(1024*1024):h.update(b)
 return h.hexdigest()
def write(p,d):p.write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def gz(p,d):
 with gzip.GzipFile(filename=str(p),mode='wb',mtime=0) as f:f.write(json.dumps(d,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode())
def witness(m):
 v=np.asarray(m['vertices'],dtype='<f4');v[v==0]=0
 return hashlib.sha256(v.tobytes()+np.asarray(m['indices'],dtype='<u4').tobytes()).hexdigest()
def material(m):
 n=m['name'].lower(); role=m.get('metadata',{}).get('role','')
 if n.startswith('photo_hardware_support_pad'):return 'elastomer'
 if 'sleeper' in n:return 'concrete'
 if 'yellow_lid' in n or 'balise_body' in n or 'fit_balise_' in n and ('body' in n or 'lid' in n):return 'polymer'
 return 'steel'
def properties(m):
 v=C.verts(m);f=C.faces(m);o=v.mean(0);t=v[f]-o
 six=np.einsum('ij,ij->i',t[:,0],np.cross(t[:,1],t[:,2]));vol=six.sum()/6
 com=o+(six[:,None]*t.sum(1)).sum(0)/(4*six.sum()) if abs(vol)>1e-20 else o
 topo=C.topology(m);mat=material(m)
 return {'name':m['name'],'mesh_collection':'full_scene_meshes.json.gz','mesh_key':m['name'],'vertices':len(v),'triangles':len(f),'geometry_witness_sha256':witness(m),'geometry_content_sha256':meshhash(m),'bounds_world_m':[v.min(0).tolist(),v.max(0).tolist()],'signed_volume_mm3':float(vol*1e9),'volume_mm3':float(abs(vol)*1e9),'center_of_mass_world_m':com.tolist(),'mass_properties_method':'uniform-density signed tetrahedral integration' if topo['closed'] and abs(vol)>1e-20 else 'surface tetrahedral diagnostic; not assigned rigid mass','closed':topo['closed'],'boundary_edges':topo['boundary_edges'],'nonmanifold_edges':topo['nonmanifold_edges'],'material_id':mat,'assumed_density_kg_m3':{'steel':7850.,'elastomer':1100.,'concrete':2400.,'polymer':1200.}[mat],'assembly_role':m.get('metadata',{}).get('role',m.get('group','source_component')),'mechanical_group':'_'.join(m['name'].split('_')[:4]) if m['name'].startswith('PHOTO_CLAMP') else ('turnout_rails' if m['name'].startswith(('PHOTO_FROG','PHOTO_WING','PHOTO_RETAIN')) else m.get('group','source_context'))}

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--photo-assets',type=Path,default=P);ap.add_argument('--source-scene',required=True,type=Path);ap.add_argument('--joint-gap-mm',type=float,default=.5);ap.add_argument('--output',type=Path,default=H);a=ap.parse_args();out=a.output;out.mkdir(parents=True,exist_ok=True)
 assert 0<a.joint_gap_mm<=2.,'gap must be in (0, 2] mm'
 pp=a.photo_assets/'photo_source_profiles.json';sp=a.photo_assets/'source_seed.json.gz';hp=a.photo_assets/'hardware_revision/candidate_meshes.json.gz'
 frozen=json.load(open(pp));old=next(p for p in frozen['profiles']if p['id']=='photo_frog_hold_down_research');seed=json.load(gzip.open(sp,'rt'));oldh=json.load(gzip.open(hp,'rt'));hm=json.load(open(P/'hardware_revision/change_manifest.json'))
 assert sha(a.source_scene)==seed['source_sha256'];assert len(old['groups'])==1
 profile=copy.deepcopy(old);group=profile['groups'][0];assert len(group['remove'])==67 and len(group['add_meshes'])==141
 before={m['name']:copy.deepcopy(m)for m in group['add_meshes']}; graph=json.load(open(P/'native_edge_graph.json'));edge=next(r for r in graph['source_curves']if r['handle']=='4ECED');p0,p1=np.array(edge['points']);normal=(p0-p1)/np.linalg.norm(p0-p1);normal=np.r_[normal,0.];origin=np.r_[p1/1000,0.];tail_trim_mm=min(a.joint_gap_mm/2,.25);frog_trim_mm=a.joint_gap_mm-tail_trim_mm;tail_trim=tail_trim_mm/1000;frog_trim=frog_trim_mm/1000
 changes=[]
 gz(out/'joint_reference_meshes.json.gz',{'units':'m','meshes':[before[n]for n in ['PHOTO_FROG_source_finite_V','PHOTO_RETAIN_left_ENV_main_rail_1','PHOTO_RETAIN_left_ENV_main_rail_top_1']]})
 for name,positive in [('PHOTO_FROG_source_finite_V',True),('PHOTO_RETAIN_left_ENV_main_rail_1',False),('PHOTO_RETAIN_left_ENV_main_rail_top_1',False)]:
  src=before[name];point=origin+normal*(frog_trim if positive else -tail_trim);m=clip_mesh_plane(src,normal,point,positive,name);m['metadata']=copy.deepcopy(src.get('metadata',{}));m['metadata'].update(operational_joint={'source_handle':'4ECED','source_endpoint_index':1,'normal_world':normal.tolist(),'cut_plane_point_m':point.tolist(),'keep_positive':positive,'gap_mm':a.joint_gap_mm,'endjoin':'common_transverse_miter'},operational_geometry=True)
  m['geometry_witness_sha256']=witness(m);m['geometry_content_sha256']=meshhash(m)
  assert C.topology(m)['closed'] and C.signed_volume(m)>0,(name,C.topology(m))
  group['add_meshes'][next(i for i,v in enumerate(group['add_meshes'])if v['name']==name)]=m
  changes.append({'name':name,'before_geometry_witness_sha256':witness(src),'after_geometry_witness_sha256':witness(m),'before_volume_mm3':C.signed_volume(src)*1e9,'after_volume_mm3':C.signed_volume(m)*1e9,'removed_volume_mm3':(C.signed_volume(src)-C.signed_volume(m))*1e9,'cut_plane_point_m':point.tolist(),'cut_plane_normal_world':normal.tolist(),'keep_positive':positive})
 profile.update(id='operational_assembly_v1',title='默认工程装配 · 完整扣件与有限端接头',requires_explicit_selection=False)
 group['id']='operational_assembly_v1_complete_source_replacement'
 profile['evidence']={'status':'OPERATIONAL_ENGINEERING_DEFAULT','base_profile':'photo_frog_hold_down_research','parameters':'parameters.json','material_audit':'geometry_audit.json','mesh_interface':'mesh_interface.json','source_identity_preserved':True}
 by={m['name']:m for m in group['add_meshes']};assert set(by)==set(before)
 for m in by.values():
  m.setdefault('metadata',{})['operational_assumptions']='ASSUMPTIONS.md'
 payload={**{k:v for k,v in frozen.items()if k!='profiles'},'profiles':[profile]}
 write(out/'operational_source_profile.json',payload)
 original={m['name']:m for m in read_meshes(a.source_scene)}; remove={r['name']:r for r in group['remove']}
 assert set(remove)<=set(original)
 for n,r in remove.items():assert witness(original[n])==r['geometry_witness_sha256'],n
 full={n:m for n,m in original.items()if n not in remove};assert not(set(full)&set(by));full.update(by);assert len(full)==2006
 regionremove={r['name']for r in hm['remove']};regional={m['name']:m for m in seed['meshes']if m['name']not in regionremove};candidate={m['name']:by[m['name']]for m in oldh['meshes']};regional.update(candidate);assert len(regional)==205 and len(candidate)==111
 gz(out/'candidate_meshes.json.gz',{'schema':'gtsd-operational-candidate-v1','units':'m','meshes':list(candidate.values())})
 gz(out/'regional_meshes.json.gz',{'schema':'gtsd-operational-region-v1','units':'m','meshes':list(regional.values())})
 gz(out/'full_scene_meshes.json.gz',{'schema':'gtsd-operational-full-scene-v1','units':'m','profile_id':profile['id'],'meshes':list(full.values())})
 relations=[]
 for d in hm['mounting_datums']:
  n=d['assembly'];relations.append({'assembly':n,'parts':[m for m in candidate if m.startswith(n+'_')],'source_station':d['source_station'],'source_handles':d['source_handles'],'bolt_center_world_xy_mm':d['bolt_center_world_xy_mm'],'toe_contact_z_mm':d['toe_contact_top_of_foot_z_mm'],'joint_model':'preloaded_bolt_nut_toe_clamp','seat_to_plate':'welded_rigid','toe_to_rail':'unilateral_contact','bolt_clearance_diameter_mm':7.,'shank_diameter_mm':6.,'nut_bore_diameter_mm':6.4})
 for p in hm['support_contacts']:relations.append({'assembly':p['name'],'parts':[p['name'],'PHOTO_HARDWARE_SUPPORT_long_plate',p['retained_sleeper']],'joint_model':'pad_compression_between_plate_and_sleeper','contact_area_mm2':p['actual_contact_area_mm2']})
 for r in relations:r.update(relation_scope='assembly_design',current_solver_model='rigid_group_finite_mount',solver_body='track_assembly',independent_joint_dof=False,independent_preload_or_pad_contact_solved=False)
 cfg={'schema':'gtsd-operational-parameters-v1','length_unit':'mm','geometry_coordinate_unit':'m','joint':{'gap_mm':a.joint_gap_mm,'endjoin':'common_transverse_miter','miter_normal_world':normal.tolist(),'reference_point_mm':(origin*1000).tolist(),'tail_trim_mm':tail_trim_mm,'frog_trim_mm':frog_trim_mm,'tail_trim_max_mm':.25,'gap_allocation':'symmetric up to 0.5 mm; left-tail trim capped at 0.25 mm to preserve its finite top segment; remaining gap assigned to frog','source_file':'../native_edge_graph.json','source_handle':'4ECED','source_endpoint_index':1,'reason':'A finite common end plane and 0.5 mm assembly allowance give positive clearance through the complete inherited rail section.'},'hardware':json.load(open(P/'hardware_revision/config.json')),'materials':{'steel':{'density_kg_m3':7850},'elastomer':{'density_kg_m3':1100},'concrete':{'density_kg_m3':2400},'polymer':{'density_kg_m3':1200}},'assumptions_page':'ASSUMPTIONS.md'}
 # Keep operational configuration concise; provenance and rationale live on one page.
 for k in ['all_dimensions_are_engineering_choices','source_section_and_axes_unchanged','fixation','photo_basis']:cfg['hardware'].pop(k,None)
 write(out/'parameters.json',cfg)
 props=[properties(m)for m in full.values()];now=datetime.datetime.now(datetime.timezone.utc)
 interface={'schema':'gtsd-operational-mesh-interface-v1','profile_id':profile['id'],'units':{'vertices':'m','center_of_mass_world_m':'m','volume_mm3':'mm3','density':'kg/m3'},'full_scene_meshes':'full_scene_meshes.json.gz','regional_meshes':'regional_meshes.json.gz','source_profile':'operational_source_profile.json','full_scene_mesh_count':len(full),'regional_mesh_count':len(regional),'source_add_count':141,'source_remove_count':67,'geometry_changes':changes,'mechanical_relations':relations,'mechanical_relations_scope':'Assembly design relations. The operational solver contract declares their rigid-group equivalents and enabled contacts.','material_assumptions':'ASSUMPTIONS.md','meshes':props,'frozen_utc':now.isoformat(),'frozen_beijing':now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat()}
 write(out/'mesh_interface.json',interface)
 manifest={'schema':'gtsd-operational-change-manifest-v1','candidate_id':profile['id'],'base_commit':'7d0e9c0440e8226a2fa67ae1b9bbf1a4b52167d4','remove':hm['remove'],'add':[{'name':n,'geometry_content_sha256':meshhash(m)}for n,m in candidate.items()],'result_mesh_count':len(full),'source_profile_remove_count':67,'source_profile_add_count':141,'source_inputs':{'source_scene_sha256':sha(a.source_scene),'photo_profile_sha256':sha(pp),'photo_seed_sha256':sha(sp),'hardware_meshes_sha256':sha(hp),'native_edge_graph_sha256':sha(P/'native_edge_graph.json')},'geometry_changes':changes,'mounting_datums':hm['mounting_datums'],'support_contacts':hm['support_contacts'],'build_utc':now.isoformat(),'build_beijing':now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat()}
 write(out/'change_manifest.json',manifest)
 print(json.dumps({'profile':profile['id'],'full_scene':len(full),'regional':len(regional),'changed':changes,'out':out.name},indent=2),flush=True)
if __name__=='__main__':main()
