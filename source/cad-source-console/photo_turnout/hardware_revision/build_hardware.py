"""Bounded six-station, outside-foot hold-down research alternative. No solver.
Uses packaged public-source seed and finite rail candidate; all sizes are choices.
"""
from pathlib import Path
import sys,gzip,json,math,copy,hashlib,itertools,datetime,argparse
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import cap_geometry as C,geos_kernel as K
from build_candidate import poly,rect,inter,union,free,write,meshhash
OUT=Path(__file__).resolve().parent

def circle(p,r,n=32):return poly(np.asarray(p)+r*np.c_[np.cos(np.arange(n)*2*np.pi/n),np.sin(np.arange(n)*2*np.pi/n)])
def diff(a,b):return K.G.GEOSDifference(a,b)
def framepoly(e,t,n,u0,u1,v0,v1):return poly([e+t*u+n*v for u,v in [(u0,v0),(u1,v0),(u1,v1),(u0,v1)]])
def extrusion(name,p,lo,hi,role,station=None):
 m=C.extrude_poly(name,p,2,np.zeros(3),lo/1000,hi/1000)
 m.update(color=[.48,.52,.57,1],group='photo_turnout_hardware_research',mechanical_fit_candidate=True,metadata={'role':role,'station':station,'geometry_status':'PHOTO_INSPIRED_FULL_HOLD_DOWN_ENGINEERING_RESEARCH_VARIANT','as_built_confirmed':False,'physics_status':'STATIC_NO_SOLVER_NO_COLLISION_ADMISSION','uniform_z_extrusion':True,'thread_geometry':'idealized clearance surfaces, no physical thread engagement'})
 return m

def footheight(meshes,q):
 hits=[]
 for m in meshes:
  tr=C.verts(m)[C.faces(m)]*1000
  for a,b,c in tr:
   mat=np.c_[(b-a)[:2],(c-a)[:2]]
   if abs(np.linalg.det(mat))<1e-9:continue
   u,v=np.linalg.solve(mat,np.array(q)-a[:2])
   if u>=-1e-7 and v>=-1e-7 and u+v<=1+1e-7:
    z=a[2]+u*(b-a)[2]+v*(c-a)[2]
    if z<=-51.999:hits.append(z)
 return max(hits)if hits else None

def build():
 seeddata=json.load(gzip.open(ROOT/'source_seed.json.gz','rt'));seed={m['name']:m for m in seeddata['meshes']};base=json.load(gzip.open(ROOT/'build/candidate_meshes.json.gz','rt'));bm=json.loads((ROOT/'build/change_manifest.json').read_text());src=json.loads((OUT/'targeted_source_evidence.json').read_text());rails=[m for m in base['meshes']if m['name'].startswith(('PHOTO_FROG','PHOTO_WING'))]
 fp=K.read('POLYGON EMPTY')
 for m in rails:fp=union(fp,C.actual_section(m,2,-.062,np.zeros(3)))
 config={'mounting_mode':'two outer-envelope hold-down assemblies per native sleeper axis','bolt_center_outward_from_foot_edge_mm':12.,'toe_overlap_mm':3.,'clamp_width_along_rail_mm':20.,'clamp_thickness_mm':4.,'bolt_shank_diameter_mm':6.,'clearance_bore_diameter_mm':7.,'washer_outer_diameter_mm':12.,'washer_thickness_mm':1.5,'hex_head_across_flats_mm':10.,'head_height_mm':4.,'carrier_width_mm':20.,'carrier_depth_mm':18.,'nut_height_mm':4.,'nut_bore_diameter_mm':6.4,'nut_pocket_square_mm':12.,'plate_thickness_mm':4.,'pad_thickness_mm':3.,'plate_edge_margin_mm':5.,'all_dimensions_are_engineering_choices':True,'source_section_and_axes_unchanged':True,'fixation':'separate welded captive-nut carrier candidate; weld size/process and plate fixation unconfirmed','photo_basis':'separate small hold-down, bolt and seat forms beside rail foot; no pixel-derived dimensions or calibrated registration'}
 meshes=[];datum=[];carrierplans=[]
 for st in src['stations']:
  a,b=np.asarray(st['axis_world_mm']);axis=b-a;axis/=np.linalg.norm(axis);hits=[]
  for rings in K.polys(fp):
   for ring in rings:
    for p,q in zip(np.asarray(ring)[:-1],np.asarray(ring)[1:]):
     d=q-p;mat=np.c_[axis,-d]
     if abs(np.linalg.det(mat))<1e-10:continue
     s,u=np.linalg.solve(mat,p-a)
     if -1e-8<=u<=1+1e-8:hits.append((s,a+s*axis,d/np.linalg.norm(d)))
  assert hits,st
  for side,hit in [('lower',min(hits,key=lambda r:r[0])),('upper',max(hits,key=lambda r:r[0]))]:
   _,e,t=hit;n=np.array([-t[1],t[0]]);outward=-axis if side=='lower'else axis
   if np.dot(n,outward)<0:n=-n
   if t[0]<0:t=-t
   bolt=e+12*n;prefix=f'PHOTO_CLAMP_{st["station"]:02d}_{side}';clamp=framepoly(e,t,n,-10,10,-3,22);bore=circle(bolt,3.5)
   # Exact highest inherited foot surface beneath the toe corners defines the
   # horizontal toe contact. No rail shaving or artificial clearance gap.
   samples=[e+t*u+n*v for u in [-10,0,10]for v in [-3,-1,0]];heights=[footheight(rails,p)for p in samples];heights=[z for z in heights if z is not None];z=max(heights);assert -60<z<-51.9,(prefix,z)
   outer=framepoly(bolt,t,n,-10,10,-9,9);pocket=framepoly(bolt,t,n,-6,6,-6,9.001);carrierplans.append(outer)
   # Three seat levels preserve visible lower base and accessible captive nut.
   parts=[('seat_base',diff(outer,bore),-65.,-63.),('seat_pocket',diff(outer,pocket),-63.,-59.),('seat_upper',diff(outer,bore),-59.,z),('toe_clamp',diff(clamp,bore),z,z+4.)]
   nut=diff(circle(bolt,10/math.sqrt(3),6),circle(bolt,3.2));parts.append(('captive_nut',nut,-63.,-59.))
   parts.extend([('bolt_shank',circle(bolt,3),-63.,z+5.5),('washer',diff(circle(bolt,6),bore),z+4.,z+5.5),('bolt_head',circle(bolt,10/math.sqrt(3),6),z+5.5,z+9.5)])
   for role,p,lo,hi in parts:meshes.append(extrusion(prefix+'_'+role,p,lo,hi,role,st['station']))
   datum.append({'assembly':prefix,'source_station':st['station'],'source_handles':st['source_handles'],'source_sleeper_axis_world_mm':st['axis_world_mm'],'rail_foot_envelope_section_z_mm':-62.,'exact_axis_foot_intersection_world_mm':e.tolist(),'outward_normal_xy':n.tolist(),'rail_tangent_xy':t.tolist(),'bolt_center_world_xy_mm':bolt.tolist(),'toe_contact_top_of_foot_z_mm':z,'toe_contact_samples_z_mm':heights,'attachment_version':'outside_foot_captive_seat_research','unresolved':'source station association exact; assembly type, offset, seat weld and dimensions unconfirmed'})
 # New long plate has explicit, inspectable carrier coverage; no hidden merged support.
 plans=[C.cap(m,2,np.zeros(3),'max')for m in base['meshes']if m['name']=='PHOTO_SUPPORT_separate_long_plate']+carrierplans
 from scipy.spatial import ConvexHull
 points=np.concatenate([np.array(r)for p in plans for rings in K.polys(p)for r in rings]);plate=K.G.GEOSBuffer(poly(points[ConvexHull(points).vertices]),5.,1)
 meshes.append(extrusion('PHOTO_HARDWARE_SUPPORT_long_plate',plate,-69,-65,'long_plate'))
 pads=[]
 for st in src['stations']:
  m=seed[st['source_sleeper']];p=inter(C.cap(m,2,np.zeros(3),'max'),plate);assert K.area(p)>0
  name=f'PHOTO_HARDWARE_SUPPORT_pad_{st["station"]:02d}';meshes.append(extrusion(name,p,-72,-69,'pad',st['station']));pads.append({'name':name,'retained_sleeper':m['name'],'actual_contact_area_mm2':K.area(p)})
 oldheads=['ENV_fastener.'+f'{i:03d}'for i in [2,3,6,7,10,11,14,15,18,19]]+['CAD_TRACK_REV_branch_clip_'+f'{i:02d}_{r}_{s}'for i,r in [(0,1),(1,1),(2,1),(5,2)]for s in [-1,1]]+['FIT_TRACK_clip_03_1_'+str(s)for s in [-1,1]]
 remove=[r['name']for r in bm['remove']]+oldheads+['CAD_TRACK_REV_photo_chair_001_source_station','CAD_TRACK_REV_branch_chair_00_1'];assert len(remove)==len(set(remove));assert all(n in seed for n in remove)
 retainedbase=[m for m in base['meshes']if not m['name'].startswith('PHOTO_SUPPORT_')];allnew=retainedbase+meshes
 topology={m['name']:C.topology(m)for m in allnew};assert all(t['closed']and t['positive_volume']for t in topology.values()),[(n,t)for n,t in topology.items()if not t['closed']]
 payload={'schema':'gtsd-photo-turnout-hardware-candidate-v1','meshes':allnew}
 with open(OUT/'candidate_meshes.json.gz','wb')as f:
  with gzip.GzipFile(fileobj=f,mode='wb',mtime=0)as g:g.write(json.dumps(payload,separators=(',',':'),ensure_ascii=False).encode())
 manifest={'schema':'gtsd-photo-turnout-change-manifest-v1','candidate_id':'photo_finite_frog_full_hold_down_research_v2','base_commit':bm['base_commit'],'source_scene_mesh_count':seeddata['scene_mesh_count'],'remove':[{'name':n,'expected_geometry_witness_sha256':seed[n]['geometry_witness_sha256']}for n in remove],'add':[{'name':m['name'],'geometry_content_sha256':meshhash(m)}for m in allnew],'result_mesh_count':seeddata['scene_mesh_count']-len(remove)+len(allnew),'as_built_confirmed':False,'physics_enabled':False,'hardware_replacement_explanation':'20 generic bolt-head proxies (17 modeled hits plus 3 paired heads) and two remaining station-00 chairs superseded only in this distinct research variant; 12 full 8-part outside-foot assemblies replace the guessed attachment interpretation. Historical proxy review preserved.','retained_unresolved_attachment_variant':'old source-position/proxy profile: unproven through-foot fastening or stale legacy offsets; no undocumented rail holes added','configuration':config,'mounting_datums':datum,'support_contacts':pads,'topology':topology,'unresolved':['0.4882476853 mm3 finite frog-foot/left-tail joint','outside-foot captive-seat family and count are unconfirmed','weld and plate-to-sleeper fixation unknown','frog nose uplift restraint and wheel swept envelope not proven','idealized nut/shank clearance surfaces do not prove threaded engagement','source guards and unchanged-versus-unchanged scene pairs remain outside this revision']}
 write(OUT/'change_manifest.json',manifest);write(OUT/'config.json',config);print('BUILT',len(allnew),'meshes; removed',len(remove),'new assemblies',len(datum),flush=True)
if __name__=='__main__':build()
