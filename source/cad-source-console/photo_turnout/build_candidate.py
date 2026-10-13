"""Build a bounded explicit photo-supported topology alternative. No solver or network.

World metres in mesh streams; source plan and engineering configuration in mm.
Photo pixels are not input to this deterministic builder and are never copied.
"""
from pathlib import Path
import argparse,copy,gzip,hashlib,json,math
import numpy as np
import geos_kernel as K
import cap_geometry as C
HERE=Path(__file__).resolve().parent
K.bind('GEOSBuffer',[K.P,K.D,K.I],K.P)

def read(p):return json.loads(Path(p).read_text())
def write(p,x):Path(p).write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def meshhash(m):return hashlib.sha256(json.dumps({k:m[k]for k in ['name','vertices','indices']},sort_keys=True,separators=(',',':')).encode()).hexdigest()
def poly(q):return K.polygon(np.asarray(q,float).tolist())
def rect(x0,y0,x1,y1):return poly([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
def inter(a,b):return K.G.GEOSIntersection(a,b)
def union(a,b):return K.G.GEOSUnion(a,b)
def free(*ps):
 for p in ps:K.G.GEOSGeom_destroy(p)
def loops(p):return K.polys(p)
def signedarea(q):return float(np.cross(q,np.roll(q,-1,axis=0)).sum()/2)
def strip(path,width,side):
 """Miter joins, square finite-end caps; side +1 is left of increasing-X path."""
 path=np.asarray(path,float);t=np.diff(path,axis=0);t/=np.linalg.norm(t,axis=1)[:,None];n=np.c_[-t[:,1],t[:,0]]*side;out=[path[0]+width*n[0]]
 for i in range(1,len(path)-1):
  a=path[i]+width*n[i-1];b=path[i]+width*n[i];mat=np.c_[t[i-1],-t[i]]
  out.append(a+t[i-1]*np.linalg.solve(mat,b-a)[0])
 out.append(path[-1]+width*n[-1]);return poly(np.vstack([path,np.asarray(out)[::-1]]))
def halfplane_line(a,b,inside,left=-20000,right=10000,bottom=-10000,top=10000):
 """Sutherland-Hodgman on a large explicit work box, no extending rail surfaces."""
 a,b,inside=map(lambda p:np.asarray(p,float),[a,b,inside]);sgn=np.sign(np.cross(b-a,inside-a));q=[np.array(p)for p in [[left,bottom],[right,bottom],[right,top],[left,top]]];out=[]
 def side(p):return sgn*np.cross(b-a,p-a)
 for p,r in zip(q,q[1:]+q[:1]):
  sp,sr=side(p),side(r)
  if sp>=0:out.append(p)
  if (sp>=0)!=(sr>=0):out.append(p+(r-p)*sp/(sp-sr))
 return poly(out)
def extrusion(name,p,z0,z1,kind,handles=(),choice=None):
 m=C.extrude_poly(name,p,2,np.zeros(3),z0/1000,z1/1000)
 m.update(color={'frog':[.60,.68,.75,1],'wing':[.34,.47,.59,1],'long_plate':[.31,.34,.37,1],'pad':[.65,.46,.24,1],'retained':[.45,.47,.49,1]}[kind],group='photo_turnout_revision',mechanical_fit_candidate=True,metadata={'geometry_status':'PHOTO_SUPPORTED_TOPOLOGY_ENGINEERING_SECTION_CANDIDATE','as_built_confirmed':False,'physics_status':'STATIC_NO_SOLVER_NO_COLLISION_ADMISSION','source_handles':list(handles),'role':kind,'engineering_choice':choice})
 return m

def clip_mesh_plane(m,normal,point,keep_positive=True,name=None):
 """Preserve original world triangles, add cap at explicit finite end plane.
 Cut vertices are welded at 1e-9 mm numerical tolerance. Existing positions
 are preserved verbatim wherever a source vertex is retained.
 """
 normal=np.asarray(normal,float);normal/=np.linalg.norm(normal);point=np.asarray(point,float)
 R=np.array([normal,[-normal[1],normal[0],0.],[0.,0.,1.]])
 vertices=C.verts(m);tri=vertices[C.faces(m)];vv=[];ff=[];lookup={}
 def index(p):
  key=tuple(round(float(x),12)for x in p)
  if key not in lookup:lookup[key]=len(vv);vv.append([float(x)for x in p])
  return lookup[key]
 for t in tri:
  if np.max(np.abs((t-point)@normal))<1e-12:continue
  out=[]
  def signed(p):
   d=np.dot(p-point,normal)*(1 if keep_positive else -1)
   return 0. if abs(d)<1e-12 else d
  for p,q in zip(t,np.roll(t,-1,axis=0)):
   a,b=signed(p),signed(q)
   if a>=0:out.append(p)
   if (a>=0)!=(b>=0):out.append(p+(q-p)*a/(a-b))
  if len(out)>=3:
   for k in range(1,len(out)-1):
    tt=np.array([out[0],out[k],out[k+1]])
    if np.linalg.norm(np.cross(tt[1]-tt[0],tt[2]-tt[0]))>1e-18:ff.append([index(p)for p in tt])
 # Cap the actual clipped surface boundary. This remains robust when the
 # finite-end plane coincides with a source station's triangle vertices.
 vf=np.asarray(vv);fi=np.asarray(ff);edges=np.sort(np.concatenate([fi[:,[0,1]],fi[:,[1,2]],fi[:,[2,0]]]),axis=1)
 unique,count=np.unique(edges,axis=0,return_counts=True);boundary=unique[count==1]
 local=(vf-point)@R.T;lines=[];cap_lookup={}
 for i,j in boundary:
  assert max(abs(local[[i,j],0]))<1e-10,(m['name'],'nonplanar boundary')
  uv=np.round(local[[i,j],1:]*1000,9);lines.append(uv.tolist())
  for k,q in zip([i,j],uv):cap_lookup[tuple(q)]=int(k)
 node=K.G.GEOSUnaryUnion(K.linework(lines))if lines else K.read('MULTILINESTRING EMPTY');arr=(K.P*1)(node);section=K.G.GEOSPolygonize(arr,1)
 def capindex(q):
  key=tuple(np.round(q,9))
  if key in cap_lookup:return cap_lookup[key]
  return index(np.array([0.,q[0]/1000,q[1]/1000])@R+point)
 for tt in K.triangulate(section):
  p=np.asarray(tt[0][:-1]);want=-1 if keep_positive else 1
  if signedarea(p)*want<0:p=p[::-1]
  ids=[capindex(q)for q in p]
  if len(set(ids))==3:ff.append(ids)
 free(node)
 free(section)
 out=copy.deepcopy(m);out.update(name=name or m['name'],vertices=np.asarray(vv).ravel().tolist(),indices=np.asarray(ff).ravel().tolist(),mechanical_fit_candidate=True)
 out['metadata']=copy.deepcopy(out.get('metadata',{}));out['metadata'].update(role='unchanged_source_remainder',source_name=m['name'],explicit_join_plane={'normal':normal.tolist(),'point_m':point.tolist()},cap_weld_tolerance_mm=1e-9,as_built_confirmed=False)
 return out

def clip_mesh_x(m,lo=None,hi=None,name=None):
 assert (lo is None)!=(hi is None)
 return clip_mesh_plane(m,[1,0,0],[lo if lo is not None else hi,0,0],lo is not None,name)

def clean_plan(p):
 polys=[]
 for rings in K.polys(p):
  result=[]
  for r in rings:
   pts=[]
   for q in r[:-1]:
    if not pts or np.linalg.norm(np.array(q)-pts[-1])>1e-8:pts.append(q)
   if len(pts)>1 and np.linalg.norm(np.array(pts[0])-pts[-1])<1e-8:pts.pop()
   result.append(pts)
  polys.append(K.polygon(result[0],result[1:]))
 out=polys[0]
 for p in polys[1:]:out=union(out,p)
 return out

def build(config_path=HERE/'config.json',out=HERE/'build'):
 cfg=read(config_path);assert cfg['as_built_confirmed']is False and cfg['physics_enabled']is False
 assert cfg['cross_section_mode']=='inherited_source_I_section_with_explicit_nose_and_slot_cuts'
 assert cfg['plan_association']=='ED_FE_frog__EE_FA_lower_wing__ED02_EC_upper_wing'
 assert cfg['head_width_mode']=='native_paired_bands_39p5mm'
 assert cfg['wing_end_join']=='normal_square_caps_at_native_finite_endpoints'
 assert cfg['frog_tail_join']=='independent_native_band_end_caps'
 top=cfg['rail_top_z_mm'];bottom=-64.99999761581421;platetop=cfg['support_top_z_mm'];platebottom=platetop-cfg['long_plate_thickness_mm'];padbottom=cfg['support_stack_bottom_z_mm']
 assert cfg['slot_depth_mm'] in [58.,60.,65.], 'Validated topology choices are 58, 60, or 65 mm; other depths require a new native topology check.'
 assert padbottom<platebottom<bottom<top
 graph=read(HERE/'native_edge_graph.json');curves={r['handle']:np.array(r['points'])for r in graph['source_curves']}
 source=json.load(gzip.open(HERE/'source_seed.json.gz','rt'));scene={m['name']:m for m in source['meshes']}
 ed,fe=curves['4ECED'],curves['4ECFE'];nose=ed[0];assert np.linalg.norm(nose-fe[0])<1e-5
 width=curves['4ED17'][0,1]-ed[0,1];assert abs(width-39.5)<1e-7
 branchwidth=abs(np.cross((fe[1]-fe[0])/np.linalg.norm(fe[1]-fe[0]),curves['4ED00'][0]-fe[0]));assert abs(branchwidth-width)<1e-7
 lower=np.array([curves['4ED5C'][1],curves['4ED5C'][0],curves['4ECEE'][0],curves['4ECFA'][0]])
 upper=np.array([curves['4ED5D'][1],curves['4ED5D'][0],curves['4ED02'][0],curves['4ECEC'][0]])
 assert np.linalg.norm(curves['4ED5C'][0]-curves['4ECEE'][1])<1e-5
 assert np.linalg.norm(curves['4ED5D'][0]-curves['4ED02'][1])<1e-5
 assert np.linalg.norm(curves['4ECEE'][0]-curves['4ECFA'][1])<1e-5
 assert np.linalg.norm(curves['4ED02'][0]-curves['4ECEC'][1])<1e-5
 # V arms use the finite source 39.5 mm bands. Clip each band to the wedge:
 # the topological candidate closes their nose, but never fills the remote V gap.
 h1=halfplane_line(*ed,nose+[0,1]);h2=halfplane_line(*fe,nose+[0,-1]);wedge=inter(h1,h2)
 straight=strip(ed[::-1],width,+1);branch=strip(fe[::-1],width,-1)
 frog=clean_plan(inter(union(straight,branch),wedge));low=strip(lower,width,-1);up=strip(upper,width,+1)
 plans={'PHOTO_FROG_source_finite_V':frog,'PHOTO_WING_lower_source_chain':low,'PHOTO_WING_upper_source_chain':up}
 meshes=[extrusion(n,p,bottom,top,'frog'if'FROG'in n else'wing',{'PHOTO_FROG_source_finite_V':['4ECED','4ECFE','4ED17','4ED00'],'PHOTO_WING_lower_source_chain':['4ED5C','4ECEE','4ECFA'],'PHOTO_WING_upper_source_chain':['4ED5D','4ED02','4ECEC']}[n],cfg['engineering_choice_note'])for n,p in plans.items()]
 from section_builder import make_sections
 new_geometry,section_evidence=make_sections(cfg,scene,curves,lower,upper,wedge,plans,out)
 for mesh in meshes:
  mesh.update({k:new_geometry[mesh['name']][k]for k in ['vertices','indices']})
  if C.signed_volume(mesh)<0:mesh['indices']=C.faces(mesh)[:,::-1].ravel().tolist()
 write(out/'inherited_section_evidence.json',section_evidence)
 # A long plate below the independently manufactured candidate rail bodies.
 # Margin and end cap are explicit design parameters; photo has no metric scale.
 full_lower=strip(lower,width+16.750007033348084,-1);full_upper=strip(upper,width+16.750007033348084,+1);allplan=union(union(frog,full_lower),full_upper);coords=np.concatenate([np.asarray(r)for loops_ in K.polys(allplan)for r in loops_]);x0=min(lower[0,0],upper[0,0])-cfg['support_end_margin_mm'];x1=max(lower[-1,0],upper[-1,0])+cfg['support_end_margin_mm'];margin=cfg['support_edge_margin_mm']
 from scipy.spatial import ConvexHull
 platepts=coords[ConvexHull(coords).vertices];plate=K.G.GEOSBuffer(poly(platepts),margin,1);plate=inter(plate,rect(x0,-1000,x1,1500))
 meshes.append(extrusion('PHOTO_SUPPORT_separate_long_plate',plate,platebottom,platetop,'long_plate',choice='Convex plate plan with explicit margins; 4 mm choice within inherited 7 mm support stack; no weld/bolt assumption.'))
 pads=[]
 for name,m in scene.items():
  if not name.startswith('CAD_TRACK_REV_long_sleeper_'):continue
  footprint=C.cap(m,2,np.zeros(3),'max');pad=inter(footprint,plate);area=K.area(pad)
  if area>1e-5:
   nm='PHOTO_SUPPORT_pad_'+name.split('sleeper_')[1].split('_')[0];meshes.append(extrusion(nm,pad,padbottom,platebottom,'pad',choice='Separate pad plan is the actual sleeper top cap intersected with the candidate long plate. Thickness is remaining support-stack height.'));pads.append({'name':nm,'retained_sleeper':name,'contact_area_mm2':area})
  free(footprint,pad)
 # Replace bounded bodies, never silently hide a full-length rail to clear a view.
 remove=['DETAIL_BENCH_turnout_inner_left_segment','ENV_main_rail_1','ENV_main_rail_top_1','CAD_TRACK_REV_inner_right_native_xy']
 x_main=upper[-1,0]/1000;x_branch=lower[-1,0]/1000;xleft=ed[1,0]/1000
 for src,x in [('ENV_main_rail_1',x_main),('ENV_main_rail_top_1',x_main)]:meshes.append(clip_mesh_x(scene[src],lo=x,name='PHOTO_RETAIN_right_'+src))
 normal=lower[-1]-lower[-2];normal/=np.linalg.norm(normal);src='CAD_TRACK_REV_inner_right_native_xy';meshes.append(clip_mesh_plane(scene[src],[*normal,0],[*(lower[-1]/1000),0],True,'PHOTO_RETAIN_right_'+src))
 for src in ['ENV_main_rail_1','ENV_main_rail_top_1']:meshes.append(clip_mesh_x(scene[src],hi=xleft,name='PHOTO_RETAIN_left_'+src))
 # Supports alone are replaced inside the new plate. Fasteners are retained;
 # any resulting actual material conflicts must remain visible in the audit.
 support_removed=[]
 for name,m in scene.items():
  if not ('chair' in name or name=='FIT_TRACK_station03_common_support'):continue
  v=C.verts(m)
  if v[:,2].min()<-0.073 or v[:,2].max()>-0.064:continue
  footprint=C.cap(m,2,np.zeros(3),'max');hit=inter(footprint,plate);area=K.area(hit)
  if area>1e-5:
   if v[:,0].min()<x0/1000-0.1 or v[:,0].max()>x1/1000+0.1:raise ValueError('Partly remote support needs explicit treatment '+name)
   remove.append(name);support_removed.append({'name':name,'plate_overlap_area_mm2':area,'disposition':'replaced_by_separate_plate_and_one_pad_per_retained_sleeper','original_bounds_m':[v.min(0).tolist(),v.max(0).tolist()]})
  free(footprint,hit)
 assert len(remove)==len(set(remove));out.mkdir(exist_ok=True,parents=True)
 for m in meshes:m['geometry_content_sha256']=meshhash(m)
 sourcechecks=[{'name':name,'expected_geometry_witness_sha256':scene[name]['geometry_witness_sha256']}for name in remove]
 facts={'nose_world_mm':nose.tolist(),'horizontal_band_width_mm':width,'diverging_perpendicular_band_width_mm':branchwidth,'lower_slot_horizontal_mm':ed[0,1]-curves['4ECEE'][0,1],'upper_slot_perpendicular_mm':abs(np.cross((fe[1]-fe[0])/np.linalg.norm(fe[1]-fe[0]),curves['4ED02'][0]-fe[0])),'source_boundaries_are_finite':True,'photo_metric_dimensions':[],'all_57_source_curves_retained_in_overlay':True,'photo_to_cad_registration':'REGION_LEVEL_CANDIDATE_NOT_CALIBRATED','section_choice':cfg['cross_section_mode'],'rail_z_mm':[bottom,top],'long_plate_z_mm':[platebottom,platetop],'pad_z_mm':[padbottom,platebottom],'new_physics':False,'source_track_axes_changed':False}
 manifest={'schema':'gtsd-photo-turnout-change-manifest-v1','candidate_id':cfg['candidate_id'],'base_commit':'ae463b951f831fb552bcd46f3528b4c10c1cf355','source_scene_mesh_count':source['scene_mesh_count'],'remove':sourcechecks,'add':[{'name':m['name'],'geometry_content_sha256':m['geometry_content_sha256'],'role':m['metadata']['role']}for m in meshes],'after_scene_mesh_count':source['scene_mesh_count']-len(remove)+len(meshes),'support_replacements':support_removed,'separate_pads':pads,'facts':facts,'config_sha256':digest(config_path),'native_edge_graph_sha256':digest(HERE/'native_edge_graph.json'),'source_seed_sha256':digest(HERE/'source_seed.json.gz'),'explicit_join_planes_mm':{'main_right':x_main*1000,'branch_right':x_branch*1000,'main_left':xleft*1000},'unresolved':['Photo/world registration, mirror and manufacturing ownership remain candidate-level.','Wing outer edges, miter/square end caps and all Z sections are engineering choices.','Source 15.333 mm association is not a photo measurement or certified operational flangeway.','Source fasteners remain; attachment holes, fastening, end transitions, and conflicts are not auto-resolved.','Outer guard/base rail overlap is unchanged and still unresolved.','No load capacity, wheel passage, switch state, as-built or solver certification.']}
 payload={'schema':'gtsd-photo-turnout-meshes-v1','units':'m','as_built':False,'new_physics':False,'meshes':meshes}
 (out/'candidate_meshes.json.gz').write_bytes(gzip.compress(json.dumps(payload,separators=(',',':'),allow_nan=False).encode(),mtime=0))
 write(out/'change_manifest.json',manifest)
 write(out/'plan_geometry.json',{'units':'mm','plans':{n:K.gj(p)for n,p in {**plans,'PHOTO_SUPPORT_separate_long_plate':plate}.items()},'lower_chain':lower.tolist(),'upper_chain':upper.tolist(),'facts':facts})
 return manifest,payload
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,default=HERE/'config.json');ap.add_argument('--output',type=Path,default=HERE/'build');a=ap.parse_args();m,p=build(a.config,a.output);print(json.dumps({'removed':len(m['remove']),'added':len(m['add']),'after':m['after_scene_mesh_count'],'facts':m['facts']},indent=2))
