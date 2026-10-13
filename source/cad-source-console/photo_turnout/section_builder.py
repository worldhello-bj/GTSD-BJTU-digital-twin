"""Source-profile sweep + explicit nose/slot CSG; no automatic clearance trimming."""
from pathlib import Path
import gzip,json,subprocess
import numpy as np
import cap_geometry as C
import geos_kernel as K
HERE=Path(__file__).resolve().parent

def sweep(name,path,profile,side):
 path=np.asarray(path,float);t=np.diff(path,axis=0);t/=np.linalg.norm(t,axis=1)[:,None];norm=np.c_[-t[:,1],t[:,0]]*side
 normals=[norm[0]]
 for i in range(1,len(path)-1):
  n=norm[i-1]+norm[i];n/=np.dot(n,norm[i]);normals.append(n)
 normals.append(norm[-1]);v=[]
 for p,n in zip(path,normals):
  for offset,z in profile:v.append([*(p+n*offset),z])
 n=len(profile);f=[]
 for k in range(len(path)-1):
  for j in range(n):
   a=k*n+j;b=k*n+(j+1)%n;c=(k+1)*n+(j+1)%n;d=(k+1)*n+j;f.extend([[a,b,c],[a,c,d]])
 # Triangulate the actual source YZ section once, then map to finite caps.
 pg=K.polygon(profile.tolist());lookup={tuple(q):i for i,q in enumerate(profile)}
 for tri in K.triangulate(pg):
  ix=[lookup[tuple(q)]for q in tri[0][:-1]];f.append(ix[::-1]);f.append([i+(len(path)-1)*n for i in ix])
 m={'name':name,'vertices':(np.asarray(v)/1000).ravel().tolist(),'indices':np.asarray(f).ravel().tolist()}
 if C.signed_volume(m)<0:m['indices']=np.asarray(f)[:,::-1].ravel().tolist()
 assert C.topology(m)['closed'],name
 return m

def remove_numeric_t_junction_slivers(m,tolerance_mm=1e-4):
 """Remove only the redundant triangle of a proven 1/3/3 T-junction pattern.
 Its longest-edge altitude must be sub-tolerance. A meaningful material face
 or an arbitrary open boundary cannot pass this repair rule.
 """
 v=C.verts(m)*1000;f=C.faces(m);ed=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1);u,counts=np.unique(ed,axis=0,return_counts=True);count={tuple(e):int(c)for e,c in zip(u,counts)};drop=[]
 for i,t in enumerate(f):
  c=sorted(count[tuple(sorted((a,b)))]for a,b in zip(t,np.roll(t,-1)))
  if c!=[1,3,3]:continue
  q=v[t];ar=np.linalg.norm(np.cross(q[1]-q[0],q[2]-q[0]))/2;long=max(np.linalg.norm(a-b)for a,b in zip(q,np.roll(q,-1,axis=0)));alt=2*ar/long
  if alt<=tolerance_mm:drop.append({'triangle':i,'area_mm2':float(ar),'long_edge_altitude_mm':float(alt),'edge_multiplicity_before':[1,3,3]})
 if drop:m['indices']=np.delete(f,[r['triangle']for r in drop],axis=0).ravel().tolist()
 if C.signed_volume(m)<0:m['indices']=C.faces(m)[:,::-1].ravel().tolist()
 assert C.topology(m)['closed'],(m['name'],C.topology(m))
 return drop

def make_sections(cfg,scene,curves,lower,upper,wedge,headplans,out):
 # Only observed source-mesh section values are inherited. They remain an
 # engineering 3D section, not a section measurement made from photo 7.
 m=scene['ENV_main_rail_1'];src=C.cap(m,0,np.zeros(3),'min');ring=np.asarray(K.polys(src)[0][0][:-1]);headmin=float(ring[ring[:,1]==ring[:,1].max(),0].min());profile=ring.copy();profile[:,0]-=headmin
 z0=float(profile[:,1].min());z1=float(profile[:,1].max());slotbottom=z1-cfg['slot_depth_mm'];origin=[* (curves['4ECED'][0]/1000),0.]
 inputs={};inputs['straight']=sweep('straight',curves['4ECED'][::-1],profile,+1);inputs['branch']=sweep('branch',curves['4ECFE'][::-1],profile,-1);inputs['lower']=sweep('lower',lower,profile,-1);inputs['upper']=sweep('upper',upper,profile,+1)
 # Wedge constrains the nose only over the explicitly selected slot depth.
 # Beneath a shallower slot the original I-section foot is allowed to remain.
 from build_candidate import extrusion,strip,rect,union
 whole=rect(-9600,-100,-8200,700)
 for key,plan in [('frog',wedge),('lower',strip(lower,120,-1)),('upper',strip(upper,120,+1))]:
  cut_top=extrusion(key+'_keep_top',plan,z0-1 if slotbottom<=z0+1e-5 else slotbottom,z1+1,'retained')
  inputs[key+'_keep_top']=cut_top
  if slotbottom>z0+1e-7:
   inputs[key+'_keep_bottom']=extrusion(key+'_keep_bottom',whole,z0-1,slotbottom,'retained')
   # Keep top+bottom are disjoint touching volumes. Joining via exact union
   # creates one cutting body and does not merge the actual rail components.
  else:inputs[key+'_keep_bottom']=None
 # Prepare keep masks as union of their top and lower unrestricted slab.
 operations=[]
 # Masks are premerged separately in native script, avoiding order ambiguity.
 for key in ['frog','lower','upper']:
  pass
 operations=[{'name':'PHOTO_FROG_source_finite_V','base':'straight','steps':[['UNION','branch'],['INTERSECT','frog_keep']]},{'name':'PHOTO_WING_lower_source_chain','base':'lower','steps':[['INTERSECT','lower_keep']]},{'name':'PHOTO_WING_upper_source_chain','base':'upper','steps':[['INTERSECT','upper_keep']]}]
 request={'origin_m':origin,'inputs':{k:v for k,v in inputs.items()if v is not None},'keep_masks':[{'key':k,'top':k+'_keep_top','bottom':k+'_keep_bottom'if inputs[k+'_keep_bottom']is not None else None}for k in ['frog','lower','upper']],'operations':operations}
 out.mkdir(exist_ok=True,parents=True);req=out/'section_csg_request.json';result=out/'section_csg_result.json';req.write_text(json.dumps(request,separators=(',',':'))+'\n')
 command=['blender','-b','--factory-startup','-t','1','--python',str(HERE/'blender_section_boolean.py'),'--',str(req),str(result)]
 with (out/'section_csg.log').open('w')as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
 data=json.loads(result.read_text())
 # Restore source Z event doubles after local-float32 native CSG. This avoids
 # inventing a 0.000002 mm plate penetration from a known shared interface.
 for mesh in data['meshes'].values():
  vv=C.verts(mesh)
  for z in sorted(set(profile[:,1].tolist())):
   use=np.abs(vv[:,2]*1000-z)<1e-4;vv[use,2]=z/1000
  if mesh['name']=='PHOTO_FROG_source_finite_V':
   target_x=curves['4ECED'][1,0];use=np.abs(vv[:,0]*1000-target_x)<1e-4;vv[use,0]=target_x/1000
  if mesh['name']=='PHOTO_WING_upper_source_chain':
   endpoint=upper[-1];t=upper[-1]-upper[-2];t=t/np.linalg.norm(t);d=(vv[:,:2]*1000-endpoint)@t;use=np.abs(d)<1e-4;vv[use,:2]-=d[use,None]*t/1000
  elif mesh['name']=='PHOTO_WING_lower_source_chain':
   endpoint=lower[-1];t=lower[-1]-lower[-2];t=t/np.linalg.norm(t);d=(vv[:,:2]*1000-endpoint)@t;use=np.abs(d)<1e-4;vv[use,:2]-=d[use,None]*t/1000
  mesh['vertices']=vv.ravel().tolist()
 repairs={n:remove_numeric_t_junction_slivers(m)for n,m in data['meshes'].items()};evidence={'source_mesh':'ENV_main_rail_1','source_geometry_witness_sha256':m['geometry_witness_sha256'],'source_section_YZ_world_mm':ring.tolist(),'profile_offset_from_inner_head_edge_Z_mm':profile.tolist(),'Z_events_mm':sorted(set(profile[:,1].tolist())),'head_width_mm':float(np.ptp(profile[profile[:,1]==0,0])),'web_width_mm':float(np.ptp(profile[np.isclose(profile[:,1],-19),0])),'foot_width_mm':float(np.ptp(profile[np.isclose(profile[:,1],-65),0])),'slot_depth_engineering_choice_mm':cfg['slot_depth_mm'],'source_section_status':'INHERITED_SOURCE_RENDER_I_SECTION_NOT_AS_BUILT_CONFIRMED','boolean_method':'native exact CSG in local millimetres; only explicit nose wedge and flangeway ownership cut','native_precision_note':'float32 local mm vertices; source boundary checked at 0.001 mm, 1e-4 mm numerical vertex weld; no collision-driven trim','blender_version':data['blender_version'],'numerical_t_junction_repairs':repairs,'numerical_tolerance_mm':1e-4}
 return data['meshes'],evidence
