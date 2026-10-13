"""Independent read-only combined profile audit; world-space triangle resources only.
Run with --source source.json.gz --combined combined.json.gz --profiles profiles.json
--replacements source_replacements.json --output output_directory.
"""
import argparse,gzip,hashlib,itertools,json,math,time
from pathlib import Path
from collections import Counter
import numpy as np
from triangle_classifier import classify,closed,TOL

def read(p):
 b=Path(p).read_bytes();return json.loads(gzip.decompress(b) if str(p).endswith('.gz') else b)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def witness(m):
 v=np.asarray(m['vertices'],dtype='<f4');v[v==0]=0
 return hashlib.sha256(v.tobytes()+np.asarray(m['indices'],dtype='<u4').tobytes()).hexdigest()
def winding(geom,p):
 v,f=geom;t=v[f]-p;a,b,c=t[:,0],t[:,1],t[:,2];la=np.linalg.norm(a,axis=1);lb=np.linalg.norm(b,axis=1);lc=np.linalg.norm(c,axis=1)
 num=np.einsum('ij,ij->i',a,np.cross(b,c));den=la*lb*lc+np.einsum('ij,ij->i',a,b)*lc+np.einsum('ij,ij->i',b,c)*la+np.einsum('ij,ij->i',c,a)*lb
 return float(np.sum(2*np.arctan2(num,den))/(4*np.pi))
def surface_distance(geom,p):
 v,f=geom;t=v[f];a,b,c=t[:,0],t[:,1],t[:,2];ab=b-a;ac=c-a;ap=p-a;n=np.cross(ab,ac);n2=(n*n).sum(1);ok=n2>1e-30
 signed=np.zeros(len(f));signed[ok]=np.einsum('ij,ij->i',ap,n)[ok]/n2[ok];proj=p-signed[:,None]*n;pp=proj-a
 d00=(ab*ab).sum(1);d01=(ab*ac).sum(1);d11=(ac*ac).sum(1);d20=(pp*ab).sum(1);d21=(pp*ac).sum(1);den=d00*d11-d01*d01;valid=den>1e-30
 u=np.zeros(len(f));w=u.copy();u[valid]=(d11*d20-d01*d21)[valid]/den[valid];w[valid]=(d00*d21-d01*d20)[valid]/den[valid]
 interior=ok&valid&(u>=0)&(w>=0)&(u+w<=1);dist=float(np.sqrt(np.min((signed[interior]**2)*n2[interior]))) if interior.any() else float('inf')
 for s,e in [(a,b),(b,c),(c,a)]:
  d=e-s;d2=(d*d).sum(1);q=np.zeros(len(f));nz=d2>0;q[nz]=np.einsum('ij,ij->i',p-s,d)[nz]/d2[nz];q=np.clip(q,0,1);dist=min(dist,float(np.linalg.norm(p-s-q[:,None]*d,axis=1).min()))
 return dist

def topo(geom):
 v,f=geom;uv,inv=np.unique(v,axis=0,return_inverse=True);ff=inv[f];e=np.concatenate([ff[:,[0,1]],ff[:,[1,2]],ff[:,[2,0]]]);ue,counts=np.unique(np.sort(e,axis=1),axis=0,return_counts=True);_,direct=np.unique(e,axis=0,return_counts=True)
 areas=np.linalg.norm(np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]),axis=1)/2
 return {'welded_vertex_count':len(uv),'triangles':len(f),'boundary_edge_count':int((counts==1).sum()),'nonmanifold_edge_count':int((counts>2).sum()),'closed_edge_two_incidence':bool((counts==2).all()),'consistent_directed_edges':bool((counts==2).all() and (direct==1).all()),'degenerate_triangle_count':int((areas<1e-20).sum())}
def main():
 ap=argparse.ArgumentParser();
 for x in ['source','combined','profiles','replacements','output']:ap.add_argument('--'+x,required=True)
 ap.add_argument('--float32',action='store_true');args=ap.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
 sd,cd,pd,rd=[read(getattr(args,x)) for x in ['source','combined','profiles','replacements']]
 sm={m['name']:m for m in sd['meshes']};new={m['name']:m for m in cd['meshes']};refs={m['name']:m for m in cd['mesh_refs']};am={n:new.get(n,sm.get(n)) for n in refs};assert all(am.values())
 assert len(sm)==1932 and len(new)==33 and len(am)==1928
 for m in list(sm.values())+list(new.values()):assert witness(m)==m['geometry_witness_sha256'],m['name']
 for n,m in am.items():assert m['geometry_witness_sha256']==refs[n]['geometry_witness_sha256'],n
 profile=next(x for x in pd['profiles'] if x['id']=='combined');removed={m['name'] for g in profile['groups'] for m in g['remove']};assert len(removed)==37 and set(am)==set(sm)-removed|set(new)
 replacement={r['old_name']:r['add_names'] for r in rd['replacements']};pred={};group={};successor={}
 for g in profile['groups']:
  for m in g['add_meshes']:
   n=m['name'];md=m['metadata'];old=md.get('replaces_mesh_names',[md.get('replaces_mesh_name',md.get('source_mesh'))]);old=[replacement[k][0] if k not in removed and k in replacement and len(replacement[k])==1 else k for k in old];assert all(k in removed for k in old),(n,old);pred[n]=old;group[n]=g['id']
   assert m['geometry_witness_sha256']==new[n]['geometry_witness_sha256']
   for k in old:assert k not in successor;successor[k]=n
 assert set(successor)==removed
 geo={n:(np.asarray(m['vertices'],np.float32 if args.float32 else float).astype(float).reshape(-1,3),np.asarray(m['indices'],int).reshape(-1,3)) for n,m in (sm|new).items()};bb={n:(g[0].min(0),g[0].max(0)) for n,g in geo.items()}
 def broad(names,changed):
  pairs=set()
  for n in changed:
   for b in names:
    if n==b:continue
    pair=tuple(sorted((n,b)))
    dep=np.minimum(bb[n][1],bb[b][1])-np.maximum(bb[n][0],bb[b][0])
    if np.all(dep>TOL):pairs.add(pair)
  return sorted(pairs)
 beforepairs=broad(sm,removed);afterpairs=broad(am,new);print('Broad phase',len(beforepairs),len(afterpairs),flush=True)
 cache={};evidence=out/'narrow_phase.jsonl'
 def narrow(pair):
  pair=tuple(sorted(pair))
  if pair in cache:return cache[pair]
  a,b=pair;start=time.time();r=classify(geo[a],geo[b]);r['aabb_overlap_volume_upper_bound_mm3']=float(np.prod(np.maximum(0,np.minimum(bb[a][1],bb[b][1])-np.maximum(bb[a][0],bb[b][0])))*1e9)
  if r['status']=='POSITIVE_MATERIAL_OVERLAP':
   p=np.array(r['witness_world_m']);winds=[winding(geo[n],p) for n in pair];dist=[surface_distance(geo[n],p) for n in pair];r['independent_witness_winding_numbers']=winds;r['independent_witness_surface_distances_mm']=[d*1000 for d in dist]
   valid=all(abs(abs(w)-1)<1e-5 for w in winds) and min(dist)>TOL and all(closed(*geo[n]) for n in pair)
   r['independently_confirmed_strict_interior']=bool(valid)
   if valid:r['guaranteed_inscribed_overlap_ball_volume_mm3']=(4*np.pi/3)*(min(dist)*1000)**3
   else:
    sampled=r;r=classify(geo[a],geo[b],find_witness=False);r['unconfirmed_sample_evidence']=sampled;r['independent_witness_confirmation_failed']=True
  cache[pair]=r;print(' | '.join(pair),r['status'],round(time.time()-start,2),flush=True)
  with evidence.open('a') as f:f.write(json.dumps({'pair':list(pair),'classification':r})+'\n')
  return r
 if evidence.exists():evidence.unlink()
 after=[]
 for pair in afterpairs:
  br=[{'pair':list(old),'classification':narrow(old)} for old in itertools.product(pred.get(pair[0],[pair[0]]),pred.get(pair[1],[pair[1]])) if old[0]!=old[1]]
  ar=narrow(pair);clear=ar['status'] in ['NO_POSITIVE_VOLUME_AABB','TRIANGULATED_MESHES_SEPARATED'];bp=any(x['classification']['status']=='POSITIVE_MATERIAL_OVERLAP' for x in br)
  after.append({'pair':list(pair),'group_pair':[group.get(n,'unchanged') for n in pair],'before':br,'after':ar,'comparison':'CLEARED_OR_SEPARATED' if clear else ('INHERITED_POSITIVE_OVERLAP_OR_INTERFACE' if bp else ('INHERITED_UNRESOLVED_CONTACT_REQUIRES_REVIEW' if any(x['classification']['status']=='SURFACE_OR_CONTAINMENT_REQUIRES_REVIEW' for x in br) else 'NEW_UNRESOLVED_INTERFACE_REQUIRES_REVIEW'))})
 before=[]
 for pair in beforepairs:
  br=narrow(pair);apair=tuple(sorted(successor.get(n,n) for n in pair));ar={'status':'MERGED_INTO_SINGLE_DESIGN_CANDIDATE','mesh':apair[0]} if apair[0]==apair[1] else narrow(apair)
  before.append({'pair':list(pair),'before':br,'after_pair':list(apair),'after':ar})
 cross=[list(pair) for pair in itertools.combinations(sorted(new),2) if group[pair[0]]!=group[pair[1]]]
 report={'schema':'gtsd-independent-combined-triangle-audit-v1','input_hashes':{k:sha(getattr(args,k)) for k in ['source','combined','profiles','replacements']},'scene':sd['source_scene'],'coordinate_mode':'BLENDER_FLOAT32_ROUNDTRIP' if args.float32 else 'EXPORTED_JSON_DOUBLE','tolerance_m':TOL,'counts':{'before_meshes':len(sm),'removed':len(removed),'added':len(new),'unchanged':len(am)-len(new),'after_meshes':len(am),'after_changed_vs_all_unique_pairs':len(new)*(len(am)-len(new))+len(new)*(len(new)-1)//2,'before_removed_vs_all_unique_pairs':len(removed)*(len(sm)-len(removed))+len(removed)*(len(removed)-1)//2,'after_positive_aabb_candidates':len(afterpairs),'before_positive_aabb_candidates':len(beforepairs),'cross_profile_changed_pairs':len(cross),'cross_profile_positive_aabb_candidates':sum(group.get(a)!=group.get(b) and a in new and b in new for a,b in afterpairs)},'predecessors':pred,'candidate_topology':{n:topo(geo[n]) for n in new},'after_positive_aabb_pairs':after,'before_positive_aabb_pairs':before,'after_status_counts':dict(Counter(x['after']['status'] for x in after)),'all_scene_collision_free':False,'scope':'All changed meshes against every retained mesh plus all changed-versus-changed pairs. Unchanged-versus-unchanged pairs are outside this exhaustive pass. No dynamics, hardware, connectivity, fastening or as-built certification.','triangle_classifier_review':{'broad_phase':'World-space AABB for complete selections, tolerance 1e-8 m','narrow_phase':'All triangle AABB pairs followed by both triangles edge/face tests and coplanar 2D SAT; containment testing; closed edge incidence required for separation claim.','positive_witness':'Original vertical ray sample independently checked with solid-angle winding number and exact point/triangle minimum surface distance, closed-edge topology; shared internal-ball volume is conservative, never an estimated complete intersection volume.','limits':'Floating-point polygonal meshes; sampled witnesses do not quantify complete volume. Surface coincidence and failed witness checks remain review items. Self-intersection and manufacturing tolerances are not globally certified.','merged_components':'37 predecessors become 33 meshes; internal overlaps of formerly distinct plates are classified as merged design, not falsely described as a separated two-body interface.'}}
 (out/'combined_interference_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'counts':report['counts'],'after_status_counts':report['after_status_counts']},indent=2),flush=True)
if __name__=='__main__':main()
