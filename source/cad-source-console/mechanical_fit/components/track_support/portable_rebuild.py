#!/usr/bin/env python3
"""Rebuild the published fit meshes solely from the checkout and small inputs.

Required: NumPy, SciPy. No Blender, solver, network, CAD database or private path.
"""
from pathlib import Path
import argparse,copy,gzip,hashlib,itertools,json,struct
import numpy as np
import mesh_geometry as g
from fit_profile import compose_fit_scene
HERE=Path(__file__).resolve().parent

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text())
def write(path,x):path.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def triangle_hash(v,f):
 a=np.asarray(v,dtype='<f4')[f].copy();a[a==0]=0;order=np.lexsort((a[:,:,2],a[:,:,1],a[:,:,0]),axis=1);a=np.take_along_axis(a,order[:,:,None],axis=1).reshape(-1,9);return hashlib.sha256(a[np.lexsort(a.T[::-1])].tobytes()).hexdigest()

def load_glb(path,catalog):
 zipped=path.read_bytes();assert hashlib.sha256(zipped).hexdigest()==catalog['base_glb']['gzip_sha256'];raw=gzip.decompress(zipped);assert hashlib.sha256(raw).hexdigest()==catalog['base_glb']['sha256'];assert struct.unpack_from('<III',raw)==(0x46546c67,2,len(raw));off=12;document=None;binary=None
 while off<len(raw):
  size,kind=struct.unpack_from('<II',raw,off);off+=8
  if kind==0x4e4f534a:document=json.loads(raw[off:off+size])
  elif kind==0x004e4942:binary=memoryview(raw)[off:off+size]
  else:raise ValueError('Unknown GLB chunk')
  off+=size
 assert document and binary is not None and not document.get('skins')and not document.get('animations')
 def accessor(i):
  a=document['accessors'][i];bv=document['bufferViews'][a['bufferView']];assert bv['buffer']==0 and not a.get('sparse');width={'VEC3':3,'SCALAR':1}[a['type']];dtype=np.dtype({5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']]);offset=bv.get('byteOffset',0)+a.get('byteOffset',0);stride=bv.get('byteStride',dtype.itemsize*width);assert offset+(a['count']-1)*stride+width*dtype.itemsize<=bv.get('byteOffset',0)+bv['byteLength'];return np.ndarray((a['count'],width),dtype,buffer=binary,offset=offset,strides=(stride,dtype.itemsize)).copy()
 result={}
 for i in document['scenes'][document.get('scene',0)]['nodes']:
  node=document['nodes'][i];assert not any(k in node for k in ['children','matrix','translation','rotation','scale']);p=document['meshes'][node['mesh']]['primitives'];assert len(p)==1 and p[0].get('mode',4)==4;v=accessor(p[0]['attributes']['POSITION']);f=accessor(p[0]['indices']).reshape(-1,3);w=np.stack([v[:,0],-v[:,2],v[:,1]],axis=1);result[node['name']]=(w,f)
 assert set(result)==set(catalog['scenes'][catalog['default_scene']]['mesh_names']);return result

def rebuild(project_root,output,inputs_path=HERE/'rebuild_inputs.json'):
 inputs=read(inputs_path);base_dir=project_root/'baseline_console/GTSD_Assembly_Console';catalog=read(base_dir/'app/data/catalog.json');base=load_glb(base_dir/'assets/whole.glb.gz',catalog);replacements_path=project_root/'integration/data/source_replacements.json';replacements=read(replacements_path);scene=dict(base)
 for r in replacements['replacements']:del scene[r['old_name']]
 for r in replacements['meshes']:scene[r['name']]=(np.array(r['vertices']).reshape(-1,3),np.array(r['indices']).reshape(-1,3))
 source={}
 for n,pose in inputs['poses_from_source_native_intersections'].items():
  v,f=base[n];m=np.array(pose['matrix']);assert np.array_equal(m[:3,:3],np.eye(3))and np.array_equal(m[3],[0,0,0,1]);source[n]=(v.astype(float)+m[:3,3],f)
 names=list(inputs['groups']);chair01='DETAIL_BENCH_branch_chair_01_0';chair03='DETAIL_BENCH_branch_chair_03_1';chair23='DETAIL_BENCH_branch_chair_23_2';main03='ENV_rail_chair.007';parameters=inputs['parameters'];pitch=parameters['pitch_mm'];offset=parameters['normal_offset_mm'];edge=parameters['outer_bolt_land_edge_allowance_mm'];assert 16<=pitch<=24 and 41.5<=offset<=42.5 and edge==1.0;output.mkdir(parents=True,exist_ok=True);reports=[]
 for mode,template in inputs['profile_metadata_templates_without_geometry'].items():
  profile=copy.deepcopy(template);assert profile['requirements']['source_replacements_sha256']==sha(replacements_path),'Wrong static source revision';built={}
  for station,constituents in inputs['shared_support_constituents'].items():
   vs=[source[n][0]for n in constituents];built['FIT_TRACK_'+station+'_common_support']=g.hull_mesh(np.concatenate(vs))if mode=='conforming_hull'else g.blank_mesh(vs)
  for chair,ns in inputs['groups'].items():
   for n in ns:
    if 'fastener'not in n:continue
    v,f=source[n];sign=int(n.rsplit('_',1)[1]);center=(v.min(0)+v.max(0))[:2]/2
    if chair==chair01:
     anchor=np.array(inputs['poses_from_source_native_intersections'][chair]['source_anchor_world_XY_m']);t=np.array(inputs['branch_tangents_world_XY'][chair]);normal=np.array([-t[1],t[0]]);target=anchor+normal*offset/1000+t*sign*pitch/2000
    elif chair==chair03:
     cc=(source[chair][0].min(0)+source[chair][0].max(0))/2;t=np.array(inputs['station03_lower_offset_tangent']);normal=np.array([-t[1],t[0]])
     if sign<0:target=cc[:2]-normal*.042
     else:
      cv=source[main03][0];target=np.array([(cv.min(0)[0]+cv.max(0)[0])/2,(cv.min(0)[1]+cv.max(0)[1])/2+.042])
    else:
     cc=(source[chair][0].min(0)+source[chair][0].max(0))/2;t=np.array(inputs['branch_tangents_world_XY'][chair]);normal=np.array([-t[1],t[0]]);target=cc[:2]+normal*sign*.042
    delta=np.zeros(3);delta[:2]=target-center;built['FIT_TRACK_clip_'+n.split('branch_fastener_')[1]]=(v+delta,f)
  old_v,old_f=source[chair01];ear=[]
  for name,(vv,_)in built.items():
   if name.startswith('FIT_TRACK_clip_01_0_'):
    xy=g.unique(vv[:,:2])
    for dx,dy in itertools.product([-edge/1000,edge/1000],repeat=2):
     for z in [old_v[:,2].min(),old_v[:,2].max()]:ear.extend(np.c_[xy+np.array([dx,dy]),np.full(len(xy),z)])
  built['FIT_TRACK_station01_source_chair']=g.hull_mesh(np.vstack([old_v,np.asarray(ear)]))
  identity=[]
  for mesh in profile['add_meshes']:
   v,f=built[mesh['name']];assert g.closed(v,f)and g.volume(v,f)>0;h=triangle_hash(v,f);assert h==mesh['metadata']['triangle_sha256'],(mesh['name'],'Regenerated source shape differs from frozen candidate');mesh['vertices']=v.ravel().tolist();mesh['indices']=f.ravel().tolist();identity.append({'name':mesh['name'],'triangle_sha256':h,'frozen_geometry_identity_verified':True})
  # Adapter checks the whole group and static flags before applying anything.
  display={n:{'name':n}for n in scene};after_map=compose_fit_scene(display,profile,sha(replacements_path));after={n:v for n,v in scene.items()if n not in profile['remove_names']};after.update(built);assert set(after_map)==set(after);checks=[];seen=set()
  for a,ga in built.items():
   for name,gb in after.items():
    key=tuple(sorted([a,name]))
    if name==a or key in seen:continue
    seen.add(key);va=ga[0];vb=gb[0]
    if np.any(np.minimum(va.max(0),vb.max(0))-np.maximum(va.min(0),vb.min(0))<=1e-8):continue
    result=g.pair_screen(ga,gb);assert result['status']in ['AABB_SEPARATED_OR_BOUNDARY_CONTACT','SEPARATING_PLANE_PROOF','EXACT_TRIANGULATED_MESHES_SEPARATED'],(key,result);checks.append({'pair':key,**result})
  write(output/('profile_'+mode+'.json'),profile);reports.append({'id':profile['id'],'pass':True,'base_glb_sha256':catalog['base_glb']['sha256'],'source_replacements_sha256':sha(replacements_path),'source_default_mesh_count':len(scene),'after_mesh_count':len(after),'geometry_identity_checks':identity,'full_scene_nontrivial_pairs':checks,'scope':'Only the nine regenerated candidate meshes against the complete actual source-default scene; global untouched pairs not certified.'})
 write(output/'portable_rebuild_validation.json',{'pass':True,'inputs_sha256':sha(inputs_path),'profiles':reports,'no_private_database_required':True,'no_solver_or_hardware':True});return reports

def main():
 p=argparse.ArgumentParser();p.add_argument('--project-root',type=Path,required=True,help='Root of source/cad-source-console after prepare_assets.py');p.add_argument('--output',type=Path,required=True);p.add_argument('--inputs',type=Path,default=HERE/'rebuild_inputs.json');a=p.parse_args();results=rebuild(a.project_root,a.output,a.inputs);print('REBUILT_AND_VERIFIED',len(results),'profiles',sum(len(r['geometry_identity_checks'])for r in results),'mesh identities')
if __name__=='__main__':main()
