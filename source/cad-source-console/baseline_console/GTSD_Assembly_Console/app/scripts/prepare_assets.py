"""Index complete accepted GLB plus exact small alternative mesh arrays; no new geometry."""
from pathlib import Path
import json,gzip,struct,mmap,hashlib,numpy as np
R=Path(__file__).resolve().parents[1];BASE=R.parent/'gtsd-whole-clamp-integration-1012'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb')as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
D=json.loads((BASE/'DELIVERY.json').read_text());assert sha(BASE/'DELIVERY.json')=='dc5f4a32e89b779e366f4a332517c735ff81f685314505f931c39f3cc8a8b804'
pins=json.loads((BASE/'inputs/build_contract.json').read_text())
for k in ['upper_delta','lower_delta','lower_selection']:assert sha(Path(pins[k+'_path']))==pins[k+'_sha256'],k
B=json.loads((BASE/'evidence/build.json').read_text());E=json.loads((BASE/'evidence/export.json').read_text());gz=BASE/'outputs/GTSD_CAD_clamp_routes_review.glb.gz';assert sha(gz)==E['gzip_sha256']
with gzip.open(gz,'rb')as f:
 header=f.read(12);size,t=struct.unpack('<II',f.read(8));g=json.loads(f.read(size))
assert struct.unpack('<4sII',header)==(b'glTF',2,E['uncompressed_bytes']);default={n['name']:n for n in g['nodes']};assert set(default)==set(B['scenes'][B['default_scene']]['mesh_names'])
class GLB:
 def __init__(self,p):
  self.file=p.open('rb');self.raw=mmap.mmap(self.file.fileno(),0,access=mmap.ACCESS_READ);off=12
  while off<len(self.raw):
   size,t=struct.unpack_from('<II',self.raw,off);off+=8
   if t==0x4e4f534a:self.g=json.loads(self.raw[off:off+size])
   if t==0x004e4942:self.binary=off
   off+=size
  self.nodes={n['name']:n for n in self.g['nodes']if'mesh'in n}
 def acc(self,i):
  a=self.g['accessors'][i];v=self.g['bufferViews'][a['bufferView']];dt=np.dtype({5123:'<u2',5125:'<u4',5126:'<f4'}[a['componentType']]);n={'SCALAR':1,'VEC3':3,'VEC2':2,'VEC4':4}[a['type']]
  return np.ndarray((a['count'],n),dtype=dt,buffer=self.raw,offset=self.binary+v.get('byteOffset',0)+a.get('byteOffset',0),strides=(v.get('byteStride',dt.itemsize*n),dt.itemsize))
 def mesh(self,n):
  node=self.nodes[n];assert not any(k in node for k in ['translation','rotation','matrix','scale']);pr=self.g['meshes'][node['mesh']]['primitives'];assert len(pr)==1;p=pr[0];v=self.acc(p['attributes']['POSITION']).copy();v=v[:,[0,2,1]];v[:,1]*=-1;f=self.acc(p['indices']).reshape(-1,3);color=self.g['materials'][p['material']].get('pbrMetallicRoughness',{}).get('baseColorFactor',[.7,.7,.7,1]);return {'id':n,'name':n,'vertices':v.reshape(-1).tolist(),'indices':f.reshape(-1).tolist(),'color':color,'metadata':node.get('extras',{})}
priorpath=R.parent/'gtsd-whole-assembly-orthogonal-1012/outputs/GTSD_CAD_orthogonal_mast_review.glb';assert sha(priorpath)=='e5c3d9175b98c68b0e469b80be8708f13d746771a07723b9fe1db2adf9df0f4e';prior=GLB(priorpath)
U=json.loads((R.parent/'gtsd-upper-clamp-mesh-1012/upper_clamp_mesh_delta.json').read_text());L=json.loads((R.parent/'gtsd-lower-support-repair-1012/lower_support_mesh_delta.json').read_text());source={m['name']:m for m in U['meshes']+L['meshes']};reflection=np.array(json.loads((R.parent/'gtsd-lower-support-repair-1012/selection_contract.json').read_text())['second_station']['candidate_reflection']['world_new_from_world_old']);allnames=set().union(*(set(s['mesh_names'])for s in B['scenes'].values()));extras=[];metadata=[]
def group(meta,name):
 q=meta.get('group');return q if q in ['body','bogie','pantograph','environment','display']else'environment'if any(t in name for t in ['CAD_51','CAD_50','C4D1','B535','REPAIR_'])else'display'
for n in sorted(allnames):
 if n in default:meta=default[n].get('extras',{})
 else:
  if n in prior.nodes:m=prior.mesh(n)
  elif n in source:
   s=source[n];m={'id':n,'name':n,'vertices':np.array(s['vertices'],dtype=np.float32).reshape(-1).tolist(),'indices':np.array(s['indices']).reshape(-1).tolist(),'color':s.get('color_rgba',[.67,.74,.78,1]),'metadata':{k:v for k,v in s.items()if k not in ['vertices','indices']}}
  elif n.endswith('_FRONT_ROUTE_REFLECTED_CANDIDATE'):
   s=source[n.removesuffix('_FRONT_ROUTE_REFLECTED_CANDIDATE')];v=np.array(s['vertices']).reshape(-1,3);v=(np.c_[v,np.ones(len(v))]@reflection.T)[:,:3].astype(np.float32);m={'id':n,'name':n,'vertices':v.reshape(-1).tolist(),'indices':np.array(s['indices']).reshape(-1,3)[:,[0,2,1]].reshape(-1).tolist(),'color':[1,.48,.08,1],'metadata':{'source_mesh_reference':s['name'],'explicit_world_transform':reflection.tolist(),'source_transform_is_native':False,'candidate':True}}
  elif n.endswith('_SECOND_PLAN_ROUTE_CANDIDATE'):
   src=n.removesuffix('_SECOND_PLAN_ROUTE_CANDIDATE')
   if src in source:s=source[src];v=np.array(s['vertices']).reshape(-1,3);f=np.array(s['indices']).reshape(-1)
   else:s=prior.mesh(src);v=np.array(s['vertices']).reshape(-1,3);f=np.array(s['indices'])
   v[:,0]-=4;m={'id':n,'name':n,'vertices':v.astype(np.float32).reshape(-1).tolist(),'indices':f.tolist(),'color':[1,.48,.08,1],'metadata':{'source_mesh_reference':src,'explicit_world_translation_m':[-4,0,0],'source_transform_is_native':False,'candidate':True}}
  else:raise ValueError(n)
  meta=m['metadata'];m['group']=group(meta,n);extras.append(m)
 metadata.append({'name':n,'id':n,'group':group(meta,n),'registered_component_id':meta.get('registered_component_id')or None,'source_metadata':meta,'as_built':False,'new_collision_enabled':False})
raw=json.dumps({'schema':'gtsd-assembly-alternative-meshes-v1','units':'m','meshes':extras},ensure_ascii=False,separators=(',',':'),allow_nan=False).encode();blob=gzip.compress(raw,mtime=0);(R/'data/scene_extras.json.gz').write_bytes(blob)
out={'schema':'gtsd-whole-assembly-console-catalog-v1','units':'m','native_sha256':D['native']['sha256'],'native_delivery_sha256':sha(BASE/'DELIVERY.json'),'default_scene':B['default_scene'],'base_glb':{'url':'/assembly/model.glb','sha256':E['uncompressed_sha256'],'bytes':E['uncompressed_bytes'],'gzip_sha256':E['gzip_sha256'],'gzip_bytes':E['gzip_bytes']},'extras':{'url':'/assembly/extras.json','sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw),'gzip_sha256':hashlib.sha256(blob).hexdigest()},'scenes':B['scenes'],'meshes':metadata,'mesh_union_count':len(allnames),'reference_registry_count':109,'as_built':False,'new_physics_enabled':False,'warnings':['Default is a candidate review configuration, not confirmed installed layout.','Second station front/plan differ490.590529mm; candidates are exclusive.','4079rod and B535bar true intersections remain; no new contact enabled.','Source geometry, visual body correspondence, and actual physical collision mapping are separate.'],'conflicts':json.loads((BASE/'evidence/interface_interpretation.json').read_text())}
(R/'data/catalog.json').write_text(json.dumps(out,ensure_ascii=False,separators=(',',':'))+'\n');(R/'evidence/assets_preparation.json').write_text(json.dumps({'all_declared_meshes_found':True,'default_meshes':len(default),'union_meshes':len(allnames),'extra_meshes':len(extras),'extra_json_bytes':len(raw),'extra_gzip_bytes':len(blob),'source_native_unchanged':True,'full_glb_copied':False,'catalog_sha256':sha(R/'data/catalog.json'),'extra_source':'OriginalacceptedGLB or exactsourceFloat32; only explicitcontractreflection/translation applied.'},indent=2)+'\n');print(len(default),len(allnames),len(extras),len(blob))
