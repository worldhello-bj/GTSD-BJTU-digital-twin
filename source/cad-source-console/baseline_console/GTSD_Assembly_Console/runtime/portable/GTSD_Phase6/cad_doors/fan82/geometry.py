import copy,hashlib,json
from functools import lru_cache
from cad_doors.geometry import resource as parent_resource
from .model import NAMES
@lru_cache(maxsize=1)
def resource():
 prior,oldsha,_,_=parent_resource();g=json.loads(prior);changes=[]
 for m in g['meshes']:
  k=m.get('door_key')
  if k in NAMES and m['name']=='CAD_fan_shroud_'+k:
   old=m['physics_body_name'];m['physics_body_name']=NAMES[k]+'_stator';changes.append({'mesh_name':m['name'],'old_physics_body_name':old,'new_physics_body_name':m['physics_body_name']})
 overlay={'schema':'gtsd-four-door-fan-overlay-v1','prior_geometry_sha256':oldsha,'rebound_exact_meshes':changes,'unchanged_vertices_and_indices':True,'rotor_visual_geometry':'None: no photographed blade geometry has been inferred; spinning coordinates and loads are native, shrouds are fixed stators.'};g['four_door_fan_overlay']=overlay
 raw=json.dumps(g,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode();return raw,hashlib.sha256(raw).hexdigest(),overlay
