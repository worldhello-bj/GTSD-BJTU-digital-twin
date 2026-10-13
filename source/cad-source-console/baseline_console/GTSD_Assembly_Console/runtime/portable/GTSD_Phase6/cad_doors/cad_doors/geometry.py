"""Only new immutable composition is served; never overwrite older geometry."""
import copy,hashlib,json
from functools import lru_cache
import numpy as np
from .bootstrap import ROOT,PATHS,MANIFEST
from cad_body.geometry import resource as body_resource

def read_sources():
 p=PATHS['doors'];return json.loads((p/'ASSEMBLY_CONTRACT.json').read_text()),json.loads((p/'outputs/door_cad_candidate_meshes.json').read_text())

@lru_cache(maxsize=1)
def resource():
 c,d=read_sources();prior=json.loads(body_resource()[0]);repl=c['replacement_manifest'];removed=set(repl['replaced_source_door_mesh_names'])|set(repl['replaced_cad_body_mesh_names']);prior_names={m['name'] for m in prior['meshes']}
 # End controls were deliberately absent from the prior body overlay. They can
 # now be reintroduced only as new exact-name door-mounted candidates.
 absent=sorted(removed-prior_names);body_removed=set(body_resource()[2]['replaced_source_mesh_names'])
 if any(n not in body_removed for n in absent):raise ValueError('Replacement target absent from both prior overlay and its explicit removals')
 additions=[];shift=np.asarray(c['component_to_reference_world_translation_m'])
 for mesh in d['meshes']:
  m=copy.deepcopy(mesh);m['vertices']=(np.asarray(m['vertices']).reshape(-1,3)+shift).reshape(-1).tolist();m['group']='body';m['display_group']='cad_end_doors';m['registered_component_id']='';m['physics_body_name']='door_'+m['door_key'] if m.get('door_key') else 'payload_secondary'
  if m['name']==c['doors']['end_neg_y_neg']['fan_target_mesh_name']:m['physics_body_name']='bay_fan_stator'
  additions.append(m)
 out=copy.deepcopy(prior);out['meshes']=[m for m in prior['meshes'] if m['name'] not in removed]+additions
 if len({m['name'] for m in out['meshes']})!=len(out['meshes']):raise ValueError('Duplicate geometry identity')
 overlay=dict(schema='gtsd-cad-door-composed-overlay-v1',prior_composed_geometry_sha256=body_resource()[1],door_resource_sha256=hashlib.sha256((PATHS['doors']/'outputs/door_cad_candidate_meshes.json').read_bytes()).hexdigest(),
  removed_mesh_names=sorted(removed),previously_removed_control_mesh_names=absent,added_mesh_names=[m['name'] for m in additions],component_to_reference_world_translation_m=shift.tolist(),
  boundary='CAD opening dimensions; leaf gaps, fan bore, hinge depth, hardware positions and masses remain research candidates. Open source grilles remain visual-only.')
 out['cad_door_overlay']=overlay;data=json.dumps(out,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode();return data,hashlib.sha256(data).hexdigest(),overlay,additions

def patch_binding(binding,model,data):
 value=copy.deepcopy(binding);raw,digest,overlay,added=resource();remove=set(overlay['removed_mesh_names']);value['meshes']=[r for r in value['meshes'] if r['mesh_name'] not in remove]
 for mesh in added:
  v=np.asarray(mesh['vertices']).reshape(-1,3);value['meshes'].append(dict(mesh_name=mesh['name'],body_name=mesh['physics_body_name'],mode='rigid',visible_in_dynamic_view=True,
   evidence=mesh['geometry_status'],source_refs=mesh['source_refs'],reference_aabb_min_m=v.min(0).tolist(),reference_aabb_max_m=v.max(0).tolist(),note='Actual native pose binding. Geometry is a CAD/photo research assembly, with source-defined uncertain mounting.'))
 for i in range(1,model.nbody):value['reference_poses'][model.body(i).name]={'position':data.xpos[i].tolist(),'quaternion':data.xquat[i].tolist()}
 value.update(source_geometry_sha256=digest,variant='cad_doors_mounted',cad_door_overlay=copy.deepcopy(overlay));return value
