"""Deterministic composition; no copy of the 13 MB baseline is stored."""
import copy,hashlib,json
from functools import lru_cache
import numpy as np
from .bootstrap import PATHS,MANIFEST

def canonical_bytes(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()

def read_sources():
    c=PATHS['cad'];b=PATHS['phase5']
    return (json.loads((c/'outputs/body_cad_candidate_meshes.json').read_text()),
            json.loads((c/'evidence/body_geometric_mass_properties.json').read_text()),
            json.loads((c/'evidence/body_geometry_validation.json').read_text()),
            json.loads((b/'assembly_contract.json').read_text()))

# Explicit source identities, not a runtime prefix deletion rule. The complete list
# is generated once against the verified baseline and committed as review evidence.
def replacement_names():
    from .bootstrap import ROOT
    return json.loads((ROOT/'replacement_manifest.json').read_text())['replaced_source_mesh_names']

@lru_cache(maxsize=1)
def resource():
    cad,moments,evidence,contract=read_sources()
    raw=(PATHS['phase5']/'web/geometry.json').read_bytes()
    baseline=json.loads(raw);removed=replacement_names();names=[m['name'] for m in baseline['meshes']]
    if len(names)!=len(set(names)) or any(names.count(n)!=1 for n in removed):raise ValueError('Replacement identity not unique')
    z=contract['body_bottom_z_m']; additions=[]
    for source in cad['meshes']:
        mesh=copy.deepcopy(source);v=np.asarray(mesh['vertices']).reshape(-1,3);v[:,2]+=z
        mesh['vertices']=v.reshape(-1).tolist();mesh['group']='body';mesh['display_group']='cad_body'
        mesh['registered_component_id']='';mesh['geometry_status']='CAD_SECTION_SWEEP_RIGID_PHYSICS_BOUND_RESEARCH'
        mesh['physics_body_name']='payload_secondary';additions.append(mesh)
    result=copy.deepcopy(baseline); result['meshes']=[m for m in baseline['meshes'] if m['name'] not in removed]+additions
    result['dimensions']['body_width_m']=evidence['source_outer_width_mm']/1000
    result['dimensions']['evidence']['body_width_m']='SOURCE_CAD_MODEL_INSTANCE_SECTION_zw$3E1B; width only, not mounting calibration'
    result['assumptions']['body_width_m']=result['dimensions']['evidence']['body_width_m']
    overlay=dict(schema='gtsd-cad-body-geometry-overlay-v1',baseline_geometry_sha256=hashlib.sha256(raw).hexdigest(),
       cad_source_identity=cad['source_identity'],cad_resource_sha256=MANIFEST['dependencies']['cad']['source_sha256']['outputs/body_cad_candidate_meshes.json'],
       replaced_source_mesh_names=removed,added_mesh_names=[m['name'] for m in additions],
       component_to_reference_world_translation_m=[0.,0.,z],render_contract='Existing reference-body rigid transform; no scaling, no prescribed animation.',
       mounting_status='Remaining doors, auxiliary units and sensor meshes retain explicitly unverified legacy candidate mounting; section-only CAD does not identify their positions.',
       shell_contact_boundary='26 exact visual solids and their volume moments; only floor is an active support collision. No shell/door collision or mounting certification.')
    result['cad_body_overlay']=overlay
    data=canonical_bytes(result)
    return data,hashlib.sha256(data).hexdigest(),overlay,additions

def visual_resource_bytes():return resource()[0]

def patch_binding(binding,model,data):
    value=copy.deepcopy(binding);raw,digest,overlay,additions=resource();removed=set(overlay['replaced_source_mesh_names'])
    value['meshes']=[r for r in value['meshes'] if r['mesh_name'] not in removed]
    for row in value['meshes']:
        if row.get('body_name')=='payload_secondary' or str(row.get('body_name','')).startswith('door_'):
            row['mounting_status']='LEGACY_CANDIDATE_NOT_MIGRATED_FROM_CAD_SECTION'
    bid=model.body('payload_secondary').id
    value['reference_poses']['payload_secondary']={'position':data.xpos[bid].tolist(),'quaternion':data.xquat[bid].tolist()}
    for mesh in additions:
        v=np.asarray(mesh['vertices']).reshape(-1,3)
        value['meshes'].append(dict(mesh_name=mesh['name'],body_name='payload_secondary',mode='rigid',visible_in_dynamic_view=True,
          evidence='SOURCE_CAD_SECTION_PLUS_EXPLICIT_LONGITUDINAL_SWEEP',source_refs=mesh['source_refs'],
          reference_aabb_min_m=v.min(axis=0).tolist(),reference_aabb_max_m=v.max(axis=0).tolist(),
          note='Exact supplied triangulated solid; rigidly bound to actual solver body. Mass included once in residual payload allocation.'))
    value['source_geometry_sha256']=digest;value['cad_body_overlay']=copy.deepcopy(overlay);value['variant']='cad_body_mounted'
    return value
