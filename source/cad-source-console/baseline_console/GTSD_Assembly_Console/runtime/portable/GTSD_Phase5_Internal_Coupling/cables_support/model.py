"""Independent cable/support study, sourced in current geometry world metres.

The solid AABB supports are deliberately conservative collision approximations,
not a reconstruction of the trough's unknown interior cross-section.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import hashlib, json, math
import numpy as np
from cables.model import CableParameters, augment_xml
ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'web/geometry.json'
NAMES = ('BODY_bay_floor', 'BAY_cable_trough', 'BAY_cable_trough.001')

@dataclass(frozen=True)
class Parameters:
    dt_s: float = .0004
    support_mass_kg: float = 2.
    support_spring_N_m: float = 20.
    support_damping_Ns_m: float = .2
    contact_time_constant_s: float = .001
    def __post_init__(self):
        for k,v in asdict(self).items():
            if type(v) not in (int,float) or not math.isfinite(v) or v <= 0: raise ValueError('finite positive parameter required: '+k)
        if not .00005 <= self.dt_s <= .001: raise ValueError('dt outside validated cost/stability envelope')
        if self.contact_time_constant_s < 2*self.dt_s: raise ValueError('contact time constant must be at least twice dt')


def source_geometry():
    raw = SOURCE.read_bytes(); source = json.loads(raw)
    meshes = {m['name']:m for m in source['meshes']}
    selected=[]; records=[]
    for name in NAMES:
        mesh=meshes[name]; points=np.asarray(mesh['vertices']).reshape(-1,3)
        lo,hi=points.min(axis=0),points.max(axis=0)
        records.append(dict(source_mesh=name,proxy_geom='cs_'+name.replace('.','_'),aabb_min_world_m=lo.tolist(),aabb_max_world_m=hi.tolist(),center_local_m=((hi+lo)/2).tolist(),half_size_m=((hi-lo)/2).tolist(),binding_body='support',collision='SOLID_CONSERVATIVE_AABB_NOT_ACTUAL_TROUGH_INTERIOR'))
        selected.append(dict(mesh,bench_body='support',group='body',display_group='independent_cable_support',registered_component_id='support'))
    return hashlib.sha256(raw).hexdigest(),records,selected


def model(parameters=None):
    p=Parameters(**parameters) if isinstance(parameters,dict) else parameters or Parameters()
    sha,records,meshes=source_geometry()
    geoms=''.join(f'<geom name="{r["proxy_geom"]}" type="box" pos="{" ".join(map(str,r["center_local_m"]))}" size="{" ".join(map(str,r["half_size_m"]))}" mass="0" contype="64" conaffinity="64" condim="3" friction=".3 .001 .0001" solref="{p.contact_time_constant_s} 1" rgba=".2 .6 .8 .25"/>' for r in records)
    xml=f'''<mujoco model="independent_cable_support"><compiler angle="radian"/><option timestep="{p.dt_s}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" jacobian="dense" iterations="100" tolerance="1e-12"><flag energy="enable"/></option><worldbody><body name="support"><joint name="support_y" type="slide" axis="0 1 0" stiffness="{p.support_spring_N_m}" damping="{p.support_damping_Ns_m}"/><inertial pos="0 0 0" mass="{p.support_mass_kg}" diaginertia=".2 .2 .2"/>{geoms}</body></worldbody></mujoco>'''
    xml,cables=augment_xml(xml,CableParameters(contact_time_constant_s=p.contact_time_constant_s),bindings={'bay_wire_0':('support','support')},route_keys=('bay_wire_0',),with_floor=False)
    if cables['source_geometry_sha256'] != sha: raise ValueError('geometry differs from cable route extraction')
    # Retain library provenance, but replace host-specific statements that do
    # not apply to this independent bench (there is no vehicle payload or ground).
    cables['mass_boundary']='Finite cable mass is separate from the declared guided support mass; no vehicle payload equivalent is present'
    cables['limits']=[x for x in cables['limits'] if not x.startswith('Numerical ground plane')] + ['No global ground plane; mapped support AABBs and permitted nonadjacent cable self-contacts only']
    spec=dict(schema='gtsd-v5-cable-support-model-v1',scope='INDEPENDENT_CABLE_SUPPORT_STUDY_NOT_VEHICLE_COUPLING',parameters=asdict(p),source_geometry_sha256=sha,cables=cables,supports=records,source_coordinates='IDENTITY_CURRENT_SOURCE_WORLD_METRES',mass_boundary='Equivalent support mass is a declared configurable study parameter; no measured real support mass or mass partition is claimed',binding_status='GEOMETRIC_CANDIDATE_ONLY_REAL_TERMINALS_AND_FIXITY_UNKNOWN',limits=['25 DOF expected: one y-only guided support plus seven finite-mass cable links; actual dimensions are in solver identity','Source trough meshes are display surfaces; solid AABBs do not identify a concave channel or its interior clearance','Both candidate cable ends share the support body; actual mechanical end terminals remain unknown','Support y rail, mass, spring, damping, cable EI/density/friction and contact softness are uncalibrated study assumptions','Rail exchanges x/z force and all torques with the environment; physical total momentum audit covers y only','Native signed constraint work and contact virtual work are not asserted to be heat dissipation','No axial stretch, fracture, fatigue, measured material law, wiring continuity or full vehicle coupling'])
    return xml,spec,meshes


def write_resources():
    xml,spec,meshes=model()
    visual=dict(schema='gtsd-new-geometry-view-v1',units='m',meshes=meshes,registry=[{'id':'support','name':'support'}],original_source_geometry_sha256=spec['source_geometry_sha256'],provenance='Source vertices retain actual world metre coordinates; support solver body has identity reference pose; wire is solver capsules only')
    raw=json.dumps(visual,separators=(',',':'),allow_nan=False).encode(); vh=hashlib.sha256(raw).hexdigest()
    binding=dict(schema='gtsd-physics-binding-v1',subsystem='cables_support',source_geometry_sha256=vh,reference_poses={'support':{'position':[0.,0.,0.],'quaternion':[1.,0.,0.,0.]}},meshes=[dict(mesh_name=m['name'],body_name='support',mode='rigid',evidence='NEW_SOURCE_WORLD_MESH_AND_INDEPENDENT_SUPPORT_BODY',note='AABB collision, source surface display, unknown real mechanical mounting') for m in meshes])
    (ROOT/'model.xml').write_text(xml); (ROOT/'parameters.json').write_text(json.dumps(spec,ensure_ascii=False,indent=2)); (ROOT/'visual_geometry.json').write_bytes(raw); (ROOT/'binding.json').write_text(json.dumps(binding,ensure_ascii=False,indent=2))
    return spec
if __name__ == '__main__': write_resources()
