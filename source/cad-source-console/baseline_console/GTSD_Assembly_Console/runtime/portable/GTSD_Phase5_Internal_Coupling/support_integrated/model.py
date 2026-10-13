"""Moving vehicle cable supports from current-source geometry; no new inertia.

Source surfaces locate conservative solid AABBs, not an identified trough
interior. Mounts, Coulomb friction and contact compliance are engineering
assumptions. The independent bench's guide/body/mass are intentionally absent.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from cables_support.model import source_geometry

ROOT = Path(__file__).resolve().parent

@dataclass(frozen=True)
class Parameters:
    contact_time_constant_s: float = .001
    friction: float = .3
    collision_bit: int = 128
    def __post_init__(self):
        for k in ('contact_time_constant_s', 'friction'):
            v = getattr(self, k)
            if type(v) not in (float, int) or not math.isfinite(v) or v <= 0:
                raise ValueError('finite positive parameter required: ' + k)
        if self.collision_bit != 128:
            raise ValueError('reserved cable-support collision bit must be 128')

def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

def vec(value):
    return ' '.join(format(float(x), '.16g') for x in value)

def augment_xml(xml, cable_spec, parameters=None, *, binding_body='payload_secondary', route_keys=('bay_wire_0',)):
    """Return augmented XML and spec; bind proxies to an existing moving body.

    Call after cables.model.augment_xml. This removes that library's numerical
    world ground, changes selected cable collision affinity, and appends three
    zero-mass support geoms. Existing bodies, DOFs, inertias and cables survive.
    """
    p = Parameters(**parameters) if isinstance(parameters, dict) else parameters or Parameters()
    root = ET.fromstring(xml)
    if any(e.get('name', '').startswith('v5s_') for e in root.iter()):
        raise ValueError('vehicle cable supports already augmented')
    if type(binding_body) is not str or binding_body == 'world':
        raise ValueError('support requires an existing dynamic vehicle body')
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, binding_body)
    if bid <= 0 or model.body_dofnum[bid] == 0:
        raise ValueError('support binding body must carry actual vehicle DOFs')
    if p.contact_time_constant_s < 2 * model.opt.timestep:
        raise ValueError('support contact time constant must be at least twice dt')
    sha, source, _ = source_geometry()
    if cable_spec['source_geometry_sha256'] != sha:
        raise ValueError('support and cable source geometry disagree')
    routes = {r['key']: r for r in cable_spec['routes']}
    if not route_keys or len(set(route_keys)) != len(route_keys) or any(k not in routes for k in route_keys):
        raise ValueError('unique cable routes already present in XML are required')
    support_body = root.find(f'.//body[@name="{binding_body}"]')
    rotation = data.xmat[bid].reshape(3, 3)
    quat = data.xquat[bid].copy(); quat[1:] *= -1
    records = []
    for row in source:
        center = (np.asarray(row['aabb_min_world_m']) + row['aabb_max_world_m']) / 2
        local = rotation.T @ (center - data.xpos[bid])
        name = 'v5s_' + row['source_mesh'].replace('.', '_')
        ET.SubElement(support_body, 'geom', dict(name=name, type='box', pos=vec(local), quat=vec(quat),
            size=vec(row['half_size_m']), mass='0', contype=str(p.collision_bit), conaffinity='0',
            condim='3', priority='1', friction=f'{p.friction} .001 .0001',
            solref=f'{p.contact_time_constant_s} 1', rgba='.2 .6 .8 .25'))
        records.append(dict(row, proxy_geom=name, binding_body=binding_body, center_local_m=local.tolist(),
            orientation_local_wxyz=quat.tolist(), incremental_mass_kg=0.,
            mounting_evidence='EXPLICIT_UNCALIBRATED_RIGID_BODY_MOUNTING_ASSUMPTION'))
    cable_geoms = []
    for key in route_keys:
        for segment in routes[key]['segments']:
            name = segment['geom_name']; geom = root.find(f'.//geom[@name="{name}"]')
            if geom is None: raise ValueError('missing cable geom ' + name)
            affinity = int(geom.get('conaffinity', '1'))
            if affinity & p.collision_bit: raise ValueError('reserved support collision bit already used')
            geom.set('conaffinity', str(affinity | p.collision_bit)); cable_geoms.append(name)
    removed = []
    for parent in root.iter():
        for geom in list(parent):
            if geom.tag == 'geom' and geom.get('name') == 'v5c_ground_proxy':
                parent.remove(geom); removed.append('v5c_ground_proxy')
    spec = dict(schema='gtsd-v5-integrated-cable-support-model-v1', parameters=asdict(p),
        source_geometry_sha256=sha, cable_spec_sha256=digest(cable_spec), supports=records,
        binding_body=binding_body, route_keys=list(route_keys), cable_geoms=cable_geoms,
        removed_world_geoms=removed, incremental_mass_kg=0., incremental_dof=0,
        reaction_owner='NATIVE_MUJOCO_CONTACT_CONSTRAINT_ONLY_NO_FORCE_REAPPLICATION',
        mass_boundary='Support mass/inertia is already included in the unchanged vehicle payload equivalent; no independent 2 kg support or guide is inserted. Explicit cable mass remains solely owned by cables.',
        limits=['UNCALIBRATED_ENGINEERING_EQUIVALENT',
            'Solid source AABBs are conservative collision proxies; actual trough interior, clearance and lip geometry remain unidentified.',
            'Source placement is preserved; attachment to payload_secondary follows an explicitly assumed rigid body mount, not verified hardware fixation.',
            'Friction and soft contact regularization are numerical assumptions, not measured material parameters.',
            'Only selected bay cables contact these supports. The inherited cable world-floor proxy is removed; other unmodelled vehicle supports remain absent.',
            'Contact force is physically coupled to the simulated moving vehicle, but does not validate real hardware contact or mounting.',
            'Signed contact virtual work is not entirely heat; no full-vehicle energy or momentum closure is asserted.'])
    ET.indent(root)
    return ET.tostring(root, encoding='unicode'), spec
