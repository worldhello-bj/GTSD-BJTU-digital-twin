"""Independent source-mapped finite-mount rig. Contact mode is gated separately."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as X
import numpy as np
from mass_properties import integrate, combine, canonical_sha

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'gtsd-finite-mount-rig-v1'

DEFAULTS = {
    'mode': 'mount_only', 'historical_profile': 'combined',
    'time_step_s': 0.0002, 'gravity_m_s2': [0., 0., -9.81],
    'rig_mass_kg': 100., 'rig_inertia_kg_m2': [25., 900., 910.],
    'source_density_kg_m3': 7850., 'balise_total_mass_kg': 0.6,
    'rig_stiffness': [200000., 200000., 10000.],
    'mount_stiffness_N_m': 20000., 'clamp_stiffness_N_m': 10000.,
    'clamp_rotational_stiffness_Nm_rad': 1., 'damping_ratio': 0.3,
    'rig_fixed': False,
}


def fmt(values):
    if np.isscalar(values): return format(float(values), '.17g')
    return ' '.join(format(float(v), '.17g') for v in values)


def source_meshes(root=ROOT):
    path = root / 'mechanical_fit/profiles.json'
    data = json.loads(path.read_text())
    profile = next(p for p in data['profiles'] if p['id'] == 'combined')
    rows = [dict(m, branch=g['id']) for g in profile['groups'] for m in g['add_meshes']]
    if len(rows) != 33 or len({m['name'] for m in rows}) != 33:
        raise ValueError('Historical combined profile membership changed')
    return rows, hashlib.sha256(path.read_bytes()).hexdigest()


def grouping(mesh):
    if mesh.get('solver_body_name'): return mesh['solver_body_name']
    n = mesh['name']
    if n.startswith('FIT_TRACK_station'): return 'support_' + n.split('_')[2][7:9]
    if n.startswith('FIT_TRACK_clip_'): return 'support_' + n.split('_')[3]
    if n.startswith('FIT_BALISE_SOURCE_branch_'): return 'support_' + n.split('_')[5]
    if n.startswith('FIT_BALISE_'): return 'balise_' + n.split('_')[2]
    if n.startswith('FIT_') and mesh['branch'] == 'clamps': return n
    raise ValueError('Unassigned source mesh ' + n)


def validate_parameters(parameters):
    cfg = dict(DEFAULTS)
    unknown = set(parameters) - set(cfg)
    if unknown: raise ValueError('Unknown parameters: ' + str(sorted(unknown)))
    cfg.update(parameters)
    if cfg['mode'] != 'mount_only' or cfg['historical_profile'] != 'combined':
        raise ValueError('No contact/photo-profile admission exists in mount-only builder')
    if type(cfg['rig_fixed']) is not bool: raise ValueError('rig_fixed must be boolean')
    for k, v in cfg.items():
        if isinstance(v, bool) or isinstance(v, str): continue
        if not np.isfinite(np.asarray(v, float)).all(): raise ValueError('Non-finite parameter ' + k)
        if k != 'gravity_m_s2' and np.any(np.asarray(v) <= 0): raise ValueError('Nonpositive parameter ' + k)
    if not 1e-6 <= cfg['time_step_s'] <= 0.001: raise ValueError('Time-step outside admitted construction range')
    if len(cfg['gravity_m_s2']) != 3 or len(cfg['rig_stiffness']) != 3 or len(cfg['rig_inertia_kg_m2']) != 3:
        raise ValueError('Wrong vector length')
    return cfg


def build(parameters=None, root=ROOT, mesh_overrides=None):
    cfg = validate_parameters(parameters or {})
    meshes, profile_sha = source_meshes(root)
    if mesh_overrides:
        if set(mesh_overrides)-{m['name'] for m in meshes}:raise ValueError('Unknown source override')
        meshes=[dict(mesh_overrides[m['name']],branch=m['branch'],solver_body_name=m['name']) if m['name'] in mesh_overrides else m for m in meshes]
    groups = {}
    for m in meshes: groups.setdefault(grouping(m), []).append(m)
    inertias = {}
    for name, members in groups.items():
        props = [integrate(m, density_kg_m3=cfg['source_density_kg_m3']) for m in members]
        if name.startswith('balise_'):
            scale = cfg['balise_total_mass_kg'] / sum(p['mass_kg'] for p in props)
            props = [integrate(m, mass_kg=p['mass_kg'] * scale) for m, p in zip(members, props)]
        inertias[name] = combine(props, hypothesis_id='DECLARED_RIGID_MESH_GROUP_' + name)
    tree = X.Element('mujoco', model=SCHEMA)
    X.SubElement(tree, 'compiler', angle='radian', inertiafromgeom='false')
    opt = X.SubElement(tree, 'option', timestep=fmt(cfg['time_step_s']), gravity=fmt(cfg['gravity_m_s2']),
                       integrator='implicitfast', solver='Newton', tolerance='1e-12', iterations='100')
    X.SubElement(opt, 'flag', energy='enable')
    world = X.SubElement(tree, 'worldbody')
    rig = X.SubElement(world, 'body', name='rig', pos='0 0 0')
    X.SubElement(rig, 'inertial', pos='0 0 0', mass=fmt(cfg['rig_mass_kg']), diaginertia=fmt(cfg['rig_inertia_kg_m2']))
    joints = []
    if not cfg['rig_fixed']:
        for axis, kind, stiffness, inertia in zip(([0,1,0],[0,0,1],[1,0,0]), ('slide','slide','hinge'), cfg['rig_stiffness'], (cfg['rig_mass_kg'],cfg['rig_mass_kg'],cfg['rig_inertia_kg_m2'][0])):
            name = 'rig_' + ('y' if axis == [0,1,0] else 'z' if axis == [0,0,1] else 'roll')
            damping = 2 * cfg['damping_ratio'] * np.sqrt(stiffness * inertia)
            X.SubElement(rig, 'joint', name=name, type=kind, axis=fmt(axis), stiffness=fmt(stiffness), damping=fmt(damping))
            joints.append({'name':name, 'kind':kind, 'stiffness':stiffness,'damping':float(damping)})
    bindings = []
    bodies = [{'name':'rig','branch':'rig','reference_position_m':[0.,0.,0.], 'mass_kg':cfg['rig_mass_kg']}]
    for name, members in groups.items():
        p = inertias[name]; center = p['center_of_mass_world_m']; tensor = np.array(p['inertia_com_world_kg_m2'])
        body = X.SubElement(rig, 'body', name=name, pos=fmt(center))
        full = [tensor[0,0],tensor[1,1],tensor[2,2],tensor[0,1],tensor[0,2],tensor[1,2]]
        X.SubElement(body, 'inertial', pos='0 0 0', mass=fmt(p['mass_kg']), fullinertia=fmt(full))
        axes = ([0,0,1],) if name.startswith('support_') else ([1,0,0],[0,1,0],[0,0,1])
        k = cfg['clamp_stiffness_N_m'] if members[0]['branch'] == 'clamps' else cfg['mount_stiffness_N_m']
        for i, axis in enumerate(axes):
            jname = name + '_translation_' + str(i)
            damping = 2 * cfg['damping_ratio'] * np.sqrt(k * p['mass_kg'])
            X.SubElement(body, 'joint', name=jname, type='slide', axis=fmt(axis), stiffness=fmt(k), damping=fmt(damping))
            joints.append({'name':jname,'kind':'slide','stiffness':k,'damping':float(damping)})
        if members[0]['branch'] == 'clamps':
            krot=cfg['clamp_rotational_stiffness_Nm_rad']
            # One MJCF ball damping value covers all three tangent axes. Using
            # minimum inertia under-damps transverse bending of a slender rod.
            # Use the maximum principal value; this is an explicit conservative
            # isotropic research damper, not a measured bearing characteristic.
            principal=np.linalg.eigvalsh(tensor)
            damping=2 * cfg['damping_ratio'] * np.sqrt(krot * principal.max())
            X.SubElement(body,'joint',name=name+'_rotation',type='ball',stiffness=fmt(krot),damping=fmt(damping))
            joints.append({'name':name+'_rotation','kind':'ball','stiffness':krot,'damping':float(damping),
                           'damping_choice':'maximum_principal_inertia_isotropic',
                           'principal_axis_damping_ratios':(damping/(2*np.sqrt(krot*principal))).tolist()})
        branch = members[0]['branch']
        bodies.append({'name':name,'branch':branch,'reference_position_m':center,'mass_kg':p['mass_kg']})
        for m in members:
            bindings.append({'mesh_name':m['name'],'body_name':name,'branch':branch,
                             'source_geometry_sha256':canonical_sha({'vertices':m['vertices'],'indices':m['indices']}),
                             'body_reference_position_m':center,'body_reference_quaternion_wxyz':[1.,0.,0.,0.],
                             'rigid_group_hypothesis':p['hypothesis_id']})
    xml = X.tostring(tree,encoding='unicode')
    code_hashes={f:hashlib.sha256((Path(__file__).parent/f).read_bytes()).hexdigest() for f in ('build_rig.py','mass_properties.py')}
    contract = {'schema':SCHEMA,'profile_sha256':profile_sha,'parameters':cfg,'bindings':bindings,
                'bodies':bodies,'inertial_properties':inertias,'joint_parameters':joints,
                'source_code_sha256':code_hashes,'xml_sha256':hashlib.sha256(xml.encode()).hexdigest(),
                'scope':'FINITE_MOUNT_ONLY_NO_CONTACT_ADMISSION',
                'historical_engineering_envelope':True,'as_built':False,
                'collision_enabled':False,'contact_acceptance':False,
                'unknown_interfaces':['4079 connection method','pin retention','balise fixing','mount stiffness/damping','physical mass/inertia'],
                'force_owners':{'body_mounts':'native_joint_spring_damping','rig_world':'native_joint_spring_damping'},
                'geometry_phase_changed':False,'old_checkpoint_allowed':False}
    contract['identity_sha256'] = canonical_sha(contract)
    return xml, contract


if __name__ == '__main__':
    a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);ns=a.parse_args()
    xml,c=build();ns.output.mkdir(parents=True,exist_ok=True)
    (ns.output/'model.xml').write_text(xml)
    (ns.output/'contract.json').write_text(json.dumps(c,indent=2)+'\n')
    print(json.dumps({'identity_sha256':c['identity_sha256'],'mesh_bindings':len(c['bindings']),'body_count':len(c['bodies']),'contact_acceptance':False}))
