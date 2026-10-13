"""Native auxiliary inertia and moving mounts, without a second state owner.

The source meshes supply candidate AABB locations only. Mass, inertia, internal
geometry, axes, mounting and the Scotch-yoke equivalent are uncalibrated research
assumptions. Four relative coordinates are exclusively integrated by MuJoCo.
All effort laws (motor, bearing, gas, yoke and world-air load) belong to runtime.
This module neither adds armature nor changes preload, solver or existing DOFs.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
from .bootstrap import BASELINE as PHASE5
GEOMETRY_SHA256 = '7d925267cbfb18d39e64041ea851fc13db2e3a13f6d403eab386235f741d8f43'
PROVENANCE = 'UNCALIBRATED_RESEARCH_EQUIVALENT_SOURCE_AABB_PLACEMENT_ONLY'
COORDINATES = ('compressor_rotor', 'compressor_piston',
               'compressor_fan_rotor', 'bay_fan_rotor')


@dataclass(frozen=True)
class Parameters:
    crank_radius_m: float = .006
    compressor_stator_mass_kg: float = .40
    compressor_rotor_mass_kg: float = .10
    compressor_piston_mass_kg: float = .04
    compressor_fan_stator_mass_kg: float = .04
    compressor_fan_rotor_mass_kg: float = .02
    cabinet_mass_kg: float = .40
    bay_fan_stator_mass_kg: float = .04
    bay_fan_rotor_mass_kg: float = .02
    compressor_rotor_spin_inertia_kgm2: float = 2e-5
    fan_rotor_spin_inertia_kgm2: float = 2e-6
    compressor_rotor_length_m: float = .03
    fan_rotor_length_m: float = .003
    piston_radius_m: float = .008
    piston_length_m: float = .008
    compressor_axis_reference_xyz: tuple = (1.,0.,0.)
    piston_axis_reference_xyz: tuple = (0.,0.,1.)
    compressor_fan_axis_reference_xyz: tuple = (1.,0.,0.)
    bay_fan_axis_reference_xyz: tuple = (1.,0.,0.)
    compressor_mount_offset_m: tuple = (0.,0.,0.)
    compressor_fan_mount_offset_m: tuple = (0.,0.,0.)
    bay_fan_mount_offset_m: tuple = (0.,0.,0.)
    cabinet_mount_offset_m: tuple = (0.,0.,0.)

    def __post_init__(self):
        for key, value in asdict(self).items():
            if key.endswith(('_reference_xyz','_offset_m')):
                if type(value) not in (tuple,list) or len(value)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in value):raise ValueError('Finite 3-vector required: '+key)
                if key.endswith('_reference_xyz') and abs(sum(x*x for x in value)-1.)>1e-12:raise ValueError('Unit axis required: '+key)
                object.__setattr__(self,key,tuple(float(x) for x in value))
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('Finite positive research parameter required: ' + key)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def vec(value):
    return ' '.join(format(float(x), '.17g') for x in value)


def parallel_axis(mass, offset):
    r = np.asarray(offset, dtype=float)
    return mass * ((r @ r) * np.eye(3) - np.outer(r, r))


def quatmat(quat):
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, np.asarray(quat, dtype=float))
    return result.reshape(3, 3)


def _physical_tensor(value, name):
    tensor = np.asarray(value, dtype=float)
    eig = np.linalg.eigvalsh(tensor)
    if (tensor.shape != (3, 3) or not np.isfinite(tensor).all()
            or np.max(abs(tensor - tensor.T)) > 1e-13 or eig[0] <= 0
            or eig[-1] > eig[0] + eig[1] + 1e-12):
        raise ValueError('Nonphysical full inertia tensor: ' + name)
    return eig


def _fullinertia(tensor):
    x = np.asarray(tensor)
    return vec([x[0, 0], x[1, 1], x[2, 2], x[0, 1], x[0, 2], x[1, 2]])


def _inertial_attributes(mass, center, tensor):
    """Serialize a full tensor without MJCF's approximate eigendecomposition.

    numpy.eigh gives an orthonormal principal basis. Use an explicit proper
    rotation and diagonal values so compilation cannot discard small products
    of inertia. The ledger retains the full tensor in the original body frame.
    """
    values, rotation = np.linalg.eigh(np.asarray(tensor, dtype=float))
    if np.linalg.det(rotation) < 0:
        rotation[:, 0] *= -1
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(rotation).reshape(-1))
    return dict(pos=vec(center), mass=str(mass), quat=vec(quat), diaginertia=vec(values))


def _box_inertia(mass, dims):
    x, y, z = np.asarray(dims)
    return np.diag(mass / 12 * np.array([y*y+z*z, x*x+z*z, x*x+y*y]))


def _rotor_inertia(mass, spin, length, axis=(1.,0.,0.)):
    # Explicit homogeneous X-cylinder: Ixx=.5*m*r^2, not an armature.
    radius = math.sqrt(2 * spin / mass)
    transverse = mass * (3 * radius**2 + length**2) / 12
    axis=np.asarray(axis,dtype=float)
    return transverse*np.eye(3)+(spin-transverse)*np.outer(axis,axis), radius


def source_geometry():
    """Read only the frozen source; never duplicate its large geometry resource."""
    path = PHASE5 / 'web/geometry.json'
    raw = path.read_bytes()
    observed = hashlib.sha256(raw).hexdigest()
    if observed != GEOMETRY_SHA256:
        raise ValueError('Source geometry hash changed; auxiliary placement needs review')
    geometry = json.loads(raw)
    wanted = ['BAY_compressor_motor', 'SHELL_fan_back_-1_-1'] + [
        'BAY_control_module_' + str(i) for i in range(9)]
    selected = {}
    for mesh in geometry['meshes']:
        name = mesh['name']
        if name not in wanted:
            continue
        if name in selected:
            raise ValueError('Duplicate source geometry name: ' + name)
        vertices = np.asarray(mesh['vertices'], dtype=float).reshape(-1, 3)
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        if not np.isfinite(vertices).all() or np.any(hi <= lo):
            raise ValueError('Invalid source AABB: ' + name)
        selected[name] = dict(source_mesh=name, aabb_min_world_m=lo.tolist(),
                              aabb_max_world_m=hi.tolist(),
                              center_world_m=((lo+hi)/2).tolist(),
                              dimensions_m=(hi-lo).tolist())
    if set(selected) != set(wanted):
        raise ValueError('Required source auxiliary mesh identities are missing')
    return selected


def spatial_mass_properties(model, data=None):
    """Full mass/first moment/tensor in world axes at the supplied configuration."""
    if data is None:
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
    mass = float(np.sum(model.body_mass[1:]))
    first = np.sum(model.body_mass[1:, None] * data.xipos[1:], axis=0)
    origin = np.zeros((3, 3))
    for i in range(1, model.nbody):
        r = data.ximat[i].reshape(3, 3)
        origin += (r @ np.diag(model.body_inertia[i]) @ r.T
                   + parallel_axis(model.body_mass[i], data.xipos[i]))
    com = first / mass
    return dict(mass_kg=mass, first_moment_world_kgm=first.tolist(),
                com_world_m=com.tolist(), inertia_world_origin_kgm2=origin.tolist(),
                inertia_world_com_kgm2=(origin-parallel_axis(mass, com)).tolist())


def _new_bodies(source, p):
    pump = source['BAY_compressor_motor']
    fan = source['SHELL_fan_back_-1_-1']
    modules = [source['BAY_control_module_' + str(i)] for i in range(9)]
    cabinet_center = np.mean([m['center_world_m'] for m in modules], axis=0)
    cabinet_inertia = sum((
        _box_inertia(p.cabinet_mass_kg/9, m['dimensions_m'])
        + parallel_axis(p.cabinet_mass_kg/9,
                        np.asarray(m['center_world_m'])-cabinet_center)
        for m in modules), np.zeros((3, 3)))
    pump_rotor, pump_radius = _rotor_inertia(p.compressor_rotor_mass_kg,
        p.compressor_rotor_spin_inertia_kgm2, p.compressor_rotor_length_m,p.compressor_axis_reference_xyz)
    cf_rotor, cf_radius = _rotor_inertia(p.compressor_fan_rotor_mass_kg,
        p.fan_rotor_spin_inertia_kgm2, p.fan_rotor_length_m,p.compressor_fan_axis_reference_xyz)
    bf_rotor, bf_radius = _rotor_inertia(p.bay_fan_rotor_mass_kg,
        p.fan_rotor_spin_inertia_kgm2, p.fan_rotor_length_m,p.bay_fan_axis_reference_xyz)
    # The piston is a Z-cylinder, and its COM is its joint origin at x=0.
    m, r, l = p.compressor_piston_mass_kg, p.piston_radius_m, p.piston_length_m
    transverse=m*(3*r*r+l*l)/12;axis=np.asarray(p.piston_axis_reference_xyz)
    piston=transverse*np.eye(3)+(.5*m*r*r-transverse)*np.outer(axis,axis)
    rows = []

    def row(name, parent, budget, mass, tensor, center, source_names, **extra):
        eig = _physical_tensor(tensor, name)
        rows.append(dict(body=name, parent_body=parent, budget_body=budget,
            mass_kg=mass, com_source_world_m=list(center),
            inertia_com_source_world_kgm2=tensor.tolist(),
            principal_inertias_kgm2=eig.tolist(), source_meshes=source_names,
            provenance=PROVENANCE, **extra))
    pc=np.asarray(pump['center_world_m'])+p.compressor_mount_offset_m
    fc=np.asarray(fan['center_world_m'])+p.bay_fan_mount_offset_m
    cfc=pc+np.asarray(p.compressor_fan_mount_offset_m)
    cabinet_center=cabinet_center+np.asarray(p.cabinet_mount_offset_m)
    row('compressor_stator', 'payload_secondary', 'payload_secondary',
        p.compressor_stator_mass_kg, _box_inertia(p.compressor_stator_mass_kg,
        pump['dimensions_m']), pc, [pump['source_mesh']])
    row('compressor_rotor', 'compressor_stator', 'payload_secondary',
        p.compressor_rotor_mass_kg, pump_rotor, pc, [pump['source_mesh']],
        equivalent_cylinder_radius_m=pump_radius, joint_type='hinge', axis=p.compressor_axis_reference_xyz)
    row('compressor_piston', 'compressor_stator', 'payload_secondary',
        m, piston, pc, [pump['source_mesh']], joint_type='slide', axis=p.piston_axis_reference_xyz)
    row('compressor_fan_stator', 'compressor_stator', 'payload_secondary',
        p.compressor_fan_stator_mass_kg,
        _box_inertia(p.compressor_fan_stator_mass_kg, [.006, .03, .03]),
        cfc, [pump['source_mesh']])
    row('compressor_fan_rotor', 'compressor_fan_stator', 'payload_secondary',
        p.compressor_fan_rotor_mass_kg, cf_rotor, cfc, [pump['source_mesh']],
        equivalent_cylinder_radius_m=cf_radius, joint_type='hinge', axis=p.compressor_fan_axis_reference_xyz)
    row('aux_cabinet', 'payload_secondary', 'payload_secondary',
        p.cabinet_mass_kg, cabinet_inertia, cabinet_center,
        [m['source_mesh'] for m in modules])
    row('bay_fan_stator', 'door_end_neg_y_neg', 'door_end_neg_y_neg',
        p.bay_fan_stator_mass_kg,
        _box_inertia(p.bay_fan_stator_mass_kg, fan['dimensions_m']),
        fc, [fan['source_mesh']])
    row('bay_fan_rotor', 'bay_fan_stator', 'door_end_neg_y_neg',
        p.bay_fan_rotor_mass_kg, bf_rotor, fc, [fan['source_mesh']],
        equivalent_cylinder_radius_m=bf_radius, joint_type='hinge', axis=p.bay_fan_axis_reference_xyz)
    return rows


def _allocate_budget(base, data, budget, rows, expected):
    bid = mujoco.mj_name2id(base, mujoco.mjtObj.mjOBJ_BODY, budget)
    if bid <= 0:
        raise ValueError('Required mass-budget body is absent: ' + budget)
    mass = float(base.body_mass[bid])
    if abs(mass-expected) > 1e-10:
        raise ValueError('Unexpected original mass budget; refuse repeat allocation: ' + budget)
    rotation = data.xmat[bid].reshape(3, 3)
    center = base.body_ipos[bid].copy()
    ir = quatmat(base.body_iquat[bid])
    inertia = ir @ np.diag(base.body_inertia[bid]) @ ir.T
    first_new, origin_new = np.zeros(3), np.zeros((3, 3))
    allocated = 0.
    for row in rows:
        if row['budget_body'] != budget:
            continue
        c = rotation.T @ (np.asarray(row['com_source_world_m'])-data.xpos[bid])
        ic = rotation.T @ np.asarray(row['inertia_com_source_world_kgm2']) @ rotation
        io = ic + parallel_axis(row['mass_kg'], c)
        row.update(com_budget_local_m=c.tolist(), inertia_com_budget_local_kgm2=ic.tolist(),
                   inertia_budget_origin_kgm2=io.tolist())
        first_new += row['mass_kg']*c
        origin_new += io
        allocated += row['mass_kg']
    remaining = mass-allocated
    if remaining <= 0:
        raise ValueError('Auxiliary allocation exhausts original mass budget: ' + budget)
    residual_center = (mass*center-first_new)/remaining
    original_origin = inertia+parallel_axis(mass, center)
    residual = original_origin-origin_new-parallel_axis(remaining, residual_center)
    residual = (residual+residual.T)/2
    eig = _physical_tensor(residual, budget+' residual')
    return dict(body=budget, original_mass_kg=mass, allocated_mass_kg=allocated,
        residual_mass_kg=remaining, original_com_local_m=center.tolist(),
        original_inertia_com_local_kgm2=inertia.tolist(),
        residual_com_local_m=residual_center.tolist(),
        residual_inertia_com_local_kgm2=residual.tolist(),
        residual_principal_inertias_kgm2=eig.tolist(),
        first_moment_error_kgm=float(np.max(abs(remaining*residual_center+first_new-mass*center))),
        inertia_origin_error_kgm2=float(np.max(abs(residual+parallel_axis(remaining,
            residual_center)+origin_new-original_origin))))


def augment_xml(xml, parameters=None):
    """Return independent MJCF plus JSON-safe spec, compiling before acceptance.

    Input should be the frozen Phase5 combined model (75 DOFs). Smaller fixtures
    with the same named 27.6 kg payload and .6 kg door budgets are also supported.
    The exact initial total mass, first moment and full tensor are preserved.
    """
    if type(xml) is not str:
        raise TypeError('XML must be a string')
    p = Parameters(**parameters) if type(parameters) is dict else parameters
    if p is None:
        p = Parameters()
    if not isinstance(p, Parameters):
        raise TypeError('parameters must be Parameters, a dict or None')
    root = ET.fromstring(xml)
    reserved = set(COORDINATES) | {'compressor_stator', 'compressor_fan_stator',
        'bay_fan_stator', 'aux_cabinet', 'gtsd_phase6_auxiliary_mounts'}
    if any(node.get('name') in reserved for node in root.iter()):
        raise ValueError('Auxiliary model already augmented or reserved names occupied')
    if root.find('keyframe') is not None:
        raise ValueError('Keyframe qpos remapping must precede auxiliary augmentation')
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler', angle='radian')
    if compiler.get('angle', 'degree') != 'radian':
        raise ValueError('Auxiliary augmentation requires radian MJCF')
    if compiler.get('fusestatic', 'false') != 'false':
        raise ValueError('Mount sensor bodies require fusestatic=false')
    if float(compiler.get('settotalmass', '-1')) > 0:
        raise ValueError('Automatic mass rescaling is incompatible with exact mass allocation')
    base = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(base)
    mujoco.mj_forward(base, data)
    source = source_geometry()
    rows = _new_bodies(source, p)
    allocations = {}
    for budget, expected in [('payload_secondary', 27.6), ('door_end_neg_y_neg', .6)]:
        node = root.find(f'.//body[@name="{budget}"]')
        if node is None or node.find('inertial') is None:
            raise ValueError('Explicit parent inertial is required: ' + budget)
        ledger = _allocate_budget(base, data, budget, rows, expected)
        allocations[budget] = ledger
        inertial = node.find('inertial')
        inertial.attrib.clear()
        inertial.attrib.update(_inertial_attributes(ledger['residual_mass_kg'],
            ledger['residual_com_local_m'], ledger['residual_inertia_com_local_kgm2']))
    added = {}
    for row in rows:
        parent_name = row['parent_body']
        parent = root.find(f'.//body[@name="{parent_name}"]')
        if parent is None:
            raise ValueError('Missing auxiliary body parent: ' + parent_name)
        if parent_name in added:
            # All new body frames are source-world aligned at the q=0 reference.
            pos = np.asarray(row['com_source_world_m'])-added[parent_name]
            quat = [1., 0., 0., 0.]
        else:
            bid = mujoco.mj_name2id(base, mujoco.mjtObj.mjOBJ_BODY, parent_name)
            pos = data.xmat[bid].reshape(3, 3).T @ (np.asarray(row['com_source_world_m'])-data.xpos[bid])
            quat = data.xquat[bid] * [1, -1, -1, -1]
        body = ET.SubElement(parent, 'body', name=row['body'], pos=vec(pos), quat=vec(quat))
        ET.SubElement(body, 'inertial', _inertial_attributes(row['mass_kg'],
                      [0, 0, 0], row['inertia_com_source_world_kgm2']))
        if 'joint_type' in row:
            ET.SubElement(body, 'joint', name=row['body'], type=row['joint_type'],
                pos='0 0 0', axis=vec(row['axis']), limited='false', damping='0',
                armature='0', frictionloss='0', stiffness='0', springdamper='0 0',
                ref='0', springref='0')
        row.update(position_parent_m=pos.tolist(), quaternion_parent_wxyz=list(quat))
        added[row['body']] = np.asarray(row['com_source_world_m'])
    sensor = root.find('sensor')
    if sensor is None:
        sensor = ET.SubElement(root, 'sensor')
    mounts = {}
    for key, body_name in [('compressor', 'compressor_stator'),
                           ('compressor_fan', 'compressor_fan_stator'),
                           ('bay_fan', 'bay_fan_stator')]:
        site_name = key+'_mount_site'
        body = root.find(f'.//body[@name="{body_name}"]')
        ET.SubElement(body, 'site', name=site_name, pos='0 0 0', quat='1 0 0 0',
                      size='.003', rgba='.9 .3 .1 1')
        for kind in ('force', 'torque'):
            ET.SubElement(sensor, kind, name=site_name+'_'+kind, site=site_name,
                          cutoff='0', noise='0')
        mounts[key] = dict(body=body_name, site=site_name,
            force_sensor=site_name+'_force', torque_sensor=site_name+'_torque',
            frame='native_site_local_xyz', reference_point='mount_site_origin',
            sensor_convention='MuJoCo native child-parent interaction, site on child; positive force points child toward parent',
            sampling='Capture accepted native solver sensor sample before any subsequent mj_forward',
            work_boundary='Internal subsystem transfer only; do not add to whole-vehicle external work')
    custom = root.find('custom')
    if custom is None:
        custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'text', name='gtsd_phase6_auxiliary_mounts',
                  data='four_native_coordinates_mass_reallocated_once_uncalibrated')
    ET.indent(root)
    augmented = ET.tostring(root, encoding='unicode')
    model = mujoco.MjModel.from_xml_string(augmented)
    new_data = mujoco.MjData(model)
    mujoco.mj_forward(model, new_data)
    before, after = spatial_mass_properties(base, data), spatial_mass_properties(model, new_data)
    errors = {key: float(np.max(abs(np.asarray(after[key])-before[key]))) for key in before}
    tolerances = dict(mass_kg=1e-10, first_moment_world_kgm=1e-10,
                      com_world_m=1e-12, inertia_world_origin_kgm2=1e-10,
                      inertia_world_com_kgm2=1e-10)
    if any(errors[key] > tolerances[key] for key in errors):
        raise ValueError('Compiled reference spatial inertia failed preservation: '+str(errors))
    if model.nv != base.nv+4 or model.nq != base.nq+4:
        raise ValueError('Native auxiliary augmentation must add exactly four scalar coordinates')
    coordinates = {}
    for name in COORDINATES:
        jid = model.joint(name).id
        bid = model.body(name).id
        dof, qadr = int(model.jnt_dofadr[jid]), int(model.jnt_qposadr[jid])
        if (model.dof_damping[dof] != 0 or model.dof_armature[dof] != 0
                or model.dof_frictionloss[dof] != 0 or model.jnt_stiffness[jid] != 0):
            raise ValueError('New relative DOF has an unexpected native passive effort: '+name)
        coordinates[name] = dict(joint=name, body=name, joint_id=jid, body_id=bid,
            dof_address=dof, qpos_address=qadr, qvel_address=dof,
            type='slide' if name=='compressor_piston' else 'hinge',
            axis_local=model.jnt_axis[jid].tolist(),
            stator_body=model.body(int(model.body_parentid[bid])).name,
            state_owner='MUJOCO_QPOS_QVEL_ONLY', damping=0., armature=0.,
            effort_owner='AUXILIARY_RUNTIME_RELATIVE_EFFORT_AND_WORLD_AIR_JACOBIAN')
    for mount in mounts.values():
        bid, sid = model.body(mount['body']).id, model.site(mount['site']).id
        parent_id = int(model.body_parentid[bid])
        mount.update(body_id=bid, parent_body_id=parent_id,
                     parent_body=model.body(parent_id).name, site_id=sid,
                     reference_position_world_m=new_data.site_xpos[sid].tolist())
        for kind in ('force', 'torque'):
            sensor_id = model.sensor(mount[kind+'_sensor']).id
            mount[kind+'_sensor_id'] = sensor_id
            mount[kind+'_sensor_address'] = int(model.sensor_adr[sensor_id])
    for row in rows:
        row['body_id'] = model.body(row['body']).id
    spec = dict(schema='gtsd-phase6-native-auxiliary-model-v1', provenance=PROVENANCE,
        parameters=asdict(p), source_geometry_sha256=GEOMETRY_SHA256,
        source_geometry_path='web/geometry.json (within verified Phase5 dependency)', source_geometry=source,
        input_xml_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        augmented_xml_sha256=hashlib.sha256(augmented.encode()).hexdigest(),
        source_model_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        coordinates=coordinates, mounts=mounts,
        nq_before=base.nq, nq_after=model.nq, nv_before=base.nv, nv_after=model.nv,
        incremental_dof=4, incremental_mass_kg=0.,
        mass_ledger=dict(reference='All newly added relative q=0; existing model qpos0',
            allocations=allocations, added_bodies=rows, before=before, after=after,
            absolute_errors=errors, tolerances=tolerances,
            allocation_boundary='Auxiliary inertia is split from existing payload/selected-door core budget; no extra preload or cable/brake mass allocation.',
            motion_boundary='Piston translation and mounted door motion may physically change total COM and inertia after the reference configuration.'),
        reaction_owner='NATIVE_MUJOCO_BODY_TREE_NO_MANUAL_STATOR_COUNTERTORQUE',
        limits=['Masses, axes, inertias and internal placements are unmeasured assumptions.',
            'Only door_end_neg_y_neg owns bay_fan; other doors have no shared fan coordinate.',
            f'Cabinet thermal node is an uncalibrated calorimeter with a fixed {p.cabinet_mass_kg:g} kg research mass package.',
            'No legacy pump/fan integrator is enabled by this geometry-only module.',
            'No collision geometry or physical clearance claim is introduced.',
            'Native mount wrench is a numerical virtual sensor, not registered measured hardware.',
            'Valve-spool mount reactions and fluid translation/jet momentum remain outside this model.'])
    spec=json.loads(json.dumps(spec,allow_nan=False))  # stable list types across JSON checkpoint round trips
    spec['identity_sha256'] = digest(spec)
    return augmented, spec
