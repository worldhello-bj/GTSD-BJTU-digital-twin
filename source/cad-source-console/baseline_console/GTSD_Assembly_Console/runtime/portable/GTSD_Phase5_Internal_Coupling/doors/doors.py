"""Four new-source end doors: torque-only MuJoCo joints and optional ideal catches.

New geometry only. Every mechanical number is an explicit uncalibrated assumption.
No generalized state writes, position servo, device I/O or historical mesh import.
Full replay state = this controller checkpoint plus mjSTATE_INTEGRATION.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent
GEOMETRY_SHA256 = '7d925267cbfb18d39e64041ea851fc13db2e3a13f6d403eab386235f741d8f43'
DOOR_KEYS = ('end_neg_y_neg', 'end_neg_y_pos', 'end_pos_y_neg', 'end_pos_y_pos')
DEFAULTS = {
    'leaf_mass_kg': .6,
    'stiffness_Nmprad': .2,
    'damping_Nmsprad': .20,
    'frictionloss_Nm': .002,
    'torque_limit_Nm': .8,
    'command_torque_Nm': .5,
    'opening_limit_rad': math.radians(100),
    'closing_preload_angle_rad': .04,
    'limit_time_constant_s': .002,
    'limit_damping_ratio': 1.,
    'latch_capture_angle_rad': .006,
    'latch_capture_velocity_radps': .03,
    'latch_enabled': True,
    'initially_latched': True,
    'original_payload_mass_kg': 30.,
}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def sha(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def vec(values):
    return ' '.join(f'{float(x):.17g}' for x in values)


def finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(name + ' must be a finite number, not a boolean/string')
    return float(value)


def parameters(params=None):
    if params is None:
        params = {}
    if type(params) is not dict or set(params) - set(DEFAULTS):
        raise ValueError('Unknown door parameters or non-dictionary parameters')
    p = DEFAULTS | params
    for k, v in p.items():
        if type(DEFAULTS[k]) is bool:
            if type(v) is not bool:
                raise ValueError(k + ' must be boolean')
        else:
            p[k] = finite(v, k)
            if p[k] < 0:
                raise ValueError(k + ' must be nonnegative')
    for k in ('leaf_mass_kg', 'torque_limit_Nm', 'opening_limit_rad', 'limit_time_constant_s',
              'limit_damping_ratio', 'original_payload_mass_kg'):
        if p[k] <= 0:
            raise ValueError(k + ' must be positive')
    if p['command_torque_Nm'] > p['torque_limit_Nm'] or p['opening_limit_rad'] > math.pi:
        raise ValueError('Command exceeds torque limit or travel exceeds pi radians')
    if p['latch_capture_angle_rad'] >= p['opening_limit_rad']:
        raise ValueError('Capture envelope must be narrower than opening travel')
    if not p['latch_enabled'] and p['initially_latched']:
        # The enabled flag alone must be sufficient to remove all ideal catches.
        p['initially_latched'] = False
    return p


def parallel_axis(mass, offset):
    r = np.asarray(offset, dtype=float)
    return mass * (float(r @ r) * np.eye(3) - np.outer(r, r))


def quatmat(q):
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, np.asarray(q, dtype=float))
    return r.reshape(3, 3)


def source_leaves(geometry=None):
    """Resolve exact mesh identities and hinge line from the new closed vertices.

    The fan/grille/spokes/badge are generated inside the leaf loop in
    component_body.py. Blender suffixes are resolved spatially and persisted by
    exact name; no suffix inference or prefix broadening occurs at runtime.
    Top-of-end stop/indicator/cap ownership is unresolved, so they remain fixed.
    """
    if geometry is None:
        raw = (SOURCE / 'web/geometry.json').read_bytes()
        if hashlib.sha256(raw).hexdigest() != GEOMETRY_SHA256:
            raise ValueError('New geometry changed; review door source mapping first')
        geometry = json.loads(raw)
    meshes = {m['name']: m for m in geometry['meshes']}
    if len(meshes) != len(geometry['meshes']):
        raise ValueError('Duplicate source mesh names')
    bounds = {n: np.asarray(m['vertices'], dtype=float).reshape(-1, 3) for n, m in meshes.items()}
    center = {n: (v.min(0) + v.max(0)) / 2 for n, v in bounds.items()}
    leaves = []
    for key, (end, side) in zip(DOOR_KEYS, ((-1, -1), (-1, 1), (1, -1), (1, 1))):
        leaf = f'SHELL_access_door_{end}_{side}'
        hinges = sorted(n for n in meshes if n.startswith(f'SHELL_hinge_{end}_{side}_'))
        if leaf not in meshes or len(hinges) != 2:
            raise ValueError('Expected one new leaf and two visible hinge meshes: ' + key)
        hinges.sort(key=lambda n: center[n][2])
        a, b = center[hinges[0]], center[hinges[1]]
        axis = b - a
        axis /= np.linalg.norm(axis)
        if np.max(abs(axis - [0., 0., 1.])) > 1e-6:
            raise ValueError('Visible hinge centers no longer establish source Z axis')
        pivot = (a + b) / 2
        names = [leaf, *hinges]
        for n in meshes:
            if n.startswith((f'SHELL_fan_back_{end}_{side}', f'SHELL_fan_grille_{end}_{side}_')):
                names.append(n)
            elif n.startswith(('SHELL_fan_spoke', 'SHELL_electrical_caution')):
                c = center[n]
                if (1 if c[0] > 0 else -1) == end and (1 if c[1] > 0 else -1) == side:
                    names.append(n)
        handle = f'SHELL_door_handle_{end}' if end == side else None
        if handle:
            if handle not in meshes:
                raise ValueError('Expected photographed handle mesh missing')
            names.append(handle)
        names = sorted(set(names))
        if len(names) != (13 if handle else 12):
            raise ValueError('Leaf membership changed; expected new-source leaf/hinge/fan/badge assembly')
        v = np.concatenate([bounds[n] for n in names])
        lo, hi = v.min(0), v.max(0)
        leaves.append({
            'key': key, 'source_leaf': leaf, 'source_parts': names,
            'visible_handle': handle, 'visible_hinges': hinges,
            'hinge_source_m': pivot.tolist(), 'axis_source': axis.tolist(),
            'hinge_center_separation_m': float(np.linalg.norm(b-a)),
            'leaf_bounds_source_m': [bounds[leaf].min(0).tolist(), bounds[leaf].max(0).tolist()],
            'assembly_bounds_source_m': [lo.tolist(), hi.tolist()],
            'com_source_m': ((lo+hi)/2).tolist(),
            # cross(z, COM-pivot).x has the sign of side; choose outward end.
            'opening_sign': end * side,
            'geometry_evidence': 'NEW_PHOTO_INFERRED_MESH_WITH_CAD_ENVELOPE_NOT_MEASURED_HINGE',
            'attachment_evidence': 'SOURCE_LOOP_AND_SPATIAL_MEMBERSHIP_CANDIDATE',
        })
    names = [n for d in leaves for n in d['source_parts']]
    if len(names) != len(set(names)):
        raise ValueError('Source mesh belongs to more than one door')
    return leaves


def mass_allocation(base_model, payload_id, leaves, p, reference):
    """Subtract all four closed leaves once, preserving original M, COM and I.

    Tensors and moments in payload reference axes. Door AABB uniform mass is an
    explicit equivalent, not mesh volume/material inference or a measured inertia.
    """
    M = float(base_model.body_mass[payload_id])
    if abs(M-p['original_payload_mass_kg']) > 1e-9:
        raise ValueError('Payload mass differs from original 30 kg budget; refuse double allocation')
    c0 = base_model.body_ipos[payload_id].copy()
    ri = quatmat(base_model.body_iquat[payload_id])
    I0 = ri @ np.diag(base_model.body_inertia[payload_id]) @ ri.T
    rp = quatmat(reference['quaternion'])
    xp = np.asarray(reference['position'])
    first = np.zeros(3)
    inertia = np.zeros((3, 3))
    rows = []
    for d in leaves:
        lo, hi = np.asarray(d['assembly_bounds_source_m'])
        dims = hi-lo
        mass = p['leaf_mass_kg']
        Ix = mass * np.array([dims[1]**2+dims[2]**2, dims[0]**2+dims[2]**2,
                              dims[0]**2+dims[1]**2]) / 12
        c = rp.T @ (np.asarray(d['com_source_m']) - xp)
        Ic = rp.T @ np.diag(Ix) @ rp
        Ir = Ic + parallel_axis(mass, c)
        first += mass*c
        inertia += Ir
        d.update(mass_kg=mass, inertia_com_source_diagonal_kgm2=Ix.tolist())
        rows.append({'key': d['key'], 'mass_kg': mass, 'com_payload_m': c.tolist(),
                     'inertia_com_payload_kgm2': Ic.tolist(), 'inertia_payload_origin_kgm2': Ir.tolist()})
    moving_mass = sum(r['mass_kg'] for r in rows)
    residual_mass = M-moving_mass
    if residual_mass <= 0:
        raise ValueError('Door masses exhaust original payload budget')
    residual_com = (M*c0-first)/residual_mass
    residual_I = I0+parallel_axis(M, c0)-inertia-parallel_axis(residual_mass, residual_com)
    eig = np.linalg.eigvalsh(residual_I)
    if not np.isfinite(eig).all() or min(eig) <= 0 or max(eig) > sum(eig)-max(eig)+1e-12:
        raise ValueError('Residual payload inertia is nonphysical; select explicit alternative mass/inertia parameters')
    reconstructed_first = residual_mass*residual_com+first
    reconstructed_I = residual_I+parallel_axis(residual_mass, residual_com)+inertia
    return {
        'basis': 'Preserve original closed 30 kg payload M/COM/I by parallel-axis subtraction; pantograph excluded unchanged.',
        'mass_assumption': '0.6 kg per complete leaf assembly by default; uniform assembly AABB inertia; uncalibrated.',
        'original_mass_kg': M, 'original_com_payload_m': c0.tolist(),
        'original_inertia_com_payload_kgm2': I0.tolist(),
        'moving_mass_kg': moving_mass, 'moving_leaves': rows,
        'residual_mass_kg': residual_mass, 'residual_com_payload_m': residual_com.tolist(),
        'residual_inertia_com_payload_kgm2': residual_I.tolist(),
        'residual_principal_inertias_kgm2': eig.tolist(),
        'closed_first_moment_error_kgm': float(np.max(abs(reconstructed_first-M*c0))),
        'closed_inertia_error_kgm2': float(np.max(abs(reconstructed_I-I0-parallel_axis(M, c0)))),
        'open_configuration': 'COM and inertia change naturally when MuJoCo integrates relative door rotation.',
    }


def augment_xml(xml, params=None):
    """Return augmented XML and metadata; leave original string/model untouched.

    Requires a named payload_secondary body with the original aggregate mass.
    MuJoCo compiles the unmodified input to obtain its actual reference frame and
    tensor, avoiding guessed world offsets and supporting suspension wrappers.
    """
    p = parameters(params)
    if type(xml) is not str:
        raise TypeError('XML must be a string')
    root = ET.fromstring(xml)
    if root.find(".//*[@name='gtsd_new_doors_added']") is not None or any(
            n.get('name', '').startswith('door_end_') for n in root.iter()):
        raise ValueError('Doors already added; allocation is one-shot')
    if root.find('keyframe') is not None:
        raise ValueError('Existing keyframes require explicit qpos/control remapping before adding door DOFs')
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler', angle='radian')
    if compiler.get('angle', 'degree') != 'radian':
        raise ValueError('Door augmentation requires radian MJCF')
    base = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(base)
    mujoco.mj_forward(base, data)
    payload_id = mujoco.mj_name2id(base, mujoco.mjtObj.mjOBJ_BODY, 'payload_secondary')
    payload = root.find(".//body[@name='payload_secondary']")
    if payload_id < 0 or payload is None or payload.find('inertial') is None:
        raise ValueError('Explicit inertial payload_secondary body required')
    reference = {'position': data.xpos[payload_id].tolist(), 'quaternion': data.xquat[payload_id].tolist()}
    leaves = source_leaves()
    ledger = mass_allocation(base, payload_id, leaves, p, reference)
    inertial = payload.find('inertial')
    I = np.asarray(ledger['residual_inertia_com_payload_kgm2'])
    inertial.attrib.clear()
    inertial.attrib.update(pos=vec(ledger['residual_com_payload_m']), mass=str(ledger['residual_mass_kg']),
                          fullinertia=vec([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]]))
    actuator = root.find('actuator')
    if actuator is None:
        actuator = ET.SubElement(root, 'actuator')
    equality = root.find('equality')
    if p['latch_enabled'] and equality is None:
        equality = ET.SubElement(root, 'equality')
    rp = quatmat(reference['quaternion'])
    xp = np.asarray(reference['position'])
    invq = np.asarray(reference['quaternion'])*[1, -1, -1, -1]
    for d in leaves:
        key = d['key']
        name = 'door_'+key
        h = np.asarray(d['hinge_source_m'])
        c = np.asarray(d['com_source_m'])
        lo, hi = np.asarray(d['assembly_bounds_source_m'])
        sign = d['opening_sign']
        d.update(body_name=name, joint_name=name+'_hinge', actuator_name=name+'_torque',
                 equality_name=name+'_ideal_catch' if p['latch_enabled'] else None,
                 hinge_payload_m=(rp.T@(h-xp)).tolist(),
                 joint_range_rad=sorted([0., sign*p['opening_limit_rad']]),
                 spring_reference_rad=-sign*p['closing_preload_angle_rad'])
        body = ET.SubElement(payload, 'body', name=name, pos=vec(d['hinge_payload_m']), quat=vec(invq))
        ET.SubElement(body, 'joint', name=d['joint_name'], type='hinge', axis=vec(d['axis_source']),
                      pos='0 0 0', limited='true', range=vec(d['joint_range_rad']),
                      stiffness=str(p['stiffness_Nmprad']), springref=str(d['spring_reference_rad']),
                      damping=str(p['damping_Nmsprad']), frictionloss=str(p['frictionloss_Nm']),
                      solreflimit=vec([p['limit_time_constant_s'],p['limit_damping_ratio']]), solimplimit='.99 .999 .001')
        ET.SubElement(body, 'inertial', pos=vec(c-h), mass=str(d['mass_kg']),
                      diaginertia=vec(d['inertia_com_source_diagonal_kgm2']))
        ET.SubElement(body, 'geom', name=name+'_noncontact_proxy', type='box', pos=vec(c-h),
                      size=vec((hi-lo)/2), contype='0', conaffinity='0', mass='0', rgba='.65 .72 .74 .3')
        ET.SubElement(actuator, 'motor', name=d['actuator_name'], joint=d['joint_name'], gear='1',
                      ctrllimited='true', ctrlrange=vec([-p['torque_limit_Nm'], p['torque_limit_Nm']]),
                      forcelimited='true', forcerange=vec([-p['torque_limit_Nm'], p['torque_limit_Nm']]))
        if p['latch_enabled']:
            ET.SubElement(equality, 'joint', name=d['equality_name'], joint1=d['joint_name'],
                          polycoef='0 0 0 0 0', active=str(p['initially_latched']).lower(),
                          solref=vec([p['limit_time_constant_s'], p['limit_damping_ratio']]), solimp='.99 .999 .001')
    custom = root.find('custom')
    if custom is None:
        custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'text', name='gtsd_new_doors_added', data='four_new_mesh_leaves_allocated_once')
    ET.indent(root)
    augmented = ET.tostring(root, encoding='unicode')
    # Compile before returning. No caller state is touched if this fails.
    model = mujoco.MjModel.from_xml_string(augmented)
    if abs(sum(model.body_mass)-sum(base.body_mass)) > 1e-10:
        raise ValueError('Door augmentation changed total vehicle mass')
    metadata = {
        'schema': 'gtsd-new-door-model-v1', 'source_geometry_sha256': GEOMETRY_SHA256,
        'input_xml_sha256': hashlib.sha256(xml.encode()).hexdigest(),
        'augmented_xml_sha256': hashlib.sha256(augmented.encode()).hexdigest(),
        'parameters': p, 'payload_reference_pose': reference, 'doors': leaves,
        'mass_ledger': ledger,
        'dynamic_mass_before_kg': float(sum(base.body_mass)), 'dynamic_mass_after_kg': float(sum(model.body_mass)),
        'evidence': 'Four end leaves and eight visible hinge meshes in new source. Candidate axes, no measured mechanism.',
        'latch_boundary': 'Four optional independent ideal scalar closed constraints. Only two visible handles; no claim of four real locks. No lock core, spindle, linkage, contact tooth or handle dynamics.',
        'collision_boundary': 'Soft hinge end stops only; door/source meshes do not collide with people, bay equipment, each other or chassis.',
        'unresolved': ['No new-source independent external cabinet door established.',
                       'End emergency stops, blank caps and indicator remain on payload: attachment to moving leaf is unresolved.',
                       'Fan grilles and backing ride with the leaf; no fan rotor dynamics or electrical operation.',
                       'Opening direction is outward engineering assumption; 100 degree travel, mass, spring, friction and damping are unmeasured.'],
    }
    metadata['identity_sha256'] = sha(metadata)
    return augmented, metadata


def patch_binding(binding, model, metadata):
    """Move exactly 50 source meshes from old payload ownership to door bodies."""
    out = copy.deepcopy(binding)
    if out.get('source_geometry_sha256') != metadata['source_geometry_sha256']:
        raise ValueError('Binding and door source hashes disagree')
    rows = {r['mesh_name']: r for r in out['meshes']}
    if len(rows) != len(out['meshes']):
        raise ValueError('Duplicate mesh bindings')
    for d in metadata['doors']:
        for name in d['source_parts']:
            if name not in rows or rows[name]['body_name'] != 'payload_secondary' or rows[name]['mode'] != 'rigid':
                raise ValueError('Source mesh is missing, already moved, or not rigid payload: '+name)
            rows[name].update(body_name=d['body_name'], mode='rigid', visible_in_dynamic_view=True,
                              evidence='NEW_SOURCE_DOOR_MEMBERSHIP_WITH_UNCALIBRATED_HINGE',
                              note='New-mesh candidate leaf; torque-integrated joint. Reference vertices unchanged. No hardware-calibrated lock/hinge claim.')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for i in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if name:
            out.setdefault('reference_poses', {})[name] = {'position': data.xpos[i].tolist(), 'quaternion': data.xquat[i].tolist()}
    out['door_augmentation'] = copy.deepcopy(metadata)
    if 'unmodeled' in out:
        out['unmodeled'] = [('Physical door lock core and calibrated hinge mechanics, fan/internal cabinet mechanisms.'
                            if item == 'All independently moving door/fan/internal cabinet mechanics.' else item)
                           for item in out['unmodeled']]
    out['classification_note'] = out.get('classification_note', '') + ' Four new end leaves now articulated with exact-name ownership; remaining cabinet/door candidates retain prior status.'
    return out


class DoorControls:
    """Intent and ideal-catch switches; MuJoCo alone advances physical state."""
    def __init__(self, model, data, metadata):
        if metadata.get('schema') != 'gtsd-new-door-model-v1':
            raise ValueError('Door metadata schema mismatch')
        check = copy.deepcopy(metadata)
        identity = check.pop('identity_sha256', None)
        if identity != sha(check):
            raise ValueError('Door metadata identity mismatch')
        self.model, self.data, self.metadata = model, data, copy.deepcopy(metadata)
        self.p = self.metadata['parameters']
        self.doors = {d['key']: d for d in self.metadata['doors']}
        self.keys = tuple(self.doors)
        self.joints, self.bodies, self.actuators, self.equalities = {}, {}, {}, {}
        self.qpos_addresses, self.qvel_addresses = {}, {}
        for key, d in self.doors.items():
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, d['joint_name'])
            b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, d['body_name'])
            a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, d['actuator_name'])
            if min(j,b,a) < 0:
                raise ValueError('Door is missing from model: '+key)
            self.joints[key], self.bodies[key], self.actuators[key] = j,b,a
            self.qpos_addresses[key] = int(model.jnt_qposadr[j])
            self.qvel_addresses[key] = int(model.jnt_dofadr[j])
            if d['equality_name']:
                eq = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, d['equality_name'])
                if eq < 0:
                    raise ValueError('Door ideal catch missing')
                self.equalities[key] = eq
        self.commands = {k: {'mode': 'release', 'opening_torque_Nm': 0.} for k in self.keys}
        self.apply_forces()

    def _keys(self, key):
        keys = list(self.keys) if key is None or key == 'all' else ([key] if isinstance(key, str) else list(key))
        if not keys or len(keys) != len(set(keys)) or set(keys)-set(self.keys):
            raise ValueError('Unknown, duplicate or empty door selection')
        return keys

    def apply(self, opening_torques=None):
        """Atomically update selected intents in opening-positive Nm; clamp bounds.

        None refreshes own actuator controls. Omitted door keys keep their intent.
        Explicit torque does not unlock; it can test a latched reaction.
        """
        if opening_torques is None:
            return self.apply_forces()
        if type(opening_torques) is not dict or set(opening_torques)-set(self.keys):
            raise ValueError('Expected known door-key to finite torque mapping')
        values = {k: max(-self.p['torque_limit_Nm'], min(self.p['torque_limit_Nm'], finite(v,k)))
                  for k,v in opening_torques.items()}
        for k,v in values.items():
            self.commands[k] = {'mode': 'torque', 'opening_torque_Nm': v}
        self.apply_forces()

    def apply_forces(self):
        for k,d in self.doors.items():
            self.data.ctrl[self.actuators[k]] = d['opening_sign']*self.commands[k]['opening_torque_Nm']

    def set_open(self, key, opened):
        keys = self._keys(key)
        if type(opened) is not bool:
            raise ValueError('opened must be a boolean')
        torque = self.p['command_torque_Nm']*(1 if opened else -1)
        for k in keys:
            self.commands[k] = {'mode': 'open' if opened else 'close', 'opening_torque_Nm': torque}
            if opened and k in self.equalities:
                self.data.eq_active[self.equalities[k]] = False
        self.apply_forces()

    def release(self, key=None):
        for k in self._keys(key):
            self.commands[k] = {'mode': 'release', 'opening_torque_Nm': 0.}
        self.apply_forces()

    def latch(self, key, engaged=True):
        keys = self._keys(key)
        if type(engaged) is not bool:
            raise ValueError('engaged must be a boolean')
        if set(keys)-set(self.equalities):
            raise ValueError('Ideal catches disabled; cannot operate absent locks')
        result = {}
        for k in keys:
            capture = abs(float(self.data.qpos[self.qpos_addresses[k]])) <= self.p['latch_capture_angle_rad']
            capture &= abs(float(self.data.qvel[self.qvel_addresses[k]])) <= self.p['latch_capture_velocity_radps']
            result[k] = bool(not engaged or capture)
        for k in keys:
            if result[k]:
                self.data.eq_active[self.equalities[k]] = engaged
        return result[key] if isinstance(key,str) and key in self.keys else result

    def command(self, name, args):
        if name != 'door' or type(args) is not dict or set(args)-{'key','action','torque_Nm'}:
            raise ValueError('Expected door command with key, action and optional torque_Nm')
        key, action = args.get('key'), args.get('action')
        if 'key' not in args or action not in ('open','close','release','latch','unlock','torque'):
            raise ValueError('Missing door selection or unsupported action')
        keys = self._keys(key)
        if action == 'torque':
            if set(args) != {'key','action','torque_Nm'}:
                raise ValueError('Torque action requires torque_Nm')
            value = finite(args['torque_Nm'],'torque_Nm')
            return self.apply({k:value for k in keys})
        if set(args) != {'key','action'}:
            raise ValueError('Unexpected command field')
        if action in ('open','close'):
            return self.set_open(key,action=='open')
        if action == 'release':
            return self.release(key)
        return self.latch(key,action=='latch')

    def _constraint_component(self, key, kind):
        """Actual solver J-transpose times lambda for this catch or joint limit."""
        if kind == 'latch' and key not in self.equalities:
            return 0.
        desired_type = (mujoco.mjtConstraint.mjCNSTR_EQUALITY if kind == 'latch'
                        else mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        desired_id = self.equalities[key] if kind == 'latch' else self.joints[key]
        force = np.zeros(self.data.nefc)
        rows = (self.data.efc_type == int(desired_type)) & (self.data.efc_id == desired_id)
        force[rows] = self.data.efc_force[rows]
        generalized = np.zeros(self.model.nv)
        mujoco.mj_mulJacTVec(self.model, self.data, generalized, force)
        return float(generalized[self.qvel_addresses[key]])

    def snapshot(self):
        out = {}
        for k,d in self.doors.items():
            q = float(self.data.qpos[self.qpos_addresses[k]])
            v = float(self.data.qvel[self.qvel_addresses[k]])
            dof = self.qvel_addresses[k]
            out[k] = {
                'angle_rad': q, 'opening_rad': d['opening_sign']*q,
                'angular_velocity_radps': v, 'opening_velocity_radps': d['opening_sign']*v,
                'latched': bool(self.data.eq_active[self.equalities[k]]) if k in self.equalities else False,
                'ideal_latch_enabled': k in self.equalities,
                'capture_angle_satisfied': abs(q) <= self.p['latch_capture_angle_rad'],
                'capture_velocity_satisfied': abs(v) <= self.p['latch_capture_velocity_radps'],
                'command': copy.deepcopy(self.commands[k]),
                'torques': {
                    'actuator_Nm': float(self.data.actuator_force[self.actuators[k]]),
                    'spring_Nm': -self.p['stiffness_Nmprad']*(q-d['spring_reference_rad']),
                    'viscous_Nm': -self.p['damping_Nmsprad']*v,
                    'constraint_total_Nm': float(self.data.qfrc_constraint[dof]),
                    'ideal_latch_reaction_Nm': self._constraint_component(k, 'latch'),
                    'joint_limit_reaction_Nm': self._constraint_component(k, 'limit'),
                    'gravity_coriolis_bias_Nm': float(self.data.qfrc_bias[dof]),
                },
            }
        return out

    def checkpoint_state(self):
        """Controller-owned state; pair with model mjSTATE_INTEGRATION for replay."""
        return {'schema': 'gtsd-door-controls-v1', 'identity_sha256': self.metadata['identity_sha256'],
                'commands': copy.deepcopy(self.commands),
                'ctrl': {k:float(self.data.ctrl[a]) for k,a in self.actuators.items()},
                'eq_active': {k:bool(self.data.eq_active[e]) for k,e in self.equalities.items()}}

    def restore(self, state):
        """Validate entire state before writing; never restores qpos/qvel itself."""
        expected = {'schema','identity_sha256','commands','ctrl','eq_active'}
        if type(state) is not dict or set(state) != expected or state['schema'] != 'gtsd-door-controls-v1':
            raise ValueError('Door control state schema mismatch')
        if state['identity_sha256'] != self.metadata['identity_sha256']:
            raise ValueError('Door source/model/parameter identity mismatch')
        for name, keys in (('commands',self.keys), ('ctrl',self.keys), ('eq_active',self.equalities)):
            if type(state[name]) is not dict or set(state[name]) != set(keys):
                raise ValueError('Door state fields mismatch: '+name)
        candidate = copy.deepcopy(state['commands'])
        ctrls = {}
        for k,c in candidate.items():
            if type(c) is not dict or set(c) != {'mode','opening_torque_Nm'} or c['mode'] not in ('release','open','close','torque'):
                raise ValueError('Door command state invalid')
            v = finite(c['opening_torque_Nm'], 'opening torque')
            ctrl = finite(state['ctrl'][k], 'actuator ctrl')
            if abs(v) > self.p['torque_limit_Nm'] or abs(ctrl-self.doors[k]['opening_sign']*v) > 1e-12:
                raise ValueError('Out-of-range or inconsistent door actuator state')
            expected_torque = {'release':0., 'open':self.p['command_torque_Nm'], 'close':-self.p['command_torque_Nm']}
            if c['mode'] != 'torque' and v != expected_torque[c['mode']]:
                raise ValueError('Door mode and torque disagree')
            ctrls[k] = ctrl
        if any(type(v) is not bool for v in state['eq_active'].values()):
            raise ValueError('Latch state must be boolean')
        # Capture is NOT reevaluated on restore; existing physically loaded catches
        # can exceed capture thresholds. State is replay, not a new latch command.
        self.commands = candidate
        for k,v in ctrls.items():
            self.data.ctrl[self.actuators[k]] = v
        for k,v in state['eq_active'].items():
            self.data.eq_active[self.equalities[k]] = v


def standalone_xml(dt=.0004, params=None, gravity=(0.,0.,-9.81), free_payload=False):
    """Diagnostic bench with original equivalent payload; no external cabinet leaf."""
    dt = finite(dt,'dt')
    if dt <= 0:
        raise ValueError('dt must be positive')
    root = ET.Element('mujoco', model='new_source_four_door_bench')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false')
    ET.SubElement(root,'option',timestep=str(dt),gravity=vec(gravity),integrator='implicitfast',
                  solver='Newton',iterations='100',tolerance='1e-12')
    world = ET.SubElement(root,'worldbody')
    body = ET.SubElement(world,'body',name='payload_secondary',pos='0 0 0.639691067284615')
    if free_payload:
        ET.SubElement(body,'freejoint',name='diagnostic_payload_free')
    ET.SubElement(body,'inertial',mass='30',pos='0 0 0',diaginertia='2.36 4.98 5.83')
    return augment_xml(ET.tostring(root,encoding='unicode'),params)
