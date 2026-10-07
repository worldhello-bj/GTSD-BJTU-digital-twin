"""Six source-bound B8 access doors, driven only by bounded hinge torque.

Source establishes hinge locations, leaf membership and travel, NOT mechanical
properties. Assumed masses/inertia, springs, damping and hand torque are explicit.
The cabinet bases are world-fixed. Car doors belong to car A's 60 kg allocation.
No position/velocity servo, pose write or animation driver. An optional ideal
closed catch is an explicit added assumption, with no handle/spindle dynamics.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from math import isfinite, pi
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, tostring, indent
import json

SOURCE_HASH = 'c96811c45220dcdfad0853b73e4743297be7e090d4fb3230a7531eb89b3a4c04'
SOURCE_CAR_CENTER = (1.3, 0., 1.095)
DOOR_KEYS = ('bay_L', 'bay_R', 'bay_L_secondary', 'bay_R_secondary', 'cabinet_PWR', 'cabinet_DAQ')
DEFAULTS = {
    'bay_mass_kg': 1.2, 'bay_fan_mass_kg': 1.4, 'cabinet_mass_kg': 6.0,
    'bay_stiffness_Nmprad': .35, 'cabinet_stiffness_Nmprad': 1.0,
    'bay_damping_Nmsprad': .32, 'cabinet_damping_Nmsprad': 1.1,
    'bay_frictionloss_Nm': .004, 'cabinet_frictionloss_Nm': .015,
    'bay_torque_limit_Nm': 1.5, 'cabinet_torque_limit_Nm': 5.,
    'closing_preload_angle_rad': .04,
    'limit_time_constant_s': .002, 'limit_damping_ratio': 1.,
    'latch_capture_angle_rad': .006, 'latch_capture_velocity_radps': .03,
}
# Generated from the audited, closed-rest Blender source; patched in below.
SOURCE_LEAVES = {'bay_L': {'source_owner': 'BayDoor_L',
           'source_owner_type': 'bone',
           'source_parts': ['CarBody_Window_03',
                            'CarBody_BayDoor_L',
                            'B1_Band_L_1',
                            'B1_Window_L_1_2',
                            'B1_Stripe_L_1_0.985',
                            'B1_Stripe_L_1_0.933',
                            'B1_DoorStiffener_L_1',
                            'B1_DoorLatch_L_1',
                            'B1_DoorHinge_L_1_0',
                            'B1_DoorHinge_L_1_1'],
           'bounds_source_m': [[1.305999994277954, 0.45100000500679016, 0.7039999961853027],
                               [2.5249998569488525, 0.49950000643730164, 1.3079999685287476]],
           'hinge_source_m': [2.5199999809265137, 0.4789999723434448, 1.0],
           'opening_sign': -1,
           'opening_limit_rad': 1.31},
 'bay_R': {'source_owner': 'BayDoor_R',
           'source_owner_type': 'bone',
           'source_parts': ['CarBody_BayDoor_R',
                            'B1_Band_R_0',
                            'B1_Window_R_0_1',
                            'B1_Window_R_0_2',
                            'B1_Stripe_R_0_0.985',
                            'B1_Stripe_R_0_0.933',
                            'B1_DoorStiffener_R_0',
                            'B1_DoorLatch_R_0',
                            'B1_DoorHinge_R_0_0',
                            'B1_DoorHinge_R_0_1'],
           'bounds_source_m': [[0.07499995082616806, -0.49950000643730164, 0.7039999961853027],
                               [1.2939999103546143, -0.45100000500679016, 1.3079999685287476]],
           'hinge_source_m': [0.07999992370605469, -0.4789999723434448, 1.0],
           'opening_sign': -1,
           'opening_limit_rad': 1.31},
 'bay_L_secondary': {'source_owner': 'BayDoor_L_Secondary',
                     'source_owner_type': 'bone',
                     'source_parts': ['CarBody_WindowBand',
                                      'CarBody_Window_01',
                                      'CarBody_Window_02',
                                      'CarBody_Stripe',
                                      'CarBody_BayDoor_L_Secondary',
                                      'B1_Stripe_L_0_0.933',
                                      'B1_DoorStiffener_L_0',
                                      'B1_DoorLatch_L_0',
                                      'B1_DoorHinge_L_0_0',
                                      'B1_DoorHinge_L_0_1'],
                     'bounds_source_m': [[0.07499995082616806,
                                          0.45100000500679016,
                                          0.7039999961853027],
                                         [1.2939999103546143,
                                          0.49950000643730164,
                                          1.3079999685287476]],
                     'hinge_source_m': [0.07999992370605469, 0.4789999723434448, 1.0],
                     'opening_sign': 1,
                     'opening_limit_rad': 1.31},
 'bay_R_secondary': {'source_owner': 'BayDoor_R_Secondary',
                     'source_owner_type': 'bone',
                     'source_parts': ['CarBody_BayDoor_R_Secondary',
                                      'B1_Band_R_1',
                                      'B1_Window_R_1_2',
                                      'B1_Window_R_1_3',
                                      'B1_Stripe_R_1_0.985',
                                      'B1_Stripe_R_1_0.933',
                                      'B1_DoorStiffener_R_1',
                                      'B1_DoorLatch_R_1',
                                      'B1_DoorHinge_R_1_0',
                                      'B1_DoorHinge_R_1_1',
                                      'B1_BayFanSquareFrame',
                                      'Aux_BayFan',
                                      'B1_BayFanGrille',
                                      'B1_BayFanBolts'],
                     'bounds_source_m': [[1.305999994277954,
                                          -0.49950000643730164,
                                          0.7039999961853027],
                                         [2.5249998569488525,
                                          -0.4212999939918518,
                                          1.3079999685287476]],
                     'hinge_source_m': [2.5199999809265137, -0.4789999723434448, 1.0],
                     'opening_sign': 1,
                     'opening_limit_rad': 1.31},
 'cabinet_PWR': {'source_owner': 'B5_PWR 01_DoorPivot',
                 'source_owner_type': 'object',
                 'source_parts': ['B4_PWR 01_Door',
                                  'B4_PWR 01_HMI',
                                  'B4_PWR 01_HMIbar',
                                  'B4_PWR 01_HMIbar.001',
                                  'B4_PWR 01_HMIbar.002',
                                  'B4_PWR 01_HMIbar.003',
                                  'B4_PWR 01_Handle',
                                  'B4_PWR 01_Vent',
                                  'B4_PWR 01_Vent.001',
                                  'B4_PWR 01_Vent.002',
                                  'B4_PWR 01_Vent.003',
                                  'B4_PWR 01_Vent.004',
                                  'B4_PWR 01_Vent.005',
                                  'B4_PWR 01_Vent.006',
                                  'B4_PWR 01_Vent.007',
                                  'B4_PWR 01_Vent.008',
                                  'B4_PWR 01_Label',
                                  'B4_PWR 01_Button',
                                  'B4_PWR 01_Button.001',
                                  'B5_PWR 01_DoorEarthLug'],
                 'bounds_source_m': [[2.2775001525878906, 2.6600000858306885, 0.06499999761581421],
                                     [3.0225000381469727, 2.740000009536743, 1.7950000762939453]],
                 'hinge_source_m': [2.2639999389648438, 2.7190001010894775, 0.05000000074505806],
                 'opening_sign': -1,
                 'opening_limit_rad': 1.9198621771937625},
 'cabinet_DAQ': {'source_owner': 'B5_DAQ 01_DoorPivot',
                 'source_owner_type': 'object',
                 'source_parts': ['B4_DAQ 01_Door',
                                  'B4_DAQ 01_HMI',
                                  'B4_DAQ 01_HMIbar',
                                  'B4_DAQ 01_HMIbar.001',
                                  'B4_DAQ 01_HMIbar.002',
                                  'B4_DAQ 01_HMIbar.003',
                                  'B4_DAQ 01_Handle',
                                  'B4_DAQ 01_Vent',
                                  'B4_DAQ 01_Vent.001',
                                  'B4_DAQ 01_Vent.002',
                                  'B4_DAQ 01_Vent.003',
                                  'B4_DAQ 01_Vent.004',
                                  'B4_DAQ 01_Vent.005',
                                  'B4_DAQ 01_Vent.006',
                                  'B4_DAQ 01_Vent.007',
                                  'B4_DAQ 01_Vent.008',
                                  'B4_DAQ 01_Label',
                                  'B4_DAQ 01_Button',
                                  'B4_DAQ 01_Button.001',
                                  'B5_DAQ 01_DoorEarthLug'],
                 'bounds_source_m': [[3.2975001335144043, 2.6600000858306885, 0.06499999761581421],
                                     [4.042500019073486, 2.740000009536743, 1.7950000762939453]],
                 'hinge_source_m': [3.2839999198913574, 2.7190001010894775, 0.05000000074505806],
                 'opening_sign': -1,
                 'opening_limit_rad': 1.9198621771937625}}


def _vec(values):
    return ' '.join(f'{float(x):.15g}' for x in values)


def _add(parent, tag, **attributes):
    return SubElement(parent, tag, {k: str(v) for k, v in attributes.items()})


def _parallel_axis(mass, r):
    import numpy as np
    r = np.asarray(r, dtype=float)
    return mass * (float(r @ r) * np.eye(3) - np.outer(r, r))


@dataclass(frozen=True)
class DoorSpec:
    key: str
    source_owner: str
    source_owner_type: str
    source_parts: tuple[str, ...]
    attached_to: str
    body_name: str
    joint_name: str
    actuator_name: str
    hinge_source_m: tuple[float, float, float]
    hinge_parent_m: tuple[float, float, float]
    hinge_world_rest_m: tuple[float, float, float]
    bounds_source_m: tuple[tuple[float, float, float], tuple[float, float, float]]
    com_source_m: tuple[float, float, float]
    com_hinge_m: tuple[float, float, float]
    mass_kg: float
    inertia_com_kgm2: tuple[float, float, float]
    opening_sign: int
    opening_limit_rad: float
    stiffness_Nmprad: float
    damping_Nmsprad: float
    frictionloss_Nm: float
    torque_limit_Nm: float
    spring_reference_rad: float
    latch_equality_name: str | None

    @property
    def joint_range_rad(self):
        q = self.opening_sign * self.opening_limit_rad
        return (min(0., q), max(0., q))

    @property
    def hinge_inertia_kgm2(self):
        x, y, _ = self.com_hinge_m
        return self.inertia_com_kgm2[2] + self.mass_kg * (x*x + y*y)


@dataclass(frozen=True)
class AccessDoorsSpec:
    doors: tuple[DoorSpec, ...]
    source_x_shift: float
    params: dict

    def bind(self, model):
        return AccessDoorsState(self, model)

    def by_key(self):
        return {d.key: d for d in self.doors}

    def source_body_mapping(self):
        """All 84 source leaf-owned parts mapped to their dynamic MJCF body."""
        return {name: d.body_name for d in self.doors for name in d.source_parts}

    def car_mass_allocation(self, original_mass_kg=60., original_com_m=(0., 0., 0.),
                            original_inertia_kgm2=(10.025, 36.7625, 35.7625)):
        """Subtract closed car leaves from A's aggregate, preserving total M/COM/I.

        The original tensor is about original_com_m, in the car root axes. Output
        residual inertia is about residual_com_m in those same axes, in MJCF
        fullinertia order xx yy zz xy xz yz. Never includes stationary cabinets.
        """
        import numpy as np
        M = float(original_mass_kg)
        c0 = np.asarray(original_com_m, dtype=float)
        inertia = np.asarray(original_inertia_kgm2, dtype=float)
        if inertia.shape == (3,):
            I0 = np.diag(inertia)
        elif inertia.shape == (6,):
            xx, yy, zz, xy, xz, yz = inertia
            I0 = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        elif inertia.shape == (3, 3):
            I0 = inertia
        else:
            raise ValueError('Original inertia requires diagonal 3, full 6 or 3x3 values')
        leaves = [d for d in self.doors if d.attached_to == 'car_A']
        masses = sum(d.mass_kg for d in leaves)
        residual_mass = M - masses
        if not isfinite(residual_mass) or residual_mass <= 0:
            raise ValueError('Door allocation exhausts original car mass')
        moving_first = np.zeros(3)
        moving_I_root = np.zeros((3, 3))
        rows = []
        for d in leaves:
            r = np.asarray(d.com_source_m) - np.asarray(SOURCE_CAR_CENTER)
            Icom = np.diag(d.inertia_com_kgm2)
            Iroot = Icom + _parallel_axis(d.mass_kg, r)
            moving_first += d.mass_kg * r
            moving_I_root += Iroot
            rows.append({'key': d.key, 'mass_kg': d.mass_kg, 'com_car_m': r.tolist(),
                         'inertia_com_kgm2': Icom.tolist(), 'inertia_car_origin_kgm2': Iroot.tolist()})
        residual_com = (M*c0 - moving_first) / residual_mass
        residual_I = I0 + _parallel_axis(M, c0) - moving_I_root - _parallel_axis(residual_mass, residual_com)
        eig = np.linalg.eigvalsh(residual_I)
        if not np.all(np.isfinite(eig)) or min(eig) <= 0 or max(eig) > sum(eig)-max(eig):
            raise ValueError('Subtracted car-body inertia is not a physically realizable positive tensor')
        full = [residual_I[0, 0], residual_I[1, 1], residual_I[2, 2],
                residual_I[0, 1], residual_I[0, 2], residual_I[1, 2]]
        return {'basis': 'Original car A aggregate includes all four closed door leaves; cabinet bases are fixed to world.',
                'original_mass_kg': M, 'original_com_m': c0.tolist(), 'original_inertia_com_kgm2': I0.tolist(),
                'moving_mass_kg': masses, 'moving_com_car_m': (moving_first/masses).tolist(),
                'moving_inertia_car_origin_kgm2': moving_I_root.tolist(), 'moving_leaves': rows,
                'residual_mass_kg': residual_mass, 'residual_com_m': residual_com.tolist(),
                'residual_inertia_com_kgm2': residual_I.tolist(), 'residual_fullinertia_kgm2': full,
                'residual_principal_inertias_kgm2': eig.tolist()}

    def apply_car_mass_allocation(self, car_A_element, **original):
        """Explicit opt-in mutation of A's inertial only; rejects double allocation.

        To compose with other allocated mechanisms, compute all their subtractions
        against the original aggregate together rather than repeatedly call this.
        """
        inertial = car_A_element.find('inertial')
        if inertial is None:
            raise ValueError('Car A needs an explicit aggregate inertial to allocate doors')
        allocation = self.car_mass_allocation(**original)
        present_mass = float(inertial.get('mass', 'nan'))
        if not isfinite(present_mass) or abs(present_mass - allocation['original_mass_kg']) > 1e-9:
            raise ValueError('Car A aggregate has changed or was already allocated; reconcile mass budget first')
        inertial.attrib.clear()
        inertial.attrib.update(pos=_vec(allocation['residual_com_m']),
                               mass=str(allocation['residual_mass_kg']),
                               fullinertia=_vec(allocation['residual_fullinertia_kgm2']))
        return allocation

    def to_dict(self):
        return {'source_hash': SOURCE_HASH, 'source_x_shift_m': self.source_x_shift, 'parameters': dict(self.params),
                'mechanical_parameter_status': 'Assumptions, not measured or source-validated',
                'inertia_basis': 'Uniform mass within the closed-rest whole-leaf source AABB',
                'cabinet_support': 'Infinite world-fixed support, no cabinet chassis dynamics',
                'collision_scope': 'Joint end stops; no source-mesh door obstacle collision',
                'closure': 'Preloaded torsion spring, viscous damping and hinge friction',
                'latch': 'Ideal closed scalar constraint; no handle/spindle dynamics' if self.doors[0].latch_equality_name else 'No latch: equality section was omitted',
                'doors': [asdict(d) | {'hinge_inertia_kgm2': d.hinge_inertia_kgm2} for d in self.doors],
                'source_body_mapping': self.source_body_mapping(), 'car_mass_allocation': self.car_mass_allocation()}


class AccessDoorsState:
    def __init__(self, spec, model):
        import mujoco
        self.spec, self.model = spec, model
        self.joint_ids, self.body_ids, self.actuator_ids = {}, {}, {}
        self.qpos_addresses, self.qvel_addresses, self.latch_ids = {}, {}, {}
        for d in spec.doors:
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, d.joint_name)
            b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, d.body_name)
            a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, d.actuator_name)
            if min(j, b, a) < 0:
                raise ValueError(f'Access door {d.key} is missing from the compiled model')
            self.joint_ids[d.key], self.body_ids[d.key], self.actuator_ids[d.key] = j, b, a
            self.qpos_addresses[d.key] = int(model.jnt_qposadr[j])
            self.qvel_addresses[d.key] = int(model.jnt_dofadr[j])
            if d.latch_equality_name is not None:
                e = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, d.latch_equality_name)
                if e < 0:
                    raise ValueError(f'Access door latch {d.key} is missing')
                self.latch_ids[d.key] = e

    def set_torques(self, data, signed_Nm):
        """Set signed source-Z hand torques. Omitted keys release to zero torque.

        Input clamps to each declared +/- maximum. Rejects NaN, unknown keys and
        invalid commands atomically, before changing any controls. No pose writes.
        """
        if not hasattr(signed_Nm, 'keys'):
            raise TypeError('Expected a door-key to signed torque [Nm] mapping')
        unknown = set(signed_Nm) - set(self.actuator_ids)
        if unknown:
            raise KeyError(f'Unknown access door keys: {sorted(unknown)}')
        torques = {d.key: float(signed_Nm.get(d.key, 0.)) for d in self.spec.doors}
        if not all(isfinite(t) for t in torques.values()):
            raise ValueError('Door torques must be finite')
        for d in self.spec.doors:
            data.ctrl[self.actuator_ids[d.key]] = min(d.torque_limit_Nm, max(-d.torque_limit_Nm, torques[d.key]))

    def set_opening_torques(self, data, opening_Nm):
        """Positive opens in the source direction; negative assists closing."""
        directions = {d.key: d.opening_sign for d in self.spec.doors}
        unknown = set(opening_Nm) - set(directions)
        if unknown:
            raise KeyError(f'Unknown access door keys: {sorted(unknown)}')
        self.set_torques(data, {k: directions[k]*float(v) for k, v in opening_Nm.items()})

    def release(self, data):
        self.set_torques(data, {})

    def _latch_keys(self, keys):
        keys = list(self.actuator_ids if keys is None else ([keys] if isinstance(keys, str) else keys))
        missing = set(keys) - set(self.latch_ids)
        if missing:
            raise KeyError(f'Unknown or unmodeled door latches: {sorted(missing)}')
        return keys

    def release_latch(self, data, keys=None):
        """Explicitly unlock ideal catches by disabling constraints, not changing pose."""
        for k in self._latch_keys(keys):
            data.eq_active[self.latch_ids[k]] = False

    def try_latch(self, data, keys=None):
        """Engage each catch only in its actual closed/slow capture envelope.

        Returns per-door success. There is no snapping, qpos or qvel write.
        """
        result = {}
        for k in self._latch_keys(keys):
            closed = abs(float(data.qpos[self.qpos_addresses[k]])) <= self.spec.params['latch_capture_angle_rad']
            slow = abs(float(data.qvel[self.qvel_addresses[k]])) <= self.spec.params['latch_capture_velocity_radps']
            result[k] = bool(closed and slow)
            if result[k]:
                data.eq_active[self.latch_ids[k]] = True
        return result

    def latched(self, data):
        return {d.key: bool(data.eq_active[self.latch_ids[d.key]]) if d.key in self.latch_ids else False
                for d in self.spec.doors}

    def angles(self, data):
        return {k: float(data.qpos[a]) for k, a in self.qpos_addresses.items()}

    def velocities(self, data):
        return {k: float(data.qvel[a]) for k, a in self.qvel_addresses.items()}

    def forces(self, data):
        """Actual hinge torques [Nm], using refreshed MuJoCo derived quantities.

        qfrc_constraint combines unilateral limits and dry friction constraints.
        Spring/viscous values are reported separately from that solver output.
        """
        out = {}
        for d in self.spec.doors:
            q = float(data.qpos[self.qpos_addresses[d.key]])
            v = float(data.qvel[self.qvel_addresses[d.key]])
            out[d.key] = {'actuator_Nm': float(data.actuator_force[self.actuator_ids[d.key]]),
                          'spring_Nm': -d.stiffness_Nmprad*(q-d.spring_reference_rad),
                          'viscous_Nm': -d.damping_Nmsprad*v,
                          'constraint_Nm': float(data.qfrc_constraint[self.qvel_addresses[d.key]])}
        return out

    def telemetry(self, data):
        angles, velocities, forces = self.angles(data), self.velocities(data), self.forces(data)
        return {d.key: {'angle_rad': angles[d.key], 'opening_rad': d.opening_sign*angles[d.key],
                        'angular_velocity_radps': velocities[d.key], 'latched': self.latched(data)[d.key], 'torques': forces[d.key]}
                for d in self.spec.doors}


def add_access_doors(car_A_element, worldbody, actuators, source_x_shift=3.0, *, params=None, equality=None):
    """Append six hinges to a radian-MJCF model and return its binding spec.

    Car A must be identity-oriented at source car center+(source_x_shift,0,0).
    This function does NOT alter A's inertia; use the explicit allocation method
    before compiling to avoid counting the 5 kg of car doors twice. World cabinet
    roots receive the same +X registration as the whole source lab.
    Pass the root equality section to add six initially latched ideal closed
    catches. If omitted, no catch is invented and closure is spring/friction only.
    """
    p = DEFAULTS | (params or {})
    unknown = set(p) - set(DEFAULTS)
    if unknown:
        raise KeyError(f'Unknown door parameters: {sorted(unknown)}')
    if not all(isfinite(float(v)) and float(v) >= 0 for v in p.values()):
        raise ValueError('Door parameters must be finite and nonnegative')
    if any(p[k] <= 0 for k in ('bay_mass_kg','bay_fan_mass_kg','cabinet_mass_kg',
                              'bay_torque_limit_Nm','cabinet_torque_limit_Nm','limit_time_constant_s')):
        raise ValueError('Door masses, torque limits and limit time constant must be positive')
    source_x_shift = float(source_x_shift)
    if not isfinite(source_x_shift):
        raise ValueError('Source X shift must be finite')
    if set(SOURCE_LEAVES) != set(DOOR_KEYS):
        raise ValueError('Audited source door geometry is missing')
    if any(e.get('name','').startswith('access_door_') for root in (car_A_element, worldbody, actuators) for e in root.iter()):
        raise ValueError('Access doors already exist in this XML tree')
    doors = []
    for key in DOOR_KEYS:
        s = SOURCE_LEAVES[key]
        is_car = key.startswith('bay_')
        kind = 'bay' if is_car else 'cabinet'
        low, high = s['bounds_source_m']
        size = tuple((b-a)/2 for a,b in zip(low, high))
        com = tuple((a+b)/2 for a,b in zip(low, high))
        pivot = tuple(s['hinge_source_m'])
        center = SOURCE_CAR_CENTER if is_car else (0.,0.,0.)
        parent_pivot = tuple(pivot[i]-center[i] for i in range(3)) if is_car else (0.,0.,0.)
        world_pivot = (pivot[0]+source_x_shift, pivot[1], pivot[2])
        com_hinge = tuple(com[i]-pivot[i] for i in range(3))
        mass = float(p['bay_fan_mass_kg' if key == 'bay_R_secondary' else kind+'_mass_kg'])
        dx,dy,dz = (2*x for x in size)
        inertia = (mass*(dy*dy+dz*dz)/12, mass*(dx*dx+dz*dz)/12, mass*(dx*dx+dy*dy)/12)
        sign, limit = s['opening_sign'], s['opening_limit_rad']
        name = 'access_door_'+key
        d = DoorSpec(key, s['source_owner'], s['source_owner_type'], tuple(s['source_parts']),
                     'car_A' if is_car else 'world', name, name+'_hinge', name+'_torque',
                     pivot, parent_pivot, world_pivot, (tuple(low),tuple(high)), com, com_hinge,
                     mass, inertia, sign, limit, float(p[kind+'_stiffness_Nmprad']),
                     float(p[kind+'_damping_Nmsprad']), float(p[kind+'_frictionloss_Nm']),
                     float(p[kind+'_torque_limit_Nm']), -sign*float(p['closing_preload_angle_rad']),
                     name+'_ideal_closed_latch' if equality is not None else None)
        parent = car_A_element if is_car else _add(worldbody,'body',name=name+'_fixed_support',pos=_vec(world_pivot))
        body = _add(parent,'body',name=d.body_name,pos=_vec(parent_pivot))
        _add(body,'joint',name=d.joint_name,type='hinge',axis='0 0 1',pos='0 0 0',
             limited='true',range=_vec(d.joint_range_rad),stiffness=d.stiffness_Nmprad,
             springref=d.spring_reference_rad,damping=d.damping_Nmsprad,frictionloss=d.frictionloss_Nm,
             solreflimit=_vec((p['limit_time_constant_s'],p['limit_damping_ratio'])),
             solimplimit='.99 .999 .001')
        _add(body,'inertial',pos=_vec(com_hinge),mass=mass,diaginertia=_vec(inertia))
        _add(body,'geom',name=name+'_visual_proxy',type='box',pos=_vec(com_hinge),size=_vec(size),
             contype='0',conaffinity='0',rgba='.62 .72 .77 .7' if is_car else '.21 .29 .36 .85',mass='0')
        _add(body,'site',name=name+'_hinge_site',pos='0 0 0',size='.009',rgba='.9 .6 .2 1')
        _add(actuators,'motor',name=d.actuator_name,joint=d.joint_name,gear='1',
             ctrllimited='true',ctrlrange=_vec((-d.torque_limit_Nm,d.torque_limit_Nm)),
             forcelimited='true',forcerange=_vec((-d.torque_limit_Nm,d.torque_limit_Nm)))
        if equality is not None:
            _add(equality,'joint',name=d.latch_equality_name,joint1=d.joint_name,
                 polycoef='0 0 0 0 0',active='true',solref='.002 1',solimp='.99 .999 .001')
        doors.append(d)
    return AccessDoorsSpec(tuple(doors), source_x_shift, p)


def standalone_xml(timestep=.0001, *, params=None, free_car=False):
    """Diagnostic fixture; optionally free car A to check action/reaction transfer."""
    root = Element('mujoco', model='B8_six_source_access_doors')
    _add(root,'compiler',angle='radian',autolimits='true')
    _add(root,'option',timestep=timestep,gravity='0 0 0' if free_car else '0 0 -9.81',
         integrator='implicitfast',solver='Newton',iterations='80',tolerance='1e-12')
    world = _add(root,'worldbody')
    car = _add(world,'body',name='A_carbody',pos='4.3 0 1.095')
    if free_car:
        _add(car,'freejoint',name='A_car_free')
    _add(car,'inertial',pos='0 0 0',mass='60',diaginertia='10.025 36.7625 35.7625')
    equality = _add(root,'equality')
    actuators = _add(root,'actuator')
    spec = add_access_doors(car, world, actuators, params=params, equality=equality)
    spec.apply_car_mass_allocation(car)
    indent(root)
    return tostring(root,encoding='unicode'), spec
