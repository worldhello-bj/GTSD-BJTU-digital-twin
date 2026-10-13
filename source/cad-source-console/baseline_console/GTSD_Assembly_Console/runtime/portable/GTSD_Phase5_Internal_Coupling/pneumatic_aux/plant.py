"""Standalone finite-volume air, valve, pump, fan and thermal equivalent.

This module contains no MuJoCo model, prescribed mechanical trajectory, device
access, network access or legacy vehicle factory calls. Mechanical coordinates
must come from the caller's actual solver. All dimensions are engineering
assumptions, not measured B32/bellows geometry.
"""
from __future__ import annotations
import copy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import zlib
from typing import Mapping
from .kernels import b8, b9, b10, VENDOR_HASHES, verify_vendor

SCHEMA = 'gtsd-pneumatic-aux-checkpoint-v1'
PROVENANCE = 'UNCALIBRATED_ENGINEERING_EQUIVALENT_NOT_MEASURED_HARDWARE'
FANS = ('compressor_fan', 'bay_fan')
AIR_SPRINGS = ('air_spring_left', 'air_spring_right')
BRAKES = ('brake_axle_1', 'brake_axle_2')
MODES = ('fill', 'hold', 'vent')

def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()

def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()

def number(value, name, low=None, high=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(name+' must be a finite number')
    if low is not None and value < low or high is not None and value > high:
        raise ValueError(name+' outside accepted range')
    return float(value)

@dataclass(frozen=True)
class PneumaticAuxiliaryParameters:
    reservoir_volume_m3: float = .002
    reservoir_initial_gauge_pressure_Pa: float = 500000.
    brake_volume_each_axle_m3: float = 1e-5
    brake_supply_area_m2: float = 5e-8
    brake_exhaust_area_m2: float = 1.5e-7
    brake_leak_area_m2: float = 3e-9
    reservoir_leak_area_m2: float = 2e-6
    pantograph_effective_area_m2: float = 2e-4
    pantograph_equivalent_lever_m: float = .05
    pantograph_phase_rad: float = 0.
    pantograph_dead_volume_m3: float = 4e-5
    pantograph_supply_area_m2: float = 6e-8
    pantograph_exhaust_area_m2: float = 6e-8
    pantograph_quick_exhaust_area_m2: float = 6e-7
    pantograph_leak_area_m2: float = 1e-7
    air_spring_area_each_m2: float = 3e-4
    air_spring_dead_volume_each_m3: float = 4e-5
    air_spring_initial_gauge_pressure_Pa: float = 0.
    air_spring_supply_area_m2: float = 6e-8
    air_spring_exhaust_area_m2: float = 6e-8
    air_spring_leak_area_m2: float = 1e-7
    gas_max_substep_s: float = .0002
    cabinet_external_heat_W: float = 80.

    def __post_init__(self):
        nonnegative = {'reservoir_initial_gauge_pressure_Pa', 'air_spring_initial_gauge_pressure_Pa', 'cabinet_external_heat_W'}
        for name, value in asdict(self).items():
            number(value, name)
            if name == 'pantograph_phase_rad':
                continue
            if value < 0 or (value == 0 and name not in nonnegative):
                raise ValueError('Invalid nonpositive parameter '+name)
        # Guarantees positive equivalent pantograph volume for every finite q.
        if self.pantograph_dead_volume_m3 <= 2*self.pantograph_effective_area_m2*self.pantograph_equivalent_lever_m:
            raise ValueError('Pantograph dead volume must exceed full equivalent swept volume')

class MeteredGasNetwork(b8.PneumaticNetwork):
    """Same pinned gas equations, plus exact work attribution by volume."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node_useful_work_J = {n: 0. for n in self.volumes}
        # Interval ledger starts from zero; never recover tiny work by subtracting cumulative totals.
        self.node_interval_work_J = {n: 0. for n in self.volumes}

    def _move_volumes(self, positions):
        for name, x in positions.items():
            v = self.volumes[name]
            dv = v.piston_area_m2*(x-v.displacement_m)
            increment = v.mass_kg*self.RT*math.log1p(dv/v.volume_m3)-self.ambient_pressure_Pa*dv
            self.node_useful_work_J[name] += increment
            self.node_interval_work_J[name] += increment
        super()._move_volumes(positions)


class CheckpointAuxiliaryNetwork(b10.AuxiliaryNetwork):
    def __getattr__(self, name):
        # The frozen B9 forwarding method recurses during copy reconstruction
        # before 'gas' exists; preserve normal forwarding with a safe guard.
        gas = self.__dict__.get('gas')
        if gas is None:
            raise AttributeError(name)
        return getattr(gas, name)


def _typed_like(value, template, path='state'):
    if type(template) is dict:
        if type(value) is not dict or set(value) != set(template):
            raise ValueError(path+' keys mismatch')
        return {k: _typed_like(value[k], v, path+'.'+k) for k, v in template.items()}
    if type(template) is list:
        if type(value) is not list or len(value) != len(template):
            raise ValueError(path+' list schema mismatch')
        return [_typed_like(v, t, path+'[]') for v,t in zip(value, template)]
    if type(template) in (float, int) and type(template) is not bool:
        if type(template) is int:
            if type(value) is not int:
                raise ValueError(path+' must be integer')
            return value
        return number(value, path)
    if type(value) is not type(template):
        raise ValueError(path+' scalar schema mismatch')
    if type(template) not in (bool, str, type(None)):
        raise TypeError(path+' unsupported checkpoint schema')
    return value

class PneumaticAuxiliaryPlant:
    """Data-only adapter for the new assembly's observed coordinate ports.

    step(dt_s, mechanical_state=None, commands=None) advances forces and gas.
    Omitted fields persist. dt=0 may change controls but never coordinates.
    Returned work-equivalent generalized efforts can be used in a mechanical
    midpoint/corrector; caller must reconcile actual work before committing.
    Checkpoint/restore permits repeatable trial integration without committing
    the gas twice. This class does not claim to integrate vehicle mechanics.
    """
    def __init__(self, parameters=None, *, pump_parameters=None, initial_mechanical_state=None):
        verify_vendor()
        self.parameters = parameters or PneumaticAuxiliaryParameters()
        if not isinstance(self.parameters, PneumaticAuxiliaryParameters):
            raise TypeError('parameters must be PneumaticAuxiliaryParameters')
        self.pump_parameters = pump_parameters or b10.PumpParameters()
        if not isinstance(self.pump_parameters, b10.PumpParameters):
            raise TypeError('pump_parameters must be PumpParameters')
        p, pump = self.parameters, self.pump_parameters
        self.mechanical_state = {'pantograph_q_rad': 0., 'air_spring_displacements_m': {n: 0. for n in AIR_SPRINGS}}
        self.mechanical_state = self._mechanical_input(initial_mechanical_state)
        positions = self._positions(self.mechanical_state)
        self.initial_mechanical_state = copy.deepcopy(self.mechanical_state)
        vv = [b8.GasVolume('reservoir', p.reservoir_volume_m3, b8.AMBIENT_PA+p.reservoir_initial_gauge_pressure_Pa)]
        oo = [b8.Orifice('reservoir_leak', 'reservoir', None, p.reservoir_leak_area_m2)]
        for name in BRAKES:
            vv.append(b8.GasVolume(name, p.brake_volume_each_axle_m3))
            oo.extend(self._ports(name, p.brake_supply_area_m2, p.brake_exhaust_area_m2, p.brake_leak_area_m2))
        vv.append(b8.GasVolume('pantograph_rotary', p.pantograph_dead_volume_m3, b8.AMBIENT_PA,
                              p.pantograph_effective_area_m2, positions['pantograph_rotary']))
        oo.extend(self._ports('pantograph_rotary', p.pantograph_supply_area_m2, p.pantograph_exhaust_area_m2, p.pantograph_leak_area_m2))
        oo.append(b8.Orifice('pantograph_quick_exhaust', 'pantograph_rotary', None, p.pantograph_quick_exhaust_area_m2))
        for name in AIR_SPRINGS:
            vv.append(b8.GasVolume(name, p.air_spring_dead_volume_each_m3, b8.AMBIENT_PA+p.air_spring_initial_gauge_pressure_Pa,
                                  p.air_spring_area_each_m2, positions[name]))
            oo.extend(self._ports(name, p.air_spring_supply_area_m2, p.air_spring_exhaust_area_m2, p.air_spring_leak_area_m2))
        vv.append(b8.GasVolume('compressor_cylinder', pump.dead_volume_m3, b8.AMBIENT_PA, pump.area_m2))
        oo.extend([b8.Orifice('compressor_intake','compressor_cylinder',None,pump.intake_area_m2),
                   b8.Orifice('compressor_discharge','compressor_cylinder','reservoir',pump.discharge_area_m2,one_way=True)])
        gas = MeteredGasNetwork(vv, oo, max_substep_s=p.gas_max_substep_s)
        self.network = CheckpointAuxiliaryNetwork(gas, pump_parameters=pump)
        # We own auto-switch*voltage control; old B10's auto path ignored voltage.
        self.network.automatic_compressor = False
        self.network.cabinet_external_heat_W = p.cabinet_external_heat_W
        self.commands = {'brake_mode':'hold', 'pantograph_mode':'hold', 'air_spring_mode':'hold',
                         'leaks':{n:0. for n in ('reservoir','brake','pantograph','air_spring')},
                         'coil_failures':{n:False for n in self.network.spools},
                         'compressor_auto':True, 'compressor_voltage_fraction':1., 'compressor_power_failed':False,
                         'compressor_intake_failed_closed':False, 'compressor_discharge_failed_closed':False,
                         'fan_voltage_fractions':{n:1. for n in FANS}, 'fan_power_failed':{n:False for n in FANS}}
        self.steps = 0
        self.last_step = {'dt_s':0., 'node_work_J':{n:0. for n in gas.volumes},
                          'pantograph_generalized_work_J':0., 'air_spring_generalized_work_J':{n:0. for n in AIR_SPRINGS},
                          'work_equivalent_pantograph_torque_Nm':0.,
                          'pantograph_geometry_roundoff_work_J':0.,
                          'work_equivalent_air_spring_forces_N':{n:0. for n in AIR_SPRINGS}}
        self._configuration = self._configuration_record()
        self._configuration_sha256 = digest(self._configuration)
        self._apply_controls(self.commands)

    @property
    def gas(self):
        return self.network.gas

    @staticmethod
    def _ports(name, supply, exhaust, leak):
        return [b8.Orifice(name+'_supply','reservoir',name,supply,one_way=True),
                b8.Orifice(name+'_exhaust',name,None,exhaust),
                b8.Orifice(name+'_leak',name,None,leak)]

    def _positions(self, state):
        p = self.parameters
        q = state['pantograph_q_rad']
        x = p.pantograph_equivalent_lever_m*(math.sin(q+p.pantograph_phase_rad)-math.sin(p.pantograph_phase_rad))
        return {'pantograph_rotary':x, **state['air_spring_displacements_m']}

    def _mechanical_input(self, patch):
        result = copy.deepcopy(self.mechanical_state)
        if patch is None:
            return result
        if not isinstance(patch, Mapping) or set(patch)-{'pantograph_q_rad','secondary_z_m','air_spring_displacements_m'}:
            raise ValueError('Unknown mechanical state key')
        if 'pantograph_q_rad' in patch:
            result['pantograph_q_rad'] = number(patch['pantograph_q_rad'],'pantograph_q_rad',-math.tau,math.tau)
        if 'secondary_z_m' in patch and 'air_spring_displacements_m' in patch:
            raise ValueError('Supply shared secondary_z_m or individual displacements, not both')
        if 'secondary_z_m' in patch:
            result['air_spring_displacements_m'] = {n:number(patch['secondary_z_m'],'secondary_z_m',-1.,1.) for n in AIR_SPRINGS}
        if 'air_spring_displacements_m' in patch:
            values = patch['air_spring_displacements_m']
            if not isinstance(values, Mapping) or set(values)-set(AIR_SPRINGS):
                raise ValueError('Unknown air-spring displacement')
            for n,x in values.items():
                result['air_spring_displacements_m'][n] = number(x,n,-1.,1.)
        for n,z in result['air_spring_displacements_m'].items():
            if self.parameters.air_spring_dead_volume_each_m3+self.parameters.air_spring_area_each_m2*z <= 1e-12:
                raise ValueError('Air-spring chamber volume must exceed numerical minimum 1e-12 m3: '+n)
        return result

    def _command_input(self, patch):
        result = copy.deepcopy(self.commands)
        if patch is None:
            return result
        if not isinstance(patch, Mapping) or set(patch)-set(result):
            raise ValueError('Unknown pneumatic auxiliary command')
        modes = {'brake_mode':('apply','hold','release'),'pantograph_mode':(*MODES,'quick_vent'),'air_spring_mode':MODES}
        for key,value in patch.items():
            if key in modes:
                if value not in modes[key]:
                    raise ValueError('Invalid mode '+key)
                result[key] = value
            elif type(result[key]) is bool:
                if type(value) is not bool:
                    raise ValueError(key+' requires boolean')
                result[key] = value
            elif type(result[key]) is dict:
                if not isinstance(value, Mapping) or set(value)-set(result[key]):
                    raise ValueError('Unknown key in '+key)
                for n,v in value.items():
                    if type(result[key][n]) is bool:
                        if type(v) is not bool:
                            raise ValueError(key+'.'+n+' requires boolean')
                        result[key][n] = v
                    else:
                        result[key][n] = number(v,key+'.'+n,0.,1.)
            else:
                result[key] = number(value,key,0.,1.)
        return result

    def _apply_controls(self, controls):
        net = self.network
        for n,failed in controls['coil_failures'].items():
            net.set_coil_failure(n, failed)
        net.pump.motor.power_failed = controls['compressor_power_failed']
        net.pump.intake_failed_closed = controls['compressor_intake_failed_closed']
        net.pump.discharge_failed_closed = controls['compressor_discharge_failed_closed']
        for n,fan in net.fans.items():
            fan.motor.power_failed = controls['fan_power_failed'][n]
        ports = {n:0. for n in net.commands if n not in ('compressor_intake','compressor_discharge')}
        for names,mode in ((BRAKES, {'apply':'fill','release':'vent','hold':'hold'}[controls['brake_mode']]),
                           (('pantograph_rotary',),controls['pantograph_mode']), (AIR_SPRINGS,controls['air_spring_mode'])):
            for name in names:
                ports[name+'_supply'] = float(mode=='fill')
                ports[name+'_exhaust'] = float(mode=='vent')
        ports['pantograph_quick_exhaust'] = float(controls['pantograph_mode']=='quick_vent')
        for category,names in (('reservoir',('reservoir',)),('brake',BRAKES),('pantograph',('pantograph_rotary',)),('air_spring',AIR_SPRINGS)):
            for name in names:
                ports[name+'_leak'] = controls['leaks'][category]
        ports.update(controls['fan_voltage_fractions'])
        ports['compressor'] = controls['compressor_voltage_fraction']
        return ports

    def forces(self):
        p, state = self.parameters, self.mechanical_state
        pressures = self.gas.pressures_Pa()
        jacobian = p.pantograph_equivalent_lever_m*math.cos(state['pantograph_q_rad']+p.pantograph_phase_rad)
        f = (pressures['pantograph_rotary']-b8.AMBIENT_PA)*p.pantograph_effective_area_m2
        air = {n:(pressures[n]-b8.AMBIENT_PA)*p.air_spring_area_each_m2 for n in AIR_SPRINGS}
        return {'pantograph_torque_Nm':f*jacobian,'pantograph_dx_dq_m_per_rad':jacobian,
                'pantograph_equivalent_force_N':f,'air_spring_forces_N':air,
                'secondary_z_force_N':math.fsum(air.values())}

    def step(self, dt_s, mechanical_state=None, commands=None, *, snapshot=True):
        h_total = number(dt_s,'dt_s',0.,1.)
        end = self._mechanical_input(mechanical_state)
        controls = self._command_input(commands)
        if h_total == 0 and end != self.mechanical_state:
            raise ValueError('Mechanical state change requires positive dt')
        # All user inputs validated before any commands, faults or states change.
        begin = copy.deepcopy(self.mechanical_state)
        old_forces = self.forces()
        initial_pantograph_x = self.gas.volumes['pantograph_rotary'].displacement_m
        edge_before = dict(self.gas.edge_mass_kg)
        ports = self._apply_controls(controls)
        self.commands = controls
        if h_total == 0:
            self.network.update(0., valve_commands=ports, snapshot=False)
            return self.snapshot() if snapshot else None
        self.gas.node_interval_work_J = {n:0. for n in self.gas.volumes}
        count = max(1, math.ceil(h_total/min(self.parameters.gas_max_substep_s,self.pump_parameters.maximum_step_s)))
        h = h_total/count
        for k in range(count):
            fraction = (k+1)/count
            observed = {'pantograph_q_rad':begin['pantograph_q_rad']+(end['pantograph_q_rad']-begin['pantograph_q_rad'])*fraction,
                        'air_spring_displacements_m':{n:begin['air_spring_displacements_m'][n]+(end['air_spring_displacements_m'][n]-begin['air_spring_displacements_m'][n])*fraction for n in AIR_SPRINGS}}
            ports['compressor'] = controls['compressor_voltage_fraction']*(self.network.pump.switch() if controls['compressor_auto'] else 1.)
            self.network.update(h, self._positions(observed), ports, snapshot=False)
        self.gas.last_flows_kg_s = {n:(dm-edge_before[n])/h_total for n,dm in self.gas.edge_mass_kg.items()}
        self.mechanical_state = end
        self.steps += 1
        work = dict(self.gas.node_interval_work_J)
        now = self.forces()
        dq = end['pantograph_q_rad']-begin['pantograph_q_rad']
        dx = self.gas.volumes['pantograph_rotary'].displacement_m-initial_pantograph_x
        # Stable divided difference: Fbar uses exactly the displacement that
        # produced gas work; the sine map derivative avoids subtracting two
        # nearby absolute sin coordinates before dividing by tiny dq.
        half_dq = .5*dq
        sinc = math.sin(half_dq)/half_dq if half_dq else 1.
        dx_dq = self.parameters.pantograph_equivalent_lever_m*math.cos(.5*(end['pantograph_q_rad']+begin['pantograph_q_rad'])+self.parameters.pantograph_phase_rad)*sinc
        if dx != 0.:
            tq = (work['pantograph_rotary']/dx)*dx_dq
        elif dq != 0. and work['pantograph_rotary'] != 0.:
            tq = work['pantograph_rotary']/dq
        else:
            tq = .5*(old_forces['pantograph_torque_Nm']+now['pantograph_torque_Nm'])
        air = {}
        for n in AIR_SPRINGS:
            dz = end['air_spring_displacements_m'][n]-begin['air_spring_displacements_m'][n]
            air[n] = work[n]/dz if abs(dz)>1e-14 else .5*(old_forces['air_spring_forces_N'][n]+now['air_spring_forces_N'][n])
        self.last_step = {'dt_s':h_total,'node_work_J':work,'pantograph_generalized_work_J':work['pantograph_rotary'],
                          'air_spring_generalized_work_J':{n:work[n] for n in AIR_SPRINGS},
                          'work_equivalent_pantograph_torque_Nm':tq,'work_equivalent_air_spring_forces_N':air,
                          'pantograph_geometry_roundoff_work_J':tq*dq-work['pantograph_rotary']}
        return self.snapshot() if snapshot else None

    def _configuration_record(self):
        config = copy.deepcopy(self.network.configuration())
        for o in config['orifices'].values():
            o.pop('opening')
        dynamic = config['dynamic_metering_ports']
        for key in ('requested_coil_force_fractions','coil_failures','audit'):
            dynamic.pop(key)
        config['auxiliary_model'].pop('automatic_compressor')
        return {'schema':'gtsd-pneumatic-aux-configuration-v1','provenance':PROVENANCE,
                'parameters':asdict(self.parameters),'network':config,'vendor_sha256':dict(VENDOR_HASHES),
                'initial_mechanical_state':self.initial_mechanical_state,
                'force_ports':{'pantograph':'q is actual panto_lower_hinge angle; x=l*(sin(q+phi)-sin(phi)); V=V0+A*x; tau=(p-pa)*A*l*cos(q+phi)',
                               'air_springs':'each actual positive-extension z: V=V0+A*z, force=(p-pa)*A; shared secondary_z force=sum(left,right)',
                               'brakes':'fixed gas volume; pressure exported to existing clamp-force equivalent, no piston stroke/work'},
                'limits':['No actual B32 or bellows geometry claimed; no old 0.16 m two-arm mechanism',
                          'Air springs start at ambient by default: no added preload; retaining old passive springs is a declared parallel equivalent',
                          'Auxiliary pump, fans and valve spools use fixed external mount; vehicle mount mass/reactions are not coupled',
                          'Thermal nodes are diagnostic; no gas-temperature or motor-resistance feedback',
                          'Caller must use observed mechanical endpoints and reconcile force/port work before accepting a coupled step']}

    def configuration(self):
        return copy.deepcopy(self._configuration) | {'configuration_sha256':self._configuration_sha256}

    def snapshot(self):
        result = self.network.snapshot()
        result['pressures_Pa'] = dict(result['pressure_Pa'])
        result['mechanical_state'] = copy.deepcopy(self.mechanical_state)
        result['commands'] = copy.deepcopy(self.commands)
        result['force_ports'] = self.forces()
        result['last_step'] = copy.deepcopy(self.last_step)
        result['node_useful_work_J'] = dict(self.gas.node_useful_work_J)
        result['node_work_ledger_residual_J'] = math.fsum(self.gas.node_useful_work_J.values())-self.gas.ledger['useful_mechanical_work_J']
        result['steps'] = self.steps
        result['configuration_sha256'] = self._configuration_sha256
        return result

    def _state(self):
        gas,net = self.gas,self.network
        gas_keys = ('time_s','ledger','edge_mass_kg','compressor_mass_kg','last_flows_kg_s','equalization_caps','substeps','node_useful_work_J','node_interval_work_J')
        return {'gas':{k:copy.deepcopy(getattr(gas,k)) for k in gas_keys},
                'volumes':{n:{'mass_kg':v.mass_kg,'displacement_m':v.displacement_m} for n,v in gas.volumes.items()},
                'orifices':{n:o.opening for n,o in gas.orifices.items()},
                'network_commands':dict(net.commands),
                'spools':{n:{k:copy.deepcopy(v) for k,v in vars(s).items() if k!='parameters'} for n,s in net.spools.items()},
                'pump':{k:copy.deepcopy(v) for k,v in vars(net.pump).items() if k not in ('gas','parameters','motor')},
                'pump_motor':{k:v for k,v in vars(net.pump.motor).items() if k!='parameters'},
                'fans':{n:{'state':{k:v for k,v in vars(f).items() if k not in ('parameters','motor')},
                           'motor':{k:v for k,v in vars(f.motor).items() if k!='parameters'}} for n,f in net.fans.items()},
                'thermal':{n:{k:getattr(t,k) for k in ('temperature_K','heat_input_J','heat_to_ambient_J')} for n,t in net.thermal.items()},
                'commands':copy.deepcopy(self.commands),'mechanical_state':copy.deepcopy(self.mechanical_state),
                'steps':self.steps,'last_step':copy.deepcopy(self.last_step)}

    def checkpoint_state(self):
        payload = {'schema':SCHEMA,'configuration_sha256':self._configuration_sha256,'state':self._state()}
        return payload | {'sha256':digest(payload)}

    def checkpoint_bytes(self):
        """Compact data-only zlib JSON; no pickle, classes, code or file references."""
        return zlib.compress(canonical(self.checkpoint_state()), level=6)

    def _load_state(self, state):
        net,gas = self.network,self.gas
        for k,v in state['gas'].items():
            setattr(gas,k,v)
        for n,fields in state['volumes'].items():
            for k,v in fields.items():
                setattr(gas.volumes[n],k,v)
        for n,v in state['orifices'].items():
            gas.orifices[n].opening = v
        net.commands = state['network_commands']
        for n,fields in state['spools'].items():
            vars(net.spools[n]).update(fields)
        vars(net.pump).update(state['pump'])
        vars(net.pump.motor).update(state['pump_motor'])
        for n,fields in state['fans'].items():
            vars(net.fans[n]).update(fields['state'])
            vars(net.fans[n].motor).update(fields['motor'])
        for n,fields in state['thermal'].items():
            vars(net.thermal[n]).update(fields)
        for k in ('commands','mechanical_state','steps','last_step'):
            setattr(self,k,state[k])

    def _validate_restored(self):
        self._command_input(self.commands)
        self._mechanical_input(self.mechanical_state)
        positions = self._positions(self.mechanical_state)
        positions['compressor_cylinder'] = self.network.pump.displacement_m
        for n,v in self.gas.volumes.items():
            if v.mass_kg <= 0 or v.volume_m3 <= 0:
                raise ValueError('Checkpoint requires positive mass and volume')
            if n in positions and abs(v.displacement_m-positions[n]) > 1e-13:
                raise ValueError('Checkpoint gas/mechanical displacement mismatch')
        if self.gas.time_s < 0 or self.steps < 0 or self.gas.substeps < 0 or self.gas.equalization_caps < 0:
            raise ValueError('Negative time/count in checkpoint')
        for u in list(self.network.commands.values())+[o.opening for o in self.gas.orifices.values()]:
            number(u,'checkpoint opening',0.,1.)
        for s in self.network.spools.values():
            number(s.command,'checkpoint spool command',0.,1.)
        if any(t.temperature_K<=0 for t in self.network.thermal.values()):
            raise ValueError('Nonpositive checkpoint temperature')
        c = self.commands
        if any(s.coil_failed != c['coil_failures'][n] for n,s in self.network.spools.items()):
            raise ValueError('Checkpoint coil fault mismatch')
        if self.network.pump.motor.power_failed != c['compressor_power_failed'] or any(f.motor.power_failed != c['fan_power_failed'][n] for n,f in self.network.fans.items()):
            raise ValueError('Checkpoint power fault mismatch')
        if self.network.pump.intake_failed_closed != c['compressor_intake_failed_closed'] or self.network.pump.discharge_failed_closed != c['compressor_discharge_failed_closed']:
            raise ValueError('Checkpoint compressor check-valve fault mismatch')
        audit = self.gas.audit()
        if abs(audit['mass_balance_residual_kg'])>1e-10 or abs(audit['gas_energy_balance_residual_J'])>1e-5 or abs(audit['exergy_balance_residual_J'])>1e-5:
            raise ValueError('Checkpoint gas conservation ledger mismatch')
        if abs(self.network.pump.audit()['mechanical_balance_residual_J'])>1e-5 or abs(self.network.spool_audit()['balance_residual_J'])>1e-7:
            raise ValueError('Checkpoint mechanical ledger mismatch')
        # Must also be finite after derived quantities, not just finite inputs.
        canonical(self.snapshot())

    def restore(self, checkpoint):
        if isinstance(checkpoint, bytes):
            decoder = zlib.decompressobj()
            raw = decoder.decompress(checkpoint, 1_000_001)
            if len(raw)>1_000_000 or not decoder.eof or decoder.unused_data:
                raise ValueError('Invalid or oversized compressed checkpoint')
            checkpoint = json.loads(raw)
        if type(checkpoint) is not dict or set(checkpoint)!={'schema','configuration_sha256','state','sha256'}:
            raise ValueError('Checkpoint schema mismatch')
        payload = {k:v for k,v in checkpoint.items() if k!='sha256'}
        if checkpoint['schema']!=SCHEMA or checkpoint['configuration_sha256']!=self._configuration_sha256 or checkpoint['sha256']!=digest(payload):
            raise ValueError('Checkpoint identity or checksum mismatch')
        state = _typed_like(checkpoint['state'], self._state())
        candidate = copy.deepcopy(self)
        candidate._load_state(state)
        candidate._validate_restored()
        self._load_state(state)  # validated scalar/dict assignment; keep gas/network object identity
        return None

# Re-export the reviewed parameter type for explicit alternative pump tests.
PumpParameters = b10.PumpParameters
