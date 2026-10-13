"""Uncalibrated finite-volume brake and RL motor equivalents, entirely software.
No prescribed pressure/position/speed and no physical device or network I/O.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import hashlib, json, math, sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'vendor/b8'))
sys.path.insert(0, str(ROOT/'vendor/b9'))
from pneumatics import GasVolume, Orifice, PneumaticNetwork, AMBIENT_PA
from valve_dynamics import DynamicValveNetwork, SpoolParameters

VENDOR_HASHES = {
 'vendor/b8/pneumatics.py': 'a96d81f70e347319ad055c0fea4aa06933f762e01910522377c81a16dc6a1915',
 'vendor/b9/valve_dynamics.py': 'acd3d855c74db93793022f4f39c6acec821c1a51f168b23d4c56f0f3b4908a50',
}

def verify_vendor():
    for rel, digest in VENDOR_HASHES.items():
        if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Pinned vendor source changed: '+rel)

@dataclass(frozen=True)
class DriveParameters:
    bus_voltage_V: float = 6.0
    resistance_ohm: float = 2.0
    inductance_H: float = 0.02
    torque_constant_Nm_A: float = 0.12
    back_emf_constant_V_s_rad: float = 0.12
    gear_ratio_motor_per_axle: float = 3.0
    rotor_inertia_kgm2: float = 0.0003
    rotor_viscous_friction_Nms_rad: float = 0.00003
    freewheel_diode_drop_V: float = 0.7

    def __post_init__(self):
        if not all(math.isfinite(v) and v>0 for v in asdict(self).values()):
            raise ValueError('Positive finite motor equivalent parameters required')
        if self.torque_constant_Nm_A != self.back_emf_constant_V_s_rad:
            raise ValueError('SI kt must equal ke for the lossless electromagnetic converter')

class Motor:
    """Ideal rigid gear + reflected rotor inertia (in MuJoCo), and explicit RL.

    Powered: L di/dt = V - R i - ke g omega. Bidirectional ideal DC supply.
    Supply loss: diode flyback conducts only while stored current remains;
    then the winding becomes open circuit. Zero voltage shorting is NOT coasting.
    Preview uses implicit midpoint current, actual axle-speed feedback, and an
    analytically located zero-current flyback endpoint. No guessed wheel speed.
    """
    def __init__(self, parameters=None):
        self.p=parameters or DriveParameters()
        self.current_A=0.0
        self.ledger={k:0.0 for k in ('electrical_supply_work_J','copper_loss_J',
          'freewheel_diode_loss_J','electromagnetic_conversion_work_J',
          'actual_axle_mechanical_work_J','coupling_split_residual_J')}

    def preview(self, h, axle_speed_rad_s, enabled):
        if h<=0 or not math.isfinite(h) or not math.isfinite(axle_speed_rad_s):
            raise ValueError('Finite speed and positive timestep required')
        p=self.p; i0=self.current_A; emf=p.back_emf_constant_V_s_rad*p.gear_ratio_motor_per_axle*axle_speed_rad_s
        v=p.bus_voltage_V if enabled else 0.0
        elapsed=h; diode=0.0; mode='powered' if enabled else 'flyback'
        if not enabled and i0==0:
            return dict(current_A=0.0, current_mean_A=0.0, torque_Nm=0.0, emf_V=emf,
                        winding_terminal_voltage_V=emf, electrical_supply_work_J=0.0,
                        copper_loss_J=0.0, freewheel_diode_loss_J=0.0,
                        electromagnetic_conversion_work_J=0.0, magnetic_energy_J=0.0,
                        electrical_balance_residual_J=0.0, conduction_time_s=0.0, mode='open_circuit')
        if not enabled: diode=math.copysign(p.freewheel_diode_drop_V,i0)
        i1=(i0+h/p.inductance_H*(v-diode-emf-p.resistance_ohm*i0/2))/(1+h*p.resistance_ohm/(2*p.inductance_H))
        if not enabled and i1*i0<=0:
            elapsed=p.inductance_H*i0/(diode+emf+p.resistance_ohm*i0/2)
            elapsed=min(h,max(0.0,elapsed));i1=0.0;mode='flyback_then_open'
        im=(i0+i1)/2; integrated_current=im*elapsed
        supplied=v*integrated_current
        copper=p.resistance_ohm*im*im*elapsed
        diode_loss=diode*integrated_current
        conversion=emf*integrated_current
        magnetic=.5*p.inductance_H*i1*i1
        residual=magnetic-.5*p.inductance_H*i0*i0-supplied+copper+diode_loss+conversion
        return dict(current_A=i1,current_mean_A=integrated_current/h,
          torque_Nm=p.gear_ratio_motor_per_axle*p.torque_constant_Nm_A*integrated_current/h,
          emf_V=emf,winding_terminal_voltage_V=v-diode if elapsed==h else emf,
          electrical_supply_work_J=supplied,copper_loss_J=copper,
          freewheel_diode_loss_J=diode_loss,electromagnetic_conversion_work_J=conversion,
          magnetic_energy_J=magnetic,electrical_balance_residual_J=residual,
          conduction_time_s=elapsed,mode=mode)

    def commit(self, state, mechanical_work_J):
        self.current_A=state['current_A']
        for key in ('electrical_supply_work_J','copper_loss_J','freewheel_diode_loss_J','electromagnetic_conversion_work_J'):
            self.ledger[key]+=state[key]
        self.ledger['actual_axle_mechanical_work_J']+=mechanical_work_J
        self.ledger['coupling_split_residual_J']+=mechanical_work_J-state['electromagnetic_conversion_work_J']

    def audit(self):
        l=self.ledger;p=self.p;magnetic=.5*p.inductance_H*self.current_A**2
        return dict(**l,current_A=self.current_A,magnetic_energy_J=magnetic,
          electrical_balance_residual_J=magnetic-l['electrical_supply_work_J']+l['copper_loss_J']+l['freewheel_diode_loss_J']+l['electromagnetic_conversion_work_J'])

@dataclass(frozen=True)
class BrakeParameters:
    reservoir_volume_m3: float=0.002
    reservoir_initial_gauge_pressure_Pa: float=500000.0
    fixed_chamber_volume_each_axle_m3: float=1e-5
    supply_orifice_area_m2: float=5e-8
    exhaust_orifice_area_m2: float=1.5e-7
    effective_clamp_area_each_axle_m2: float=8e-5
    equivalent_friction_interfaces: float=2.0
    friction_coefficient: float=0.3
    effective_friction_radius_m: float=0.065
    friction_regularization_speed_rad_s: float=0.1
    gas_substep_s: float=0.0005

    def __post_init__(self):
        if not all(math.isfinite(v) and v>0 for v in asdict(self).values()):
            raise ValueError('Positive finite brake equivalents required')

class FixedVolumeBrake:
    def __init__(self, parameters=None):
        verify_vendor();self.p=parameters or BrakeParameters();p=self.p
        volumes=[GasVolume('reservoir',p.reservoir_volume_m3,AMBIENT_PA+p.reservoir_initial_gauge_pressure_Pa)]
        ports=[]
        for n in ('brake_axle_1','brake_axle_2'):
            # Explicitly zero piston area: this is a fixed-volume force equivalent.
            volumes.append(GasVolume(n,p.fixed_chamber_volume_each_axle_m3,AMBIENT_PA,piston_area_m2=0.0))
            ports.extend([Orifice(n+'_supply','reservoir',n,p.supply_orifice_area_m2,one_way=True),
                          Orifice(n+'_exhaust',n,None,p.exhaust_orifice_area_m2)])
        self.network=DynamicValveNetwork(PneumaticNetwork(volumes,ports,max_substep_s=p.gas_substep_s))
        self.brake_heat_J=0.0

    def advance(self,h,supply,exhaust):
        if supply and exhaust:raise ValueError('Supply and exhaust command conflict')
        self.network.update(h,valve_commands={n:float(supply if n.endswith('_supply') else exhaust)
          for n in self.network.spools},snapshot=False)

    def force_torque(self,axle,omega,pressure_Pa=None):
        p=self.p;pressure=self.network.pressure_Pa(f'brake_axle_{axle}') if pressure_Pa is None else pressure_Pa
        clamp=max(0.0,pressure-AMBIENT_PA)*p.effective_clamp_area_each_axle_m2
        cap=clamp*p.equivalent_friction_interfaces*p.friction_coefficient*p.effective_friction_radius_m
        torque=-cap*math.tanh(omega/p.friction_regularization_speed_rad_s)
        return dict(gauge_pressure_Pa=pressure-AMBIENT_PA,clamp_force_equivalent_N=clamp,
                    friction_torque_limit_Nm=cap,torque_Nm=torque,power_dissipated_W=-torque*omega)

    def snapshot(self):
        return self.network.snapshot()|{'model_scope':'fixed-volume clamp-force equivalent; no cylinder travel or piston work',
          'brake_heat_from_wheel_rotation_J':self.brake_heat_J}


def command_at(case,t):
    if case not in ('traction_supply_exhaust','power_loss_coast'):raise ValueError(case)
    return {'traction_enable':.3<=t<2.5,
            'brake_supply':case=='traction_supply_exhaust' and 2.6<=t<4.6,
            'brake_exhaust':case=='traction_supply_exhaust' and t>=4.6}


class CouplingStepRejected(ValueError):
    """A trial step cannot be accepted under the discrete energy contract."""
    def __init__(self, diagnostics):
        self.diagnostics=diagnostics
        super().__init__('Discrete coupling step rejected: '+json.dumps(diagnostics,sort_keys=True))


def validate_discrete_coupling_step(dt_s, used_midpoint_speed_rad_s,
        actual_midpoint_speed_rad_s, brake_torques_Nm, *,
        speed_tolerance_rad_s=1e-7, positive_brake_work_tolerance_J=1e-12):
    """Fail closed on unresolved co-simulation or positive discrete brake work.

    Call before committing motor/heat ledgers or publishing an endpoint. The
    used feedback must be copied BEFORE the final trial, not a guess overwritten
    with the observed result afterward. Tolerances are numerical SI thresholds,
    not physical calibration. This validator itself never mutates state.
    """
    used=list(map(float,used_midpoint_speed_rad_s))
    actual=list(map(float,actual_midpoint_speed_rad_s))
    torques=list(map(float,brake_torques_Nm))
    if (not used or len(used)!=len(actual) or len(used)!=len(torques) or
        not all(math.isfinite(x) for x in used+actual+torques+[dt_s,speed_tolerance_rad_s,positive_brake_work_tolerance_J]) or
        dt_s<=0 or speed_tolerance_rad_s<0 or positive_brake_work_tolerance_J<0):
        raise ValueError('Finite matching axle arrays, positive dt, and nonnegative tolerances required')
    residual=max(abs(a-u) for a,u in zip(actual,used))
    works=[tau*omega*dt_s for tau,omega in zip(torques,actual)]
    diagnostics={'feedback_residual_rad_s':residual,
      'speed_tolerance_rad_s':speed_tolerance_rad_s,'per_axle_brake_work_J':works,
      'positive_brake_work_tolerance_J':positive_brake_work_tolerance_J,
      'feedback_converged':residual<=speed_tolerance_rad_s,
      'discrete_brake_passive':all(work<=positive_brake_work_tolerance_J for work in works)}
    if not diagnostics['feedback_converged'] or not diagnostics['discrete_brake_passive']:
        raise CouplingStepRejected(diagnostics)
    return diagnostics
