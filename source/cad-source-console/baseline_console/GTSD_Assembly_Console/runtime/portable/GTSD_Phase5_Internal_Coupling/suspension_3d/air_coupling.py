"""Optional work-consistent coupling to pneumatic_aux's finite-volume plant.

No 40 kN/m parallel support: air-enabled XML replaces it with a declared backup
spring and explicit remaining mechanical preload. Gas displacements are actual
native spatial-tendon lengths, including all six secondary coordinates.
"""
import copy
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from .model import preload_allocation

AIR_NAMES=('air_spring_left','air_spring_right')

def air_spring_kinematics(m,d):
    result={}
    for name in AIR_NAMES:
        tid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_TENDON,name)
        if tid<0: raise ValueError('Air-enabled XML required; missing '+name)
        lo=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,name+'_lower')
        up=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,name+'_upper')
        delta=d.site_xpos[up]-d.site_xpos[lo]
        result[name]={'length_m':float(d.ten_length[tid]),'reference_length_m':float(m.tendon_length0[tid]),
            'displacement_m':float(d.ten_length[tid]-m.tendon_length0[tid]),
            'velocity_mps':float(d.ten_velocity[tid]),'unit_lower_to_upper':(delta/np.linalg.norm(delta)).tolist(),
            'lower_world_m':d.site_xpos[lo].tolist(),'upper_world_m':d.site_xpos[up].tolist()}
    return result

def observed_state(m,d):
    return {'air_spring_displacements_m':{k:v['displacement_m'] for k,v in air_spring_kinematics(m,d).items()}}

def apply_air_forces(m,d,forces):
    """Positive force extends the native tendon; actual sign verified in tests."""
    for name in AIR_NAMES:
        aid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,name+'_force')
        if aid<0: raise ValueError('Missing air motor '+name)
        force=float(forces[name])
        if not np.isfinite(force): raise ValueError('Nonfinite air force')
        d.ctrl[aid]=force

def make_air_plant(m,d,xml,suspension_parameters=None,area_each_m2=.0004):
    """Initialize both chamber pressures from explicit mass/preload accounting."""
    from pneumatic_aux.plant import PneumaticAuxiliaryParameters,PneumaticAuxiliaryPlant
    allocation=preload_allocation(xml,suspension_parameters)
    declared=ET.fromstring(xml).find("custom/numeric[@name='suspension_3d_preload_N']")
    enabled=ET.fromstring(xml).find("custom/numeric[@name='suspension_3d_air_enabled']")
    if declared is None or enabled is None or enabled.get('data')!='1':
        raise ValueError('make_air_plant requires the matching air-enabled augmented XML')
    values=np.fromstring(declared.get('data'),sep=' ')
    if len(values)!=3 or abs(values[0]-allocation['secondary_mechanical_preload_N'])>1e-8 or abs(values[1]-allocation['air_required_force_N'])>1e-8:
        raise ValueError('Pass exact suspension parameters matching XML preload allocation; refusing double support')
    required=allocation['air_required_force_N']
    if area_each_m2<=0:raise ValueError('Area must be positive')
    p=PneumaticAuxiliaryParameters(air_spring_area_each_m2=area_each_m2,
        air_spring_initial_gauge_pressure_Pa=required/(2*area_each_m2))
    plant=PneumaticAuxiliaryPlant(p,initial_mechanical_state=observed_state(m,d))
    plant.step(0.,commands={'compressor_auto':False,'compressor_voltage_fraction':0.,
        'fan_voltage_fractions':{'compressor_fan':0.,'bay_fan':0.}})
    return plant,allocation

class AirSpringCoupler:
    """One MuJoCo step plus finite-volume endpoint-work fixed-point correction.

    All plant trial states are rolled back until accepted. Existing non-air
    actuator controls and external body forces are left in the model state.
    This adapter only couples the two gas springs; console-wide panto/brake/pump
    coupling must be performed by its owner, without stepping this plant twice.
    The returned gas/mechanical port residual is separate from MuJoCo's own
    time-integration/contact dissipation; this method does not erase that error.
    """
    def __init__(self,m,d,plant,force_tolerance_N=1e-7,work_tolerance_J=1e-10,max_iterations=12):
        self.m=m;self.plant=plant;self.force_tolerance_N=force_tolerance_N
        self.work_tolerance_J=work_tolerance_J;self.max_iterations=max_iterations
        self.state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
        self.state=np.zeros(mujoco.mj_stateSize(m,self.state_spec))
        expected=observed_state(m,d)['air_spring_displacements_m']
        if any(abs(expected[n]-plant.mechanical_state['air_spring_displacements_m'][n])>1e-10 for n in AIR_NAMES):
            raise ValueError('Plant initial volumes must match actual MuJoCo endpoint lengths')
        self.cumulative_gas_work_J=0.;self.cumulative_actuator_endpoint_work_J=0.
        self.last={}

    def _check_trial(self, trial, expected_time, baseline_warnings):
        arrays=(trial.qpos,trial.qvel,trial.qacc,trial.ctrl,trial.qfrc_actuator,trial.qfrc_constraint,trial.energy)
        if any(not np.isfinite(a).all() for a in arrays):
            raise RuntimeError('Nonfinite MuJoCo air-spring trial; step rejected')
        warnings=tuple(int(w.number) for w in trial.warning)
        if warnings!=baseline_warnings:
            raise RuntimeError('MuJoCo warning during air-spring trial; step rejected')
        if not np.isfinite(trial.time) or abs(float(trial.time)-expected_time)>1e-12*max(1.,abs(expected_time)):
            raise RuntimeError('MuJoCo time failed to advance exactly once; step rejected')

    def step(self,d,commands=None):
        # Trials are genuinely isolated MjData copies. mjSTATE_INTEGRATION alone
        # cannot restore warning/timer/contact diagnostics after an engine reset,
        # so the caller's MjData is never touched before a validated acceptance.
        m=self.m
        baseline_warnings=tuple(int(w.number) for w in d.warning)
        begin_time=float(d.time);target_time=begin_time+float(m.opt.timestep)
        if not np.isfinite(begin_time) or any(not np.isfinite(a).all() for a in (d.qpos,d.qvel,d.ctrl,d.xfrc_applied)):
            raise RuntimeError('Nonfinite initial MuJoCo state; step rejected')
        base=copy.copy(d);mujoco.mj_forward(m,base)
        self._check_trial(base,begin_time,baseline_warnings)
        begin=air_spring_kinematics(m,base)
        checkpoint=self.plant.checkpoint_state();forces=self.plant.forces()['air_spring_forces_N']
        try:
            for iteration in range(1,self.max_iterations+1):
                trial=copy.copy(base)
                apply_air_forces(m,trial,forces);mujoco.mj_forward(m,trial)
                self._check_trial(trial,begin_time,baseline_warnings)
                mujoco.mj_step(m,trial);mujoco.mj_forward(m,trial)
                self._check_trial(trial,target_time,baseline_warnings)
                end=air_spring_kinematics(m,trial)
                if iteration>1:self.plant.restore(checkpoint)
                self.plant.step(float(m.opt.timestep),observed_state(m,trial),commands,snapshot=False)
                equivalent=self.plant.last_step['work_equivalent_air_spring_forces_N']
                work=self.plant.last_step['air_spring_generalized_work_J']
                mechanical={n:forces[n]*(end[n]['length_m']-begin[n]['length_m']) for n in AIR_NAMES}
                residual={n:mechanical[n]-work[n] for n in AIR_NAMES}
                error=max(abs(equivalent[n]-forces[n]) for n in AIR_NAMES)
                if error<=self.force_tolerance_N and max(abs(x) for x in residual.values())<=self.work_tolerance_J:
                    gas_total=self.cumulative_gas_work_J+sum(work.values())
                    mechanical_total=self.cumulative_actuator_endpoint_work_J+sum(mechanical.values())
                    last={'iterations':iteration,'force_iteration_error_N':error,
                        'gas_work_J':dict(work),'actuator_endpoint_work_J':mechanical,
                        'port_work_residual_J':residual,'used_force_N':dict(forces),
                        'endpoint_kinematics':end,'cumulative_gas_work_J':gas_total,
                        'cumulative_actuator_endpoint_work_J':mechanical_total,
                        'accepted_time_s':target_time,'new_solver_warnings':0}
                    # Commit only already-validated state and controls. Complete
                    # integration state includes warm start and user force inputs.
                    mujoco.mj_getState(m,trial,self.state,self.state_spec)
                    mujoco.mj_setState(m,d,self.state,self.state_spec);mujoco.mj_forward(m,d)
                    self.cumulative_gas_work_J=gas_total
                    self.cumulative_actuator_endpoint_work_J=mechanical_total
                    self.last=last
                    return copy.deepcopy(last)
                forces=dict(equivalent)
            raise RuntimeError('Air-spring work corrector did not converge; step rejected')
        except Exception:
            # Original d and own accepted ledger remain untouched on every
            # rejected trial, including BADQVEL/BADQACC engine auto-resets.
            self.plant.restore(checkpoint)
            raise
