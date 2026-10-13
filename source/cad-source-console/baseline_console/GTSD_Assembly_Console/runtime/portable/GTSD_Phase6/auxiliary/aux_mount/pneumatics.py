"""B8/B9 gas/valves and B10 electrical/thermal laws with observed native motion.

No call to ReciprocatingPump.advance_geometry or Fan.advance is permitted.
Copies of native endpoints are observations required by gas volume integration;
only MuJoCo integrates their coordinates and velocities.
"""
from __future__ import annotations
import copy, math
from collections.abc import Mapping
from . import bootstrap
from brake_integrated.pneumatics import (PneumaticAuxiliaryPlant,PneumaticAuxiliaryParameters,CheckpointAuxiliaryNetwork,
    AIR_SPRINGS,BRAKES,FANS,number,digest,canonical)
from pneumatic_aux.kernels import b8,b9,b10
from .runtime import ROTORS,FAN_KEYS

DEFAULT_NATIVE={'angle_rad':{n:0. for n in ROTORS},'omega_rad_s':{n:0. for n in ROTORS},
                'piston_displacement_m':0.,'piston_velocity_m_s':0.}

def validate_native(value):
    if type(value) is not dict or set(value)!=set(DEFAULT_NATIVE):raise ValueError('Native auxiliary observation fields mismatch')
    out=copy.deepcopy(value)
    for field in ('angle_rad','omega_rad_s'):
        if type(value[field]) is not dict or set(value[field])!=set(ROTORS):raise ValueError('Native rotor observation keys mismatch')
        out[field]={n:number(v,field+'.'+n) for n,v in value[field].items()}
    for field in ('piston_displacement_m','piston_velocity_m_s'):out[field]=number(value[field],field)
    return out

def fan_parameter_map(value=None):
    """Normalize shared or per-fan research parameters without changing defaults."""
    def one(p):
        if p is None:return b10.FanParameters()
        if isinstance(p,Mapping):p=b10.FanParameters(**dict(p))
        if not isinstance(p,b10.FanParameters):raise TypeError('Fan parameters require FanParameters or a parameter mapping')
        return p
    if isinstance(value,Mapping) and set(value).issubset(FAN_KEYS):
        return {n:one(value.get(n)) for n in FAN_KEYS}
    shared=one(value)
    return {n:shared for n in FAN_KEYS}

class ObservedPump:
    def __init__(self,gas,parameters):
        self.gas=gas;self.parameters=parameters;self.motor=b10.Motor()
        self.angle_rad=0.;self.phase_rad=0.;self.omega_rad_s=0.;self.displacement_m=0.;self.piston_velocity_m_s=0.
        self.pressure_switch_on=False;self.intake_failed_closed=False;self.discharge_failed_closed=False
        self.gas_work_J=0.;self.friction_heat_J=0.
    switch=b10.ReciprocatingPump.switch
    check_commands=b10.ReciprocatingPump.check_commands
    def audit(self):
        return self.motor.audit()|{'gas_work_J':self.gas_work_J,'compressor_work_to_gas_J':-self.gas_work_J,
            'friction_heat_J':self.friction_heat_J,
            'relative_motion_kinetic_diagnostic_J':.5*self.parameters.flywheel_inertia_kg_m2*self.omega_rad_s**2+.5*self.parameters.piston_mass_kg*self.piston_velocity_m_s**2,
            'mechanical_balance_scope':'Native full-body kinetic energy and mounting transfer audited by host, no independent scalar balance asserted'}
    def snapshot(self):
        return self.audit()|{n:getattr(self,n) for n in ('angle_rad','phase_rad','omega_rad_s','displacement_m','piston_velocity_m_s','pressure_switch_on','intake_failed_closed','discharge_failed_closed')}|{
            'piston_displacement_m':self.displacement_m,'cylinder_pressure_Pa':self.gas.pressure_Pa('compressor_cylinder'),
            'state_owner':'Observed MuJoCo endpoint; no scalar pump integrator','power_failed':self.motor.power_failed}

class ObservedFan:
    def __init__(self,parameters=None):
        self.parameters=parameters or b10.FanParameters();self.motor=b10.Motor(b10.MotorParameters(resistance_ohm=24,torque_back_emf_constant=.03))
        self.angle_rad=0.;self.omega_rad_s=0.;self.air_work_J=0.;self.bearing_heat_J=0.;self.air_virtual_speed_rad_s=0.
    def snapshot(self):
        return self.motor.audit()|{'angle_rad':self.angle_rad,'omega_rad_s':self.omega_rad_s,
            'relative_spin_kinetic_diagnostic_J':.5*self.parameters.inertia_kg_m2*self.omega_rad_s**2,
            'air_work_J':self.air_work_J,'bearing_heat_J':self.bearing_heat_J,
            'flow_proxy_m3_s':self.parameters.flow_m3_per_rad*abs(self.omega_rad_s),'power_failed':self.motor.power_failed,
            'air_virtual_speed_rad_s':self.air_virtual_speed_rad_s,'state_owner':'Observed MuJoCo endpoint; no scalar fan integrator',
            'mechanical_balance_scope':'Native full-body kinetic energy and mounting transfer audited by host'}

class MountedNetwork(CheckpointAuxiliaryNetwork):
    def __init__(self,gas,*,pump_parameters=None,fan_parameters=None):
        super().__init__(gas,pump_parameters=pump_parameters)
        self.pump=ObservedPump(gas,pump_parameters or b10.PumpParameters())
        fp=fan_parameter_map(fan_parameters)
        self.fans={n:ObservedFan(fp[n]) for n in FANS}
        self.automatic_compressor=False
    def update(self,dt,displacements_m=None,valve_commands=None,*,snapshot=True):
        if dt!=0:raise RuntimeError('Use native observed-motion advance, never B10 scalar mechanical update')
        for n,u in (valve_commands or {}).items():
            if n not in self.commands or not math.isfinite(u) or not 0<=u<=1:raise ValueError('Invalid native auxiliary control')
            self.commands[n]=u
        for n,s in self.spools.items():s.command=self.commands[n]
        return self.snapshot() if snapshot else None
    def configuration(self):
        value=super().configuration();a=value['auxiliary_model']
        a.update(provenance='NATIVE_4_DOF_MOUNTED_RESEARCH_EQUIVALENT',auxiliary_mechanical_coordinates=4,
            auxiliary_canonical_states=0,mechanical_state_owner='HOST_MUJOCO_ONLY',
            limits=['Uncalibrated finite-flexibility Scotch-yoke and candidate mounts','B9 spool reactions remain external fixed bench',
                'World-rest air angular drag; no fluid linear inertia or jet thrust','Thermal calorimeters diagnostic, no gas temperature feedback'])
        return value

class MountedPneumatics(PneumaticAuxiliaryPlant):
    def __init__(self,parameters=None,*,pump_parameters=None,fan_parameters=None,initial_mechanical_state=None):
        # The inherited constructor creates the cylinder at its cold, zero-slide
        # volume and fixes the initial gas conservation reference there. Reject
        # a different initial slide instead of returning inconsistent gas/native
        # observations or silently changing mass/energy reference after creation.
        if isinstance(initial_mechanical_state,Mapping) and 'auxiliary' in initial_mechanical_state:
            initial_native=validate_native(initial_mechanical_state['auxiliary'])
            if initial_native['piston_displacement_m']!=0.:
                raise ValueError('Native compressor initialization requires piston displacement 0 m; restore a validated nonzero checkpoint to continue motion')
        super().__init__(parameters,pump_parameters=pump_parameters,initial_mechanical_state=initial_mechanical_state)
        self.network=MountedNetwork(self.gas,pump_parameters=self.pump_parameters,fan_parameters=fan_parameters)
        self.network.cabinet_external_heat_W=self.parameters.cabinet_external_heat_W
        self.last_step.update(compressor_gas_work_J=0.,work_equivalent_compressor_force_N=0.,
            motor_shaft_work_J={n:0. for n in ROTORS},auxiliary_motor_torque_Nm={n:0. for n in ROTORS},
            bearing_heat_J=0.,fan_air_work_J=0.,native_air_virtual_speed_rad_s={n:0. for n in FAN_KEYS})
        self.network.update(0.,valve_commands=self._apply_controls(self.commands),snapshot=False)
        self._observe(self.mechanical_state['auxiliary'])
        self._configuration=self._configuration_record();self._configuration_sha256=digest(self._configuration)

    def _mechanical_input(self,patch):
        aux=copy.deepcopy(self.mechanical_state.get('auxiliary',DEFAULT_NATIVE))
        old={k:v for k,v in self.mechanical_state.items() if k!='auxiliary'}
        if patch is not None:
            if not isinstance(patch,Mapping):raise ValueError('Mechanical observations require mapping')
            patch=dict(patch)
            if 'auxiliary' in patch:aux=validate_native(patch.pop('auxiliary'))
        saved=self.mechanical_state;self.mechanical_state=old
        try:result=super()._mechanical_input(patch)
        finally:self.mechanical_state=saved
        p=getattr(self,'pump_parameters',b10.PumpParameters())
        if p.dead_volume_m3+p.area_m2*aux['piston_displacement_m']<=1e-12:raise ValueError('Nonpositive native compressor chamber volume')
        result['auxiliary']=aux
        return result

    def _positions(self,state):
        value=super()._positions(state)
        value['compressor_cylinder']=state.get('auxiliary',DEFAULT_NATIVE)['piston_displacement_m']
        return value

    def _observe(self,value):
        p=self.network.pump;p.angle_rad=value['angle_rad']['compressor_rotor'];p.phase_rad=math.remainder(p.angle_rad,math.tau)
        p.omega_rad_s=value['omega_rad_s']['compressor_rotor'];p.displacement_m=value['piston_displacement_m'];p.piston_velocity_m_s=value['piston_velocity_m_s']
        for n,f in self.network.fans.items():f.angle_rad=value['angle_rad'][n+'_rotor'];f.omega_rad_s=value['omega_rad_s'][n+'_rotor']

    def initial_motor_torques(self):
        c=self.commands;p=self.network.pump
        command=c['compressor_voltage_fraction']*(p.switch() if c['compressor_auto'] else 1.)
        torques={}
        for n,motor,u in [('compressor_rotor',p.motor,command),*[(n+'_rotor',f.motor,c['fan_voltage_fractions'][n]) for n,f in self.network.fans.items()]]:
            drive,damp=motor.coefficients(u);torques[n]=drive-damp*self.mechanical_state['auxiliary']['omega_rad_s'][n]
        return torques

    def forces(self):
        value=super().forces()
        value['compressor_force_N']=(self.gas.pressure_Pa('compressor_cylinder')-self.gas.ambient_pressure_Pa)*self.pump_parameters.area_m2
        return value

    def step(self,dt_s,mechanical_state=None,commands=None,*,snapshot=True,air_virtual_speed_rad_s=None):
        htotal=number(dt_s,'dt_s',0.,1.);end=self._mechanical_input(mechanical_state);controls=self._command_input(commands)
        begin=copy.deepcopy(self.mechanical_state)
        if htotal==0 and end!=begin:raise ValueError('Mechanical change requires positive dt')
        air={n:0. for n in FAN_KEYS} if air_virtual_speed_rad_s is None else dict(air_virtual_speed_rad_s)
        if set(air)!=set(FAN_KEYS):raise ValueError('Air speed keys mismatch')
        air={n:number(v,'air speed') for n,v in air.items()}
        oldforce=self.forces();ports=self._apply_controls(controls);self.commands=controls
        self.network.update(0.,valve_commands=ports,snapshot=False)
        if htotal==0:return self.snapshot() if snapshot else None
        net,gas=self.network,self.gas;p=net.pump
        gas.node_interval_work_J={n:0. for n in gas.volumes};edge0=dict(gas.edge_mass_kg)
        start=self._positions(begin);finish=self._positions(end)
        rel={n:(end['auxiliary']['angle_rad'][n]-begin['auxiliary']['angle_rad'][n])/htotal for n in ROTORS}
        motor_work={n:0. for n in ROTORS};motor_torque={n:0. for n in ROTORS};bearing=0.;airwork=0.
        count=max(1,math.ceil(htotal/min(self.parameters.gas_max_substep_s,self.pump_parameters.maximum_step_s)));h=htotal/count
        for k in range(count):
            actual={n:u for n,u in net.commands.items() if n not in ('compressor',*FAN_KEYS)}
            actual.update(p.check_commands())
            for n,s in net.spools.items():actual[n]=s.advance(h,net.commands[n])
            command=controls['compressor_voltage_fraction']*(p.switch() if controls['compressor_auto'] else 1.)
            net.commands['compressor']=command
            mid={n:a+(finish[n]-a)*(k+.5)/count for n,a in start.items()}
            stop={n:a+(finish[n]-a)*(k+1)/count for n,a in start.items()}
            gas.update(h/2,mid,actual,snapshot=False)
            actual.update(p.check_commands());gas.update(h/2,stop,actual,snapshot=False)
            for n,s in net.spools.items():gas.orifices[n].opening=s.opening
            before=p.motor.copper_heat_J+p.friction_heat_J
            p.motor.record(h,command,rel['compressor_rotor'])
            p.friction_heat_J+=p.parameters.viscous_friction_Nm_s*rel['compressor_rotor']**2*h
            tau=p.motor.parameters.torque_back_emf_constant*p.motor.current_A
            motor_torque['compressor_rotor']+=tau/count;motor_work['compressor_rotor']+=tau*rel['compressor_rotor']*h
            bearing+=p.parameters.viscous_friction_Nm_s*rel['compressor_rotor']**2*h
            pumpheat=p.motor.copper_heat_J+p.friction_heat_J-before;fanheat={}
            for n,f in net.fans.items():
                before=f.motor.copper_heat_J+f.bearing_heat_J;omega=rel[n+'_rotor']
                f.motor.record(h,controls['fan_voltage_fractions'][n],omega)
                q=f.parameters.bearing_drag_Nm_s*omega**2*h;wa=f.parameters.aerodynamic_drag_Nm_s2*abs(air[n])**3*h
                f.bearing_heat_J+=q;f.air_work_J+=wa;f.air_virtual_speed_rad_s=air[n];bearing+=q;airwork+=wa
                tau=f.motor.parameters.torque_back_emf_constant*f.motor.current_A
                motor_torque[n+'_rotor']+=tau/count;motor_work[n+'_rotor']+=tau*omega*h
                fanheat[n]=f.motor.copper_heat_J+f.bearing_heat_J-before
            net.thermal['compressor_motor'].advance(h,pumpheat+fanheat['compressor_fan'],rel['compressor_fan_rotor'])
            net.thermal['cabinet'].advance(h,self.parameters.cabinet_external_heat_W*h+fanheat['bay_fan'],rel['bay_fan_rotor'])
        gas.last_flows_kg_s={n:(v-edge0[n])/htotal for n,v in gas.edge_mass_kg.items()}
        self.mechanical_state=end;self._observe(end['auxiliary']);self.steps+=1
        work=dict(gas.node_interval_work_J);p.gas_work_J+=work['compressor_cylinder'];now=self.forces()
        def force(n,dx,a,b):return work[n]/dx if dx else .5*(a+b)
        da=end['pantograph_q_rad']-begin['pantograph_q_rad'];xx0=start['pantograph_rotary'];xx1=finish['pantograph_rotary']
        dx=xx1-xx0;df=self.parameters.pantograph_equivalent_lever_m*math.cos(.5*(end['pantograph_q_rad']+begin['pantograph_q_rad'])+self.parameters.pantograph_phase_rad)*(math.sin(.5*da)/(.5*da) if da else 1.)
        tq=work['pantograph_rotary']/dx*df if dx else (work['pantograph_rotary']/da if da and work['pantograph_rotary'] else .5*(oldforce['pantograph_torque_Nm']+now['pantograph_torque_Nm']))
        af={n:force(n,finish[n]-start[n],oldforce['air_spring_forces_N'][n],now['air_spring_forces_N'][n]) for n in AIR_SPRINGS}
        bf={n:force(n,finish[n]-start[n],oldforce['brake_piston_forces_N'][n],now['brake_piston_forces_N'][n]) for n in BRAKES}
        cf=force('compressor_cylinder',finish['compressor_cylinder']-start['compressor_cylinder'],oldforce['compressor_force_N'],now['compressor_force_N'])
        self.last_step={'dt_s':htotal,'node_work_J':work,'pantograph_generalized_work_J':work['pantograph_rotary'],
            'air_spring_generalized_work_J':{n:work[n] for n in AIR_SPRINGS},'brake_piston_generalized_work_J':{n:work[n] for n in BRAKES},
            'work_equivalent_pantograph_torque_Nm':tq,'work_equivalent_air_spring_forces_N':af,'work_equivalent_brake_piston_forces_N':bf,
            'pantograph_geometry_roundoff_work_J':tq*da-work['pantograph_rotary'],'compressor_gas_work_J':work['compressor_cylinder'],
            'work_equivalent_compressor_force_N':cf,'motor_shaft_work_J':motor_work,'auxiliary_motor_torque_Nm':motor_torque,
            'bearing_heat_J':bearing,'fan_air_work_J':airwork,'native_air_virtual_speed_rad_s':air}
        return self.snapshot() if snapshot else None

    def _configuration_record(self):
        value=super()._configuration_record()
        value['schema']='gtsd-phase6-observed-native-aux-configuration-v1'
        value['limits']=[x for x in value['limits'] if not x.startswith('Auxiliary pump')]+[
            'Pump/fan scalar mechanical advance disabled; native rotor/piston state is sole dynamics owner',
            'Four mechanical DOFs, finite-flexibility yoke; valve spool mount reactions still external']
        value['force_ports']['compressor']='Actual native slide x; V=V0+A*x; gauge force; port work only once'
        return value

    def _validate_restored(self):
        # Inherited restore validates a private candidate and only then loads the
        # original values. All checks here must reject bad data, not repair it.
        self._command_input(self.commands);self._mechanical_input(self.mechanical_state)
        gas,net,c,last=self.gas,self.network,self.commands,self.last_step
        if gas.time_s<0 or self.steps<0 or gas.substeps<0 or gas.equalization_caps<0:
            raise ValueError('Negative native pneumatic time/count')
        if gas.substeps<2*self.steps or gas.substeps%2 or (self.steps==0 and gas.substeps!=0):
            raise ValueError('Native pneumatic split-substep count mismatch')
        if (self.steps==0 and (gas.time_s!=0 or last['dt_s']!=0)) or (self.steps>0 and not 0<last['dt_s']<=gas.time_s+1e-12):
            raise ValueError('Native pneumatic interval/time mismatch')
        for u in list(net.commands.values())+[o.opening for o in gas.orifices.values()]:
            number(u,'restored opening',0.,1.)
        # Check faults BEFORE _apply_controls; applying controls first would mask
        # a forged motor/valve fault in the validation candidate.
        if any(s.coil_failed!=c['coil_failures'][n] for n,s in net.spools.items()):
            raise ValueError('Native spool coil fault/command mismatch')
        if net.pump.motor.power_failed!=c['compressor_power_failed'] or any(f.motor.power_failed!=c['fan_power_failed'][n] for n,f in net.fans.items()):
            raise ValueError('Native motor power fault/command mismatch')
        if net.pump.intake_failed_closed!=c['compressor_intake_failed_closed'] or net.pump.discharge_failed_closed!=c['compressor_discharge_failed_closed']:
            raise ValueError('Native compressor check-valve fault/command mismatch')
        positions=self._positions(self.mechanical_state)
        for n,v in gas.volumes.items():
            if v.mass_kg<=0 or v.volume_m3<=0:raise ValueError('Invalid restored gas mass/volume')
            if n in positions and abs(v.displacement_m-positions[n])>1e-13:raise ValueError('Observed native/gas volume mismatch')
        native=self.mechanical_state['auxiliary'];p=net.pump
        if p.angle_rad!=native['angle_rad']['compressor_rotor'] or p.omega_rad_s!=native['omega_rad_s']['compressor_rotor'] or p.displacement_m!=native['piston_displacement_m'] or p.piston_velocity_m_s!=native['piston_velocity_m_s']:raise ValueError('Pump observation mismatch')
        if p.phase_rad!=math.remainder(p.angle_rad,math.tau):raise ValueError('Pump phase/observed angle mismatch')
        if p.friction_heat_J<0:raise ValueError('Negative pump bearing heat')
        for n,f in net.fans.items():
            if f.angle_rad!=native['angle_rad'][n+'_rotor'] or f.omega_rad_s!=native['omega_rad_s'][n+'_rotor']:raise ValueError('Fan observation mismatch')
            if f.bearing_heat_J<0 or f.air_work_J<0:raise ValueError('Negative fan bearing/air dissipation')
            if f.air_virtual_speed_rad_s!=last['native_air_virtual_speed_rad_s'][n]:raise ValueError('Fan last world-air speed mismatch')
        audit=gas.audit()
        for key,tol in [('mass_balance_residual_kg',1e-10),('gas_energy_balance_residual_J',1e-5),('combined_energy_balance_residual_J',1e-5),('exergy_balance_residual_J',1e-5)]:
            if abs(audit[key])>tol:raise ValueError('Gas conservation mismatch')
        for key in ('external_mass_in_kg','external_mass_out_kg','external_enthalpy_in_J','external_enthalpy_out_J','flow_exergy_destroyed_J'):
            if gas.ledger[key]<-1e-12:raise ValueError('Negative gas transfer/dissipation ledger: '+key)
        for key in ('compressor_work_in_J','compressor_heat_to_bath_J','compressor_exergy_destroyed_J'):
            if gas.ledger[key]!=0.:raise ValueError('Disabled ideal compressor ledger must remain zero')
        if any(v!=0. for v in gas.compressor_mass_kg.values()):raise ValueError('Disabled ideal compressor mass ledger must remain zero')
        if abs(gas.ledger['gas_boundary_work_J']-gas.ledger['useful_mechanical_work_J']-gas.ledger['ambient_displacement_work_J'])>1e-8:raise ValueError('Gas boundary/useful/ambient work split mismatch')
        if abs(math.fsum(gas.node_useful_work_J.values())-gas.ledger['useful_mechanical_work_J'])>1e-8:raise ValueError('Node gas work mismatch')
        if last['node_work_J']!=gas.node_interval_work_J:raise ValueError('Interval gas work mismatch')
        if last['compressor_gas_work_J']!=last['node_work_J']['compressor_cylinder']:raise ValueError('Compressor gas interval mismatch')
        if any(last['brake_piston_generalized_work_J'][n]!=last['node_work_J'][n] for n in BRAKES):raise ValueError('Brake piston/node interval mismatch')
        if last['pantograph_generalized_work_J']!=last['node_work_J']['pantograph_rotary'] or any(last['air_spring_generalized_work_J'][n]!=last['node_work_J'][n] for n in AIR_SPRINGS):raise ValueError('Gas generalized port/node interval mismatch')
        if abs(p.gas_work_J-gas.node_useful_work_J['compressor_cylinder'])>1e-9:raise ValueError('Compressor cumulative gas work mismatch')
        motors={'compressor_rotor':p.motor,**{n+'_rotor':f.motor for n,f in net.fans.items()}}
        for n,motor in motors.items():
            if abs(motor.audit()['electrical_balance_residual_J'])>1e-8 or motor.copper_heat_J<0:raise ValueError('Aux motor electrical ledger mismatch')
            # Fans use one fixed voltage throughout the interval. Pump current
            # can change with automatic switching between gas substeps, so its
            # average torque equals final current only in a one-substep interval.
            one_substep=last['dt_s']<=min(self.parameters.gas_max_substep_s,self.pump_parameters.maximum_step_s)
            if n!='compressor_rotor' or one_substep:
                if abs(last['auxiliary_motor_torque_Nm'][n]-motor.parameters.torque_back_emf_constant*motor.current_A)>1e-12:raise ValueError('Last auxiliary motor torque/current mismatch')
        if last['bearing_heat_J']<0 or last['fan_air_work_J']<0:raise ValueError('Negative last auxiliary dissipation')
        expected_air=math.fsum(net.fans[n].parameters.aerodynamic_drag_Nm_s2*abs(last['native_air_virtual_speed_rad_s'][n])**3*last['dt_s'] for n in FAN_KEYS)
        if abs(last['fan_air_work_J']-expected_air)>1e-10:raise ValueError('Last fan world-air work/speed mismatch')
        expected=self._apply_controls(self.commands)
        for n,value in expected.items():
            if n!='compressor' and abs(net.commands[n]-value)>1e-14:raise ValueError('Command mismatch')
        for n,s in net.spools.items():
            number(s.command,'restored spool command',0.,1.)
            if s.command!=net.commands[n] or abs(s.opening-gas.orifices[n].opening)>1e-12:raise ValueError('Spool mismatch')
            if s.initial_energy_J!=0. or s.damping_loss_J<0 or s.max_stop_penetration_m<0 or abs(s.audit()['balance_residual_J'])>1e-7:raise ValueError('Individual spool reference/dissipation/energy mismatch')
        for t in net.thermal.values():
            if t.temperature_K<=0 or t.heat_input_J<0 or abs(t.snapshot()['balance_residual_J'])>1e-7:raise ValueError('Thermal ledger mismatch')
        cf,bf=net.fans['compressor_fan'],net.fans['bay_fan']
        expected_pump_heat=p.motor.copper_heat_J+p.friction_heat_J+cf.motor.copper_heat_J+cf.bearing_heat_J
        expected_cabinet_heat=self.parameters.cabinet_external_heat_W*gas.time_s+bf.motor.copper_heat_J+bf.bearing_heat_J
        if abs(net.thermal['compressor_motor'].heat_input_J-expected_pump_heat)>1e-8 or abs(net.thermal['cabinet'].heat_input_J-expected_cabinet_heat)>1e-8:raise ValueError('Auxiliary thermal source ledger mismatch')
        canonical(self.snapshot())
