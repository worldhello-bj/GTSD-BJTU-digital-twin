"""Phase6 mounted auxiliary research extension of the frozen Phase5 solve."""
from __future__ import annotations
import copy,hashlib,json,math
from pathlib import Path
from dataclasses import asdict
import mujoco
import numpy as np
from . import bootstrap
from brake_integrated.plant import IntegratedBrakePlant,NativeBrakeDiagnostics,STATE_KIND,sha,canonical,AIR_NAMES,PISTONS,DOOR_KEYS,pack
from brake_integrated.plant import WORK_COLUMNS as BASE_WORK_COLUMNS
from console.live_plant import unpack_like
from support_integrated.model import augment_xml as add_support,Parameters as SupportParameters
from support_integrated.runtime import SupportContactRuntime
from .model import augment_xml as add_auxiliary,Parameters as AuxiliaryParameters
from .pneumatics import MountedPneumatics
from .runtime import AuxiliaryMountRuntime,COLUMNS as AUX_COLUMNS,ROTORS
from pneumatic_aux.kernels import b10
ROOT=Path(__file__).resolve().parent
WORK_COLUMNS=(*BASE_WORK_COLUMNS,*AUX_COLUMNS)
SCHEMA='gtsd-phase6-native-mounted-host-checkpoint-v1'

class MountedAuxiliaryPlant(IntegratedBrakePlant):
    def __init__(self,dt=.0002,*,include_cables=True,brake_parameters=None,support_parameters=None,auxiliary_parameters=None):
        if include_cables is not True:raise ValueError('Phase6 mounted assembly requires the frozen cable/support assembly')
        self.support_parameters=SupportParameters(**support_parameters) if isinstance(support_parameters,dict) else support_parameters or SupportParameters()
        self.auxiliary_parameters=AuxiliaryParameters(**auxiliary_parameters) if isinstance(auxiliary_parameters,dict) else auxiliary_parameters or AuxiliaryParameters()
        self._building_base=True
        super().__init__(dt,include_cables=True,brake_parameters=brake_parameters)
        self._building_base=False
        self.mount_runtime=AuxiliaryMountRuntime(self.model,self.data,self.auxiliary_spec)
        ap=self.auxiliary_parameters
        pump=b10.PumpParameters(crank_radius_m=ap.crank_radius_m,flywheel_inertia_kg_m2=ap.compressor_rotor_spin_inertia_kgm2,piston_mass_kg=ap.compressor_piston_mass_kg)
        self.mount_runtime.p=pump
        fans=b10.FanParameters(inertia_kg_m2=ap.fan_rotor_spin_inertia_kgm2)
        self.mount_runtime.fp={n:fans for n in self.mount_runtime.fp}
        self.aux=MountedPneumatics(self.aux.parameters,pump_parameters=pump,fan_parameters=fans,initial_mechanical_state=self._observations())
        self.brake=NativeBrakeDiagnostics(self.aux)
        self.support=SupportContactRuntime(self.model,self.data,self.support_spec)
        self.command_registry.update(self.support.command_registry)
        self.work=np.zeros(len(WORK_COLUMNS));self.last_step_work=np.zeros(len(WORK_COLUMNS))
        self.last_coupling.update(auxiliary_effort_iteration_error=0.,auxiliary_compressor_work_residual_J=0.,auxiliary_motor_work_residual_J=0.)
        self.acceptance.update(auxiliary_effort=1e-9,auxiliary_work_J=1e-10,max_iterations=40)
        self.model_version='phase6-native-79dof-mounted-auxiliary-research'
        self.limits=[x for x in self.limits if '标量动力学' not in x]+['辅机转子/活塞为同一原生求解器；安装、轴向、质量及有限柔性yoke均为可替换研究假设。','阀芯安装反力仍为固定台座边界；流体平动/喷流惯量未建。']
        self.capabilities.update(integrated_dof=self.model.nv,native_auxiliary_rotors=True,native_compressor_piston=True,auxiliary_mount_reactions=True,cable_support_native_contact=True,cable_support_vehicle_reactions=True,whole_vehicle_energy_closed=False)
        self.identity.update(schema='gtsd-phase6-mounted-model-identity-v1',pneumatic_configuration=self.aux.configuration())
        self.identity['extensions'].update(phase5_verified_dependency_sha256=bootstrap.BASELINE_IDENTITY_SHA256,auxiliary_spec=copy.deepcopy(self.auxiliary_spec),support_spec=copy.deepcopy(self.support_spec),support_runtime_identity=copy.deepcopy(self.support.identity),work_columns_J=list(WORK_COLUMNS))
        self.identity['extensions']['source_sha256'].update({'phase6/aux_mount/'+p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/n for n in ('__init__.py','bootstrap.py','model.py','runtime.py','pneumatics.py','plant.py')]})
        self.binding.update(auxiliary_augmentation=copy.deepcopy(self.auxiliary_spec),support_augmentation=copy.deepcopy(self.support_spec))
        self.mass_ledger.update(auxiliary_allocation=copy.deepcopy(self.auxiliary_spec['mass_ledger']),auxiliary_incremental_mass_kg=0.,cable_support_incremental_mass_kg=0.,
            boundary='Frozen core/cable/native-brake totals retained. Auxiliary masses split from payload and one selected door with reference mass/first-moment/full-inertia preservation. No rotor armature duplication. All allocations are research assumptions.')
        self._apply_forces(self.aux.forces(),self.last_motor,self.last_brake)
        mujoco.mj_forward(self.model,self.data);self.energy0=float(sum(self.data.energy))

    def augment_extra_xml(self,xml,metadata):
        xml,self.support_spec=add_support(xml,metadata['cables'],self.support_parameters)
        xml,self.auxiliary_spec=add_auxiliary(xml,self.auxiliary_parameters)
        return xml

    def _observations(self):
        result=super()._observations()
        if not self._building_base:result['auxiliary']=self.mount_runtime.observe()
        return result

    def command(self,name,args):
        if name=='support_contact':
            if self.failed:raise ValueError('plant stopped after rejected interval')
            return self.support.command(name,args)
        return super().command(name,args)

    def _step(self):
        state=self._mechanical_state();aux=self.aux.checkpoint_state();dynamic=copy.deepcopy(self._dynamic());motors=copy.deepcopy(self.motors)
        heat=self.brake.brake_heat_J;support=self.support.checkpoint();runtime=self.mount_runtime.checkpoint();warnings=[(w.number,w.lastinfo) for w in self.data.warning]
        try:
            super()._step()
            velocity=np.asarray(self.last_native_brake['generalized_displacement'])/self.dt
            self.support.capture_interval(state,self.data,velocity,expected_constraint=self.last_constraint_generalized)
        except Exception:
            mujoco.mj_setState(self.model,self.data,state,STATE_KIND);self.aux.restore(aux);self.motors=motors;self.brake.brake_heat_J=heat
            for name,value in dynamic.items():setattr(self,name,value)
            self.support.restore(support);self.mount_runtime.restore(runtime)
            warm=self.data.qacc_warmstart.copy();mujoco.mj_forward(self.model,self.data);self.data.qacc_warmstart[:]=warm
            for w,(number,lastinfo) in zip(self.data.warning,warnings):w.number,w.lastinfo=number,lastinfo
            raise

    def _trial_interval(self,state,aux0):
        m,d,h=self.model,self.data,self.dt
        q0=d.qpos.copy();v0=d.qvel.copy();ten0=d.ten_length.copy();energy0=float(sum(d.energy));begin=self._observations()
        feedback=v0[self.wheel_dofs].copy();forces=self.aux.forces();accepted=False
        ar=self.mount_runtime;abegin=ar.observe();basis=ar.basis();adelta=h*v0.copy();aend=copy.deepcopy(abegin)
        for n in ROTORS:aend['angle_rad'][n]+=adelta[ar.dof[n]]
        aend['piston_displacement_m']+=adelta[ar.dof['compressor_piston']]
        auxiliary=ar.preview(abegin,aend,adelta,h,basis,self.aux.initial_motor_torques(),forces['compressor_force_N'])
        for iteration in range(1,self.acceptance['max_iterations']+1):
            used_feedback=feedback.copy();used_forces=copy.deepcopy(forces);used_auxiliary=auxiliary
            ms=[self.motors[i].preview(h,float(used_feedback[i]),self.commands['drive_enabled']) for i in range(2)]
            mujoco.mj_setState(m,d,state,STATE_KIND);mujoco.mj_forward(m,d)
            qhead,qbody,qcable=self._apply_forces(used_forces,ms,None)
            d.qfrc_applied[:]+=used_auxiliary['total']
            mujoco.mj_forward(m,d)
            gas_ports=[(self.panto_actuator,used_forces['pantograph_torque_Nm'])]+[(m.actuator(n+'_force').id,used_forces['air_spring_forces_N'][n]) for n in AIR_NAMES]
            for aid,effort in gas_ports:
                if abs(float(d.actuator_force[aid])-effort)>1e-12*max(1.,abs(effort)):raise RuntimeError('Gas actuator effort clipped')
            mujoco.mj_step(m,d)
            auxiliary_sensors=ar.capture_sensors()
            actual_constraint=d.qfrc_constraint.copy();actual_limit=np.zeros(m.nv)
            is_limit=np.isin(d.efc_type,[int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT),int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)])
            mujoco.mj_mulJacTVec(m,d,actual_limit,np.where(is_limit,d.efc_force,0.))
            native,qnormal,qtangent=self._native_contact_projection()
            delta=np.zeros(m.nv);mujoco.mj_differentiatePos(m,delta,1.,q0,d.qpos)
            vmid=.5*(v0+d.qvel);observed_feedback=vmid[self.wheel_dofs]
            mujoco.mj_forward(m,d);end=self._observations()
            if iteration>1:self.aux.restore(aux0)
            self.aux.step(h,end,snapshot=False,air_virtual_speed_rad_s=ar.air_speeds(basis,delta,h));gas=self.aux.last_step
            aend=ar.observe();auxiliary_next=ar.preview(abegin,aend,delta,h,basis,gas['auxiliary_motor_torque_Nm'],gas['work_equivalent_compressor_force_N'])
            auxiliary_error=float(max(abs(auxiliary_next['total']-used_auxiliary['total'])))
            auxiliary_gas_error=float(used_auxiliary['parts']['compressor_gas']@delta)-gas['compressor_gas_work_J']
            auxiliary_motor_error=max(abs(used_auxiliary['motor_torque_Nm'][n]*delta[ar.dof[n]]-gas['motor_shaft_work_J'][n]) for n in ROTORS)
            equivalent={'pantograph_torque_Nm':gas['work_equivalent_pantograph_torque_Nm'],
                'air_spring_forces_N':dict(gas['work_equivalent_air_spring_forces_N']),
                'brake_piston_forces_N':dict(gas['work_equivalent_brake_piston_forces_N'])}
            gas_work=np.array([gas['pantograph_generalized_work_J']]+[gas['air_spring_generalized_work_J'][n] for n in AIR_NAMES]+[gas['brake_piston_generalized_work_J'][n] for n in PISTONS])
            mechanical_work=np.array([used_forces['pantograph_torque_Nm']*(end['pantograph_q_rad']-begin['pantograph_q_rad'])]+
                [used_forces['air_spring_forces_N'][n]*(end['air_spring_displacements_m'][n]-begin['air_spring_displacements_m'][n]) for n in AIR_NAMES]+
                [used_forces['brake_piston_forces_N'][n]*(end['brake_piston_displacements_m'][n]-begin['brake_piston_displacements_m'][n]) for n in PISTONS])
            residual=mechanical_work-gas_work
            force_error=max(abs(equivalent['air_spring_forces_N'][n]-used_forces['air_spring_forces_N'][n]) for n in AIR_NAMES)
            torque_error=abs(equivalent['pantograph_torque_Nm']-used_forces['pantograph_torque_Nm'])
            brake_error=max(abs(equivalent['brake_piston_forces_N'][n]-used_forces['brake_piston_forces_N'][n]) for n in PISTONS)
            speed_error=float(max(abs(observed_feedback-used_feedback)))
            if force_error<=self.acceptance['force_N'] and torque_error<=self.acceptance['torque_Nm'] and brake_error<=self.acceptance['brake_force_N'] and speed_error<=self.acceptance['feedback_rad_s'] and max(abs(residual))<=self.acceptance['port_work_J'] and auxiliary_error<=self.acceptance['auxiliary_effort'] and max(abs(auxiliary_gas_error),auxiliary_motor_error)<=self.acceptance['auxiliary_work_J']:
                accepted=True;break
            forces=equivalent;feedback=observed_feedback.copy();auxiliary=auxiliary_next
        if not accepted:raise RuntimeError('Native brake coupled interval rejected: '+json.dumps({'iteration':iteration,'force_N':force_error,'torque_Nm':torque_error,'piston_force_N':brake_error,'speed_rad_s':speed_error,'port_work_J':residual.tolist(),'auxiliary_effort':auxiliary_error,'auxiliary_gas_J':auxiliary_gas_error,'auxiliary_motor_J':auxiliary_motor_error}))
        if abs(float(d.time)-(float(state[0])+h))>1e-10 or not np.isfinite(np.r_[d.qpos,d.qvel,d.qacc]).all():raise ArithmeticError('Invalid coupled endpoint')
        if any(w.number>int(old) for w,old in zip(d.warning,self.warnings)):raise ArithmeticError('MuJoCo warning; coupled interval rejected')
        tangent_work=float(qtangent@delta);normal_work=float(qnormal@delta)
        if tangent_work>1e-10:raise ArithmeticError('Native brake tangent added discrete energy: '+str(tangent_work))
        self.last_guard={'feedback_residual_rad_s':speed_error,'speed_tolerance_rad_s':self.acceptance['feedback_rad_s'],
          'per_axle_brake_work_J':(qtangent[self.wheel_dofs]*delta[self.wheel_dofs]).tolist(),'positive_brake_work_tolerance_J':1e-10,
          'feedback_converged':True,'discrete_brake_passive':True}
        self.last_corrector_error=observed_feedback-used_feedback;self.last_constraint_generalized=actual_constraint;self.last_limit_generalized=actual_limit
        self.last_mid_velocity=vmid.copy();self.last_generalized_head_force=qhead;self.last_body_generalized=qbody;self.last_cable_generalized=qcable
        self.constraint_work_parts+=np.array([float(actual_limit@delta),float((actual_constraint-actual_limit)@delta)])
        damping=-float((m.dof_damping*d.qvel)@delta+np.sum(m.tendon_damping*d.ten_velocity*(d.ten_length-ten0)))
        motor_work=sum(ms[i]['torque_Nm']*delta[self.wheel_dofs[i]] for i in range(2))
        motor_mid=sum(ms[i]['torque_Nm']*observed_feedback[i]*h for i in range(2))
        door_work=sum(d.ctrl[self.doors.actuators[k]]*delta[self.doors.qvel_addresses[k]] for k in DOOR_KEYS)
        all_constraint=float(actual_constraint@delta)
        # Tangential brake work has its own column; remove it from total constraint
        # column so summing all columns never counts native friction twice.
        self.last_step_work=np.array([motor_work,tangent_work,float(qhead@delta),damping,all_constraint-tangent_work,
            mechanical_work[0],sum(mechanical_work[1:3]),door_work,float(qbody@delta),float(qcable@delta),sum(mechanical_work[3:])]+[float(used_auxiliary['parts'][n]@delta) for n in AUX_COLUMNS])
        self.work+=self.last_step_work;self.port_work+=mechanical_work
        self.native_brake_ledger['tangent_work_J']+=tangent_work;self.native_brake_ledger['normal_work_J']+=normal_work
        self.native_brake_ledger['native_contact_total_work_J']+=float((qnormal+qtangent)@delta)
        self.native_brake_ledger['piston_gas_work_J']+=float(sum(mechanical_work[3:]));self.native_brake_ledger['coupling_residual_J']+=float(sum(residual[3:]))
        self.native_brake_ledger['max_positive_tangent_step_J']=max(self.native_brake_ledger['max_positive_tangent_step_J'],tangent_work)
        self.brake.brake_heat_J=-self.native_brake_ledger['tangent_work_J']
        for i in range(2):self.motors[i].commit(ms[i],ms[i]['torque_Nm']*float(delta[self.wheel_dofs[i]]))
        midpoint=np.array([h*used_forces['pantograph_torque_Nm']*vmid[self.panto_dof]]+
            [mechanical_work[k+1] for k in range(2)]+[h*used_forces['brake_piston_forces_N'][n]*vmid[self.piston_dofs[n]] for n in PISTONS])
        self.last_coupling={'iterations':iteration,'force_iteration_error_N':force_error,'torque_iteration_error_Nm':torque_error,
            'brake_force_iteration_error_N':brake_error,'auxiliary_effort_iteration_error':auxiliary_error,'auxiliary_compressor_work_residual_J':auxiliary_gas_error,'auxiliary_motor_work_residual_J':auxiliary_motor_error,'gas_endpoint_work_J':gas_work.tolist(),'mechanical_endpoint_work_J':mechanical_work.tolist(),
            'port_work_residual_J':residual.tolist(),'midpoint_actuator_work_J':midpoint.tolist(),'midpoint_minus_endpoint_work_J':(midpoint-mechanical_work).tolist()}
        native.update(tangent_work_J=tangent_work,normal_work_J=normal_work,per_axle_tangent_work_J=(qtangent[self.wheel_dofs]*delta[self.wheel_dofs]).tolist(),
          work_reconstruction_residual_J=float((qnormal+qtangent)@delta-normal_work-tangent_work),generalized_displacement=delta.tolist(),qpos_start=q0.tolist(),qvel_start=v0.tolist(),
          gas_piston_forces_N=dict(used_forces['brake_piston_forces_N']),gas_piston_work_J={n:float(mechanical_work[3+k]) for k,n in enumerate(PISTONS)},
          mechanical_energy_start_J=energy0,mechanical_energy_end_J=float(sum(d.energy)),mechanical_unclosed_residual_J=float(sum(d.energy))-energy0-float(sum(self.last_step_work)),
          motor_midpoint_minus_endpoint_work_J=float(motor_mid-motor_work),all_constraint_work_J=all_constraint,other_constraint_work_J=all_constraint-tangent_work-normal_work,
          input_drive_enabled=self.commands['drive_enabled'],input_brake_mode=self.commands['brake_mode'])
        ar.commit(abegin,aend,delta,h,basis,used_auxiliary,gas,auxiliary_sensors)
        self.last_native_brake=native;self.last_motor=ms
        self.last_brake=[self._brake_diagnostic(i+1,native['per_axle_contact_torque_Nm'][i]) for i in range(2)]
        self.last_head_force=self.commands['head_force_N'];self.steps+=1
        if self.commands['body_test_force']['remaining_steps']>0:self.commands['body_test_force']['remaining_steps']-=1
        self.warnings=np.maximum(self.warnings,[w.number for w in d.warning])

    def _restore_validated_candidate(self,value):
        expected={'schema','identity','identity_sha256','mujoco_integration_state','mujoco_warning_state','pneumatic','motors','dynamic','brake_heat_J','doors','cables','sha256'}
        if type(value) is not dict or set(value)!=expected or value['schema']!=SCHEMA:raise ValueError('extended checkpoint schema mismatch')
        canonical(value)
        if value['sha256']!=sha({k:v for k,v in value.items() if k!='sha256'}):raise ValueError('extended checksum mismatch')
        if value['identity']!=self.identity or value['identity_sha256']!=sha(self.identity):raise ValueError('extended model identity mismatch')
        raw_state=value['mujoco_integration_state']
        if type(raw_state) is not list or any(type(x) not in (int,float) or not math.isfinite(x) for x in raw_state):raise ValueError('mechanical state requires finite numeric scalars, not booleans')
        state=np.asarray(raw_state,dtype=float)
        if state.shape!=self._mechanical_state().shape or not np.isfinite(state).all():raise ValueError('invalid mechanical state')
        self._validate_mechanical_state(state)
        initial_energy_reference=self.energy0
        self.aux.restore(value['pneumatic']);self.motors=unpack_like(value['motors'],self.motors,'motors')
        dynamic=unpack_like(value['dynamic'],self._dynamic(),'dynamic')
        for name,v in dynamic.items():setattr(self,name,v)
        if type(value['brake_heat_J']) not in (float,int) or not math.isfinite(value['brake_heat_J']) or value['brake_heat_J']<0:raise ValueError('invalid brake heat')
        self.brake.brake_heat_J=float(value['brake_heat_J'])
        if abs(self.native_brake_ledger['piston_gas_work_J']-self.work[10])>1e-10 or abs(self.native_brake_ledger['tangent_work_J']-self.work[1])>1e-10:raise ValueError('native brake work ledger mismatch')
        if self.steps<0 or abs(state[0]-self.steps*self.dt)>1e-7 or abs(self.aux.gas.time_s-state[0])>1e-7 or self.aux.steps!=self.steps:raise ValueError('inconsistent time/step counts')
        self._validate_command_state()
        if self.commands['brake_mode']!=self.aux.commands['brake_mode'] or abs(self.brake.brake_heat_J+self.native_brake_ledger['tangent_work_J'])>1e-10:raise ValueError('inconsistent brake command/ledger')
        mujoco.mj_setState(self.model,self.data,state,STATE_KIND)
        original_ctrl=self.data.ctrl.copy();original_eq=self.data.eq_active.copy()
        self.doors.restore(value['doors'])
        if self.cables:self.cables.restore(value['cables'])
        elif value['cables'] is not None:raise ValueError('unexpected cable state')
        if not np.array_equal(original_ctrl,self.data.ctrl) or not np.array_equal(original_eq,self.data.eq_active):raise ValueError('controller/MuJoCo controls or constraints disagree')
        warm=self.data.qacc_warmstart.copy();mujoco.mj_forward(self.model,self.data);self.data.qacc_warmstart[:]=warm
        obs=self._observations();auxobs=self.aux.mechanical_state
        if any(abs(obs['brake_piston_displacements_m'][n]-auxobs['brake_piston_displacements_m'][n])>1e-12 for n in PISTONS):raise ValueError('gas/piston endpoint mismatch')
        if abs(obs['pantograph_q_rad']-auxobs['pantograph_q_rad'])>1e-12 or any(abs(obs['air_spring_displacements_m'][n]-auxobs['air_spring_displacements_m'][n])>1e-12 for n in AIR_NAMES):raise ValueError('gas/mechanical endpoint mismatch')
        warnings=value['mujoco_warning_state']
        if type(warnings) is not list or len(warnings)!=len(self.data.warning):raise ValueError('warning schema mismatch')
        for w,v in zip(self.data.warning,warnings):
            if type(v) is not dict or set(v)!={'number','lastinfo'} or any(type(x) is not int or x<0 for x in v.values()):raise ValueError('invalid warning record')
            w.number,w.lastinfo=v['number'],v['lastinfo']
        if not np.isfinite(np.r_[self.data.qpos,self.data.qvel,self.data.qacc]).all():raise ValueError('nonfinite restored state')
        if self.energy0!=initial_energy_reference:raise ValueError('Changed fixed initial mechanical energy reference')
        if not np.array_equal(self.warnings,np.array([w.number for w in self.data.warning])) or any(self.warnings):raise ValueError('Accepted checkpoint cannot contain solver warnings')
        if self.steps and (type(self.last_coupling['iterations']) is not int or not 1<=self.last_coupling['iterations']<=self.acceptance['max_iterations']):raise ValueError('Invalid coupling iteration count')
        for field,tolerance in (('force_iteration_error_N',self.acceptance['force_N']),('torque_iteration_error_Nm',self.acceptance['torque_Nm']),('brake_force_iteration_error_N',self.acceptance['brake_force_N'])):
            if not 0<=self.last_coupling[field]<=tolerance:raise ValueError('Invalid coupling convergence error')
        for motor,last in zip(self.motors,self.last_motor):
            audit=motor.audit();l=motor.ledger
            if motor.current_A!=last['current_A']:raise ValueError('Motor current/last preview mismatch')
            if l['copper_loss_J']<0 or l['freewheel_diode_loss_J']<0:raise ValueError('Negative motor dissipation')
            if abs(audit['electrical_balance_residual_J'])>1e-8:raise ValueError('Motor electrical ledger mismatch')
            if abs(l['actual_axle_mechanical_work_J']-l['electromagnetic_conversion_work_J']-l['coupling_split_residual_J'])>1e-9:raise ValueError('Motor conversion split mismatch')
            if abs(last['magnetic_energy_J']-.5*motor.p.inductance_H*motor.current_A**2)>1e-10 or abs(last['electrical_balance_residual_J'])>1e-9:raise ValueError('Last motor magnetic energy mismatch')
            if abs(last['torque_Nm']-motor.p.gear_ratio_motor_per_axle*motor.p.torque_constant_Nm_A*last['current_mean_A'])>1e-12:raise ValueError('Motor torque/current mismatch')
        if abs(sum(m.ledger['actual_axle_mechanical_work_J'] for m in self.motors)-self.work[0])>1e-8:raise ValueError('Motor/mechanical work mismatch')
        if abs(self.port_work[0]-self.work[5])>1e-9 or abs(sum(self.port_work[1:3])-self.work[6])>1e-9 or abs(sum(self.port_work[3:])-self.work[10])>1e-9:raise ValueError('Cumulative gas port work/column mismatch')
        for i,n in enumerate(('pantograph_rotary',*AIR_NAMES,*PISTONS)):
            if abs(self.port_work[i]-self.aux.gas.node_useful_work_J[n])>max(1,self.steps)*self.acceptance['port_work_J']:raise ValueError('Cumulative gas/mechanical port mismatch')
        if abs(sum(self.constraint_work_parts)-self.work[1]-self.work[4])>1e-8:raise ValueError('Constraint work split mismatch')
        # Verify restored native-brake work and effort independently of hashes.
        r=self.last_native_brake;ledger=self.native_brake_ledger
        if abs(ledger['native_contact_total_work_J']-ledger['normal_work_J']-ledger['tangent_work_J'])>1e-9:raise ValueError('Native contact cumulative work split mismatch')
        if ledger['max_positive_tangent_step_J']>1e-10 or ledger['max_positive_tangent_step_J']<0:raise ValueError('Nonpassive brake history')
        if abs(sum(self.aux.gas.node_useful_work_J[n] for n in PISTONS)+ledger['coupling_residual_J']-ledger['piston_gas_work_J'])>1e-8:raise ValueError('Cumulative piston gas/mechanical mismatch')
        if abs(ledger['coupling_residual_J'])>max(1,self.steps)*4*self.acceptance['port_work_J']:raise ValueError('Invalid cumulative piston residual')
        if any(abs(z['torque_Nm']-r['per_axle_contact_torque_Nm'][i])>1e-12 for i,z in enumerate(self.last_brake)):raise ValueError('Last brake native torque mismatch')
        if not np.allclose(self.data.ctrl[self.axle_actuators],[z['torque_Nm'] for z in self.last_motor],atol=1e-13,rtol=0):raise ValueError('Axle effort includes unowned torque')
        expected_force=np.zeros(self.model.nv)
        for n in PISTONS:expected_force[self.piston_dofs[n]]=r['gas_piston_forces_N'][n]
        if self.mount_runtime.last:expected_force+=np.asarray(self.mount_runtime.last['generalized_force'])
        if not np.allclose(self.data.qfrc_applied,expected_force,atol=1e-13,rtol=0):raise ValueError('Piston applied force/last accepted effort mismatch')
        if self.steps:
            delta=np.zeros(self.model.nv);mujoco.mj_differentiatePos(self.model,delta,1.,np.asarray(r['qpos_start']),self.data.qpos)
            if not np.allclose(delta,r['generalized_displacement'],atol=1e-13,rtol=0):raise ValueError('Last native displacement mismatch')
            tangent=np.asarray(r['tangent_generalized']);normal=np.asarray(r['normal_generalized'])
            close=lambda a,b:abs(float(a)-float(b))<=1e-9
            if not close(tangent@delta,r['tangent_work_J']) or not close(normal@delta,r['normal_work_J']):raise ValueError('Last native work/projection mismatch')
            if r['tangent_work_J']>1e-10 or r['projection_force_residual']>1e-8 or r['projection_force_residual']<0:raise ValueError('Invalid last native guard')
            if not np.allclose(tangent[self.wheel_dofs],r['per_axle_contact_torque_Nm'],atol=1e-12,rtol=0):raise ValueError('Native axle torque mismatch')
            if sum(int(z['active']) for z in r['contacts'].values())!=r['contact_count']:raise ValueError('Native contact count mismatch')
            for c in r['contacts'].values():
                if c['normal_force_N']< -1e-12 or c['normal_force_N']!=c['wrench_contact_frame'][0] or c['tangent_force_N']!=c['wrench_contact_frame'][1:3]:raise ValueError('Native contact wrench mismatch')
            for n in PISTONS:
                if not close(r['gas_piston_forces_N'][n]*delta[self.piston_dofs[n]],r['gas_piston_work_J'][n]):raise ValueError('Piston endpoint work mismatch')
            if not close(sum(r['gas_piston_work_J'].values()),self.last_step_work[10]) or not close(r['tangent_work_J'],self.last_step_work[1]):raise ValueError('Last work column mismatch')
            if not close(r['all_constraint_work_J']-r['tangent_work_J'],self.last_step_work[4]):raise ValueError('Double counted native work')
            if not close(sum(self.data.energy),r['mechanical_energy_end_J']) or not close(r['mechanical_energy_end_J']-r['mechanical_energy_start_J']-sum(self.last_step_work),r['mechanical_unclosed_residual_J']):raise ValueError('Mechanical residual mismatch')
            gas=np.asarray(self.last_coupling['gas_endpoint_work_J']);mech=np.asarray(self.last_coupling['mechanical_endpoint_work_J'])
            if not np.allclose(mech-gas,self.last_coupling['port_work_residual_J'],atol=1e-12,rtol=0) or max(abs(mech-gas))>self.acceptance['port_work_J']:raise ValueError('Port work residual mismatch')
        canonical(self.snapshot())

    def snapshot(self):
        value=super().snapshot()
        if not hasattr(self,'mount_runtime'):return value
        t=value['telemetry'];t['auxiliary_mount']=self.mount_runtime.snapshot();t['cable_support']=self.support.snapshot()
        t['energy_work_columns']=list(WORK_COLUMNS);t['energy_work_columns_J']=self.work.tolist()
        t['cumulative_external_work_J']=float(sum(self.work[i] for i in (0,2,5,6,7,8,9,10,11,13,16)))
        t['energy_note']='Eight observed gas ports and three native motor ports; signed native constraints and nonlinear integrator residual remain reported. Auxiliary mechanical energy lives only in MuJoCo. Mount wrench transfer is not another global power input. Yoke spring work is minus its separately reported potential change. No whole-system closure claim.'
        value['variant']='mounted_auxiliary';value['binding_subsystem']='integrated'
        return value

    def checkpoint(self):
        host=super().checkpoint();host['schema']=SCHEMA;host['sha256']=sha({k:v for k,v in host.items() if k!='sha256'})
        value={'schema':'gtsd-phase6-mounted-assembly-checkpoint-v1','plant':host,'support':self.support.checkpoint(),'auxiliary':self.mount_runtime.checkpoint()}
        return value|{'sha256':sha(value)}

    def restore(self,value):
        expected={'schema','plant','support','auxiliary','sha256'}
        if type(value) is not dict or set(value)!=expected or value['schema']!='gtsd-phase6-mounted-assembly-checkpoint-v1':raise ValueError('Phase6 checkpoint schema mismatch')
        canonical(value)
        if value['sha256']!=sha({k:v for k,v in value.items() if k!='sha256'}):raise ValueError('Phase6 checkpoint checksum mismatch')
        candidate=type(self)(self.dt,include_cables=True,brake_parameters=self.brake_parameters,support_parameters=self.support_parameters,auxiliary_parameters=self.auxiliary_parameters)
        candidate.mount_runtime.restore(value['auxiliary'],validate_native=False);candidate._restore_validated_candidate(value['plant']);candidate.support.restore(value['support'])
        if candidate.steps!=candidate.mount_runtime.steps or candidate.steps!=candidate.support.steps:raise ValueError('Host/support/aux count mismatch')
        native=candidate.mount_runtime.observe()
        if native!=candidate.aux.mechanical_state['auxiliary']:raise ValueError('Native/auxiliary observed endpoint mismatch')
        if candidate.steps:
            last=candidate.mount_runtime.last
            if last['end']!=native:raise ValueError('Runtime last endpoint differs from true native state')
            start=candidate.last_native_brake
            q0=np.asarray(start['qpos_start']);v0=np.asarray(start['qvel_start']);r=candidate.mount_runtime
            observed0={'angle_rad':{n:float(q0[r.qadr[n]]) for n in ROTORS},'omega_rad_s':{n:float(v0[r.dof[n]]) for n in ROTORS},'piston_displacement_m':float(q0[r.qadr['compressor_piston']]),'piston_velocity_m_s':float(v0[r.dof['compressor_piston']])}
            if last['begin']!=observed0:raise ValueError('Aux beginning differs from native recorded start')
            if not np.allclose(last['generalized_displacement'],start['generalized_displacement'],atol=1e-13,rtol=0):raise ValueError('Aux/native interval displacement mismatch')
            scratch=mujoco.MjData(candidate.model);scratch.qpos[:]=q0;scratch.qvel[:]=v0;mujoco.mj_forward(candidate.model,scratch)
            old_data=r.data;r.data=scratch
            try:basis=r.basis()
            finally:r.data=old_data
            for n,b in basis.items():
                if not np.allclose(b['projection'],last['air_projection'][n],atol=1e-12,rtol=0) or not np.allclose(b['axis_world'],last['air_axis_world'][n],atol=1e-12,rtol=0):raise ValueError('Air world basis differs from native interval start')
            if abs(last['work_J']['compressor_gas']-candidate.aux.last_step['compressor_gas_work_J']-last['residual']['gas_J'])>1e-10:raise ValueError('Last auxiliary gas residual mismatch')
            for n in ROTORS:
                if abs(last['motor_torque_Nm'][n]-candidate.aux.last_step['auxiliary_motor_torque_Nm'][n])>candidate.acceptance['auxiliary_effort']:raise ValueError('Last native/electric motor torque mismatch')
                if abs(last['motor_work_J'][n]-candidate.aux.last_step['motor_shaft_work_J'][n]-last['motor_residual_J'][n])>1e-10:raise ValueError('Last native/electric motor work mismatch')
        for i,n in enumerate(AUX_COLUMNS):
            if abs(candidate.work[len(BASE_WORK_COLUMNS)+i]-candidate.mount_runtime.work[n])>1e-9:raise ValueError('Auxiliary work double-count/mismatch')
        motors={'compressor_rotor':candidate.aux.network.pump.motor,**{n+'_rotor':f.motor for n,f in candidate.aux.network.fans.items()}}
        if abs(sum(m.shaft_work_J for m in motors.values())-candidate.mount_runtime.work['auxiliary_motor'])>max(1,candidate.steps)*candidate.acceptance['auxiliary_work_J']:raise ValueError('Native/electrical shaft ledger mismatch')
        if abs(candidate.mount_runtime.work['compressor_gas']-candidate.aux.gas.node_useful_work_J['compressor_cylinder'])>max(1,candidate.steps)*candidate.acceptance['auxiliary_work_J']:raise ValueError('Native compressor/gas cumulative mismatch')
        bearing=candidate.aux.network.pump.friction_heat_J+sum(f.bearing_heat_J for f in candidate.aux.network.fans.values())
        air=sum(f.air_work_J for f in candidate.aux.network.fans.values())
        if abs(candidate.mount_runtime.work['auxiliary_bearing']+bearing)>max(1,candidate.steps)*1e-10 or abs(candidate.mount_runtime.work['fan_external_air']+air)>max(1,candidate.steps)*1e-10:raise ValueError('Aux native dissipation/gas-electric ledger mismatch')
        if abs(candidate.mount_runtime.work['yoke_spring']+candidate.mount_runtime.yoke_energy(native)-candidate.mount_runtime.initial_yoke_energy_J)>1e-8:raise ValueError('Yoke conservative cumulative work mismatch')
        original=np.asarray(value['plant']['mujoco_integration_state'],dtype=float);mujoco.mj_setState(candidate.model,candidate.data,original,STATE_KIND)
        if not np.array_equal(candidate._mechanical_state(),original):raise ValueError('Exact Phase6 native restore failed')
        self.__dict__.clear();self.__dict__.update(candidate.__dict__)
