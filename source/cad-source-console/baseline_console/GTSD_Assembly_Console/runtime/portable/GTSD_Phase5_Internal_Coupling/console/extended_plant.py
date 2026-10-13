"""New-source coupled vehicle: spatial suspension, finite-volume air and four doors.

All commands act through forces/electrical ports/constraint activation. There is
one gas owner. Trial mechanics and scalar dynamics are restored until accepted.
This is an uncalibrated engineering assembly, not hardware-control software.
"""
from __future__ import annotations
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent
for path in (SOURCE, ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
from live_plant import LivePlant, STATE_KIND, pack, unpack_like, canonical, sha, im, apply_head
from pneumatic_aux.plant import PneumaticAuxiliaryParameters, PneumaticAuxiliaryPlant
from suspension_3d import model as suspension
from suspension_3d.air_coupling import AIR_NAMES, air_spring_kinematics, apply_air_forces
from doors.doors import augment_xml as add_doors, patch_binding, DoorControls, DOOR_KEYS

SCHEMA = 'gtsd-v5-extended-coupled-checkpoint-v1'
WORK_COLUMNS = ('traction_motor', 'wheel_brake', 'head_test_force', 'viscous_damping',
                'signed_solver_constraint', 'pantograph_gas', 'air_spring_gas',
                'door_motor', 'body_test_force', 'cable_test_force')


def _number(low, high):
    return {'type':'number', 'minimum':low, 'maximum':high}


def _enum(values):
    return {'type':'string', 'enum':list(values)}


class ExtendedPlant(LivePlant):
    """Controller-compatible force-driven plant. Cables are an explicit variant."""
    def __init__(self, dt=im.DEFAULT_DT, *, include_cables=False):
        if type(include_cables) is not bool:
            raise ValueError('include_cables must be boolean')
        self.include_cables = include_cables
        self.model_version = 'v5-extended-coupled-new-geometry'
        self.limits = ['全部为未标定工程等效，未验证真实硬件性能。', '四扇门质量/铰轴/锁扣与气簧和线缆参数为明确假设；未使用旧车几何。', '压缩机/风机标量动力学真实推进，但其安装反力未耦合整车；不声称全系统能量闭合。', '保留弓网接触及残余弓振荡；任何未收敛数值区间原子拒绝。']
        self.suspension_parameters = suspension.parameters({'air_enabled':True})
        metadata = {}
        def builder(h):
            _, xml, bp, pp, mp, geometry = im.build_model(h)
            xml = suspension.augment_xml(xml, self.suspension_parameters)
            xml, metadata['doors'] = add_doors(xml)
            metadata['preload'] = suspension.preload_allocation(xml, self.suspension_parameters)
            if include_cables:
                from cables.model import augment_xml
                xml, metadata['cables'] = augment_xml(xml)
            metadata['xml'] = xml
            return mujoco.MjModel.from_xml_string(xml), xml, bp, pp, mp, geometry
        def binding_builder(model, geometry):
            value = suspension.binding(model, im.binding(model, geometry))
            value = patch_binding(value, model, metadata['doors'])
            value['subsystem'] = 'integrated'
            value['variant'] = 'extended_coupled_new_geometry'
            if include_cables:
                hidden = set(metadata['cables']['hidden_source_meshes'])
                for row in value['meshes']:
                    if row['mesh_name'] in hidden:
                        row.update(mode='excluded_deformable', visible_in_dynamic_view=False,
                                   note='Replaced only by explicitly labelled finite-mass solver capsule proxies.')
                value['cable_augmentation'] = copy.deepcopy(metadata['cables'])
            return value
        super().__init__(dt, builder=builder, binding_builder=binding_builder)
        self.xml = metadata['xml']
        self.preload_allocation = metadata['preload']
        self.door_metadata = metadata['doors']
        self.doors = DoorControls(self.model, self.data, self.door_metadata)
        self.panto_qadr = int(self.model.jnt_qposadr[self.model.joint('panto_lower_hinge').id])
        self.panto_dof = int(self.model.jnt_dofadr[self.model.joint('panto_lower_hinge').id])
        self.panto_actuator = self.model.actuator('root_torque_port').id
        area = .0004
        parameters = PneumaticAuxiliaryParameters(air_spring_area_each_m2=area,
            air_spring_initial_gauge_pressure_Pa=self.preload_allocation['air_required_force_N']/(2*area))
        self.aux = PneumaticAuxiliaryPlant(parameters, initial_mechanical_state=self._observations())
        # Keep the reviewed pressure-to-friction law, discard its original gas
        # network. This alias has no independent gas advance or state ownership.
        self.brake.network = self.aux.network
        self.cables = None
        if include_cables:
            from cables.runtime import CableRuntime
            self.cables = CableRuntime(self.model, self.data, metadata['cables'])
        self.commands.update(body_test_force={'body':'payload_secondary','force_N':[0.,0.,0.], 'remaining_steps':0})
        self.work = np.zeros(len(WORK_COLUMNS))
        self.last_step_work = np.zeros(len(WORK_COLUMNS))
        self.port_work = np.zeros(3)
        self.last_coupling = {'iterations':0, 'force_iteration_error_N':0., 'torque_iteration_error_Nm':0.,
            'brake_iteration_error_Nm':0., 'gas_endpoint_work_J':[0.,0.,0.],
            'mechanical_endpoint_work_J':[0.,0.,0.], 'port_work_residual_J':[0.,0.,0.],
            'midpoint_actuator_work_J':[0.,0.,0.], 'midpoint_minus_endpoint_work_J':[0.,0.,0.]}
        self.last_body_generalized = np.zeros(self.model.nv)
        self.last_cable_generalized = np.zeros(self.model.nv)
        self.acceptance = {'max_iterations':24, 'force_N':1e-7, 'torque_Nm':1e-9,
                           'brake_torque_Nm':1e-10, 'feedback_rad_s':1e-10, 'port_work_J':1e-10}
        self.command_registry.update({
            'pantograph':{'label':'弓气阀（真实压力力矩）','args':{'mode':_enum(('fill','hold','vent','quick_vent'))}},
            'air_spring':{'label':'二系气簧阀','args':{'mode':_enum(('fill','hold','vent'))}},
            'leak':{'label':'气路泄漏故障开口','args':{'subsystem':_enum(('reservoir','brake','pantograph','air_spring')),'fraction':_number(0.,1.)}},
            'coil_failure':{'label':'阀线圈断电故障','args':{'spool':_enum(self.aux.network.spools),'failed':{'type':'boolean'}}},
            'compressor':{'label':'压缩机电源/自动压力开关','args':{'automatic':{'type':'boolean'},'voltage_fraction':_number(0.,1.),'power_failed':{'type':'boolean'}}},
            'compressor_valve_fault':{'label':'压缩机止回阀卡闭','args':{'port':_enum(('intake','discharge')),'failed_closed':{'type':'boolean'}}},
            'fan':{'label':'辅助风机电压/断路','args':{'key':_enum(('compressor_fan','bay_fan')),'voltage_fraction':_number(0.,1.),'power_failed':{'type':'boolean'}}},
            'door':{'label':'四扇候选检修门','args':{'key':_enum((*DOOR_KEYS,'all')),'action':_enum(('open','close','release','latch','unlock'))}},
            'door_torque':{'label':'门铰轴外部力矩','args':{'key':_enum((*DOOR_KEYS,'all')),'torque_Nm':_number(-.8,.8)}},
            'body_force':{'label':'车体/构架外部试验脉冲','args':{'body':_enum(('payload_secondary','frame')),
                **{k:_number(-100.,100.) for k in ('fx_N','fy_N','fz_N')},'duration_steps':{'type':'integer','minimum':0,'maximum':100000}}},
        })
        if self.cables:
            self.command_registry.update(self.cables.command_registry)
        self.capabilities.update(integrated_dof=self.model.nv, fixed_volume_air_brake=True,
            pantograph_pneumatic_actuator=True, spatial_suspension=True, air_springs=True,
            doors=True, ideal_door_catches=True, auxiliaries=True, cables=include_cables,
            whole_vehicle_energy_closed=False, calibrated_hardware_sensors=False, hardware_control=False)
        paths = [ROOT/'extended_plant.py', SOURCE/'suspension_3d/model.py', SOURCE/'suspension_3d/air_coupling.py',
                 SOURCE/'doors/doors.py', SOURCE/'pneumatic_aux/plant.py', SOURCE/'pneumatic_aux/kernels.py']
        if include_cables:
            paths += [SOURCE/'cables'/n for n in ('model.py','runtime.py','source_routes.json')]
        self.identity.update(schema='gtsd-v5-extended-model-identity-v1',
            pneumatic_configuration=self.aux.configuration(),
            extensions={'source_sha256':{str(p.relative_to(SOURCE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
                'suspension_parameters':self.suspension_parameters,'door_identity':self.door_metadata['identity_sha256'],
                'include_cables':include_cables,'acceptance':self.acceptance,'work_columns_J':list(WORK_COLUMNS)})
        self.mass_ledger = self._mass_ledger(metadata)
        self._apply_forces(self.aux.forces(), self.last_motor, self.last_brake)
        mujoco.mj_forward(self.model, self.data)
        self.energy0 = float(sum(self.data.energy))

    def _mass_ledger(self, metadata):
        m,d=self.model,self.data
        total=float(sum(m.body_mass));com=np.sum(m.body_mass[:,None]*d.xipos,axis=0)/total
        inertia=np.zeros((3,3))
        for i in range(1,m.nbody):
            r=d.ximat[i].reshape(3,3);v=d.xipos[i]-com
            inertia+=r@np.diag(m.body_inertia[i])@r.T+m.body_mass[i]*((v@v)*np.eye(3)-np.outer(v,v))
        cable_mass=metadata.get('cables',{}).get('total_mass_kg',0.)
        return {'core_equivalent_mass_kg':67.3,'incremental_cable_mass_kg':cable_mass,
            'total_dynamic_mass_kg':total,'reference_com_world_m':com.tolist(),
            'reference_inertia_about_com_world_kgm2':inertia.tolist(),
            'bodies_kg':{m.body(i).name:float(m.body_mass[i]) for i in range(1,m.nbody) if m.body_mass[i]},
            'door_allocation':copy.deepcopy(metadata['doors']['mass_ledger']),
            'support_allocation':copy.deepcopy(self.preload_allocation),
            'boundary':'67.3 kg is an uncalibrated core budget. Four door masses are subtracted from the original 30 kg payload, not added. If enabled, explicit flexible cable bodies are additional mass excluded from that core budget by this stated engineering allocation; no physical measured total is claimed. Auxiliary lumped pump/fan mount masses and reaction torques are not separately coupled; motor rotor inertia is already joint armature.'}

    def _observations(self):
        return {'pantograph_q_rad':float(self.data.qpos[self.panto_qadr]),
            'air_spring_displacements_m':{k:v['displacement_m'] for k,v in air_spring_kinematics(self.model,self.data).items()}}

    def _mechanical_state(self):
        out=np.zeros(mujoco.mj_stateSize(self.model,STATE_KIND))
        mujoco.mj_getState(self.model,self.data,out,STATE_KIND)
        return out

    def _apply_forces(self, forces, ms, bs):
        m,d=self.model,self.data
        d.ctrl[self.axle_actuators]=[ms[i]['torque_Nm']+bs[i]['torque_Nm'] for i in range(2)]
        d.ctrl[self.panto_actuator]=forces['pantograph_torque_Nm']
        apply_air_forces(m,d,forces['air_spring_forces_N'])
        self.doors.apply_forces()
        qhead=apply_head(m,d,self.commands['head_force_N'])
        qbody=np.zeros(m.nv);qcable=np.zeros(m.nv)
        force=self.commands['body_test_force']
        if force['remaining_steps']>0:
            bid=m.body(force['body']).id;f=np.asarray(force['force_N'])
            d.xfrc_applied[bid,:3]+=f
            mujoco.mj_applyFT(m,d,f,np.zeros(3),d.xipos[bid],bid,qbody)
        if self.cables:
            self.cables.apply_forces()
            for key,f in self.cables.loads.items():
                bid=m.body(self.cables.routes[key]['segments'][-1]['body_name']).id
                mujoco.mj_applyFT(m,d,np.asarray(f),np.zeros(3),d.xipos[bid],bid,qcable)
        return qhead,qbody,qcable

    def command(self,name,args):
        from controller import validate_command
        validate_command({'session_id':'plant','request_id':'direct','action':name,'args':args},self.command_registry)
        if self.failed:
            raise ValueError('plant stopped after rejected numerical interval')
        patch={}
        if name=='drive':self.commands['drive_enabled']=args['enabled']
        elif name=='head_load':self.commands['head_force_N']=float(args['force_N'])
        elif name=='brake':patch={'brake_mode':args['mode']}
        elif name in ('pantograph','air_spring'):patch={name+'_mode':args['mode']}
        elif name=='leak':patch={'leaks':{args['subsystem']:float(args['fraction'])}}
        elif name=='coil_failure':patch={'coil_failures':{args['spool']:args['failed']}}
        elif name=='compressor':patch={'compressor_auto':args['automatic'],'compressor_voltage_fraction':float(args['voltage_fraction']),'compressor_power_failed':args['power_failed']}
        elif name=='compressor_valve_fault':patch={'compressor_'+args['port']+'_failed_closed':args['failed_closed']}
        elif name=='fan':patch={'fan_voltage_fractions':{args['key']:float(args['voltage_fraction'])},'fan_power_failed':{args['key']:args['power_failed']}}
        elif name=='body_force':self.commands['body_test_force']={'body':args['body'],'force_N':[float(args[k]) for k in ('fx_N','fy_N','fz_N')],'remaining_steps':args['duration_steps']}
        elif name=='door_torque':self.doors.apply({k:args['torque_Nm'] for k in self.doors._keys(args['key'])})
        elif name=='door':
            old=self.doors.checkpoint_state()
            result=self.doors.command('door',args)
            if args['action']=='latch' and (result is False or type(result) is dict and not all(result.values())):
                self.doors.restore(old)
                raise ValueError('Door capture rejected: every selected leaf must satisfy angle and velocity limits; no latch changed')
        elif self.cables and name in self.cables.command_registry:self.cables.command(name,args)
        else:raise ValueError('unsupported extended command')
        if patch:
            self.aux.step(0.,commands=patch,snapshot=False)
            self.commands['brake_mode']=self.aux.commands['brake_mode']

    def _dynamic(self):
        values=super()._dynamic()
        for name in ('port_work','last_coupling','last_body_generalized','last_cable_generalized'):
            values[name]=getattr(self,name)
        return values

    def _step(self):
        m,d,h=self.model,self.data,self.dt
        mujoco.mj_forward(m,d)
        accepted=self._mechanical_state()
        aux0=self.aux.checkpoint_state()
        dynamics=copy.deepcopy(self._dynamic());motor0=copy.deepcopy(self.motors)
        heat0=self.brake.brake_heat_J
        warning0=[(w.number,w.lastinfo) for w in d.warning]
        try:
            self._trial_interval(accepted,aux0)
        except Exception:
            mujoco.mj_setState(m,d,accepted,STATE_KIND)
            self.aux.restore(aux0);self.motors=motor0
            self.brake.brake_heat_J=heat0
            for name,value in dynamics.items():setattr(self,name,value)
            warm=d.qacc_warmstart.copy();mujoco.mj_forward(m,d);d.qacc_warmstart[:]=warm
            for w,(number,lastinfo) in zip(d.warning,warning0):w.number,w.lastinfo=number,lastinfo
            raise

    def _trial_interval(self,state,aux0):
        m,d,h=self.model,self.data,self.dt
        v0=d.qvel.copy();ten0=d.ten_velocity.copy();begin=self._observations()
        omega0=v0[self.wheel_dofs].copy();feedback=omega0.copy()
        old_p=np.array([self.aux.gas.pressure_Pa('brake_axle_'+str(i)) for i in (1,2)])
        mid_p=old_p.copy();forces=self.aux.forces();accepted=False
        for iteration in range(1,self.acceptance['max_iterations']+1):
            used_feedback=feedback.copy();used_forces=copy.deepcopy(forces)
            ms=[self.motors[i].preview(h,float(used_feedback[i]),self.commands['drive_enabled']) for i in range(2)]
            bs=[self.brake.force_torque(i+1,float(used_feedback[i]),float(mid_p[i])) for i in range(2)]
            mujoco.mj_setState(m,d,state,STATE_KIND);mujoco.mj_forward(m,d)
            qhead,qbody,qcable=self._apply_forces(used_forces,ms,bs)
            mujoco.mj_forward(m,d)
            gas_ports=[(self.panto_actuator,used_forces['pantograph_torque_Nm'])]+[(m.actuator(n+'_force').id,used_forces['air_spring_forces_N'][n]) for n in AIR_NAMES]
            for aid,effort in gas_ports:
                actual=float(d.actuator_force[aid])
                if abs(actual-effort)>1e-12*max(1.,abs(effort)):
                    raise RuntimeError('Pneumatic actuator effort was clipped or altered: '+m.actuator(aid).name)
            mujoco.mj_step(m,d)
            actual_constraint=d.qfrc_constraint.copy();actual_limit=np.zeros(m.nv)
            is_limit=np.isin(d.efc_type,[int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT),int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)])
            mujoco.mj_mulJacTVec(m,d,actual_limit,np.where(is_limit,d.efc_force,0.))
            vmid=.5*(v0+d.qvel);observed_feedback=vmid[self.wheel_dofs]
            mujoco.mj_forward(m,d)
            end=self._observations()
            if iteration>1:self.aux.restore(aux0)
            self.aux.step(h,end,snapshot=False)
            gas=self.aux.last_step
            equivalent={'pantograph_torque_Nm':gas['work_equivalent_pantograph_torque_Nm'],
                        'air_spring_forces_N':dict(gas['work_equivalent_air_spring_forces_N'])}
            next_p=.5*(old_p+np.array([self.aux.gas.pressure_Pa('brake_axle_'+str(i)) for i in (1,2)]))
            next_bs=[self.brake.force_torque(i+1,float(observed_feedback[i]),float(next_p[i])) for i in range(2)]
            gas_work=np.array([gas['pantograph_generalized_work_J']]+[gas['air_spring_generalized_work_J'][n] for n in AIR_NAMES])
            mechanical_work=np.array([used_forces['pantograph_torque_Nm']*(end['pantograph_q_rad']-begin['pantograph_q_rad'])]+
                [used_forces['air_spring_forces_N'][n]*(end['air_spring_displacements_m'][n]-begin['air_spring_displacements_m'][n]) for n in AIR_NAMES])
            residual=mechanical_work-gas_work
            force_error=max(abs(equivalent['air_spring_forces_N'][n]-used_forces['air_spring_forces_N'][n]) for n in AIR_NAMES)
            torque_error=abs(equivalent['pantograph_torque_Nm']-used_forces['pantograph_torque_Nm'])
            brake_error=max(abs(next_bs[i]['torque_Nm']-bs[i]['torque_Nm']) for i in range(2))
            speed_error=float(max(abs(observed_feedback-used_feedback)))
            if force_error<=self.acceptance['force_N'] and torque_error<=self.acceptance['torque_Nm'] and brake_error<=self.acceptance['brake_torque_Nm'] and speed_error<=self.acceptance['feedback_rad_s'] and max(abs(residual))<=self.acceptance['port_work_J']:
                accepted=True;break
            forces=equivalent;mid_p=next_p;feedback=observed_feedback.copy()
        if not accepted:
            raise RuntimeError('Extended coupled interval rejected: '+json.dumps({'iteration':iteration,'force_N':force_error,'torque_Nm':torque_error,'brake_Nm':brake_error,'speed_rad_s':speed_error,'port_work_J':residual.tolist()}))
        guard=im.coupling.validate_discrete_coupling_step(h,used_feedback,observed_feedback,[b['torque_Nm'] for b in bs])
        if abs(float(d.time)-(float(state[0])+h))>1e-10:raise ArithmeticError('solver did not advance exactly one coupled interval')
        if not np.isfinite(np.r_[d.qpos,d.qvel,d.qacc]).all():raise ArithmeticError('nonfinite mechanical endpoint')
        if any(w.number>int(old) for w,old in zip(d.warning,self.warnings)):
            raise ArithmeticError('MuJoCo warning raised; coupled interval rejected')
        self.last_guard=guard;self.last_corrector_error=observed_feedback-used_feedback
        self.last_constraint_generalized=actual_constraint;self.last_limit_generalized=actual_limit
        self.last_mid_velocity=vmid.copy()
        self.last_generalized_head_force=qhead;self.last_body_generalized=qbody;self.last_cable_generalized=qcable
        self.constraint_work_parts+=h*np.array([float(actual_limit@vmid),float((actual_constraint-actual_limit)@vmid)])
        ten1=d.ten_velocity.copy()
        damping=-h*float(m.dof_damping@(vmid*vmid)+m.tendon_damping@((.5*(ten0+ten1))**2))
        motor_work=sum(ms[i]['torque_Nm']*observed_feedback[i]*h for i in range(2))
        brake_work=sum(bs[i]['torque_Nm']*observed_feedback[i]*h for i in range(2))
        door_work=h*sum(d.ctrl[self.doors.actuators[k]]*vmid[self.doors.qvel_addresses[k]] for k in DOOR_KEYS)
        self.last_step_work=np.array([motor_work,brake_work,h*float(qhead@vmid),damping,h*float(actual_constraint@vmid),
            mechanical_work[0],sum(mechanical_work[1:]),door_work,h*float(qbody@vmid),h*float(qcable@vmid)])
        self.work+=self.last_step_work;self.port_work+=mechanical_work
        self.brake.brake_heat_J=-self.work[1]
        for i in range(2):self.motors[i].commit(ms[i],ms[i]['torque_Nm']*float(observed_feedback[i])*h)
        midpoint=np.array([h*used_forces['pantograph_torque_Nm']*vmid[self.panto_dof]]+
            [h*used_forces['air_spring_forces_N'][n]*.5*(ten0[m.tendon(n).id]+ten1[m.tendon(n).id]) for n in AIR_NAMES])
        self.last_coupling={'iterations':iteration,'force_iteration_error_N':force_error,'torque_iteration_error_Nm':torque_error,
            'brake_iteration_error_Nm':brake_error,'gas_endpoint_work_J':gas_work.tolist(),
            'mechanical_endpoint_work_J':mechanical_work.tolist(),'port_work_residual_J':residual.tolist(),
            'midpoint_actuator_work_J':midpoint.tolist(),'midpoint_minus_endpoint_work_J':(midpoint-mechanical_work).tolist()}
        self.last_motor,self.last_brake=ms,bs;self.last_head_force=self.commands['head_force_N'];self.steps+=1
        if self.commands['body_test_force']['remaining_steps']>0:self.commands['body_test_force']['remaining_steps']-=1
        self.warnings=np.maximum(self.warnings,[w.number for w in d.warning])

    def snapshot(self):
        value=super().snapshot();m,d=self.model,self.data
        t=value['telemetry'];t['pneumatic']=self.aux.snapshot()
        t.update(doors=self.doors.snapshot(),suspension=suspension.telemetry(m,d),
            coupling=copy.deepcopy(self.last_coupling),mass_ledger=copy.deepcopy(self.mass_ledger),
            energy_work_columns=list(WORK_COLUMNS),energy_work_columns_J=self.work.tolist(),
            cumulative_external_work_J=float(sum(self.work[i] for i in (0,1,2,5,6,7,8,9))),
            energy_residual_after_reported_work_J=float(sum(d.energy))-self.energy0-float(sum(self.work)),
            energy_note='Gas and constant-effort mechanical endpoint port work share actual displacements and are checked each step. Signed solver constraint work, midpoint-versus-endpoint quadrature difference, and total integrator residual are separate. This does not certify whole-system energy closure. Auxiliary pump/fan mounting reactions remain uncoupled.',
            calibration_status='UNCALIBRATED_ENGINEERING_EQUIVALENT')
        value['commands']['auxiliary']=copy.deepcopy(self.aux.commands)
        value['commands']['doors']=copy.deepcopy(self.doors.commands)
        value['variant']='extended';value['binding_subsystem']='integrated'
        if self.cables:
            t['cables']=self.cables.snapshot();value['solver_proxies']=t['cables']['solver_proxies']
        else:value['solver_proxies']=[]
        return value

    def checkpoint(self):
        if self.failed:raise ValueError('cannot checkpoint a rejected interval')
        payload={'schema':SCHEMA,'identity':copy.deepcopy(self.identity),'identity_sha256':sha(self.identity),
            'mujoco_integration_state':self._mechanical_state().tolist(),
            'mujoco_warning_state':[{'number':w.number,'lastinfo':w.lastinfo} for w in self.data.warning],
            'pneumatic':self.aux.checkpoint_state(),'motors':pack(self.motors),'dynamic':pack(self._dynamic()),
            'brake_heat_J':float(self.brake.brake_heat_J),'doors':self.doors.checkpoint_state(),
            'cables':self.cables.checkpoint() if self.cables else None}
        return payload|{'sha256':sha(payload)}

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
        self.aux.restore(value['pneumatic']);self.motors=unpack_like(value['motors'],self.motors,'motors')
        dynamic=unpack_like(value['dynamic'],self._dynamic(),'dynamic')
        for name,v in dynamic.items():setattr(self,name,v)
        if type(value['brake_heat_J']) not in (float,int) or not math.isfinite(value['brake_heat_J']) or value['brake_heat_J']<0:raise ValueError('invalid brake heat')
        self.brake.brake_heat_J=float(value['brake_heat_J'])
        if self.steps<0 or abs(state[0]-self.steps*self.dt)>1e-7 or abs(self.aux.gas.time_s-state[0])>1e-7 or self.aux.steps!=self.steps:raise ValueError('inconsistent time/step counts')
        self._validate_command_state()
        if self.commands['brake_mode']!=self.aux.commands['brake_mode'] or abs(self.brake.brake_heat_J+self.work[1])>1e-10:raise ValueError('inconsistent brake command/ledger')
        mujoco.mj_setState(self.model,self.data,state,STATE_KIND)
        original_ctrl=self.data.ctrl.copy();original_eq=self.data.eq_active.copy()
        self.doors.restore(value['doors'])
        if self.cables:self.cables.restore(value['cables'])
        elif value['cables'] is not None:raise ValueError('unexpected cable state')
        if not np.array_equal(original_ctrl,self.data.ctrl) or not np.array_equal(original_eq,self.data.eq_active):raise ValueError('controller/MuJoCo controls or constraints disagree')
        warm=self.data.qacc_warmstart.copy();mujoco.mj_forward(self.model,self.data);self.data.qacc_warmstart[:]=warm
        obs=self._observations();auxobs=self.aux.mechanical_state
        if abs(obs['pantograph_q_rad']-auxobs['pantograph_q_rad'])>1e-12 or any(abs(obs['air_spring_displacements_m'][n]-auxobs['air_spring_displacements_m'][n])>1e-12 for n in AIR_NAMES):raise ValueError('gas/mechanical endpoint mismatch')
        warnings=value['mujoco_warning_state']
        if type(warnings) is not list or len(warnings)!=len(self.data.warning):raise ValueError('warning schema mismatch')
        for w,v in zip(self.data.warning,warnings):
            if type(v) is not dict or set(v)!={'number','lastinfo'} or any(type(x) is not int or x<0 for x in v.values()):raise ValueError('invalid warning record')
            w.number,w.lastinfo=v['number'],v['lastinfo']
        if not np.isfinite(np.r_[self.data.qpos,self.data.qvel,self.data.qacc]).all():raise ValueError('nonfinite restored state')
        canonical(self.snapshot())

    def _validate_mechanical_state(self,state):
        m=self.model
        def component(kind):
            offset=mujoco.mj_stateSize(m,int(STATE_KIND)&(int(kind)-1))
            return state[offset:offset+mujoco.mj_stateSize(m,kind)]
        eq=component(mujoco.mjtState.mjSTATE_EQ_ACTIVE)
        if not np.isin(eq,[0.,1.]).all():raise ValueError('equality activation state must be zero or one')
        q=component(mujoco.mjtState.mjSTATE_QPOS)
        for j in range(m.njnt):
            kind=int(m.jnt_type[j]);a=int(m.jnt_qposadr[j])
            if kind==int(mujoco.mjtJoint.mjJNT_FREE):quat=q[a+3:a+7]
            elif kind==int(mujoco.mjtJoint.mjJNT_BALL):quat=q[a:a+4]
            else:continue
            if abs(float(np.linalg.norm(quat))-1.)>1e-8:raise ValueError('non-unit mechanical quaternion')

    def _validate_command_state(self):
        from controller import validate_command
        f=self.commands['body_test_force']
        checks=[('drive',{'enabled':self.commands['drive_enabled']}),
                ('brake',{'mode':self.commands['brake_mode']}),
                ('head_load',{'force_N':self.commands['head_force_N']}),
                ('body_force',{'body':f['body'],'fx_N':f['force_N'][0],
                    'fy_N':f['force_N'][1],'fz_N':f['force_N'][2],'duration_steps':f['remaining_steps']})]
        for name,args in checks:
            validate_command({'session_id':'plant','request_id':'restore','action':name,'args':args},self.command_registry)

    def restore(self,value):
        # The public restore is atomic even when called directly rather than by
        # Controller. Invalid data is never partially installed in the live object.
        candidate=type(self)(self.dt,include_cables=self.include_cables)
        candidate._restore_validated_candidate(value)
        self.__dict__.clear();self.__dict__.update(candidate.__dict__)
