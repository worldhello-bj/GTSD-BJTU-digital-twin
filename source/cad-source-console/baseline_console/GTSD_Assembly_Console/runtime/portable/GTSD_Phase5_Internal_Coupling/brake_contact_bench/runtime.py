"""Force-only controls, endpoint-volume/work coupling, atomic steps and restart.

This independent numerical bench does not import a vehicle factory, open a
hardware/network port, or map an engineering proxy to an unidentified CAD part.
"""
from __future__ import annotations
import copy,hashlib,json,math
from dataclasses import asdict
import mujoco
import numpy as np
from .model import model,BrakeParameters,ROOT,PROVENANCE,BODIES
from .kernels import b8,b9,VENDOR_HASHES,verify_vendor
STATE=mujoco.mjtState.mjSTATE_INTEGRATION
PISTONS=BODIES[1:]
SCHEMA='gtsd-independent-piston-disc-checkpoint-v1'

def canonical(value):return json.dumps(value,ensure_ascii=False,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def digest(value):return hashlib.sha256(canonical(value)).hexdigest()
def number(value,name,low=None,high=None):
    if type(value) not in (int,float) or not math.isfinite(value):raise ValueError(name+' requires a finite number')
    if (low is not None and value<low) or (high is not None and value>high):raise ValueError(name+' outside bounds')
    return float(value)
def typed_like(value,template,path='state'):
    if type(template) is dict:
        if type(value) is not dict or set(value)!=set(template):raise ValueError(path+' keys mismatch')
        return {k:typed_like(value[k],v,path+'.'+k) for k,v in template.items()}
    if type(template) is list:
        if type(value) is not list or len(value)!=len(template):raise ValueError(path+' list mismatch')
        return [typed_like(v,t,path+'[]') for v,t in zip(value,template)]
    if type(template) is float:return number(value,path)
    if type(value) is not type(template):raise ValueError(path+' type mismatch')
    return value

class MeteredGas(b8.PneumaticNetwork):
    """Pinned gas laws, stable per-node work increments (no large-total subtraction)."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.node_work_J={n:0. for n in self.volumes}
        self.interval_work_J={n:0. for n in self.volumes}
    def _move_volumes(self,positions):
        for name,x in positions.items():
            v=self.volumes[name];dv=v.piston_area_m2*(x-v.displacement_m)
            boundary=v.mass_kg*self.RT*math.log1p(dv/v.volume_m3)
            ambient=self.ambient_pressure_Pa*dv;useful=boundary-ambient
            self.node_work_J[name]+=useful;self.interval_work_J[name]+=useful
            v.displacement_m=x
            self.ledger['gas_boundary_work_J']+=boundary
            self.ledger['useful_mechanical_work_J']+=useful
            self.ledger['ambient_displacement_work_J']+=ambient
            self.ledger['gas_heat_from_bath_J']+=boundary

class RestartableValves(b9.DynamicValveNetwork):
    def __getattr__(self,name):
        gas=self.__dict__.get('gas')
        if gas is None:raise AttributeError(name)
        return getattr(gas,name)

class StepRejected(RuntimeError):pass

class Bench:
    """Owns a 3-DOF MuJoCo solver and finite gas/dynamic spool state.

    step(n) integrates a detached state and commits the entire batch atomically.
    On success data/network references can change. On failure original objects,
    controls, ledgers, time, warnings and integration state are preserved.
    """
    def __init__(self,parameters=None):
        verify_vendor();self.xml,self.p=model(parameters);self.parameters=self.p
        self.model=mujoco.MjModel.from_xml_string(self.xml);self.data=mujoco.MjData(self.model)
        mujoco.mj_forward(self.model,self.data)
        p=self.p;volumes=[b8.GasVolume('reservoir',p.reservoir_volume_m3,b8.AMBIENT_PA+p.reservoir_initial_gauge_pressure_Pa)]
        ports=[b8.Orifice('reservoir_leak','reservoir',None,p.reservoir_leak_orifice_area_m2)]
        for name in PISTONS:
            volumes.append(b8.GasVolume(name,p.chamber_dead_volume_m3,b8.AMBIENT_PA,p.piston_area_m2))
            ports.extend([b8.Orifice(name+'_supply','reservoir',name,p.supply_orifice_area_m2,one_way=True),b8.Orifice(name+'_exhaust',name,None,p.exhaust_orifice_area_m2),b8.Orifice(name+'_leak',name,None,p.chamber_leak_orifice_area_m2)])
        self.network=RestartableValves(MeteredGas(volumes,ports,max_substep_s=p.timestep_s))
        self.commands={'brake_mode':'hold','disc_torque_Nm':0.,'brake_leak':0.,'reservoir_leak':0.}
        self.command_registry={
            'brake_mode':{'label':'独立台施闸/保压/缓解','args':{'mode':{'type':'string','enum':['apply','hold','release']}}},
            'disc_torque':{'label':'独立盘轴外加力矩','args':{'torque_Nm':{'type':'number','minimum':-1.,'maximum':1.}}},
            'brake_leak':{'label':'执行缸下游泄漏口','args':{'opening':{'type':'number','minimum':0.,'maximum':1.}}},
            'reservoir_leak':{'label':'有限储气罐泄漏口','args':{'opening':{'type':'number','minimum':0.,'maximum':1.}}}}
        self.controls=self.command_registry;self.steps=0;self.last_step={};self.events=[]
        self.ledger={k:0. for k in ('gas_to_pistons_work_J','disc_drive_work_J','native_constraint_work_J','pad_normal_constraint_work_J','disc_friction_work_J','disc_friction_dissipation_J','guide_and_bearing_loss_J','coupling_work_residual_J','mechanical_balance_residual_J','native_integration_energy_defect_J','discrete_identity_residual_J','max_positive_disc_friction_step_J')}
        self.energy0=self.mechanical_energy_J()
        self.identity={'schema':'gtsd-independent-piston-disc-model-v1','solver':'INDEPENDENT_PISTON_DISC_MUJOCO','mujoco_version':mujoco.__version__,'provenance':PROVENANCE,'new_CAD_mapping_confirmed':False,'parameters':asdict(p),'xml_sha256':hashlib.sha256(self.xml.encode()).hexdigest(),'state_kind':int(STATE),'state_size':mujoco.mj_stateSize(self.model,STATE),'dimensions':{'nq':self.model.nq,'nv':self.model.nv,'nbody':self.model.nbody},'source_sha256':{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ('model.py','runtime.py','kernels.py')},'vendor_sha256':dict(VENDOR_HASHES),'spool_parameters':asdict(b9.SpoolParameters())}
        self._gas_template=self._network_state()
    def command(self,name,args):
        if type(name) is not str or name not in self.command_registry or type(args) is not dict or set(args)!=set(self.command_registry[name]['args']):raise ValueError('unknown command or invalid arguments')
        candidate=copy.deepcopy(self.commands)
        if name=='brake_mode':
            if type(args['mode']) is not str or args['mode'] not in ('apply','hold','release'):raise ValueError('invalid brake mode')
            candidate['brake_mode']=args['mode']
        elif name=='disc_torque':candidate['disc_torque_Nm']=number(args['torque_Nm'],'torque_Nm',-1.,1.)
        else:candidate[name]=number(args['opening'],'opening',0.,1.)
        self.commands=candidate
        self.events.append({'step':self.steps,'time_s':float(self.data.time),'name':name,'args':copy.deepcopy(args)})
    def _ports(self):
        result={'reservoir_leak':self.commands['reservoir_leak']}
        for name in PISTONS:
            result[name+'_supply']=float(self.commands['brake_mode']=='apply')
            result[name+'_exhaust']=float(self.commands['brake_mode']=='release')
            result[name+'_leak']=self.commands['brake_leak']
        return result
    def mechanical_energy_J(self,data=None):
        d=self.data if data is None else data;p=self.p
        return .5*p.disc_spin_inertia_kgm2*float(d.qvel[0])**2+sum(.5*p.pad_mass_kg*float(d.qvel[i])**2+.5*p.pad_return_stiffness_N_m*float(d.qpos[i])**2 for i in (1,2))
    def _contact_records(self,data):
        result=[]
        for k in range(data.ncon):
            c=data.contact[k];wrench=np.zeros(6);mujoco.mj_contactForce(self.model,data,k,wrench)
            result.append({'geoms':[self.model.geom(c.geom1).name,self.model.geom(c.geom2).name],'distance_m':float(c.dist),'position_m':c.pos.tolist(),'frame':c.frame.tolist(),'wrench_contact_frame_N_Nm':wrench.tolist(),'normal_force_N':float(wrench[0]),'tangential_force_norm_N':float(np.linalg.norm(wrench[1:3]))})
        return result
    def step(self,n=1):
        if type(n) is not int or not 1<=n<=5000:raise ValueError('step count must be integer 1..5000')
        original_data,original_network=self.data,self.network
        saved={k:copy.deepcopy(getattr(self,k)) for k in ('steps','ledger','last_step')}
        self.data=copy.copy(self.data);self.network=copy.deepcopy(self.network)
        try:
            for _ in range(n):self._step()
        except Exception:
            self.data,self.network=original_data,original_network
            for k,v in saved.items():setattr(self,k,v)
            raise
        return copy.deepcopy(self.last_step)
    def _step(self):
        m,p=self.model,self.p;d0=self.data;net0=self.network;h=p.timestep_s
        x0=d0.qpos.copy();v0=d0.qvel.copy();E0=self.mechanical_energy_J(d0)
        f0=net0.forces_N();forces=np.array([f0[n] for n in PISTONS]);ports=self._ports()
        converged=False
        for iteration in range(1,17):
            used=forces.copy();trial=copy.copy(d0)
            trial.qfrc_applied[:]=[self.commands['disc_torque_Nm'],*used]
            mujoco.mj_step(m,trial)
            if not np.isfinite(trial.qpos).all() or not np.isfinite(trial.qvel).all() or np.any(trial.warning.number>d0.warning.number) or abs(trial.time-d0.time-h)>1e-10:raise StepRejected('invalid native mechanics interval')
            candidate=copy.deepcopy(net0)
            candidate.gas.interval_work_J={n:0. for n in candidate.volumes}
            candidate.update(h,{n:float(trial.qpos[i]) for i,n in enumerate(PISTONS,1)},ports,snapshot=False)
            dx=trial.qpos[1:]-x0[1:]
            work=np.array([candidate.gas.interval_work_J[n] for n in PISTONS])
            end_force=candidate.forces_N()
            forces=np.array([work[k]/dx[k] if dx[k]!=0 else .5*(f0[n]+end_force[n]) for k,n in enumerate(PISTONS)])
            force_residual=float(np.max(np.abs(forces-used)))
            work_residual=float(np.sum(used*dx-work))
            if force_residual<=1e-8 and abs(work_residual)<=1e-12:
                converged=True;break
        if not converged:raise StepRejected('endpoint/work fixed point did not converge')
        dq=trial.qpos-x0
        constraint=trial.qfrc_constraint.copy();contacts=self._contact_records(trial)
        Wgas=float(used@dq[1:]);Wdrive=float(self.commands['disc_torque_Nm']*dq[0]);Wconstraint=float(constraint@dq)
        Wfriction=float(constraint[0]*dq[0]);Wnormal=float(constraint[1:]@dq[1:])
        # Native implicitfast integrates linear joint damping at endpoint velocity.
        damping=np.array([p.bearing_damping_Nms_rad,p.pad_guide_damping_Ns_m,p.pad_guide_damping_Ns_m])
        loss=float(np.dot(damping*trial.qvel,dq))
        E=self.mechanical_energy_J(trial);residual=E-E0-Wgas-Wdrive-Wconstraint+loss
        mass=np.array([p.disc_spin_inertia_kgm2,p.pad_mass_kg,p.pad_mass_kg])
        spring=np.array([0.,p.pad_return_stiffness_N_m*x0[1],p.pad_return_stiffness_N_m*x0[2]])
        momentum=mass*(trial.qvel-v0)-h*(trial.qfrc_applied-spring-damping*trial.qvel+constraint)
        integration_defect=-.5*float(mass@((trial.qvel-v0)**2))+.5*p.pad_return_stiffness_N_m*float(dq[1:]@dq[1:])
        identity_residual=residual-integration_defect
        # Disc normal contact is parallel to its hinge axis: its constraint work
        # is tangential friction, never a pressure-to-prescribed-torque formula.
        if Wfriction>1e-12:raise StepRejected('native disc friction added excessive discrete work')
        if loss < -1e-12:raise StepRejected('joint damper added work')
        if abs(work_residual)>1e-12 or np.max(np.abs(momentum))>1e-8:raise StepRejected('unclosed force/work coupling')
        if max(abs(float(x)) for x in trial.qpos[1:])>.02:raise StepRejected('piston left numerical research domain')
        mujoco.mj_forward(m,trial)
        if not all(np.isfinite(x).all() for x in (trial.qpos,trial.qvel,trial.qacc,trial.energy)) or np.any(trial.warning.number>d0.warning.number):raise StepRejected('invalid endpoint')
        self.data,self.network=trial,candidate;self.steps+=1
        increments={'gas_to_pistons_work_J':Wgas,'disc_drive_work_J':Wdrive,'native_constraint_work_J':Wconstraint,'pad_normal_constraint_work_J':Wnormal,'disc_friction_work_J':Wfriction,'disc_friction_dissipation_J':-Wfriction,'guide_and_bearing_loss_J':loss,'coupling_work_residual_J':work_residual,'mechanical_balance_residual_J':residual,'native_integration_energy_defect_J':integration_defect,'discrete_identity_residual_J':identity_residual}
        for k,v in increments.items():self.ledger[k]+=v
        self.ledger['max_positive_disc_friction_step_J']=max(self.ledger['max_positive_disc_friction_step_J'],Wfriction)
        self.last_step={'step':self.steps,'time_s':float(trial.time),'dt_s':h,'input':copy.deepcopy(self.commands),'valve_coil_force_commands':ports,'qpos_start':x0.tolist(),'qvel_start':v0.tolist(),'qpos':trial.qpos.tolist(),'qvel':trial.qvel.tolist(),'qacc':trial.qacc.tolist(),'used_gauge_forces_N':dict(zip(PISTONS,map(float,used))),'endpoint_gauge_forces_N':end_force,'actual_piston_delta_m':dict(zip(PISTONS,map(float,dq[1:]))),'gas_interval_work_by_node_J':dict(candidate.gas.interval_work_J),'coupling_iterations':iteration,'coupling_force_residual_N':force_residual,'coupling_work_residual_J':work_residual,'mechanical_energy_J':E,'energy_step_residual_J':residual,'native_integration_energy_defect_J':integration_defect,'discrete_identity_residual_J':identity_residual,'generalized_impulse_step_residual_Ns_Nms':momentum.tolist(),'native_constraint_generalized_force_Nm_N':constraint.tolist(),'solver_contacts':contacts,'solver_contact_count':len(contacts),'endpoint_contact_count':int(trial.ncon),'work_increments':increments,'warning_count':int(sum(trial.warning.number))}
    def snapshot(self):
        d=self.data;gas=self.network.snapshot();E=self.mechanical_energy_J()
        return {'schema':'gtsd-independent-piston-disc-snapshot-v1','identity':copy.deepcopy(self.identity),'time_s':float(d.time),'steps':self.steps,'dt_s':self.p.timestep_s,'scope':PROVENANCE,'new_CAD_mapping_confirmed':False,'vehicle_mechanically_coupled':False,'hardware_control':False,'bodies':{n:{'position':d.body(n).xpos.tolist(),'quaternion':d.body(n).xquat.tolist()} for n in BODIES},'commands':copy.deepcopy(self.commands),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'qacc':d.qacc.tolist(),'disc_angle_rad':float(d.qpos[0]),'disc_speed_rad_s':float(d.qvel[0]),'piston_displacement_m':dict(zip(PISTONS,map(float,d.qpos[1:]))),'nominal_flat_face_gap_m':{n:self.p.pad_release_gap_m-float(d.qpos[i]) for i,n in enumerate(PISTONS,1)},'gas':gas,'contacts':self._contact_records(d),'contact_count':int(d.ncon),'mechanical_energy_J':E,'ledger':copy.deepcopy(self.ledger),'energy_balance_residual_J':E-self.energy0-self.ledger['gas_to_pistons_work_J']-self.ledger['disc_drive_work_J']-self.ledger['native_constraint_work_J']+self.ledger['guide_and_bearing_loss_J'],'last_step':copy.deepcopy(self.last_step),'warning_count':int(sum(d.warning.number))}
    def _network_state(self):
        net=self.network;gas=net.gas
        return {'time_s':gas.time_s,'nodes':{n:{'mass_kg':v.mass_kg,'displacement_m':v.displacement_m} for n,v in gas.volumes.items()},'opening':{n:o.opening for n,o in gas.orifices.items()},'ledger':dict(gas.ledger),'edge_mass_kg':dict(gas.edge_mass_kg),'last_flows_kg_s':dict(gas.last_flows_kg_s),'equalization_caps':gas.equalization_caps,'substeps':gas.substeps,'node_work_J':dict(gas.node_work_J),'interval_work_J':dict(gas.interval_work_J),'commands':dict(net.commands),'spools':{n:{k:v for k,v in s.__dict__.items() if k!='parameters'} for n,s in net.spools.items()}}
    def checkpoint(self):
        state=np.empty(self.identity['state_size']);mujoco.mj_getState(self.model,self.data,state,STATE)
        result={'schema':SCHEMA,'identity':copy.deepcopy(self.identity),'state':state.tolist(),'warnings':{'number':self.data.warning.number.tolist(),'lastinfo':self.data.warning.lastinfo.tolist()},'network':self._network_state(),'steps':self.steps,'commands':copy.deepcopy(self.commands),'events':copy.deepcopy(self.events),'ledger':copy.deepcopy(self.ledger),'energy0':self.energy0,'last_step':copy.deepcopy(self.last_step)}
        return result|{'payload_sha256':digest(result)}
    def restore(self,value):
        # Decode and validate into an independent candidate. No live mutation
        # occurs before every integrity/physical/cross-state check has passed.
        if type(value) is not dict or set(value)!={'schema','identity','state','warnings','network','steps','commands','events','ledger','energy0','last_step','payload_sha256'}:raise ValueError('invalid checkpoint schema')
        payload={k:v for k,v in value.items() if k!='payload_sha256'}
        if value['schema']!=SCHEMA or value['identity']!=self.identity or value['payload_sha256']!=digest(payload):raise ValueError('checkpoint identity or payload hash mismatch')
        if type(value['steps']) is not int or value['steps']<0:raise ValueError('invalid steps')
        if type(value['state']) is not list or len(value['state'])!=self.identity['state_size']:raise ValueError('invalid native state size')
        native=np.array([number(x,'native state') for x in value['state']]);trial=mujoco.MjData(self.model)
        mujoco.mj_setState(self.model,trial,native,STATE)
        if trial.time<0 or abs(trial.time-value['steps']*self.p.timestep_s)>1e-8:raise ValueError('native time/step mismatch')
        if np.max(np.abs(trial.qpos[1:]))>.02:raise ValueError('piston outside numerical domain')
        warnings=typed_like(value['warnings'],{'number':[0]*len(trial.warning),'lastinfo':[0]*len(trial.warning)})
        if any(n!=0 for n in warnings['number']):raise ValueError('warning-bearing checkpoints are not accepted')
        gasstate=typed_like(value['network'],self._gas_template)
        if gasstate['time_s']!=trial.time or gasstate['substeps']!=value['steps'] or gasstate['equalization_caps']<0:raise ValueError('gas time/counter mismatch')
        for n,x in gasstate['nodes'].items():
            if x['mass_kg']<=0:raise ValueError('nonpositive gas mass')
            if n=='reservoir':
                if x['displacement_m']!=0:raise ValueError('reservoir cannot change volume')
            elif x['displacement_m']!=float(trial.qpos[1+PISTONS.index(n)]):raise ValueError('piston/gas coordinate mismatch')
        for values in (gasstate['opening'],gasstate['commands']):
            if any(not 0<=v<=1 for v in values.values()):raise ValueError('invalid port opening')
        commands=typed_like(value['commands'],self.commands)
        if commands['brake_mode'] not in ('apply','hold','release') or not -1<=commands['disc_torque_Nm']<=1 or not 0<=commands['brake_leak']<=1 or not 0<=commands['reservoir_leak']<=1:raise ValueError('invalid control state')
        for n,s in gasstate['spools'].items():
            if not 0<=s['command']<=1 or min(s['damping_loss_J'],s['initial_energy_J'],s['max_stop_penetration_m'])<0 or s['command']!=gasstate['commands'][n]:raise ValueError('invalid spool state')
        ledger=typed_like(value['ledger'],self.ledger);energy0=number(value['energy0'],'initial energy')
        if energy0!=0:raise ValueError('initial energy mismatch')
        if type(value['last_step']) is not dict or type(value['events']) is not list:raise ValueError('invalid records')
        json.dumps(value['last_step'],allow_nan=False)
        if value['steps'] and (value['last_step'].get('step')!=value['steps'] or value['last_step'].get('time_s')!=trial.time or value['last_step'].get('qpos')!=trial.qpos.tolist() or value['last_step'].get('qvel')!=trial.qvel.tolist()):raise ValueError('last step/native mismatch')
        if not value['steps'] and value['last_step']:raise ValueError('unexpected initial step record')
        prior=-1
        for event in value['events']:
            if type(event) is not dict or set(event)!={'step','time_s','name','args'} or type(event['step']) is not int or not prior<=event['step']<=value['steps'] or not math.isfinite(event['time_s']) or abs(event['time_s']-event['step']*self.p.timestep_s)>1e-8 or event['name'] not in self.command_registry:raise ValueError('invalid event history')
            prior=event['step']
        net=copy.deepcopy(self.network);g=net.gas
        for n,x in gasstate['nodes'].items():g.volumes[n].mass_kg=x['mass_kg'];g.volumes[n].displacement_m=x['displacement_m']
        for n,v in gasstate['opening'].items():g.orifices[n].opening=v
        for key in ('time_s','ledger','edge_mass_kg','last_flows_kg_s','equalization_caps','substeps','node_work_J','interval_work_J'):setattr(g,key,copy.deepcopy(gasstate[key]))
        net.commands=copy.deepcopy(gasstate['commands'])
        for n,state in gasstate['spools'].items():
            for k,v in state.items():setattr(net.spools[n],k,v)
            if abs(net.spools[n].opening-gasstate['opening'][n])>1e-14:raise ValueError('spool/port mismatch')
        if any(v.volume_m3<=0 for v in g.volumes.values()):raise ValueError('nonpositive volume')
        a=g.audit()
        if abs(a['mass_balance_residual_kg'])>1e-11 or abs(a['gas_energy_balance_residual_J'])>1e-7 or abs(a['exergy_balance_residual_J'])>1e-7:raise ValueError('gas ledger is inconsistent')
        mujoco.mj_forward(self.model,trial)
        if any(trial.warning.number) or not np.isfinite(trial.qacc).all():raise ValueError('invalid restored mechanics')
        # Forward computes render/sensor state; reapply every integration field,
        # including warmstart. Exact restart must not silently use new warmstart.
        mujoco.mj_setState(self.model,trial,native,STATE)
        trial.warning.number[:]=warnings['number'];trial.warning.lastinfo[:]=warnings['lastinfo']
        self._validate_restored_semantics(value,trial,net)
        self.data,self.network=trial,net;self.steps=value['steps'];self.commands=copy.deepcopy(commands);self.events=copy.deepcopy(value['events']);self.ledger=copy.deepcopy(ledger);self.energy0=energy0;self.last_step=copy.deepcopy(value['last_step'])

    def _validate_restored_semantics(self, value, trial, net):
        """Cross-check data-only checkpoints, even after a caller re-signs them."""
        p=self.p;h=p.timestep_s;steps=value['steps'];ledger=value['ledger'];last=value['last_step']
        def close(a,b,label,tol=1e-9):
            number(a,label);b=float(b)
            if not math.isfinite(b):raise ValueError('nonfinite '+label)
            if abs(a-b)>tol:raise ValueError('inconsistent '+label)
        def vec(x,name,length=3):
            if type(x) is not list or len(x)!=length:raise ValueError('invalid '+name)
            return np.array([number(z,name) for z in x])
        def control_patch(commands,name,args):
            if name not in self.command_registry or type(args) is not dict or set(args)!=set(self.command_registry[name]['args']):raise ValueError('invalid event command schema')
            commands=copy.deepcopy(commands)
            if name=='brake_mode':
                if type(args['mode']) is not str or args['mode'] not in ('apply','hold','release'):raise ValueError('invalid event brake mode')
                commands['brake_mode']=args['mode']
            elif name=='disc_torque':commands['disc_torque_Nm']=number(args['torque_Nm'],'event torque',-1,1)
            else:commands[name]=number(args['opening'],'event aperture',0,1)
            return commands
        events=value['events'];commands={'brake_mode':'hold','disc_torque_Nm':0.,'brake_leak':0.,'reservoir_leak':0.};last_input=copy.deepcopy(commands)
        for event in events:
            commands=control_patch(commands,event['name'],event['args'])
            if event['step']<steps:last_input=copy.deepcopy(commands)
        if commands!=value['commands']:raise ValueError('event history/final commands mismatch')
        for key in ('guide_and_bearing_loss_J','disc_friction_dissipation_J','max_positive_disc_friction_step_J'):
            if ledger[key]<-1e-12:raise ValueError('negative '+key)
        if ledger['max_positive_disc_friction_step_J']>1e-12:raise ValueError('nonpassive brake history')
        close(ledger['native_constraint_work_J'],ledger['pad_normal_constraint_work_J']+ledger['disc_friction_work_J'],'cumulative constraint split')
        close(ledger['disc_friction_dissipation_J'],-ledger['disc_friction_work_J'],'cumulative friction heat')
        close(ledger['gas_to_pistons_work_J']-net.gas.ledger['useful_mechanical_work_J'],ledger['coupling_work_residual_J'],'cumulative gas/mechanical work')
        close(sum(net.gas.node_work_J.values()),net.gas.ledger['useful_mechanical_work_J'],'cumulative per-node gas work')
        close(net.gas.ledger['gas_boundary_work_J']-net.gas.ledger['ambient_displacement_work_J'],net.gas.ledger['useful_mechanical_work_J'],'gas boundary work split')
        E=self.mechanical_energy_J(trial)
        balance=E-value['energy0']-ledger['gas_to_pistons_work_J']-ledger['disc_drive_work_J']-ledger['native_constraint_work_J']+ledger['guide_and_bearing_loss_J']
        close(balance,ledger['mechanical_balance_residual_J'],'cumulative mechanical energy')
        close(ledger['mechanical_balance_residual_J']-ledger['native_integration_energy_defect_J'],ledger['discrete_identity_residual_J'],'cumulative numerical defect')
        if abs(ledger['discrete_identity_residual_J'])>1e-8:raise ValueError('unclosed discrete mechanical identity')
        for name,spool in net.spools.items():
            if spool.initial_energy_J!=0 or abs(spool.audit()['balance_residual_J'])>1e-9:raise ValueError('inconsistent spool energy')
            if spool.max_stop_penetration_m+1e-12<max(-spool.position_m,spool.position_m-spool.parameters.travel_m,0):raise ValueError('inconsistent spool stop history')
        if not steps:
            if last or any(abs(v)>1e-15 for v in ledger.values()) or any(trial.qpos) or any(trial.qvel):raise ValueError('inconsistent initial mechanics')
            return
        required={'step','time_s','dt_s','input','valve_coil_force_commands','qpos_start','qvel_start','qpos','qvel','qacc','used_gauge_forces_N','endpoint_gauge_forces_N','actual_piston_delta_m','gas_interval_work_by_node_J','coupling_iterations','coupling_force_residual_N','coupling_work_residual_J','mechanical_energy_J','energy_step_residual_J','native_integration_energy_defect_J','discrete_identity_residual_J','generalized_impulse_step_residual_Ns_Nms','native_constraint_generalized_force_Nm_N','solver_contacts','solver_contact_count','endpoint_contact_count','work_increments','warning_count'}
        if set(last)!=required:raise ValueError('last-step schema mismatch')
        if type(last['step']) is not int or last['step']!=steps or last['time_s']!=trial.time or last['dt_s']!=h or last['input']!=last_input or type(last['warning_count']) is not int or last['warning_count']!=0:raise ValueError('invalid last-step identity/input')
        typed_like(last['input'],self.commands,'last input')
        x0=vec(last['qpos_start'],'start position');v0=vec(last['qvel_start'],'start velocity')
        x=vec(last['qpos'],'endpoint position');v=vec(last['qvel'],'endpoint velocity');qacc=vec(last['qacc'],'endpoint acceleration')
        if not np.array_equal(x,trial.qpos) or not np.array_equal(v,trial.qvel) or not np.allclose(qacc,trial.qacc,atol=1e-8,rtol=1e-11):raise ValueError('inconsistent endpoint mechanics')
        dx=x-x0
        if not np.allclose(dx,h*v,atol=1e-12,rtol=1e-10):raise ValueError('inconsistent native endpoint displacement')
        for key,template in [('used_gauge_forces_N',{n:0. for n in PISTONS}),('endpoint_gauge_forces_N',{n:0. for n in PISTONS}),('actual_piston_delta_m',{n:0. for n in PISTONS}),('gas_interval_work_by_node_J',{n:0. for n in net.volumes}),('valve_coil_force_commands',{n:0. for n in net.orifices})]:typed_like(last[key],template,key)
        ports={'reservoir_leak':last_input['reservoir_leak']}
        for name in PISTONS:
            ports[name+'_supply']=float(last_input['brake_mode']=='apply');ports[name+'_exhaust']=float(last_input['brake_mode']=='release');ports[name+'_leak']=last_input['brake_leak']
        if last['valve_coil_force_commands']!=ports or net.commands!=ports:raise ValueError('inconsistent last valve commands')
        if last['gas_interval_work_by_node_J']!=net.gas.interval_work_J:raise ValueError('inconsistent gas interval ledger')
        force=np.array([last['used_gauge_forces_N'][n] for n in PISTONS])
        if not np.array_equal(trial.qfrc_applied,np.r_[last_input['disc_torque_Nm'],force]):raise ValueError('inconsistent native applied effort')
        for i,name in enumerate(PISTONS):
            close(last['actual_piston_delta_m'][name],dx[i+1],'piston delta',1e-15)
            close(last['endpoint_gauge_forces_N'][name],net.forces_N()[name],'endpoint pressure force',1e-10)
        constraint=vec(last['native_constraint_generalized_force_Nm_N'],'native constraint force')
        damping=np.array([p.bearing_damping_Nms_rad,p.pad_guide_damping_Ns_m,p.pad_guide_damping_Ns_m]);mass=np.array([p.disc_spin_inertia_kgm2,p.pad_mass_kg,p.pad_mass_kg])
        spring=np.r_[0.,p.pad_return_stiffness_N_m*x0[1:]]
        old_energy=.5*float(mass@(v0*v0))+.5*p.pad_return_stiffness_N_m*float(x0[1:]@x0[1:])
        Wgas=float(force@dx[1:]);Wdrive=float(last_input['disc_torque_Nm']*dx[0]);Wconstraint=float(constraint@dx);Wfriction=float(constraint[0]*dx[0]);Wnormal=float(constraint[1:]@dx[1:]);loss=float((damping*v)@dx)
        residual=E-old_energy-Wgas-Wdrive-Wconstraint+loss
        defect=-.5*float(mass@((v-v0)**2))+.5*p.pad_return_stiffness_N_m*float(dx[1:]@dx[1:])
        coupling=Wgas-sum(net.gas.interval_work_J.values())
        increments={'gas_to_pistons_work_J':Wgas,'disc_drive_work_J':Wdrive,'native_constraint_work_J':Wconstraint,'pad_normal_constraint_work_J':Wnormal,'disc_friction_work_J':Wfriction,'disc_friction_dissipation_J':-Wfriction,'guide_and_bearing_loss_J':loss,'coupling_work_residual_J':coupling,'mechanical_balance_residual_J':residual,'native_integration_energy_defect_J':defect,'discrete_identity_residual_J':residual-defect}
        typed_like(last['work_increments'],increments,'step increments')
        for k,z in increments.items():close(last['work_increments'][k],z,'step '+k,1e-11)
        for key,z in [('mechanical_energy_J',E),('energy_step_residual_J',residual),('native_integration_energy_defect_J',defect),('discrete_identity_residual_J',residual-defect),('coupling_work_residual_J',coupling)]:close(last[key],z,key,1e-11)
        impulse=mass*(v-v0)-h*(trial.qfrc_applied-spring-damping*v+constraint)
        if not np.allclose(vec(last['generalized_impulse_step_residual_Ns_Nms'],'step impulse'),impulse,atol=1e-12,rtol=1e-10) or max(abs(impulse))>1e-8:raise ValueError('inconsistent native impulse balance')
        if type(last['coupling_iterations']) is not int or not 1<=last['coupling_iterations']<=16 or not 0<=number(last['coupling_force_residual_N'],'force residual')<=1e-8 or abs(coupling)>1e-12 or Wfriction>1e-12:raise ValueError('invalid accepted-step guard')
        if type(last['solver_contacts']) is not list or type(last['solver_contact_count']) is not int or last['solver_contact_count']!=len(last['solver_contacts']) or type(last['endpoint_contact_count']) is not int or last['endpoint_contact_count']!=trial.ncon:raise ValueError('invalid contact count')
        for c in last['solver_contacts']:
            if type(c) is not dict or set(c)!={'geoms','distance_m','position_m','frame','wrench_contact_frame_N_Nm','normal_force_N','tangential_force_norm_N'}:raise ValueError('invalid contact record')
            if type(c['geoms']) is not list or len(c['geoms'])!=2 or any(type(n) is not str or mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,n)<0 for n in c['geoms']):raise ValueError('invalid contact geom')
            number(c['distance_m'],'contact distance');vec(c['position_m'],'contact position');vec(c['frame'],'contact frame',9);w=vec(c['wrench_contact_frame_N_Nm'],'contact wrench',6)
            close(c['normal_force_N'],w[0],'contact normal',1e-12);close(c['tangential_force_norm_N'],float(np.linalg.norm(w[1:3])),'contact tangent',1e-12)
            if w[0]<-1e-10:raise ValueError('tensile normal contact')
