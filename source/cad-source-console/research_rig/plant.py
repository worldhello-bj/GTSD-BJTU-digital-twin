"""Real MuJoCo finite-mount dynamics with detached identity-bound checkpoints.

Contact ownership is limited to the two explicitly admitted pin-core interfaces.
Other interfaces retain finite mounts or remain outside contact acceptance.
"""
import copy
import hashlib
from pathlib import Path
import numpy as np
import mujoco
from build_rig import build
from mass_properties import canonical_sha


class Plant:
    def __init__(self, parameters=None, root=None):
        args={} if root is None else {'root':root}
        builder=build
        if (parameters or {}).get('mode')=='pin_core_contact':
            from build_contact_rig import build as builder
        self.xml, self.contract=builder(parameters, **args)
        self.model=mujoco.MjModel.from_xml_string(self.xml)
        self.data=mujoco.MjData(self.model)
        self.state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
        self.identity=canonical_sha({'assembly_identity':self.contract['identity_sha256'],
            'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'mujoco_version':mujoco.__version__,'state_spec':int(self.state_spec),
            'nq':self.model.nq,'nv':self.model.nv})
        self.body_ids={b['name']:mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,b['name']) for b in self.contract['bodies']}
        self.steps=0
        self.ledger={'input_work_J':0.,'known_damping_J':0.,'native_constraint_work_J':0.}
        if self.contract['collision_enabled']:
            for station in ('50DF4','51409'):
                for quantity in ('normal_impulse_Ns','tangential_impulse_Ns','contact_work_J'):
                    self.ledger[station+'_'+quantity]=0.
        mujoco.mj_forward(self.model,self.data)
        self.energy_reference_J=float(sum(self.data.energy))
        self.reference_positions={name:self.data.xpos[i].copy() for name,i in self.body_ids.items()}
        self.command_force={}
        self.admission=self._verify_initial_mapping()

    def _verify_initial_mapping(self):
        worst=0.
        for b in self.contract['bindings']:
            i=self.body_ids[b['body_name']]
            error=np.max(abs(self.data.xpos[i]-b['body_reference_position_m']))
            worst=max(worst,float(error))
            if error>1e-12 or np.max(abs(self.data.xquat[i]-[1.,0.,0.,0.]))>1e-12:
                raise ValueError('Initial source/body registration failed')
        return {'mesh_bindings':len(self.contract['bindings']),'initial_pose_max_error_m':worst,
                'nq':self.model.nq,'nv':self.model.nv,'contact_admitted':self.contract['collision_enabled']}

    def _energy(self):
        return {'potential_J':float(self.data.energy[0]),'kinetic_J':float(self.data.energy[1]),
                'total_J':float(sum(self.data.energy))}

    def checkpoint(self):
        values=np.empty(mujoco.mj_stateSize(self.model,self.state_spec))
        mujoco.mj_getState(self.model,self.data,values,self.state_spec)
        return copy.deepcopy({'schema':'gtsd-rig-checkpoint-v1','identity_sha256':self.identity,
            'state_spec':int(self.state_spec),'native_state':values.tolist(),'steps':self.steps,
            'ledger':self.ledger,'energy_reference_J':self.energy_reference_J,
            'command_force':self.command_force})

    def restore(self, checkpoint):
        cp=copy.deepcopy(checkpoint)
        keys={'schema','identity_sha256','state_spec','native_state','steps','ledger','energy_reference_J','command_force'}
        if set(cp)!=keys or cp['schema']!='gtsd-rig-checkpoint-v1' or cp['identity_sha256']!=self.identity or cp['state_spec']!=int(self.state_spec):
            raise ValueError('Checkpoint identity/schema mismatch')
        values=np.asarray(cp['native_state'],float)
        if values.shape!=(mujoco.mj_stateSize(self.model,self.state_spec),) or not np.isfinite(values).all():
            raise ValueError('Invalid checkpoint native state')
        if type(cp['steps']) is not int or cp['steps']<0 or cp['steps']>100000:
            raise ValueError('Invalid checkpoint integer clock')
        if abs(values[0]-cp['steps']*self.model.opt.timestep)>1e-10:
            raise ValueError('Native time and integer clock disagree')
        if set(cp['ledger'])!=set(self.ledger) or not np.isfinite(list(cp['ledger'].values())+[cp['energy_reference_J']]).all():
            raise ValueError('Invalid checkpoint energy ledger')
        forces=self._forces(cp['command_force'])
        # Validate by forward on a new native object before changing this plant.
        candidate=mujoco.MjData(self.model)
        mujoco.mj_setState(self.model,candidate,values,self.state_spec)
        mujoco.mj_forward(self.model,candidate)
        if not np.isfinite(candidate.qacc).all(): raise ValueError('Invalid checkpoint dynamics')
        self.data=candidate
        self.steps=cp['steps'];self.ledger=cp['ledger']
        self.energy_reference_J=cp['energy_reference_J'];self.command_force=forces

    def _forces(self, loads):
        if not isinstance(loads,dict) or len(loads)>len(self.body_ids): raise ValueError('Invalid load map')
        answer={}
        for name,value in loads.items():
            if name not in self.body_ids: raise ValueError('Unknown load body '+name)
            f=np.asarray(value,dtype=float)
            if f.shape!=(3,) or not np.isfinite(f).all() or np.linalg.norm(f)>20.:
                raise ValueError('Load must be finite world force, at most 20 N')
            answer[name]=f.tolist()
        return answer

    def step(self, loads=None):
        forces=self._forces(loads or {})
        # A fixed 25,000-step cap incorrectly shortened h/2 and h/4 studies.
        # Bound physical time consistently, while retaining a hard work budget.
        if self.steps>=min(100000,int(5./self.model.opt.timestep)):
            raise ValueError('Research horizon exhausted (5 seconds or 100000 native steps)')
        before=self.checkpoint()
        try:
            return self._advance(forces)
        except Exception:
            self.restore(before)
            raise

    def _advance(self, forces):
        e0=self._energy();v0=self.data.qvel.copy()
        positions={n:self.data.xipos[self.body_ids[n]].copy() for n in forces}
        warnings=np.array([w.number for w in self.data.warning])
        self.data.qfrc_applied[:]=0;self.data.xfrc_applied[:]=0
        for name,force in forces.items():
            body=self.body_ids[name]
            mujoco.mj_applyFT(self.model,self.data,np.array(force),np.zeros(3),self.data.xipos[body],body,self.data.qfrc_applied)
        # mj_step leaves interval-used dynamics available until next mj_forward.
        mujoco.mj_step(self.model,self.data)
        used_constraint=self.data.qfrc_constraint.copy()
        warning_delta=np.array([w.number for w in self.data.warning])-warnings
        if np.any(warning_delta): raise RuntimeError('New MuJoCo warning: '+str(warning_delta.tolist()))
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError('Non-finite native state')
        v1=self.data.qvel.copy();dt=float(self.model.opt.timestep)
        used_contacts=self._used_contact_diagnostics(0.5*(v0+v1),dt)
        # Recompute resulting poses/energy, never substitute them for used forces.
        mujoco.mj_forward(self.model,self.data)
        if self.data.ncon and not self.contract['collision_enabled']: raise RuntimeError('Contact appeared in non-contact admission')
        if self.data.ncon and min(c.dist for c in self.data.contact)<-50e-6:
            raise RuntimeError('Native penetration exceeded 50 micrometre research stop')
        work=sum(float(np.dot(force,self.data.xipos[self.body_ids[n]]-positions[n])) for n,force in forces.items())
        damping=float(np.dot(self.model.dof_damping,0.5*(v0*v0+v1*v1))*dt)
        constraint=float(np.dot(used_constraint,0.5*(v0+v1))*dt)
        self.ledger['input_work_J']+=work
        self.ledger['known_damping_J']+=damping
        self.ledger['native_constraint_work_J']+=constraint
        for c in used_contacts:
            station=c['station']
            self.ledger[station+'_normal_impulse_Ns']+=c['normal_impulse_Ns']
            self.ledger[station+'_tangential_impulse_Ns']+=c['tangential_impulse_Ns']
            self.ledger[station+'_contact_work_J']+=c['work_J']
        self.steps+=1;self.command_force=copy.deepcopy(forces)
        return {'step':self.steps,'time_s':self.steps*dt,'input_world_force_N':forces,
            'energy_before':e0,'energy_after':self._energy(),
            'increment_input_work_J':work,'increment_damping_J':damping,
            'increment_native_constraint_work_J':constraint,
            'used_contacts':used_contacts,
            'contact_generalized_work_closure_error_J':sum(c['work_J'] for c in used_contacts)-constraint,
            'warning_delta':warning_delta.tolist(),'state':self.snapshot()}

    def _used_contact_diagnostics(self, velocity,dt):
        result=[]
        for i,c in enumerate(self.data.contact):
            g1,g2=int(c.geom1),int(c.geom2)
            n1=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,g1)
            n2=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,g2)
            station=next((s for s in ('50DF4','51409') if s in n1 and s in n2),None)
            if station is None:raise RuntimeError('Contact outside named force-coverage interface')
            wrench=np.zeros(6);mujoco.mj_contactForce(self.model,self.data,i,wrench)
            frame=np.asarray(c.frame).reshape(3,3)
            force=frame.T@wrench[:3];torque=frame.T@wrench[3:]
            jac=[]
            for geom in (g1,g2):
                jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
                mujoco.mj_jac(self.model,self.data,jp,jr,c.pos,int(self.model.geom_bodyid[geom]))
                jac.append((jp,jr))
            relative_velocity=(jac[1][0]-jac[0][0])@velocity
            relative_omega=(jac[1][1]-jac[0][1])@velocity
            work=float((force@relative_velocity+torque@relative_omega)*dt)
            result.append({'station':station,'geom1':n1,'geom2':n2,'signed_gap_m':float(c.dist),
                'point_world_m':c.pos.tolist(),'normal_geom1_to_geom2':frame[0].tolist(),
                'force_on_geom2_world_N':force.tolist(),'torque_on_geom2_world_Nm':torque.tolist(),
                'normal_impulse_Ns':float(wrench[0]*dt),
                'tangential_impulse_Ns':float(np.linalg.norm(wrench[1:3])*dt),'work_J':work})
        return result

    def snapshot(self):
        e=self._energy()
        residual=e['total_J']-self.energy_reference_J-self.ledger['input_work_J']-self.ledger['native_constraint_work_J']+self.ledger['known_damping_J']
        poses={n:{'position':self.data.xpos[i].tolist(),'quaternion_wxyz':self.data.xquat[i].tolist()} for n,i in self.body_ids.items()}
        mounts={}
        for j in range(self.model.njnt):
            name=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_JOINT,j)
            start=int(self.model.jnt_dofadr[j]);stop=int(self.model.jnt_dofadr[j+1]) if j+1<self.model.njnt else self.model.nv
            mounts[name]={'units':'N' if self.model.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else 'N m',
                'generalized_spring_force':self.data.qfrc_spring[start:stop].tolist(),
                'generalized_damping_force':self.data.qfrc_damper[start:stop].tolist()}
        return {'identity_sha256':self.identity,'time_s':self.steps*float(self.model.opt.timestep),
            'steps':self.steps,'qpos':self.data.qpos.tolist(),'qvel':self.data.qvel.tolist(),
            'body_poses':poses,'mount_generalized_forces':mounts,'energy':e,'ledger':copy.deepcopy(self.ledger),
            'energy_balance_residual_J':residual,'native_contact_count':int(self.data.ncon),
            'scope':self.contract['scope'],'contact_admitted':self.contract['collision_enabled'],
            'minimum_contact_gap_m':min((float(c.dist) for c in self.data.contact),default=None),
            'contact_scope':self.contract.get('collision_scope','none'),'as_built':False}
