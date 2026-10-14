"""Native MuJoCo plant for the complete photo-derived operational assembly."""
import copy
import hashlib
from pathlib import Path
import numpy as np
import mujoco
try:
    from .build_rig import build
    from .mass_properties import canonical_sha
    from .collision_guard import install as install_guard
except ImportError:
    from build_rig import build
    from mass_properties import canonical_sha
    from collision_guard import install as install_guard

class Plant:
    def __init__(self,parameters=None,root=None):
        self.xml,self.contract=build(parameters,**({} if root is None else {'root':root}))
        self.model=mujoco.MjModel.from_xml_string(self.xml);self.data=mujoco.MjData(self.model)
        self.state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
        self.identity=canonical_sha({'assembly':self.contract['identity_sha256'],'runner':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'mujoco':mujoco.__version__,'state_spec':int(self.state_spec),'nq':self.model.nq,'nv':self.model.nv})
        self.body_ids={b['name']:mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,b['name']) for b in self.contract['bodies']}
        self.collision_guard=install_guard(self.model)
        self.steps=0;self.command_force={}
        self.interfaces=[x['interface_id'] for x in self.contract['native_contact_interfaces']]
        self.ledger={'input_work_J':0.,'actuator_work_J':0.,'known_damping_J':0.,'native_constraint_work_J':0.}
        for name in self.interfaces:
            for key in ('normal_impulse_Ns','tangential_impulse_Ns','contact_work_J'):self.ledger[name+'_'+key]=0.
        self.last_contacts=[]
        mujoco.mj_forward(self.model,self.data)
        self.energy_reference_J=float(sum(self.data.energy))
        self.reference_positions={n:self.data.xpos[i].copy() for n,i in self.body_ids.items()}
        worst=max(float(np.max(abs(self.data.xpos[self.body_ids[b['body_name']]]-b['body_reference_position_m']))) for b in self.contract['bindings'])
        if worst>1e-12:raise ValueError('Source-body mapping mismatch')
        self.admission={'mesh_bindings':len(self.contract['bindings']),'initial_pose_max_error_m':worst,'nq':self.model.nq,'nv':self.model.nv,'contact_admitted':True}

    def _energy(self):return {'potential_J':float(self.data.energy[0]),'kinetic_J':float(self.data.energy[1]),'total_J':float(sum(self.data.energy))}

    def checkpoint(self):
        s=np.empty(mujoco.mj_stateSize(self.model,self.state_spec));mujoco.mj_getState(self.model,self.data,s,self.state_spec)
        return copy.deepcopy({'schema':'gtsd-operational-checkpoint-v1','identity_sha256':self.identity,'state_spec':int(self.state_spec),'native_state':s.tolist(),'steps':self.steps,'ledger':self.ledger,'energy_reference_J':self.energy_reference_J,'command_force':self.command_force,'last_contacts':self.last_contacts})

    def restore(self,checkpoint):
        cp=copy.deepcopy(checkpoint)
        keys={'schema','identity_sha256','state_spec','native_state','steps','ledger','energy_reference_J','command_force','last_contacts'}
        if set(cp)!=keys or cp['schema']!='gtsd-operational-checkpoint-v1' or cp['identity_sha256']!=self.identity or cp['state_spec']!=int(self.state_spec):raise ValueError('Checkpoint identity/schema mismatch')
        s=np.asarray(cp['native_state'],float)
        if s.shape!=(mujoco.mj_stateSize(self.model,self.state_spec),) or not np.isfinite(s).all():raise ValueError('Invalid native state')
        if type(cp['steps']) is not int or not 0<=cp['steps']<=100000 or abs(s[0]-cp['steps']*self.model.opt.timestep)>1e-9:raise ValueError('Invalid integer time')
        if set(cp['ledger'])!=set(self.ledger) or not np.isfinite(list(cp['ledger'].values())+[cp['energy_reference_J']]).all():raise ValueError('Invalid ledger')
        self._forces(cp['command_force'])
        d=mujoco.MjData(self.model);mujoco.mj_setState(self.model,d,s,self.state_spec);mujoco.mj_forward(self.model,d)
        if not np.isfinite(d.qacc).all():raise ValueError('Invalid checkpoint dynamics')
        self.data=d;self.steps=cp['steps'];self.ledger=cp['ledger'];self.energy_reference_J=cp['energy_reference_J'];self.command_force=cp['command_force'];self.last_contacts=cp['last_contacts']

    def _forces(self,loads):
        if not isinstance(loads,dict):raise ValueError('Load map required')
        out={}
        for name,value in loads.items():
            if name not in self.body_ids:raise ValueError('Unknown body '+name)
            f=np.asarray(value,float)
            if f.shape!=(3,) or not np.isfinite(f).all() or np.linalg.norm(f)>20:raise ValueError('Finite world force <=20 N required')
            out[name]=f.tolist()
        return out

    def demo_loads(self,time_s):
        """Smooth explicit body forces, independent of hidden model state."""
        t=float(time_s);c=self.contract['parameters'];out={}
        def pulse(start,width):return np.sin(np.pi*(t-start)/width)**2 if start<t<start+width else 0.
        for name in ('balise_02','balise_03'):
            if name in self.body_ids:out[name]=[0.,c['demo_force_N']*pulse(.15,.25),0.]
        for name in ('catenary_50DF4','catenary_51409'):
            out[name]=[0.,c['demo_force_N']*pulse(.35,.25),0.]
        for name in ('pin_50DF4','pin_51409'):out[name]=[0.,0.,-.02*c['demo_force_N']*pulse(.2,.2)]
        out['messenger_wire']=[0.,0.,-c['demo_force_N']*pulse(.45,.25)]
        return out

    def _drive_target(self):
        c=self.contract['parameters'];t=self.steps*float(self.model.opt.timestep);r=c['demo_ramp_s'];end=c['demo_duration_s']
        rise=.5*(1-np.cos(np.pi*min(max(t/r,0),1)))
        fall=.5*(1-np.cos(np.pi*min(max((end-t)/r,0),1)))
        return float(c['wheel_speed_m_s']*rise*fall)

    def step(self,loads=None):
        forces=self._forces(loads or {})
        if self.steps>=min(100000,int(5/self.model.opt.timestep)):raise ValueError('Five-second native horizon complete')
        before=self.checkpoint()
        try:return self._advance(forces)
        except Exception:
            self.restore(before);raise

    def _advance(self,forces):
        d=self.data;m=self.model;dt=float(m.opt.timestep);v0=d.qvel.copy();e0=self._energy()
        positions={n:d.xipos[self.body_ids[n]].copy() for n in forces};warnings=np.array([w.number for w in d.warning])
        d.qfrc_applied[:]=0;d.xfrc_applied[:]=0
        for name,f in forces.items():mujoco.mj_applyFT(m,d,np.array(f),np.zeros(3),d.xipos[self.body_ids[name]],self.body_ids[name],d.qfrc_applied)
        d.ctrl[0]=self._drive_target()
        mujoco.mj_step(m,d)
        v1=d.qvel.copy();v=.5*(v0+v1);qc=d.qfrc_constraint.copy();qa=d.qfrc_actuator.copy()
        contact=self._contacts(v,dt)
        change=np.array([w.number for w in d.warning])-warnings
        if np.any(change):raise RuntimeError('MuJoCo warning '+str(change.tolist()))
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise RuntimeError('Non-finite native dynamics')
        mujoco.mj_forward(m,d)
        work=sum(float(np.dot(f,d.xipos[self.body_ids[n]]-positions[n])) for n,f in forces.items())
        damp=float(m.dof_damping@(.5*(v0*v0+v1*v1))*dt);constraint=float(qc@v*dt);actuator=float(qa@v*dt)
        self.ledger['input_work_J']+=work;self.ledger['actuator_work_J']+=actuator;self.ledger['known_damping_J']+=damp;self.ledger['native_constraint_work_J']+=constraint
        for c in contact:
            s=c['interface_id'];self.ledger[s+'_normal_impulse_Ns']+=c['normal_impulse_Ns'];self.ledger[s+'_tangential_impulse_Ns']+=c['tangential_impulse_Ns'];self.ledger[s+'_contact_work_J']+=c['work_J']
        self.last_contacts=contact;self.steps+=1;self.command_force=copy.deepcopy(forces)
        return {'step':self.steps,'time_s':self.steps*dt,'input_world_force_N':forces,'drive_target_m_s':float(d.ctrl[0]),'energy_before':e0,'energy_after':self._energy(),'increment_input_work_J':work,'increment_actuator_work_J':actuator,'increment_damping_J':damp,'increment_native_constraint_work_J':constraint,'used_contacts':contact,'contact_generalized_work_closure_error_J':sum(c['work_J'] for c in contact)-constraint,'warning_delta':change.tolist(),'state':self.snapshot()}

    def _contacts(self,velocity,dt):
        result=[];m=self.model;d=self.data;mapping=self.contract['geometry_to_interface']
        for i,c in enumerate(d.contact):
            g1,g2=int(c.geom1),int(c.geom2);n1=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g1);n2=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g2)
            if mapping.get(n1)!=mapping.get(n2):raise RuntimeError('Unmapped contact pair '+n1+' / '+n2)
            station=mapping[n1];w=np.zeros(6);mujoco.mj_contactForce(m,d,i,w);frame=np.array(c.frame).reshape(3,3);force=frame.T@w[:3];torque=frame.T@w[3:];j=[]
            for g in (g1,g2):
                jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jac(m,d,jp,jr,c.pos,int(m.geom_bodyid[g]));j.append((jp,jr))
            rv=(j[1][0]-j[0][0])@velocity;rw=(j[1][1]-j[0][1])@velocity
            result.append({'interface_id':station,'station':station,'geom1':n1,'geom2':n2,'signed_gap_m':float(c.dist),'point_world_m':c.pos.tolist(),'normal_geom1_to_geom2':frame[0].tolist(),'force_on_geom2_world_N':force.tolist(),'torque_on_geom2_world_Nm':torque.tolist(),'normal_force_N':float(w[0]),'tangential_force_N':float(np.linalg.norm(w[1:3])),'normal_impulse_Ns':float(w[0]*dt),'tangential_impulse_Ns':float(np.linalg.norm(w[1:3])*dt),'work_J':float((force@rv+torque@rw)*dt)})
        return result

    def snapshot(self):
        d=self.data;m=self.model;e=self._energy();l=self.ledger
        residual=e['total_J']-self.energy_reference_J-l['input_work_J']-l['actuator_work_J']-l['native_constraint_work_J']+l['known_damping_J']
        poses={n:{'position':d.xpos[i].tolist(),'quaternion_wxyz':d.xquat[i].tolist()} for n,i in self.body_ids.items()}
        branches={}
        for b in self.contract['bodies']:
            delta=float(np.linalg.norm(d.xpos[self.body_ids[b['name']]]-self.reference_positions[b['name']]))
            branches.setdefault(b['branch'],[]).append(delta)
        branch_response={k:{'max_displacement_m':max(v),'mean_displacement_m':float(np.mean(v))} for k,v in branches.items()}
        interfaces={n:{'normal_force_N':0.,'tangential_force_N':0.,'normal_impulse_Ns':l[n+'_normal_impulse_Ns'],'count':0} for n in self.interfaces}
        for c in self.last_contacts:
            it=interfaces[c['interface_id']];it['normal_force_N']+=c['normal_force_N'];it['tangential_force_N']+=c['tangential_force_N'];it['count']+=1
        mounts={}
        for j in range(m.njnt):
            start=int(m.jnt_dofadr[j]);stop=int(m.jnt_dofadr[j+1]) if j+1<m.njnt else m.nv
            n=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,j);mounts[n]={'units':'N' if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else 'N m','generalized_spring_force':d.qfrc_spring[start:stop].tolist(),'generalized_damping_force':d.qfrc_damper[start:stop].tolist()}
        wheel=d.xpos[self.body_ids['wheel']]
        track=self.body_ids['track_assembly'];track_rotation=np.array(d.xmat[track]).reshape(3,3)
        wheel_in_source=track_rotation.T@(wheel-d.xpos[track])+self.reference_positions['track_assembly']
        wheel_drop=float(self.contract['parameters']['wheel_radius_m']+self.contract['parameters']['wheel_initial_clearance_m']-wheel_in_source[2])
        return {'identity_sha256':self.identity,'time_s':self.steps*float(m.opt.timestep),'steps':self.steps,'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'body_poses':poses,'mount_generalized_forces':mounts,'energy':e,'ledger':copy.deepcopy(l),'energy_balance_residual_J':float(residual),'native_contact_count':int(d.ncon),'minimum_contact_gap_m':min((float(c.dist) for c in d.contact),default=None),'contact_interfaces':interfaces,'branch_response':branch_response,'wheel_vertical_drop_m':wheel_drop,'wheel_track_relative_position_m':wheel_in_source.tolist(),'wheel_travel_m':float(wheel[0]-self.reference_positions['wheel'][0]),'wheel_position_world_m':wheel.tolist(),'wheel_drive_target_m_s':float(d.ctrl[0]),'scope':self.contract['scope'],'contact_scope':self.contract['collision_scope'],'contact_admitted':True,'as_built':False}
