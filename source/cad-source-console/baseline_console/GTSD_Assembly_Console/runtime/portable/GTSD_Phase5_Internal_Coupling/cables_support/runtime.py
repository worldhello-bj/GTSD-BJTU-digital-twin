"""Finite-mass support contact, explicit force ports and atomic solver state.

Only integration and validated checkpoint restore may change generalized state.
Contacts are measured with mj_contactForce and point Jacobians, independently of
native generalized signed constraint work. Neither quantity is labelled heat.
"""
import copy, hashlib, json, math
from pathlib import Path
import mujoco
import numpy as np
from cables.runtime import CableRuntime
from .model import model, ROOT, Parameters
STATE=mujoco.mjtState.mjSTATE_INTEGRATION

def digest(v): return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def finite(v): return type(v) in (int,float) and math.isfinite(v)
def scalar(v,bound): return finite(v) and abs(v)<=bound

class Bench:
    def __init__(self,parameters=None):
        self.xml,self.spec,_=model(parameters)
        self.model=mujoco.MjModel.from_xml_string(self.xml); self.data=mujoco.MjData(self.model)
        mujoco.mj_forward(self.model,self.data)
        self.cable=CableRuntime(self.model,self.data,self.spec['cables']); self.records={'support':self.spec['supports']}
        self.support_id=self.model.body('support').id; self.support_dof=int(self.model.joint('support_y').dofadr[0]); self.support_q=int(self.model.joint('support_y').qposadr[0])
        self.support_geoms={self.model.geom(r['proxy_geom']).id for r in self.spec['supports']}
        self.dynamic_bodies=[self.support_id]+[self.model.body(s['body_name']).id for s in self.spec['cables']['routes'][0]['segments']]
        self.contact_enabled=True; self.support_load_N=0.; self.steps=0; self.contact_events=[]
        self.command_registry=copy.deepcopy(self.cable.command_registry) | {
            'support_force':{'label':'支承 y 向外部试验力（N）','args':{'fy_N':{'type':'number','minimum':-10.,'maximum':10.}}},
            'support_contact':{'label':'支承实体 AABB 接触开关（研究假设）','args':{'enabled':{'type':'boolean'}}}}
        self.controls=self.command_registry
        self.identity={'schema':'gtsd-v5-cable-support-identity-v1','solver':'INDEPENDENT_CABLE_SUPPORT','mujoco_version':mujoco.__version__,'xml_sha256':hashlib.sha256(self.xml.encode()).hexdigest(),'spec_sha256':digest(self.spec),'source_geometry_sha256':self.spec['source_geometry_sha256'],'state_kind':int(STATE),'state_size':mujoco.mj_stateSize(self.model,STATE),'dimensions':{'nq':self.model.nq,'nv':self.model.nv,'nbody':self.model.nbody,'neq':self.model.neq},'source_sha256':{str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'model.py',ROOT/'runtime.py',ROOT.parent/'cables/model.py',ROOT.parent/'cables/runtime.py',ROOT.parent/'cables/source_routes.json')}}
        self.energy0=float(sum(self.data.energy)); self.Py0=self.momentum_y(); self.impulse_y=0.; self.work_external=0.; self.work_constraint=0.; self.work_damping=0.; self.work_contact=0.; self.contact_impulse_support=np.zeros(3); self.attachment_impulse_support_y=0.; self.last_step={}
    @property
    def loads(self): return {'support_fy_N':self.support_load_N,'cables':copy.deepcopy(self.cable.loads),'support_contact':self.contact_enabled}
    @property
    def events(self): return copy.deepcopy(self.cable.events)
    def _toggle_contact(self,enabled):
        for gid in self.support_geoms:
            self.model.geom_contype[gid]=64 if enabled else 0
            self.model.geom_conaffinity[gid]=64 if enabled else 0
        self.contact_enabled=enabled
    def command(self,name,args):
        if type(args) is not dict or name not in self.command_registry or set(args)!=set(self.command_registry[name]['args']): raise ValueError('invalid cable support command schema')
        if name.startswith('cable_'): self.cable.command(name,args)
        elif name=='support_force':
            if not scalar(args['fy_N'],10): raise ValueError('finite support force within +/-10 N required')
            self.support_load_N=float(args['fy_N'])
        else:
            if type(args['enabled']) is not bool: raise ValueError('contact switch must be boolean')
            if self.contact_enabled!=args['enabled']:
                self._toggle_contact(args['enabled']); self.contact_events.append({'time_s':float(self.data.time),'enabled':args['enabled']})
    def _forces(self):
        self.data.xfrc_applied[:]=0.; self.data.qfrc_applied[:]=0.
        self.cable.apply_forces(); self.data.xfrc_applied[self.support_id,1]=self.support_load_N
    def momentum_y(self):
        total=0.
        for bid in self.dynamic_bodies:
            jp=np.zeros((3,self.model.nv)); jr=np.zeros_like(jp); mujoco.mj_jacBodyCom(self.model,self.data,jp,jr,bid)
            total+=float(self.model.body_mass[bid]*(jp@self.data.qvel)[1])
        return total
    def _contact_audit(self,velocity):
        m,d=self.model,self.data; records=[]; Q=np.zeros(m.nv); support_force=np.zeros(3); virtual_power=0.; friction_power=0.; support_count=0
        for i in range(d.ncon):
            c=d.contact[i]
            if c.efc_address<0: continue
            f=np.zeros(6); mujoco.mj_contactForce(m,d,i,f)
            R=np.asarray(c.frame).reshape(3,3).T; F=R@f[:3]; T=R@f[3:]
            g1,g2=int(c.geom1),int(c.geom2); b1,b2=int(m.geom_bodyid[g1]),int(m.geom_bodyid[g2])
            jp1=np.zeros((3,m.nv)); jr1=np.zeros_like(jp1); jp2=np.zeros_like(jp1); jr2=np.zeros_like(jp1)
            mujoco.mj_jac(m,d,jp1,jr1,c.pos,b1); mujoco.mj_jac(m,d,jp2,jr2,c.pos,b2)
            dv=(jp2-jp1)@velocity; dw=(jr2-jr1)@velocity
            generalized=(jp2-jp1).T@F+(jr2-jr1).T@T; Q+=generalized
            power=float(F@dv+T@dw); virtual_power+=power
            local_v=R.T@dv; tangential_power=float(f[1:3]@local_v[1:3]); friction_power+=tangential_power
            is_support=(g1 in self.support_geoms) != (g2 in self.support_geoms)
            on_support=F if g2 in self.support_geoms else -F if g1 in self.support_geoms else np.zeros(3)
            if is_support: support_count+=1; support_force+=on_support
            records.append({'geom1':m.geom(g1).name,'geom2':m.geom(g2).name,'distance_m':float(c.dist),'position_world_m':c.pos.tolist(),'contact_frame_rows':np.asarray(c.frame).tolist(),'force_on_geom2_world_N':F.tolist(),'force_on_geom1_world_N':(-F).tolist(),'torque_on_geom2_world_Nm':T.tolist(),'force_on_support_world_N':on_support.tolist(),'pair_force_sum_N':(F-F).tolist(),'relative_point_velocity_world_m_s':dv.tolist(),'virtual_power_W':power,'tangential_signed_power_W':tangential_power,'is_cable_support':bool(is_support)})
        # Independent solver-row projection. Elliptic contact rows have dim rows.
        row_Q=np.zeros(m.nv); jac=np.asarray(d.efc_J).reshape(d.nefc,m.nv) if d.nefc else np.empty((0,m.nv))
        for i in range(d.ncon):
            c=d.contact[i]
            if c.efc_address>=0:
                a=int(c.efc_address); row_Q+=jac[a:a+c.dim].T@d.efc_force[a:a+c.dim]
        return dict(records=records,Q=Q,row_Q=row_Q,projection_error=float(np.linalg.norm(Q-row_Q)),support_force=support_force,virtual_power=virtual_power,friction_power=friction_power,support_count=support_count)
    def step(self,n=1):
        if type(n) is not int or not 1<=n<=2500: raise ValueError('step count must be integer 1..2500')
        original=self.data; fields=('steps','energy0','Py0','impulse_y','work_external','work_constraint','work_damping','work_contact','contact_impulse_support','attachment_impulse_support_y','last_step')
        saved={k:copy.deepcopy(getattr(self,k)) for k in fields}
        self.data=copy.copy(original); self.cable.data=self.data
        try:
            for _ in range(n): self._step()
        except Exception:
            self.data=original; self.cable.data=original
            for k,v in saved.items(): setattr(self,k,v)
            raise
        return copy.deepcopy(self.last_step)
    def _step(self):
        m,d=self.model,self.data; h=float(m.opt.timestep); w0=d.warning.number.copy(); t0=float(d.time)
        self._forces(); mujoco.mj_forward(m,d)
        v0=d.qvel.copy(); E0=float(sum(d.energy)); Py0=self.momentum_y(); y0=float(d.qpos[self.support_q])
        mujoco.mj_step(m,d); mid=(v0+d.qvel)/2
        contact=self._contact_audit(mid)
        wc=float(d.qfrc_constraint@mid)*h; wd=-float(np.sum(m.dof_damping*mid**2))*h; we=0.
        for bid in self.dynamic_bodies:
            jp=np.zeros((3,m.nv)); jr=np.zeros_like(jp); mujoco.mj_jacBodyCom(m,d,jp,jr,bid)
            f=d.xfrc_applied[bid]; we+=float(f[:3]@(jp@mid)+f[3:]@(jr@mid))*h
        # The support spring is part of native potential energy. Its impulse is
        # external to system momentum; implicitfast uses new velocity damping.
        p=self.spec['parameters']; rail_spring=-p['support_spring_N_m']*y0; rail_damp=-p['support_damping_Ns_m']*float(d.qvel[self.support_dof])
        Jy=(self.support_load_N+sum(f[1] for f in self.cable.loads.values())+rail_spring+rail_damp)*h
        constraint_q=d.qfrc_constraint.copy(); attachment_y=float(constraint_q[self.support_dof]-contact['Q'][self.support_dof])
        contact_work=float(contact['virtual_power']*h)
        native_constraint_power=float(constraint_q@mid); point_power_projection_error=abs(float(contact['Q']@mid)-contact['virtual_power'])
        mujoco.mj_forward(m,d)
        if not all(np.isfinite(x).all() for x in (d.qpos,d.qvel,d.qacc,d.energy)) or np.any(d.warning.number>w0) or abs(d.time-t0-h)>1e-10: raise RuntimeError('rejected support interval: non-finite state, warning or time reset')
        self.steps+=1; self.work_external+=we; self.work_constraint+=wc; self.work_damping+=wd; self.work_contact+=contact_work; self.impulse_y+=Jy; self.contact_impulse_support+=contact['support_force']*h; self.attachment_impulse_support_y+=attachment_y*h
        E=float(sum(d.energy)); Py=self.momentum_y()
        self.last_step={'step':self.steps,'time_s':float(d.time),'dt_s':h,'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'energy_J':E,'energy_step_residual_J':E-E0-we-wc-wd,'energy_residual_J':E-self.energy0-self.work_external-self.work_constraint-self.work_damping,'external_work_step_J':we,'constraint_signed_work_step_J':wc,'joint_damping_work_step_J':wd,'contact_virtual_work_step_J':contact_work,'contact_tangential_signed_work_step_J':contact['friction_power']*h,'equality_signed_work_step_J':wc-contact_work,'native_constraint_power_W':native_constraint_power,'contact_virtual_power_projection_error_W':point_power_projection_error,'contact_generalized_force_projection_error_N':contact['projection_error'],'contact_generalized_force':contact['Q'].tolist(),'native_constraint_generalized_force':constraint_q.tolist(),'momentum_y_Ns':Py,'external_impulse_y_step_Ns':Jy,'momentum_y_step_residual_Ns':Py-Py0-Jy,'momentum_y_residual_Ns':Py-self.Py0-self.impulse_y,'support_contact_force_world_N':contact['support_force'].tolist(),'support_attachment_reaction_y_N':attachment_y,'support_spring_force_y_N':rail_spring,'support_damping_force_y_N':rail_damp,'support_position_y_m':float(d.qpos[self.support_q]),'support_velocity_y_m_s':float(d.qvel[self.support_dof]),'support_contact_count':contact['support_count'],'contact_count':len(contact['records']),'contacts':contact['records'],'routes':self.cable.snapshot()['routes'],'contact_enabled':self.contact_enabled,'warning_count':int(sum(d.warning.number))}
    def snapshot(self):
        m,d=self.model,self.data; cable=copy.deepcopy(self.cable.snapshot())
        return {'schema':'gtsd-v5-cable-support-snapshot-v1','identity':copy.deepcopy(self.identity),'scope':self.spec['scope'],'time_s':float(d.time),'steps':self.steps,'dt_s':float(m.opt.timestep),'bodies':{'support':{'position':d.xpos[self.support_id].tolist(),'quaternion':d.xquat[self.support_id].tolist()}},'solver_proxies':cable['solver_proxies'],'hidden_source_meshes':cable['hidden_source_meshes'],'routes':cable['routes'],'release_events':copy.deepcopy(self.cable.events),'contact_switch_events':copy.deepcopy(self.contact_events),'loads':self.loads,'energy_J':float(sum(d.energy)),'energy_residual_J':float(sum(d.energy))-self.energy0-self.work_external-self.work_constraint-self.work_damping,'momentum_y_residual_Ns':self.momentum_y()-self.Py0-self.impulse_y,'momentum_scope':'SYSTEM_Y_ONLY; OTHER_AXES_AND_TORQUES_EXCHANGE_WITH_GUIDE','contact_impulse_on_support_world_Ns':self.contact_impulse_support.tolist(),'attachment_impulse_on_support_y_Ns':self.attachment_impulse_support_y,'signed_contact_virtual_work_J':self.work_contact,'signed_constraint_work_J':self.work_constraint,'joint_damping_work_J':self.work_damping,'work_interpretation':'SIGNED_NUMERICAL_WORK; NOT_ALL_HEAT','last_step':copy.deepcopy(self.last_step),'contact_count':int(d.ncon),'warning_count':int(sum(d.warning.number))}
    def checkpoint(self):
        state=np.empty(self.identity['state_size']); mujoco.mj_getState(self.model,self.data,state,STATE)
        payload={'schema':'gtsd-v5-cable-support-checkpoint-v1','identity':copy.deepcopy(self.identity),'state':state.tolist(),'steps':self.steps,'support_load_N':self.support_load_N,'contact_enabled':self.contact_enabled,'contact_events':copy.deepcopy(self.contact_events),'cable':copy.deepcopy(self.cable.checkpoint()),'ledger':{k:copy.deepcopy(getattr(self,k)) for k in ('energy0','Py0','impulse_y','work_external','work_constraint','work_damping','work_contact','attachment_impulse_support_y')} | {'contact_impulse_support':self.contact_impulse_support.tolist()},'last_step':copy.deepcopy(self.last_step),'warning_counts':self.data.warning.number.tolist()}
        return payload | {'payload_sha256':digest(payload)}
    def restore(self,value):
        expected={'schema','identity','state','steps','support_load_N','contact_enabled','contact_events','cable','ledger','last_step','warning_counts','payload_sha256'}
        if type(value) is not dict or set(value)!=expected or value['schema']!='gtsd-v5-cable-support-checkpoint-v1' or value['identity']!=self.identity: raise ValueError('support checkpoint identity/schema mismatch')
        payload={k:v for k,v in value.items() if k!='payload_sha256'}
        try:
            if digest(payload)!=value['payload_sha256']: raise ValueError('checkpoint checksum mismatch')
        except (TypeError,OverflowError): raise ValueError('invalid checkpoint encoding')
        if type(value['state']) is not list or len(value['state'])!=self.identity['state_size'] or not all(finite(v) for v in value['state']): raise ValueError('invalid native integration state')
        if type(value['steps']) is not int or value['steps']<0 or not scalar(value['support_load_N'],10) or type(value['contact_enabled']) is not bool: raise ValueError('invalid input or step state')
        ledger=value['ledger']; ledger_keys={'energy0','Py0','impulse_y','work_external','work_constraint','work_damping','work_contact','contact_impulse_support','attachment_impulse_support_y'}
        if type(ledger) is not dict or set(ledger)!=ledger_keys: raise ValueError('invalid ledger fields')
        for k,v in ledger.items():
            if k=='contact_impulse_support':
                if type(v) is not list or len(v)!=3 or not all(finite(x) for x in v): raise ValueError('invalid contact impulse ledger')
            elif not finite(v): raise ValueError('invalid scalar ledger')
        if type(value['warning_counts']) is not list or len(value['warning_counts'])!=len(self.data.warning.number) or any(type(v) is not int or v!=0 for v in value['warning_counts']): raise ValueError('warning-bearing checkpoints are not admitted')
        trial=Bench(self.spec['parameters'])
        if not math.isclose(ledger['energy0'],trial.energy0,rel_tol=1e-12,abs_tol=1e-12) or ledger['Py0']!=trial.Py0: raise ValueError('initial energy/momentum baseline mismatch')
        if ledger['work_damping']>0: raise ValueError('positive passive damping work is impossible')
        if value['steps']==0 and any(np.any(np.asarray(v)!=0) for k,v in ledger.items() if k not in ('energy0','Py0')): raise ValueError('nonzero initial cumulative ledger')
        trial._toggle_contact(value['contact_enabled']); state=np.asarray(value['state'],dtype=float)
        one=np.empty(len(state)); zero=np.empty(len(state)); trial.data.eq_active[:]=1; mujoco.mj_getState(trial.model,trial.data,one,STATE); trial.data.eq_active[:]=0; mujoco.mj_getState(trial.model,trial.data,zero,STATE)
        if any(v not in (0.,1.) for v in state[one!=zero]): raise ValueError('invalid native equality flag')
        mujoco.mj_setState(trial.model,trial.data,state,STATE)
        if trial.data.time<0 or abs(trial.data.time-value['steps']*trial.model.opt.timestep)>1e-8: raise ValueError('time/step mismatch')
        for j in range(trial.model.njnt):
            typ=int(trial.model.jnt_type[j]); a=int(trial.model.jnt_qposadr[j]); q=trial.data.qpos[a+3:a+7] if typ==int(mujoco.mjtJoint.mjJNT_FREE) else trial.data.qpos[a:a+4] if typ==int(mujoco.mjtJoint.mjJNT_BALL) else None
            if q is not None and abs(float(np.linalg.norm(q))-1)>1e-6: raise ValueError('invalid joint quaternion')
        native_eq=trial.data.eq_active.copy(); trial.cable.restore(copy.deepcopy(value['cable']))
        if not np.array_equal(native_eq,trial.data.eq_active): raise ValueError('cable/native equality disagreement')
        events=value['contact_events']
        if type(events) is not list: raise ValueError('invalid contact event list')
        active=True; last=-1.
        for e in events:
            if type(e) is not dict or set(e)!={'time_s','enabled'} or not finite(e['time_s']) or e['time_s']<0 or e['time_s']<last or e['time_s']>trial.data.time+1e-12 or type(e['enabled']) is not bool or e['enabled']==active: raise ValueError('invalid contact event')
            active=e['enabled']; last=e['time_s']
        if active!=value['contact_enabled']: raise ValueError('contact event final state mismatch')
        last_step=value['last_step']
        if type(last_step) is not dict or (value['steps']==0 and last_step): raise ValueError('invalid last-step state')
        if value['steps']:
            if last_step.get('step')!=value['steps'] or last_step.get('time_s')!=trial.data.time or last_step.get('qpos')!=trial.data.qpos.tolist() or last_step.get('qvel')!=trial.data.qvel.tolist(): raise ValueError('last step disagrees with state')
            # Exact schema is generated by the same identity, separately from values.
            schema_probe=Bench(self.spec['parameters']); schema_probe.step()
            if set(last_step)!=set(schema_probe.last_step): raise ValueError('last step schema mismatch')
            vector_fields={'qpos':trial.model.nq,'qvel':trial.model.nv,'contact_generalized_force':trial.model.nv,'native_constraint_generalized_force':trial.model.nv,'support_contact_force_world_N':3}
            structured={'contacts','routes','contact_enabled'} | set(vector_fields)
            for k,v in last_step.items():
                if k in vector_fields:
                    if type(v) is not list or len(v)!=vector_fields[k] or not all(finite(x) for x in v): raise ValueError('invalid last-step vector')
                elif k not in structured and not finite(v): raise ValueError('invalid last-step scalar')
            for k in ('step','support_contact_count','contact_count','warning_count'):
                if type(last_step[k]) is not int or last_step[k]<0: raise ValueError('invalid last-step count')
            if type(last_step['contact_enabled']) is not bool or last_step['dt_s']!=trial.model.opt.timestep: raise ValueError('invalid last-step inputs')
            contacts=last_step['contacts']; contact_keys={'geom1','geom2','distance_m','position_world_m','contact_frame_rows','force_on_geom2_world_N','force_on_geom1_world_N','torque_on_geom2_world_Nm','force_on_support_world_N','pair_force_sum_N','relative_point_velocity_world_m_s','virtual_power_W','tangential_signed_power_W','is_cable_support'}
            if type(contacts) is not list or len(contacts)!=last_step['contact_count']: raise ValueError('invalid last-step contacts')
            for c in contacts:
                if type(c) is not dict or set(c)!=contact_keys: raise ValueError('invalid contact schema')
                for k,v in c.items():
                    if k in ('geom1','geom2'):
                        if type(v) is not str or mujoco.mj_name2id(trial.model,mujoco.mjtObj.mjOBJ_GEOM,v)<0: raise ValueError('invalid contact geom')
                    elif k=='is_cable_support':
                        if type(v) is not bool: raise ValueError('invalid contact role')
                    elif k in ('distance_m','virtual_power_W','tangential_signed_power_W'):
                        if not finite(v): raise ValueError('invalid contact scalar')
                    elif type(v) is not list or len(v)!=(9 if k=='contact_frame_rows' else 3) or not all(finite(x) for x in v): raise ValueError('invalid contact vector')
                if any(x+y!=0 for x,y in zip(c['force_on_geom1_world_N'],c['force_on_geom2_world_N'])) or any(c['pair_force_sum_N']): raise ValueError('invalid contact pair forces')
                g1=trial.model.geom(c['geom1']).id; g2=trial.model.geom(c['geom2']).id
                is_support=(g1 in trial.support_geoms)!=(g2 in trial.support_geoms)
                force=np.asarray(c['force_on_geom2_world_N']); velocity=np.asarray(c['relative_point_velocity_world_m_s']); frame=np.asarray(c['contact_frame_rows']).reshape(3,3)
                on_support=force if g2 in trial.support_geoms else -force if g1 in trial.support_geoms else np.zeros(3)
                if is_support!=c['is_cable_support'] or not np.allclose(c['force_on_support_world_N'],on_support,rtol=1e-12,atol=1e-12): raise ValueError('contact support reaction disagrees with pair')
                if not np.allclose(frame@frame.T,np.eye(3),rtol=1e-12,atol=1e-12) or any(c['torque_on_geom2_world_Nm']): raise ValueError('invalid condim-3 contact frame or torque')
                tangential=float((frame@force)[1:3]@(frame@velocity)[1:3])
                if not math.isclose(c['virtual_power_W'],float(force@velocity),rel_tol=1e-12,abs_tol=1e-12) or not math.isclose(c['tangential_signed_power_W'],tangential,rel_tol=1e-12,abs_tol=1e-12): raise ValueError('contact force/point-velocity virtual work disagreement')
            if sum(c['is_cable_support'] for c in contacts)!=last_step['support_contact_count']: raise ValueError('support contact count mismatch')
            work_checks={'constraint_signed_work_step_J':last_step['contact_virtual_work_step_J']+last_step['equality_signed_work_step_J'],'contact_virtual_work_step_J':sum(c['virtual_power_W'] for c in contacts)*trial.model.opt.timestep,'contact_tangential_signed_work_step_J':sum(c['tangential_signed_power_W'] for c in contacts)*trial.model.opt.timestep,'support_attachment_reaction_y_N':last_step['native_constraint_generalized_force'][trial.support_dof]-last_step['contact_generalized_force'][trial.support_dof]}
            if any(not math.isclose(last_step[k],v,rel_tol=1e-12,abs_tol=1e-12) for k,v in work_checks.items()) or not math.isclose(last_step['constraint_signed_work_step_J'],last_step['native_constraint_power_W']*trial.model.opt.timestep,rel_tol=1e-12,abs_tol=1e-12): raise ValueError('last-step work/reaction decomposition disagreement')
            summed_reaction=sum((np.asarray(c['force_on_support_world_N']) for c in contacts if c['is_cable_support']),np.zeros(3))
            if not np.allclose(last_step['support_contact_force_world_N'],summed_reaction,rtol=1e-12,atol=1e-12): raise ValueError('last-step contact reaction sum mismatch')
            routes=last_step['routes']
            if type(routes) is not dict or set(routes)!=set(trial.cable.routes): raise ValueError('invalid last-step routes')
            for r in routes.values():
                if type(r) is not dict or set(r)!={'endpoint_error_m','attachments_active','external_force_N','candidate_binding'} or r['candidate_binding'] is not True: raise ValueError('invalid last-step route')
                if type(r['endpoint_error_m']) is not list or len(r['endpoint_error_m'])!=2 or not all(finite(x) and x>=0 for x in r['endpoint_error_m']): raise ValueError('invalid endpoint errors')
                if type(r['attachments_active']) is not list or len(r['attachments_active'])!=2 or any(type(x) is not bool for x in r['attachments_active']): raise ValueError('invalid last-step attachments')
                if type(r['external_force_N']) is not list or len(r['external_force_N'])!=3 or not all(scalar(x,2) for x in r['external_force_N']): raise ValueError('invalid last-step loads')
        mujoco.mj_forward(trial.model,trial.data)
        if not all(np.isfinite(x).all() for x in (trial.data.qpos,trial.data.qvel,trial.data.qacc,trial.data.energy)) or any(trial.data.warning.number): raise ValueError('invalid solver checkpoint')
        if value['steps']:
            energy=float(sum(trial.data.energy)); momentum=trial.momentum_y()
            crosschecks={'energy_J':energy,'momentum_y_Ns':momentum,'energy_residual_J':energy-ledger['energy0']-ledger['work_external']-ledger['work_constraint']-ledger['work_damping'],'momentum_y_residual_Ns':momentum-ledger['Py0']-ledger['impulse_y']}
            if any(not math.isclose(last_step[k],v,rel_tol=1e-12,abs_tol=1e-12) for k,v in crosschecks.items()) or last_step['warning_count']!=0: raise ValueError('last-step/native state/cumulative ledger disagreement')
        elif not np.array_equal(trial.data.qpos,trial.model.qpos0) or np.any(trial.data.qvel): raise ValueError('non-reference initial state')
        # Forward recomputes derived arrays; reinstate ALL official native state,
        # notably warmstart, then verify exact byte-level numeric continuation.
        mujoco.mj_setState(trial.model,trial.data,state,STATE); check=np.empty_like(state); mujoco.mj_getState(trial.model,trial.data,check,STATE)
        if not np.array_equal(state,check): raise ValueError('native state restore not exact')
        trial.steps=value['steps']; trial.support_load_N=float(value['support_load_N']); trial.contact_events=copy.deepcopy(events); trial.last_step=copy.deepcopy(last_step)
        for k,v in ledger.items(): setattr(trial,k,np.asarray(v,dtype=float) if k=='contact_impulse_support' else v)
        self.__dict__=trial.__dict__
