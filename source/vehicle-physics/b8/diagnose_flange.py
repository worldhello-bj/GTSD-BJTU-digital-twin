"""Frozen-B8 flange-contact diagnostics. Does not modify the frozen implementation.

The driver instruments the source of simulate_full.run without changing its
force, pneumatic-volume, command, settling, or integration operations. Output
is redirected to contact_diagnostics. Contacts are inspected on EVERY step.
A contact index is only a per-step index, not a persistent manifold identity.
"""
from pathlib import Path
import sys, os, inspect, json, gzip, pickle, time, hashlib
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'vendor'))
import numpy as np
import mujoco
FROZEN_SOURCE=ROOT/'contact_diagnostics'/'frozen_project'/'vehicle-physics'/'b8'
if not (FROZEN_SOURCE/'simulate_full.py').exists():raise RuntimeError('Original frozen runner is missing; do not diagnose against a changed current model')
sys.path.insert(0,str(FROZEN_SOURCE))
import simulate_full as frozen

OUT=ROOT/'contact_diagnostics'
OUT.mkdir(exist_ok=True)

class Tracer:
 def __init__(self,name):
  self.name=name;self.out=OUT/name;self.out.mkdir(exist_ok=True)
  self.events=gzip.open(self.out/'active_contacts.jsonl.gz','wt')
  self.rows=[];self.peak=0.;self.peak_event=None;self.impulse=0.;self.vector_impulse=np.zeros(3)
  self.checkpoint_index=[];self.prev=[];self.first=None;self.last=None;self.active_steps=0
 def initialize(self,m,d,net,airrefs,start,pref,p,xml):
  self.m=m;self.d=d;self.names=[m.geom(i).name or '' for i in range(m.ngeom)]
  self.flange=np.array(['flange_' in n for n in self.names])
  self.force=np.zeros(6);self.jp=np.zeros((3,m.nv));self.jr=np.zeros_like(self.jp);self.jq=np.zeros_like(self.jp);self.jt=np.zeros_like(self.jp)
  self.start=start;self.airrefs=airrefs;self.xml=xml;self.p=p
  self.state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
  self.state=np.zeros(mujoco.mj_stateSize(m,self.state_spec))
  self.wheel_joints=[m.joint(f'{b}_{a}_axle_y').id for b in ['A','B'] for a in ['front','rear']]
  self.bodyids=[i for i in range(1,m.nbody) if m.body(i).name.endswith(('_carrier','_wheelset'))]
  self.checkpoint(m,d,net,0,0)
 def checkpoint(self,m,d,net,t,k):
  mujoco.mj_getState(m,d,self.state,self.state_spec)
  # net includes every gas mass, actual displacement, valve state, flow and audit.
  obj=dict(state=self.state.copy(),state_spec=int(self.state_spec),network=net,airrefs=self.airrefs,start=self.start,t=t,k=k,xml=self.xml,params=self.p)
  path=self.out/f'checkpoint_{k:07}.pkl'
  path.write_bytes(pickle.dumps(obj,protocol=5));self.checkpoint_index.append(dict(time_s=t,step=k,path=path.name))
 def capture(self,m,d,net,t,k,dt):
  if k and k%round(.02/dt)==0:self.checkpoint(m,d,net,t,k)
  ids=np.flatnonzero(self.flange[d.contact.geom[:,0]]|self.flange[d.contact.geom[:,1]]) if d.ncon else []
  cs=[];normal=0.;world=np.zeros(3);single=0.;mindist=0.;normals=[]
  for ci in ids:
   c=d.contact[int(ci)];active=c.efc_address>=0
   self.force[:]=0
   if active:mujoco.mj_contactForce(m,d,int(ci),self.force)
   frame=c.frame.reshape(3,3);wf=frame.T@self.force[:3]
   # Contact force convention is on geom2; report the force on the wheel/flange.
   wheel_force=wf if self.flange[c.geom[1]] else -wf
   mujoco.mj_jac(m,d,self.jp,self.jr,c.pos,int(m.geom_bodyid[c.geom[0]]))
   mujoco.mj_jac(m,d,self.jq,self.jt,c.pos,int(m.geom_bodyid[c.geom[1]]))
   rel=frame@(self.jq-self.jp)@d.qvel
   fn=max(0.,float(self.force[0]));normal+=fn;world+=wheel_force;single=max(single,fn);mindist=min(mindist,float(c.dist))
   cs.append(dict(index=int(ci),efc_address=int(c.efc_address),active=bool(active),geom_ids=c.geom.tolist(),geoms=[self.names[g] for g in c.geom],position_m=c.pos.tolist(),distance_m=float(c.dist),frame=frame.tolist(),force_contact_N=self.force.tolist(),force_on_flange_world_N=wheel_force.tolist(),relative_velocity_contact_mps=rel.tolist(),friction=c.friction.tolist(),solref=c.solref.tolist(),solimp=c.solimp.tolist(),efc_aref=d.efc_aref[c.efc_address:c.efc_address+c.dim].tolist() if active else [],efc_vel=d.efc_vel[c.efc_address:c.efc_address+c.dim].tolist() if active else [],efc_R=d.efc_R[c.efc_address:c.efc_address+c.dim].tolist() if active else [],efc_state=d.efc_state[c.efc_address:c.efc_address+c.dim].tolist() if active else []))
  niter=int(max(d.solver_niter));ii=max(0,niter-1)
  wheel_omega=[float(d.qvel[m.jnt_dofadr[j]]) for j in self.wheel_joints]
  self.rows.append([t,len(cs),sum(c['active'] for c in cs),single,normal,*world,mindist,niter,float(d.solver.gradient[ii]),float(d.solver.improvement[ii]),*wheel_omega,float(d.qvel[0]),float(d.qvel[1])])
  self.impulse+=normal*dt;self.vector_impulse+=world*dt
  event=dict(time_s=t,step=k,contacts=cs,total_normal_N=normal,total_world_on_flanges_N=world.tolist(),normal_impulse_step_Ns=normal*dt,solver_iterations=niter,solver_gradient=float(d.solver.gradient[ii]),solver_improvement=float(d.solver.improvement[ii]),nefc=d.nefc,wheel_omegas_radps=wheel_omega,carbody_velocity=d.qvel[:6].tolist(),carbody_position=d.qpos[:7].tolist())
  if cs:
   self.active_steps+=1
   if self.first is None:self.first=t
   self.last=t
   event['bodies']={m.body(i).name:dict(position_m=d.xpos[i].tolist(),cvel_com_based_spatial_vector=d.cvel[i].tolist()) for i in self.bodyids}
   # mj_objectVelocity gives rotation, translation in global orientation.
   for i in self.bodyids:
    vel=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,i,vel,0);event['bodies'][m.body(i).name]['object_velocity_world']=vel.tolist()
   self.events.write(json.dumps(event,separators=(',',':'))+'\n')
  if single>self.peak:
   self.peak=single;self.peak_event=event
   (self.out/'peak_event.json').write_text(json.dumps(event,indent=2))
   # Store complete MjData as well, preserving warmstart and solver workspaces.
   (self.out/'peak_full.pkl').write_bytes(pickle.dumps(dict(model=m,data=d,network=net,start=self.start,airrefs=self.airrefs,params=self.p,xml=self.xml),protocol=5))
   (self.out/'peak_preceding_steps.json').write_text(json.dumps(self.prev[-5:]+[event],indent=2))
   if single>20:print(f'DIAG {self.name}: new peak {single:.9g} N at {t:.9f} s; {[(c["geoms"], c["distance_m"], c["force_contact_N"][0]) for c in cs]}',flush=True)
  self.prev.append(event)
  if len(self.prev)>6:self.prev.pop(0)
 def finish(self,metrics):
  self.events.close()
  columns=['time_s','flange_contacts','active_flange_contacts','single_normal_N','sum_normal_N','world_force_x_N','world_force_y_N','world_force_z_N','minimum_flange_distance_m','solver_niter','solver_gradient','solver_improvement','A_front_omega_radps','A_rear_omega_radps','B_front_omega_radps','B_rear_omega_radps','car_vx_mps','car_vy_mps']
  np.savez_compressed(self.out/'every_step.npz',columns=columns,values=np.asarray(self.rows))
  (self.out/'checkpoint_index.json').write_text(json.dumps(self.checkpoint_index,indent=2))
  result=dict(name=self.name,peak_N=self.peak,peak_time_s=self.peak_event['time_s'] if self.peak_event else None,normal_impulse_Ns=self.impulse,world_impulse_Ns=self.vector_impulse.tolist(),active_steps=self.active_steps,first_contact_s=self.first,last_contact_s=self.last,samples=len(self.rows),frozen_peak_reproduced_N=metrics['peak_flange_single_contact_N'],frozen_model_hash=metrics['model_xml_sha256'])
  (self.out/'summary.json').write_text(json.dumps(result,indent=2));print('DIAG SUMMARY '+json.dumps(result),flush=True)
  return result

def run(name,duration=10):
 tracer=Tracer(name);source=inspect.getsource(frozen.run)
 source=source.replace('start=d.time;initial_qpos=', 'start=d.time;tracer.initialize(m,d,net,airrefs,start,pref,p,xml);initial_qpos=')
 source=source.replace('  maxgear=max(maxgear,', '  tracer.capture(m,d,net,t,k,dt)\n  maxgear=max(maxgear,')
 env=dict(frozen.__dict__);env['OUT']=tracer.out;env['tracer']=tracer
 exec(compile(source, str(ROOT/'simulate_full.py')+' [instrumented in memory]', 'exec'),env)
 metrics=env['run'](name,dict(frozen.CASES[name]),duration,False)
 return tracer.finish(metrics)

if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('--case',default='low_adhesion');ap.add_argument('--duration',type=float,default=10);args=ap.parse_args();run(args.case,args.duration)
