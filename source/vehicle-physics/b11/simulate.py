"""B11 source-cable derivative of frozen B8: coupled forward dynamics and finite air mass states. Never writes qpos/qvel.
At each dt: pressure forces drive mechanical solve, then observed new chamber
volumes and valve flow update gas states. This first-order partitioned coupling
is checked against half-step runs. Replay contains only solver-generated poses.
"""
import os,sys
os.environ.setdefault('MPLCONFIGDIR','/tmp/b8-mpl')
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'vendor'))
sys.path.append(str(ROOT.parent/'b8'))
sys.path.append(str(ROOT.parent/'b10'))
import argparse,json,csv,time,hashlib
import numpy as np,mujoco
from full_model import PARAMS as B8_PARAMS
PARAMS=B8_PARAMS|dict(cable_maximum_link_length_m=.6)
import cables
from cables import model
from cable_contact_audit import CableContactAudit
from pneumatics import GasVolume,AMBIENT_PA
from auxiliaries import make_vehicle_network
OUT=ROOT/'results';OUT.mkdir(exist_ok=True)
CASES={
 'service_brake':dict(torque=True,brake=True),
 'half_dt':dict(torque=True,brake=True,timestep_s=.000025),
 'spatial_refinement':dict(torque=True,brake=True,cable_maximum_link_length_m=.3),
 'doors_open_close':dict(torque=False,brake=False,panto=False,doors=True),
 'doors_half_dt':dict(torque=False,brake=False,panto=False,doors=True,timestep_s=.000025),
 'tether_release':dict(torque=True,brake=True,tether_release=True),
}


def run(name,cfg,duration=10.,replay=True):
 startwall=time.time()
 sources={str(f.relative_to(ROOT.parent)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [ROOT/'simulate.py',ROOT/'cables.py',ROOT/'source_cables.json',ROOT/'cable_contact_audit.py',ROOT.parent/'build_model.py',ROOT.parent/'b10/auxiliaries.py',ROOT.parent/'b9/valve_dynamics.py',*[ROOT.parent/'b8'/n for n in ('full_model.py','compliant_contact.py','pantograph.py','access_doors.py','pneumatics.py')]]}
 p=PARAMS|dict(include_pantograph=True)|{k:v for k,v in cfg.items() if k in PARAMS}
 p['secondary_z_stiffness_Npm']=p['secondary_backup_stiffness_Npm']
 xml,pspec,doorspec=model(p,return_all_specs=True);xmlhash=hashlib.sha256(xml.encode()).hexdigest();(OUT/f'{name}.xml').write_text(xml)
 m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);dt=float(m.opt.timestep)
 cable_spec=cables.LAST_SPEC
 cable_ends={r['key']:(int(m.site('cable_'+r['key']+'_end').id),int(m.site('cable_'+r['key']+'_anchor').id)) for r in cable_spec['routes']}
 tether_eq=[int(m.equality('cable_'+r['key']+'_attachment').id) for r in cable_spec['routes'] if r['key'].startswith('supply_')]
 if cfg.get('solver_tight'):m.opt.iterations=200;m.opt.tolerance=1e-12
 mujoco.mj_forward(m,d)
 door_metadata=(doorspec.to_dict()|{'car_mass_allocation':doorspec.car_mass_allocation(original_mass_kg=p['car_mass_kg'],original_inertia_kgm2=p['car_inertia_kgm2'])}) if doorspec else None
 pref=pspec.bind(m);door_state=doorspec.bind(m) if doorspec else None
 door_vs=list(door_state.qvel_addresses.values()) if door_state else []
 hc=None;hcdata=None;hc_wi=[];hc_fi=[]
 if p.get('compliant_wheel_rail'):
  from compliant_contact import CompliantWheelRail,ContactParameters
  hc=CompliantWheelRail(m,ContactParameters(p['contact_stiffness_Npm'],p['contact_dissipation_spm'],p['contact_regularization_mps'],p['wheel_rail_mu'],p['contact_distance_tolerance']))
  hcdata=hc.evaluate(d);hc_wi=[i for i,pair in enumerate(hc.pairs) if pair[-1]=='wheel'];hc_fi=[i for i,pair in enumerate(hc.pairs) if pair[-1]=='flange']
 bodyrefs={m.body(i).name:dict(position=d.xpos[i].tolist(),quaternion_wxyz=d.xquat[i].tolist()) for i in range(1,m.nbody)}
 padnames=[m.body(i).name for i in range(1,m.nbody) if m.body(i).name.endswith('_pad')]
 pads={n:(int(m.joint(n+'_slide').qposadr[0]),int(m.actuator(n+'_force').id)) for n in padnames}
 airs={f'air_{b}_{s}':(int(m.tendon(f'air_{b}_{s}').id),int(m.actuator(f'air_{b}_{s}_force').id)) for b in ['A','B'] for s in ['FL','FR','RL','RR']}
 airrefs={n:float(d.ten_length[ids[0]]) for n,ids in airs.items()}
 motoracts=[int(m.actuator(b+'_motor_torque').id) for b in p['powered_bogies']]
 motorvs=[int(m.joint(b+'_motor_z').dofadr[0]) for b in p['powered_bogies']]
 rootbid=m.body('A_carbody').id;headbid=pref.head_body_id
 trainroots=[m.body('A_carbody').id,m.body('B_carbody').id];trainmass=float(sum(m.body_subtreemass[i] for i in trainroots));train_bodies=set(i for i in range(m.nbody) if int(m.body_rootid[i]) in trainroots)
 momentum_impulse=np.zeros(3)
 root_linear_dofs=[int(m.joint(b+'_car_free').dofadr[0]) for b in ('A','B')]
 def comvelocity():
  mujoco.mj_subtreeVel(m,d)
  return sum(m.body_subtreemass[i]*d.subtree_linvel[i] for i in trainroots)/trainmass
 extras=[GasVolume(n,p['airspring_dead_volume_m3'],p['airspring_initial_pressure_Pa'],p['airspring_area_m2'],0) for n in airs]
 def displacement():return {**{n:float(d.qpos[ids[0]]) for n,ids in pads.items()},**{n:float(d.ten_length[ids[0]])-airrefs[n] for n,ids in airs.items()},'pantograph':pref.travel(d)}
 net=make_vehicle_network(padnames,extra_chambers=extras,brake_dead_volume_m3=p['brake_dead_volume_m3'],pantograph_area_m2=pspec.params['cylinder_area_m2'],brake_area_m2=p['brake_piston_area_m2'],initial_displacements_m=displacement(),reservoir_initial_pressure_Pa=cfg.get('reservoir_initial_pressure_Pa',AMBIENT_PA+500000))
 if cfg.get('brake_leak'):
  for n in pads:net.orifices[n+'_leak'].area_m2=2e-7
 def setair():
  forces=net.forces_N()
  for n,ids in pads.items():d.ctrl[ids[1]]=forces[n]
  for n,ids in airs.items():d.ctrl[ids[1]]=forces[n]
  pref.set_force(d,forces['pantograph'])
 settling_cable_contact_audit=CableContactAudit(m)
 observation_cable_contact_audit=CableContactAudit(m)
 active_cable_contact_audit=settling_cable_contact_audit
 active_cable_contact_audit.record(d)
 def step(commands):
  nonlocal hcdata,momentum_impulse
  if hcdata is not None:d.qfrc_applied[:]=hcdata['qforce']
  old=d.qvel.copy();old_tendon_vel=d.ten_velocity.copy();setair();mujoco.mj_step(m,d)
  mid=(old+d.qvel)/2
  if hcdata is not None:
   netforce=hcdata['total_force_world_N'].copy()+trainmass*m.opt.gravity
   for bid in train_bodies:netforce+=d.xfrc_applied[bid,:3]
   # World translation DOFs of each train root include all native external
   # constraints, including cable attachment and collector contact reactions.
   # Internal train contacts cancel. Cable floor force is outside train mass.
   for vi in root_linear_dofs:netforce+=d.qfrc_constraint[vi:vi+3]
   momentum_impulse+=netforce*dt
  work=dict(actuator=float(d.qfrc_actuator@mid)*dt,damping=-float((m.dof_damping*d.qvel)@mid)*dt,constraint=float(d.qfrc_constraint@mid)*dt,external=float(d.qfrc_smooth@mid)*0)
  work['compliant_contact']=float(hcdata['qforce']@mid)*dt if hcdata is not None else 0.
  work['contact_dissipation']=hcdata['normal_dissipation_power_W']*dt if hcdata is not None else 0.
  work['contact_friction']=hcdata['friction_power_W']*dt if hcdata is not None else 0.
  work['door']=sum(float(d.qfrc_actuator[j]*mid[j])*dt for j in door_vs)
  work['motor']=sum(float(d.qfrc_actuator[j]*mid[j])*dt for j in motorvs)
  # External Cartesian work is calculated at a body's COM with the solver Jacobian.
  ext=0.
  for bid in np.flatnonzero(np.any(d.xfrc_applied,axis=1)):
   if np.any(d.xfrc_applied[bid]):
    jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jacBodyCom(m,d,jp,jr,bid)
    ext+=float(d.xfrc_applied[bid,:3]@(jp@mid)+d.xfrc_applied[bid,3:]@(jr@mid))*dt
  work['external']=ext
  mujoco.mj_forward(m,d)
  active_cable_contact_audit.record(d)
  work['damping']-=float((m.tendon_damping*old_tendon_vel)@((old_tendon_vel+d.ten_velocity)/2))*dt
  net.update(dt,displacement(),commands,snapshot=False)
  if hc is not None:hcdata=hc.evaluate(d)
  if np.any(d.warning.number):raise RuntimeError('MuJoCo warning during B11 integration: '+str(d.warning.number.tolist()))
  return work
 # Passive settling with sealed bellows and vented uncommanded service cylinders.
 settlecmd={n+'_exhaust':1 for n in pads}|{'pantograph_exhaust':1}
 settle_s=cfg.get('settle_s',6.)
 for jj in range(round(settle_s/dt)):
  step(settlecmd)
  if (jj+1)%round(1/dt)==0:print(f'{name}: settling {(jj+1)*dt:.1f}s wall={time.time()-startwall:.1f}s',flush=True)
 net.close_all_valves();mujoco.mj_forward(m,d)
 active_cable_contact_audit=observation_cable_contact_audit
 active_cable_contact_audit.record(d)
 print(f'{name}: settled in {time.time()-startwall:.1f}s',flush=True)
 start=d.time;initial_qpos=d.qpos.copy();initial_qvel=d.qvel.copy();x0=float(d.qpos[0]);e0=float(sum(d.energy));contact_u0=hcdata['spring_storage_J'] if hcdata is not None else 0.;momentum_impulse[:]=0.;com_v0=comvelocity().copy();air0=net.audit()
 sums=dict(actuator=0.,motor=0.,door=0.,damping=0.,constraint=0.,external=0.,compliant_contact=0.,contact_dissipation=0.,contact_friction=0.);doors_unlocked=False;door_max={k:0. for k in door_state.actuator_ids} if door_state else {}
 rows=[];frames=[];aux_rows=[];tether_released=False;force=np.zeros(6);maxpen=0.;worst_contact=None;contactless=0;mingap=1.;maxgear=0.;maxloop=0.;maxmass=0.;maxwarnings=0;minwheelcontacts=8; wheel_partial_contact_steps=0; peakcollector=0.;peakflange=0.;raw_contactless=0
 wheel_ids=np.array([m.geom(f'{b}_{a}_wheel_{s}').id for b in ['A','B'] for a in ['front','rear'] for s in ['L','R']])
 geom_kind=np.array([3 if 'panto_' in (m.geom(i).name or '') else 2 if 'flange_' in (m.geom(i).name or '') else 0 if 'rail_' in (m.geom(i).name or '') or '_wheel_' in (m.geom(i).name or '') else 1 for i in range(m.ngeom)])
 sample=max(1,round(.01/dt));replayn=max(1,round((1/30)/dt))
 qadr={m.joint(i).name:int(m.jnt_qposadr[i]) for i in range(m.njnt)}
 def contacts():
  vals=dict(cable_normal_N=0.,cable_contacts=0,cable_friction_power_W=0.,rail_normal_N=0.,brake_normal_N=0.,flange_normal_N=0.,collector_normal_N=0.,rail_contacts=0,brake_contacts=0,flange_contacts=0,collector_contacts=0,brake_friction_power_W=0.,rail_friction_power_W=0.,normal_contact_power_W=0.,flange_friction_power_W=0.)
  jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));jq=np.zeros((3,m.nv));jt=np.zeros((3,m.nv))
  for k in range(d.ncon):
   c=d.contact[k]
   if c.efc_address<0:continue
   mujoco.mj_contactForce(m,d,k,force);nn=[m.geom(int(g)).name for g in c.geom]
   if any(n.startswith('cable_') for n in nn):kind='cable'
   elif any('flange_' in n for n in nn):kind='flange'
   elif any('rail_' in n for n in nn):kind='rail'
   elif any('collector_contact' in n for n in nn):kind='collector'
   else:kind='brake'
   vals[kind+'_normal_N']+=max(0.,float(force[0]));vals[kind+'_contacts']+=1
   mujoco.mj_jac(m,d,jp,jr,c.pos,int(m.geom_bodyid[c.geom[0]]));mujoco.mj_jac(m,d,jq,jt,c.pos,int(m.geom_bodyid[c.geom[1]]))
   rel=np.asarray(c.frame).reshape(3,3)@(jq-jp)@d.qvel
   vals['normal_contact_power_W']+=float(force[0]*rel[0])
   if kind in ['brake','rail','cable']:vals[kind+'_friction_power_W']+=float(force[1:3]@rel[1:3])
  if hcdata is not None:
   nn=hcdata['normals_N'];vals['rail_normal_N']=float(sum(nn[hc_wi]));vals['flange_normal_N']=float(sum(nn[hc_fi]));vals['rail_contacts']=int(sum(nn[hc_wi]>0));vals['flange_contacts']=int(sum(nn[hc_fi]>0))
   vals['rail_friction_power_W']=hcdata['wheel_friction_power_W'];vals['flange_friction_power_W']=hcdata['flange_friction_power_W'];vals['normal_contact_power_W']+=hcdata['normal_power_W']
  return vals
 for k in range(round(duration/dt)+1):
  t=d.time-start
  if cfg.get('tether_release') and t>=2 and not tether_released:
   d.eq_active[tether_eq]=False;tether_released=True
  if door_state:
   if cfg.get('doors') and t>=.5 and not doors_unlocked:door_state.release_latch(d);doors_unlocked=True
   if cfg.get('doors') and .5<=t<3.5:door_state.set_opening_torques(d,{dd.key:dd.torque_limit_Nm for dd in doorspec.doors})
   else:door_state.release(d)
   if cfg.get('doors') and t>=3.5:door_state.try_latch(d)
   for dd in doorspec.doors:door_max[dd.key]=max(door_max[dd.key],dd.opening_sign*float(d.qpos[door_state.qpos_addresses[dd.key]]))
  if k%round(2/dt)==0:print(f'{name}: t={t:.1f}s wall={time.time()-startwall:.1f}s',flush=True)
  cmds={n+'_supply':float(cfg.get('brake',False) and t>=4 and not (cfg.get('brake_release') and t>=6)) for n in pads}
  cmds|={n+'_exhaust':float(cfg.get('brake_release',False) and t>=6) for n in pads}
  cmds|={'pantograph_supply':float(cfg.get('panto',True) and .5<=t<8),'pantograph_exhaust':float(t>=8 and cfg.get('normal_exhaust',False)),'pantograph_quick_exhaust':float(t>=8 and not cfg.get('normal_exhaust',False))}
  if cfg.get('brake_leak'):cmds|={n+'_leak':float(t>=5.2) for n in pads}
  if cfg.get('reservoir_loss'):cmds['reservoir_leak']=float(t>=2)
  if cfg.get('air_spring_leak'):cmds['air_A_FL_exhaust']=float(t>=2)
  ramp=min(1.,max(0.,t/.15))*min(1.,max(0.,(3-t)/.15)) if cfg.get('torque') else 0.
  if cfg.get('brake_release') and 6.5<=t<8:ramp=min(1.,(t-6.5)/.15,(8-t)/.15)
  for ai,vi in zip(motoracts,motorvs):d.ctrl[ai]=-min(p['motor_torque_cap_Nm'],p['motor_power_cap_W']/max(abs(d.qvel[vi]),1e-9))*ramp
  d.xfrc_applied[:]=0
  if cfg.get('lateral') and 1<=t<1.2:d.xfrc_applied[rootbid,1]=cfg.get('lateral_force_N',250);d.xfrc_applied[rootbid,5]=cfg.get('yaw_moment_Nm',35)
  if cfg.get('axle_lateral') and 1<=t<1.1:
   for ax in ['front','rear']:d.xfrc_applied[m.body('A_'+ax+'_carrier').id,1]=300.
  if cfg.get('collector_disturbance') and 5<=t<5.2:d.xfrc_applied[headbid,2]=-50
  maxgear=max(maxgear,*[abs(d.qpos[qadr[b+'_motor_z']]+p['gear_ratio']*d.qpos[qadr[b+'_front_axle_y']]) for b in p['powered_bogies']])
  if hcdata is not None:
   nn=hcdata['normals_N'];wc=int(sum(nn[hc_wi]>0));minwheelcontacts=min(minwheelcontacts,wc)
   if wc<8:wheel_partial_contact_steps+=1
   if wc==0:raw_contactless+=1
   peakflange=max(peakflange,float(max(nn[hc_fi])))
   hpen=max(0.,-float(min(hcdata['gaps_m'])))
   if hpen>maxpen:
    ii=int(np.argmin(hcdata['gaps_m']));pair=hc.pairs[ii];maxpen=hpen;worst_contact=dict(time_s=t,penetration_m=hpen,geoms=[pair[0],m.geom(pair[2]).name],law='explicit HuntCrossley')
  if d.ncon:
   pen=-float(np.min(d.contact.dist))
   if pen>maxpen:
    ci=int(np.argmin(d.contact.dist));cc=d.contact[ci]
    worst_contact=dict(time_s=t,penetration_m=pen,geoms=[m.geom(int(g)).name for g in cc.geom]);maxpen=pen
   kinds=np.maximum(geom_kind[d.contact.geom[:,0]],geom_kind[d.contact.geom[:,1]])
   active=d.contact.efc_address>=0
   if hcdata is None:
    wheelcount=len(set(d.contact.geom[active].flatten())&set(wheel_ids))
    minwheelcontacts=min(minwheelcontacts,wheelcount)
    if wheelcount<8:wheel_partial_contact_steps+=1
    if not np.any((kinds==0)&active):raw_contactless+=1
   for ci in np.flatnonzero(((kinds==3)|(kinds==2))&active):
    mujoco.mj_contactForce(m,d,int(ci),force)
    if kinds[ci]==3:peakcollector=max(peakcollector,float(force[0]))
    else:peakflange=max(peakflange,float(force[0]))
  elif hcdata is None:raw_contactless+=1
  maxloop=max(maxloop,float(np.linalg.norm(d.site('panto_left_top_pin').xpos-d.site('panto_right_top_pin').xpos)),float(np.linalg.norm(d.site('panto_lower_carrier_pin').xpos-d.site('panto_lower_balance_pin').xpos)))
  if k%sample==0 or k==round(duration/dt):
   cf=contacts();air=net.audit();press=net.pressures_Pa();energy=float(sum(d.energy));kinetic_resid=energy-e0-sum(sums[v] for v in ['actuator','damping','constraint','external','compliant_contact'])
   contact_u=hcdata['spring_storage_J'] if hcdata is not None else 0.
   contact_resid=sums['compliant_contact']+contact_u-contact_u0-sums['contact_dissipation']-sums['contact_friction']
   resid=kinetic_resid+contact_resid
   cv=comvelocity();momentum_resid=float(np.linalg.norm(trainmass*(cv-com_v0)-momentum_impulse)) if hcdata is not None else 0.
   railforce=hcdata['total_force_world_N'] if hcdata is not None else np.zeros(3)
   maxmass=max(maxmass,abs(air['mass_balance_residual_kg']))
   if not cf['rail_contacts']:contactless+=1
   maxpen=max(maxpen,max((-float(c.dist) for c in d.contact),default=0))
   quat=d.qpos[3:7];w,x,y,z=quat;roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
   airwork=air['useful_mechanical_work_J']-air0['useful_mechanical_work_J'];pneumawork=sums['actuator']-sums['motor']-sums['door']
   row=dict(time_s=t,coupler_length_m=float(d.ten_length[m.tendon('coupler').id]),B_x_m=float(d.body('B_carbody').xpos[0]),B_y_m=float(d.body('B_carbody').xpos[1]),x_m=float(d.qpos[0]-x0),y_m=float(d.qpos[1]),z_m=float(d.qpos[2]),speed_mps=float(d.qvel[0]),lateral_speed_mps=float(d.qvel[1]),roll_rad=float(roll),yaw_rad=float(yaw),reservoir_pressure_Pa=press['reservoir'],brake_pressure_Pa=press[padnames[0]],pantograph_pressure_Pa=press['pantograph'],pantograph_cylinder_m=pref.travel(d),collector_height_m=float(d.body('panto_collector').xpos[2]),air_A_FL_pressure_Pa=press['air_A_FL'],air_A_FR_pressure_Pa=press['air_A_FR'],primary_A_front_m=float(d.qpos[qadr['A_front_primary_z']]),secondary_A_z_m=float(d.qpos[qadr['A_secondary_z']]),secondary_A_y_m=float(d.qpos[qadr['A_secondary_y']]),secondary_A_roll_rad=float(d.qpos[qadr['A_secondary_roll']]),secondary_A_yaw_rad=float(d.qpos[qadr['A_secondary_yaw']]),motor_power_W=sum(float(d.ctrl[a]*d.qvel[v]) for a,v in zip(motoracts,motorvs)),front_wheel_omega_radps=float(d.joint('A_front_axle_y').qvel[0]),front_slip_mps=float(.14*d.joint('A_front_axle_y').qvel[0]-d.qvel[0]),mechanical_energy_delta_J=energy-e0+contact_u-contact_u0,mechanical_balance_residual_J=resid,kinetic_native_balance_residual_J=kinetic_resid,contact_constitutive_balance_residual_J=contact_resid,contact_spring_storage_J=contact_u,train_com_vx_mps=float(cv[0]),train_com_vy_mps=float(cv[1]),train_com_vz_mps=float(cv[2]),rail_force_world_x_N=float(railforce[0]),rail_force_world_y_N=float(railforce[1]),rail_force_world_z_N=float(railforce[2]),momentum_balance_residual_Ns=momentum_resid,pneumatic_work_J=pneumawork,gas_useful_work_J=airwork,coupling_work_discrepancy_J=pneumawork-airwork,air_mass_residual_kg=air['mass_balance_residual_kg'],gas_energy_residual_J=air['gas_energy_balance_residual_J'],exergy_residual_J=air['exergy_balance_residual_J'],**{k+'_work_J':v for k,v in sums.items()},**cf)
   if door_state:row.update({'door_'+key+'_angle_rad':value for key,value in door_state.angles(d).items()})
   row.update({'cable_'+key+'_attachment_error_m':float(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b])) for key,(a,b) in cable_ends.items()})
   for key in cable_ends:
    eqid=int(m.equality('cable_'+key+'_attachment').id)
    mask=(d.efc_type[:d.nefc]==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))&(d.efc_id[:d.nefc]==eqid)
    row['cable_'+key+'_attachment_force_N']=float(np.linalg.norm(d.efc_force[:d.nefc][mask]))
   row['native_train_constraint_force_x_N']=sum(float(d.qfrc_constraint[vi]) for vi in root_linear_dofs)
   row['pump_geometry_work_J']=net.pump.gas_geometry_work_J
   aux=dict(time_s=t,network_time_s=net.time_s,spool_balance_residual_J=net.spool_audit()['balance_residual_J'])
   for obj,state in {'pump':net.pump.snapshot(),**{n:f.snapshot() for n,f in net.fans.items()},**{n:th.snapshot() for n,th in net.thermal.items()}}.items():
    aux.update({obj+'_'+key:float(value) for key,value in state.items() if isinstance(value,(int,float,bool))})
   aux_rows.append(aux)
   rows.append(row)
  if replay and (k%replayn==0 or k==round(duration/dt)):
   frames.append(dict(time_s=t,qpos=d.qpos.tolist(),body_transforms={m.body(b).name:dict(position=d.xpos[b].tolist(),quaternion_wxyz=d.xquat[b].tolist()) for b in range(1,m.nbody)},speed_mps=float(d.qvel[0]),pressure_Pa=net.pressures_Pa(),panto_cylinder_m=pref.travel(d),collector_normal_N=rows[-1]['collector_normal_N'],brake_normal_N=rows[-1]['brake_normal_N'],door_angles_rad=door_state.angles(d) if door_state else {},door_latched=door_state.latched(d) if door_state else {}))
  if k==round(duration/dt):break
  works=step(cmds)
  for key in sums:sums[key]+=works[key]
  if not np.isfinite(d.qpos).all():raise RuntimeError('non-finite mechanical state')
 at4=min(rows,key=lambda r:abs(r['time_s']-4))
 # A crossing of zero during suspension rebound is not a sustained stop.
 # Require every later sampled carbody speed to remain below 0.01 m/s.
 suffix=np.maximum.accumulate(np.abs([r['speed_mps'] for r in rows])[::-1])[::-1]
 stop=next((r for i,r in enumerate(rows) if cfg.get('brake') and not cfg.get('brake_release') and abs(at4['speed_mps'])>.05 and 4.3<=r['time_s']<=duration-.5 and suffix[i]<.01),None)
 final=rows[-1]
 metrics=dict(case=name,model_xml_sha256=xmlhash,solver_version=mujoco.__version__,momentum_audit_enabled=hc is not None,dt_s=dt,duration_s=duration,total_mass_kg=float(m.body_subtreemass[m.body('A_carbody').id]+m.body_subtreemass[m.body('B_carbody').id]),total_model_moving_and_fixed_mass_kg=float(sum(m.body_mass)),dofs=m.nv,initial_settled_speed_mps=float(initial_qvel[0]),final_speed_mps=final['speed_mps'],travel_m=final['x_m'],speed_at_brake_start_mps=at4['speed_mps'],stop_distance_m=stop['x_m']-at4['x_m'] if stop else None,stopped_after_brake_s=stop['time_s']-4 if stop else None,max_lateral_m=max(abs(r['y_m']) for r in rows),max_roll_rad=max(abs(r['roll_rad']) for r in rows),max_yaw_rad=max(abs(r['yaw_rad']) for r in rows),max_contact_penetration_m=maxpen,worst_contact=worst_contact,max_gear_error_rad=maxgear,max_panto_loop_error_m=maxloop,minimum_wheel_treads_in_contact=minwheelcontacts,wheel_partial_contact_steps=wheel_partial_contact_steps,rail_contactless_samples=contactless,rail_contactless_steps=raw_contactless,peak_flange_single_contact_N=peakflange,peak_collector_single_contact_N=peakcollector,peak_flange_normal_N=max(r['flange_normal_N'] for r in rows),max_collector_normal_N=max(r['collector_normal_N'] for r in rows),max_air_mass_residual_kg=maxmass,warning_counts=d.warning.number.tolist(),runtime_s=time.time()-startwall,final=final,air_audit=net.audit(),air_audit_at_settled_start=air0,parameters=p,config=cfg,door_max_opening_rad=door_max,door_final_angles_rad=door_state.angles(d) if door_state else {},door_final_latched=door_state.latched(d) if door_state else {},access_doors=door_metadata,input_schedule='motor torque 0-3 s; brake supply from 4 s; pantograph supply .5-8 s; exhaust from 8 s; fault/disturbance variants in config',pneumatic_configuration=net.configuration(),sampled_brake_friction_work_J=float(np.trapezoid([r['brake_friction_power_W'] for r in rows],[r['time_s'] for r in rows])),sampled_rail_friction_work_J=float(np.trapezoid([r['rail_friction_power_W'] for r in rows],[r['time_s'] for r in rows])))
 with (OUT/f'{name}.csv').open('w') as f:
  wr=csv.DictWriter(f,fieldnames=rows[0]);wr.writeheader();wr.writerows(rows)
 metrics.update(cable_contact_geometry_all_steps=dict(settling=settling_cable_contact_audit.snapshot(),observation=observation_cable_contact_audit.snapshot()))
 metrics.update(version='B11 source-anchored articulated cable equivalent',settle_s=settle_s,cable_configuration=cable_spec,auxiliary_final_state=net.snapshot(),momentum_scope='HC rail + gravity + applied forces + all native train root constraint reactions')
 for relative,expected in sources.items():
  if hashlib.sha256((ROOT.parent/relative).read_bytes()).hexdigest()!=expected:raise RuntimeError('Executed source changed during B11 run')
 with (OUT/f'{name}_auxiliaries.csv').open('w',newline='',encoding='utf-8') as f:
  writer=csv.DictWriter(f,fieldnames=aux_rows[0]);writer.writeheader();writer.writerows(aux_rows)
 (OUT/f'{name}_metrics.json').write_text(json.dumps(metrics,indent=2))
 if replay:(OUT/f'{name}_replay.json').write_text(json.dumps(dict(schema='B11 force driven source-cable full3D replay v1',model_xml_sha256=xmlhash,units='SI',solver='MuJoCo '+mujoco.__version__,scope='Uncalibrated source-topology two-car/two-bogie rigid-body mechanics; ideal gears; primitive wheel/rail and rigid overhead contact; isothermal finite-volume air.',reference_zero_qpos_body_transforms=bodyrefs,initial_qpos=initial_qpos.tolist(),initial_qvel=initial_qvel.tolist(),parameters=p,access_doors=door_metadata,cable_configuration=cable_spec,frames=frames),separators=(',',':')))
 print(json.dumps({k:v for k,v in metrics.items() if k not in ['parameters','final','air_audit','air_audit_at_settled_start','config','pneumatic_configuration','access_doors','door_max_opening_rad','door_final_angles_rad','door_final_latched','cable_configuration','auxiliary_final_state']}),flush=True)
 paths=[OUT/f'{name}{suffix}' for suffix in ('.xml','.csv','_metrics.json','_auxiliaries.csv')]
 if replay:paths.append(OUT/f'{name}_replay.json')
 xmlpath=OUT/f'{name}.xml';xmlpath.write_bytes(xmlpath.read_text(encoding='utf-8').encode('utf-8'))
 identity=dict(case=name,solver=mujoco.__version__,numpy=np.__version__,sources_sha256=sources,outputs_sha256={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},configuration=cable_spec)
 (OUT/f'{name}_identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
 return metrics
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--case',default='service_brake');ap.add_argument('--duration',type=float,default=10);ap.add_argument('--no-replay',action='store_true');args=ap.parse_args()
 cases=CASES if args.case=='all' else {c:CASES[c] for c in args.case.split(',')}
 for n,c in cases.items():run(n,c,args.duration,not args.no_replay)
