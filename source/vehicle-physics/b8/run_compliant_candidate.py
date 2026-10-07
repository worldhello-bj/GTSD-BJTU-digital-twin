"""Separate headless candidate runner; the frozen B8 files/results are read-only.

Same six-second passive settle, motor/brake/pantograph schedule, mechanics-first
actual-volume gas update as simulate_full.py. No qpos/qvel assignments occur.
"""
from pathlib import Path
import argparse,csv,hashlib,json,time,sys,pickle
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'vendor'))
import mujoco
from full_model import model,PARAMS
from pneumatics import make_vehicle_network,GasVolume,AMBIENT_PA
from compliant_contact import ContactParameters,CompliantWheelRail,candidate_xml

class CandidateVehicle:
    def __init__(self,dt,mu,k,alpha,vreg):
        self.parameters=PARAMS|dict(timestep_s=dt,wheel_rail_mu=mu,include_pantograph=True,include_access_doors=False,compliant_wheel_rail=False)
        original,pspec=model(self.parameters,return_spec=True)
        self.xml,self.removed_pairs=candidate_xml(original)
        self.model=m=mujoco.MjModel.from_xml_string(self.xml);self.data=d=mujoco.MjData(m)
        self.dt=dt;mujoco.mj_forward(m,d);self.panto=pspec.bind(m)
        self.contact=CompliantWheelRail(m,ContactParameters(k,alpha,vreg,mu))
        self.pads={m.body(i).name:(int(m.joint(m.body(i).name+'_slide').qposadr[0]),int(m.actuator(m.body(i).name+'_force').id)) for i in range(1,m.nbody) if m.body(i).name.endswith('_pad')}
        self.airs={f'air_{b}_{s}':(int(m.tendon(f'air_{b}_{s}').id),int(m.actuator(f'air_{b}_{s}_force').id)) for b in ('A','B') for s in ('FL','FR','RL','RR')}
        self.airrefs={n:float(d.ten_length[v[0]]) for n,v in self.airs.items()}
        self.motoracts=[int(m.actuator(b+'_motor_torque').id) for b in self.parameters['powered_bogies']]
        self.motorvs=[int(m.joint(b+'_motor_z').dofadr[0]) for b in self.parameters['powered_bogies']]
        p=self.parameters
        extras=[GasVolume(n,p['airspring_dead_volume_m3'],p['airspring_initial_pressure_Pa'],p['airspring_area_m2'],0) for n in self.airs]
        self.network=make_vehicle_network(self.pads,extra_chambers=extras,brake_dead_volume_m3=p['brake_dead_volume_m3'],pantograph_area_m2=pspec.params['cylinder_area_m2'],brake_area_m2=p['brake_piston_area_m2'],initial_displacements_m=self.displacements(),reservoir_initial_pressure_Pa=AMBIENT_PA+500000)
    def displacements(self):
        d=self.data
        return {**{n:float(d.qpos[v[0]]) for n,v in self.pads.items()},**{n:float(d.ten_length[v[0]])-self.airrefs[n] for n,v in self.airs.items()},'pantograph':self.panto.travel(d)}
    def step(self,commands,contact):
        m,d,dt=self.model,self.data,self.dt
        old=d.qvel.copy();old_tendon=d.ten_velocity.copy()
        forces=self.network.forces_N()
        for n,v in self.pads.items():d.ctrl[v[1]]=forces[n]
        for n,v in self.airs.items():d.ctrl[v[1]]=forces[n]
        self.panto.set_force(d,forces['pantograph'])
        d.qfrc_applied[:]=contact['qforce']
        mujoco.mj_step(m,d);mid=(old+d.qvel)*.5
        work=dict(actuator=float(d.qfrc_actuator@mid)*dt,motor=sum(float(d.qfrc_actuator[j]*mid[j])*dt for j in self.motorvs),damping=-float((m.dof_damping*d.qvel)@mid)*dt,constraint=float(d.qfrc_constraint@mid)*dt,explicit_contact=float(contact['qforce']@mid)*dt,contact_normal_dissipation=contact['normal_dissipation_power_W']*dt,contact_friction=contact['friction_power_W']*dt)
        mujoco.mj_forward(m,d)
        work['damping']-=float((m.tendon_damping*old_tendon)@((old_tendon+d.ten_velocity)*.5))*dt
        self.network.update(dt,self.displacements(),commands,snapshot=False)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all() or np.any(d.warning.number):raise RuntimeError('Mechanical warning/nonfinite state '+str(d.warning.number))
        return work

def run(out,dt=1e-4,mu=.015,k=2e6,alpha=40.,vreg=.005,duration=10.,settle=6.,drive=True,brake=True,panto=True):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);wall=time.time();v=CandidateVehicle(dt,mu,k,alpha,vreg);m,d=v.model,v.data
    (out/'candidate.xml').write_text(v.xml)
    c=v.contact.evaluate(d)
    settlecmd={n+'_exhaust':1 for n in v.pads}|{'pantograph_exhaust':1}
    for j in range(round(settle/dt)):
        v.step(settlecmd,c);c=v.contact.evaluate(d)
        if (j+1)%round(2/dt)==0:print(f'{out.name}: settling {(j+1)*dt:.1f}s; wall {time.time()-wall:.1f}s',flush=True)
    v.network.close_all_valves();mujoco.mj_forward(m,d)
    settled=c=v.contact.evaluate(d,True)
    settled_weight=float(sum(m.body_mass))*9.81
    settled_vertical=sum(x['force_on_wheel_N'][2] for x in c['details'])
    settled_check=dict(weight_N=settled_weight,explicit_rail_vertical_force_N=settled_vertical,relative_weight_imbalance=(settled_vertical-settled_weight)/settled_weight,speed_mps=float(d.qvel[0]))
    start=d.time;x0=float(d.qpos[0]);e0=float(sum(d.energy));u0=c['spring_storage_J'];air0=v.network.audit();nsteps=round(duration/dt)
    sums={x:0. for x in ('actuator','motor','damping','constraint','explicit_contact','contact_normal_dissipation','contact_friction')}
    raw=np.zeros((nsteps+1,1+16+16+6));rows=[];peak=None;max_mech=0.;max_aug=0.;maxcontactres=0.;maxmass=0.;maxloop=0.;maxgear=0.
    fi=np.array([i for i,pair in enumerate(v.contact.pairs) if pair[-1]=='flange']);wi=np.array([i for i,pair in enumerate(v.contact.pairs) if pair[-1]=='wheel'])
    for j in range(nsteps+1):
        t=j*dt;nn=c['normals_N'];gap=c['gaps_m']
        mech=float(sum(d.energy))-e0-sums['actuator']-sums['damping']-sums['constraint']-sums['explicit_contact']
        cres=sums['explicit_contact']+c['spring_storage_J']-u0-sums['contact_normal_dissipation']-sums['contact_friction']
        aug=mech+cres
        max_mech=max(max_mech,abs(mech));max_aug=max(max_aug,abs(aug));maxcontactres=max(maxcontactres,abs(cres))
        raw[j]=[t,*nn,*gap,c['spring_storage_J'],c['normal_dissipation_power_W'],c['friction_power_W'],float(d.qvel[0]),float(d.joint('A_front_axle_y').qvel[0]),cres]
        if peak is None or nn[fi].max()>peak['single_force_N']:
            peak=dict(time_s=t,single_force_N=float(nn[fi].max()),total_flange_force_N=float(nn[fi].sum()),tread_loads_N={v.contact.pairs[i][0]:float(nn[i]) for i in wi},details=v.contact.evaluate(d,True)['details'])
            (out/'peak_state.pkl').write_bytes(pickle.dumps(dict(model=m,data=d,network=v.network,airrefs=v.airrefs,t=t,contact_parameters=v.contact.parameters),protocol=5))
        maxloop=max(maxloop,float(np.linalg.norm(d.site('panto_left_top_pin').xpos-d.site('panto_right_top_pin').xpos)),float(np.linalg.norm(d.site('panto_lower_carrier_pin').xpos-d.site('panto_lower_balance_pin').xpos)))
        maxgear=max(maxgear,abs(float(d.joint('A_motor_z').qpos[0])+v.parameters['gear_ratio']*float(d.joint('A_front_axle_y').qpos[0])))
        if j%max(1,round(.01/dt))==0 or j==nsteps:
            air=v.network.audit();maxmass=max(maxmass,abs(air['mass_balance_residual_kg']))
            rows.append(dict(time_s=t,x_m=float(d.qpos[0])-x0,speed_mps=float(d.qvel[0]),y_m=float(d.qpos[1]),flange_peak_N=float(nn[fi].max()),flange_total_N=float(nn[fi].sum()),tread_total_N=float(nn[wi].sum()),spring_storage_J=c['spring_storage_J'],mechanical_balance_residual_J=mech,augmented_balance_residual_J=aug,contact_energy_residual_J=cres,gas_mechanical_coupling_discrepancy_J=sums['actuator']-sums['motor']-(air['useful_mechanical_work_J']-air0['useful_mechanical_work_J']),air_mass_residual_kg=air['mass_balance_residual_kg']))
        if j%max(1,round(2/dt))==0:print(f'{out.name}: t={t:.1f}; peak flange {peak["single_force_N"]:.4f}N; wall {time.time()-wall:.1f}s',flush=True)
        if j==nsteps:break
        cmds={n+'_supply':float(brake and t>=4) for n in v.pads}|{n+'_exhaust':0. for n in v.pads}|{'pantograph_supply':float(panto and .5<=t<8),'pantograph_exhaust':0.,'pantograph_quick_exhaust':float(panto and t>=8)}
        ramp=min(1.,max(0.,t/.15))*min(1.,max(0.,(3-t)/.15)) if drive else 0.
        for ai,vi in zip(v.motoracts,v.motorvs):d.ctrl[ai]=-min(v.parameters['motor_torque_cap_Nm'],v.parameters['motor_power_cap_W']/max(abs(d.qvel[vi]),1e-9))*ramp
        d.xfrc_applied[:]=0
        works=v.step(cmds,c)
        for key in sums:sums[key]+=works[key]
        c=v.contact.evaluate(d)
    names=[pair[0] for pair in v.contact.pairs];cols=['time_s']+[n+'_N' for n in names]+[n+'_gap_m' for n in names]+['spring_storage_J','normal_dissipation_power_W','friction_power_W','car_speed_mps','driven_axle_radps','contact_energy_residual_J']
    np.savez_compressed(out/'every_step.npz',columns=np.asarray(cols),values=raw)
    with (out/'samples.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    at4=min(rows,key=lambda x:abs(x['time_s']-4));suffix=np.maximum.accumulate(np.abs([r['speed_mps'] for r in rows])[::-1])[::-1]
    stop=next((r for i,r in enumerate(rows) if abs(at4['speed_mps'])>.05 and 4.3<=r['time_s']<=duration-.5 and suffix[i]<.01),None)
    fn=raw[:,1:17][:,fi];tn=raw[:,1:17][:,wi];gaps=raw[:,17:33]
    summary=dict(status='experimental numerical-compliance candidate; not calibrated or GUI/hardware validated',dt_s=dt,mu=mu,stiffness_Npm=k,dissipation_spm=alpha,vreg_mps=vreg,distance_query_tolerance_m=v.contact.parameters.distance_tolerance_m,settle_s=settle,duration_s=duration,solver_version=mujoco.__version__,drive_enabled=drive,brake_enabled=brake,pantograph_enabled=panto,settled_force_balance=settled_check,input_clock='Integer j*dt, unlike accumulated d.time-start in frozen runner; jump timing can differ by one step',nonpenetrating_gap_semantics='Censored at zero query bound; negative gaps are raw mj_geomDistance penetrations',model_sha256=hashlib.sha256(v.xml.encode()).hexdigest(),geometry_modified=False,native_pairs_removed=v.removed_pairs,peak_flange=peak,peak_flange_single_contact_N=peak['single_force_N'],peak_total_flange_N=float(fn.sum(1).max()),flange_impulse_Ns=float(fn[:-1].sum()*dt),flange_positive_duration_s=float(np.any(fn[:-1]>0,axis=1).sum()*dt),minimum_loaded_treads=int((tn>0).sum(1).min()),partial_tread_steps=int(((tn>0).sum(1)<8).sum()),minimum_total_tread_load_N=float(tn.sum(1).min()),maximum_total_tread_load_N=float(tn.sum(1).max()),maximum_contact_penetration_m=max(0.,float(-gaps.min())),contact_penetration_scope='Explicit wheel/rail tread and flange pairs only; native brake and collector penetration is not included',maximum_tread_penetration_m=max(0.,float(-gaps[:,wi].min())),maximum_flange_penetration_m=max(0.,float(-gaps[:,fi].min())),max_mechanical_balance_residual_J=max_mech,max_augmented_balance_residual_J=max_aug,max_contact_energy_residual_J=maxcontactres,initial_contact_spring_storage_J=u0,final_contact_spring_storage_J=c['spring_storage_J'],integrated_work_J=sums,max_mass_residual_kg=maxmass,final_air_audit=v.network.audit(),max_panto_loop_error_m=maxloop,max_gear_error_rad=maxgear,final=rows[-1],speed_at_brake_start_mps=at4['speed_mps'],stop_distance_m=stop['x_m']-at4['x_m'] if stop else None,stop_delay_s=stop['time_s']-4 if stop else None,warnings=d.warning.number.tolist(),runtime_s=time.time()-wall)
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({x:summary[x] for x in ('dt_s','peak_total_flange_N','flange_impulse_Ns','minimum_loaded_treads','maximum_contact_penetration_m','max_augmented_balance_residual_J','max_contact_energy_residual_J','runtime_s')}),flush=True)
    return summary

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--out',required=True);a.add_argument('--dt',type=float,default=1e-4);a.add_argument('--mu',type=float,default=.015);a.add_argument('--k',type=float,default=2e6);a.add_argument('--alpha',type=float,default=40.);a.add_argument('--vreg',type=float,default=.005);a.add_argument('--duration',type=float,default=10.);a.add_argument('--settle',type=float,default=6.);a.add_argument('--no-drive',action='store_true');a.add_argument('--no-brake',action='store_true');a.add_argument('--no-panto',action='store_true');r=a.parse_args();run(r.out,r.dt,r.mu,r.k,r.alpha,r.vreg,r.duration,r.settle,not r.no_drive,not r.no_brake,not r.no_panto)
