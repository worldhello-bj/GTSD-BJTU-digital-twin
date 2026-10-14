"""One bounded zero-target continuation, preserving the completed passage identity."""
import argparse
import csv
from datetime import datetime,timezone,timedelta
import gzip
import json
from pathlib import Path
import time
import numpy as np
import mujoco
try:from .plant import Plant
except ImportError:from plant import Plant

def observe(source,output,duration=.4):
    source=Path(source);output=Path(output);output.mkdir(parents=True,exist_ok=True);results=[]
    for case in sorted(source.glob('h*us'),reverse=True):
        cp=json.loads((case/'checkpoint.json').read_text());contract=json.loads((case/'contract.json').read_text());p=Plant(contract['parameters']);p.restore(cp)
        if p.identity!=cp['identity_sha256']:raise ValueError('Continuation identity changed')
        dt=float(p.model.opt.timestep);count=round(duration/dt);dest=output/case.name;dest.mkdir(exist_ok=True);initial=p.snapshot();window=[];peakres=abs(initial['energy_balance_residual_J']);closure=0.;started=time.perf_counter()
        with gzip.open(dest/'steps.csv.gz','wt',newline='') as out,gzip.open(dest/'native_states.jsonl.gz','wt') as states:
            writer=csv.writer(out);writer.writerow(['step','time_s','wheel_x_m','wheel_vx_m_s','wheel_vz_m_s','wheel_spin_rad_s','wheel_normal_force_N','native_contacts','minimum_gap_m','energy_J','energy_residual_J'])
            for i in range(count):
                t=p.step({});s=t['state'];force=s['contact_interfaces']['wheel_rail']['normal_force_N'];peakres=max(peakres,abs(s['energy_balance_residual_J']));closure=max(closure,abs(t['contact_generalized_work_closure_error_J']))
                writer.writerow([s['steps'],s['time_s'],s['wheel_position_world_m'][0],*s['qvel'][-3:],force,s['native_contact_count'],s['minimum_contact_gap_m'],s['energy']['total_J'],s['energy_balance_residual_J']])
                states.write(json.dumps({k:s[k] for k in ('steps','time_s','qpos','qvel')},separators=(',',':'))+'\n')
                if i>=count-round(.1/dt):window.append([*s['qvel'],force])
        final=p.snapshot();w=np.array(window);limits=np.full(p.model.nv,.001);units=[]
        for j in range(p.model.njnt):
            dof=int(p.model.jnt_dofadr[j]);hinge=p.model.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE
            if hinge:limits[dof]=.01
            units.append({'joint':mujoco.mj_id2name(p.model,mujoco.mjtObj.mjOBJ_JOINT,j),'unit':'rad/s' if hinge else 'm/s','dof':dof,'speed_limit':float(limits[dof])})
        peak=np.max(abs(w[:,:-1]),axis=0);force_fraction=float(np.mean(w[:,-1]>1e-6));meanforce=float(np.mean(w[:,-1]));settled=bool(np.all(peak<limits) and force_fraction>.95)
        r={'case':case.name,'identity_sha256':p.identity,'start_time_s':initial['time_s'],'end_time_s':final['time_s'],'duration_s':count*dt,'native_steps':count,'wall_s':time.perf_counter()-started,'input':'No external body loads; unchanged native drive law returns zero after passage duration','source_checkpoint_sha256':__import__('hashlib').sha256((case/'checkpoint.json').read_bytes()).hexdigest(),'last_100ms_peak_per_dof_velocity':peak.tolist(),'velocity_observation_limits':units,'last_100ms_wheel_contact_fraction':force_fraction,'last_100ms_mean_wheel_normal_force_N':meanforce,'final_wheel_qvel':final['qvel'][-3:],'final_wheel_normal_force_N':final['contact_interfaces']['wheel_rail']['normal_force_N'],'final_energy_residual_J':final['energy_balance_residual_J'],'maximum_energy_residual_J':peakres,'max_contact_work_closure_error_J':closure,'naturally_settled_under_stated_observation_thresholds':settled,'original_passage_endpoint_unchanged':True,'velocities_forced_to_zero':False,'warning_counts':[x.number for x in p.data.warning]}
        for name,obj in [('initial_snapshot',initial),('final_snapshot',final),('checkpoint',p.checkpoint()),('summary',r)]:
            (dest/(name+'.json')).write_text(json.dumps(obj,indent=2)+'\n')
        results.append(r);print(json.dumps(r),flush=True)
    now=datetime.now(timezone.utc);result={'schema':'gtsd-bounded-continuation-observation-v1','scope':'one 0.4 s continuation per case; observation only, no tuning and no change to passage acceptance','cases':results,'finished_utc':now.isoformat(),'finished_beijing':now.astimezone(timezone(timedelta(hours=8))).isoformat()};(output/'observation.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--source',type=Path,required=True);a.add_argument('--output',type=Path,required=True);args=a.parse_args();observe(args.source,args.output)
