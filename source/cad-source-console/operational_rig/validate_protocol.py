"""Full passage, force/energy ledger, time-step convergence and exact recovery."""
import argparse
import hashlib
import csv
from datetime import datetime,timezone,timedelta
import gzip
import json
from pathlib import Path
import time
import numpy as np
try:from .plant import Plant
except ImportError:from plant import Plant

def native_run(dt,output):
    started=time.perf_counter();p=Plant({'time_step_s':dt});build_s=time.perf_counter()-started
    count=round(p.contract['demo']['duration_s']/dt);samples=[];contacts_seen=set();geoms_seen=set();impulses={k:0. for k in p.interfaces};peaks={k:0. for k in p.interfaces}
    maximum_penetration=0.;penetration_event=None;closure=0.;max_energy_residual=0.;energy_residual_event=None;energy_scale=1e-9;max_wheel_drop=0.;events={};pose_max={n:0. for n in p.body_ids};last=None;recovery_cp=None;recovery_rows=[]
    fields=['step','time_s','energy_J','residual_J','input_work_J','actuator_work_J','damping_J','constraint_work_J','penetration_m','wheel_x','wheel_y','wheel_z']+[i+'_normal_N' for i in p.interfaces]
    out=output/('h'+str(round(dt*1e6))+'us');out.mkdir(parents=True,exist_ok=True)
    (out/'initial_snapshot.json').write_text(json.dumps(p.snapshot()))
    run_started=time.perf_counter()
    with gzip.open(out/'steps.csv.gz','wt',newline='') as f,gzip.open(out/'poses.jsonl.gz','wt') as poses:
        writer=csv.writer(f);writer.writerow(fields)
        for i in range(count):
            if i==count-100:recovery_cp=p.checkpoint()
            trace=p.step(p.demo_loads(p.steps*dt));s=trace['state'];last=s
            max_wheel_drop=max(max_wheel_drop,s['wheel_vertical_drop_m'])
            energy_scale=max(energy_scale,abs(s['energy']['total_J']-p.energy_reference_J),abs(s['ledger']['input_work_J'])+abs(s['ledger']['actuator_work_J'])+s['ledger']['known_damping_J']+abs(s['ledger']['native_constraint_work_J']))
            if abs(s['energy_balance_residual_J'])>max_energy_residual:
                max_energy_residual=abs(s['energy_balance_residual_J']);energy_residual_event={'time_s':s['time_s'],'wheel_x_m':s['wheel_position_world_m'][0],'residual_J':s['energy_balance_residual_J']}
            for event,x in p.contract['demo']['route_events_x_m'].items():
                if event not in events and s['wheel_position_world_m'][0]>=x:
                    events[event]={'time_s':s['time_s'],'wheel_position_world_m':s['wheel_position_world_m'],'wheel_vertical_drop_m':s['wheel_vertical_drop_m'],'wheel_normal_force_N':s['contact_interfaces']['wheel_rail']['normal_force_N']}
                    (out/(event+'_snapshot.json')).write_text(json.dumps(s))
            if i>=count-100:recovery_rows.append((p.data.qpos.copy(),p.data.qvel.copy(),dict(p.ledger)))
            current={k:0. for k in p.interfaces}
            for c in trace['used_contacts']:
                if c['normal_force_N']>1e-8:contacts_seen.add(c['interface_id']);geoms_seen.update([c['geom1'],c['geom2']])
                current[c['interface_id']]+=c['normal_force_N'];impulses[c['interface_id']]+=c['normal_impulse_Ns']
                if -c['signed_gap_m']>maximum_penetration:
                    maximum_penetration=-c['signed_gap_m'];penetration_event={'time_s':s['time_s'],'interface_id':c['interface_id'],'geom1':c['geom1'],'geom2':c['geom2'],'penetration_m':maximum_penetration}
            for k,v in current.items():peaks[k]=max(peaks[k],v)
            closure=max(closure,abs(trace['contact_generalized_work_closure_error_J']))
            for n,b in s['body_poses'].items():pose_max[n]=max(pose_max[n],float(np.linalg.norm(np.array(b['position'])-p.reference_positions[n])))
            row=[s['steps'],s['time_s'],s['energy']['total_J'],s['energy_balance_residual_J'],p.ledger['input_work_J'],p.ledger['actuator_work_J'],p.ledger['known_damping_J'],p.ledger['native_constraint_work_J'],maximum_penetration,*s['wheel_position_world_m'],*[current[k] for k in p.interfaces]]
            writer.writerow(row)
            # Complete native trajectories and body pose traces, no motion scaling.
            poses.write(json.dumps({'step':s['steps'],'time_s':s['time_s'],'qpos':s['qpos'],'qvel':s['qvel'],'body_poses':s['body_poses'],'input_world_force_N':trace['input_world_force_N'],'drive_target_m_s':trace['drive_target_m_s'],'used_contacts':trace['used_contacts'],'energy':s['energy'],'ledger':s['ledger'],'energy_balance_residual_J':s['energy_balance_residual_J']},separators=(',',':'))+'\n')
            samples.append([*s['qpos'],*s['qvel'],*[current[k] for k in p.interfaces],s['energy_balance_residual_J'],*[x for b in s['body_poses'].values() for x in b['position']]])
    native_wall_s=time.perf_counter()-run_started
    (out/'final_snapshot.json').write_text(json.dumps(last))
    terminal=p.checkpoint();p.restore(recovery_cp);recovery_error=0.;ledger_error=0.
    for expected in recovery_rows:
        p.step(p.demo_loads(p.steps*dt));recovery_error=max(recovery_error,float(np.max(abs(p.data.qpos-expected[0]))),float(np.max(abs(p.data.qvel-expected[1]))));ledger_error=max(ledger_error,max(abs(p.ledger[k]-expected[2][k]) for k in p.ledger))
    if p.checkpoint()!=terminal:raise AssertionError('Checkpoint continuation changed complete terminal state')
    scale=max(abs(last['energy']['total_J']-p.energy_reference_J),abs(p.ledger['input_work_J'])+abs(p.ledger['actuator_work_J'])+p.ledger['known_damping_J']+abs(p.ledger['native_constraint_work_J']),1e-9)
    result={'identity_sha256':p.identity,'assembly_identity_sha256':p.contract['identity_sha256'],'time_step_s':dt,'duration_s':count*dt,'steps':count,'nq':p.model.nq,'nv':p.model.nv,'body_count':len(p.body_ids),'mass_accounting':p.contract['mass_accounting'],'maximum_wheel_vertical_drop_m':max_wheel_drop,'route_events':events,'native_run_wall_s':native_wall_s,'sim_seconds_per_wall_second':count*dt/native_wall_s,'build_wall_s':build_s,'total_wall_s':time.perf_counter()-started,'interfaces_with_force':sorted(contacts_seen),'normal_impulse_Ns':impulses,'raw_force_peak_N':peaks,'maximum_energy_residual_J':max_energy_residual,'maximum_energy_residual_fraction':max_energy_residual/energy_scale,'maximum_energy_residual_event':energy_residual_event,'energy_scale_J':energy_scale,'maximum_penetration_event':penetration_event,'maximum_penetration_m':maximum_penetration,'max_contact_generalized_work_closure_error_J':closure,'energy_residual_J':last['energy_balance_residual_J'],'energy_residual_fraction':abs(last['energy_balance_residual_J'])/scale,'ledger':p.ledger,'max_body_displacement_m':pose_max,'final_wheel_position_m':last['wheel_position_world_m'],'wheel_travel_m':last['wheel_travel_m'],'wheel_contact_geoms_seen':sorted(g for g in geoms_seen if g.startswith('rail_')),'checkpoint_continuation_max_error':recovery_error,'checkpoint_ledger_max_error_J':ledger_error,'complete_checkpoint_exact':True,'warning_counts':[w.number for w in p.data.warning]}
    (out/'contract.json').write_text(json.dumps(p.contract,indent=2)+'\n');(out/'model.xml').write_text(p.xml);(out/'checkpoint.json').write_text(json.dumps(terminal)+'\n');(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    return result,np.array(samples),p.model.nq,p.model.nv

def run(output):
    output.mkdir(parents=True,exist_ok=True);coarse,a,nq,nv=native_run(.00005,output);print(json.dumps({'phase':'h_done','summary':coarse}),flush=True)
    fine,b,_,_=native_run(.000025,output);print(json.dumps({'phase':'h2_done','summary':fine}),flush=True)
    b_full=b.copy();b=b[1::2];nforce=4
    dq=np.max(abs(a[:,:nq]-b[:,:nq]),axis=0);dv=np.max(abs(a[:,nq:nq+nv]-b[:,nq:nq+nv]),axis=0)
    # All body world positions (last 3*body_count columns) compared at aligned time.
    posecols=nq+nv+nforce+1;position_error=float(np.max(abs(a[:,posecols:]-b[:,posecols:])))
    impulses={k:abs(coarse['normal_impulse_Ns'][k]-fine['normal_impulse_Ns'][k])/max(abs(fine['normal_impulse_Ns'][k]),1e-9) for k in coarse['normal_impulse_Ns']}
    blocks=100 # 5 ms at 50 microseconds; fine uses all 200 native samples.
    fa=a[:,nq+nv:nq+nv+4];fb=b_full[:,nq+nv:nq+nv+4]
    aa=fa[:len(fa)//blocks*blocks].reshape(-1,blocks,4).mean(1);bb=fb[:len(fb)//(2*blocks)*(2*blocks)].reshape(-1,2*blocks,4).mean(1)
    force_error=np.max(abs(aa-bb),axis=0);force_scale=np.maximum(np.max(abs(bb),axis=0),.05)
    velocity_scale=np.max(abs(b[:,nq:nq+nv]),axis=0);velocity_tolerance=np.maximum(.002,.1*velocity_scale)
    velocity_tolerance[-1]=max(.1,.1*velocity_scale[-1])
    metrics={'per_dof_qvel_tolerance':velocity_tolerance.tolist(),'maximum_body_position_error_m':position_error,'per_dof_qpos_max_error':dq.tolist(),'per_dof_qvel_max_error':dv.tolist(),'normal_impulse_relative_error':impulses,'five_ms_force_max_error_N':force_error.tolist(),'five_ms_force_relative_error':(force_error/force_scale).tolist()}
    required_route_sources=('rail_OP_approach_','rail_PHOTO_FROG_','rail_PHOTO_WING_upper_','rail_PHOTO_RETAIN_right_ENV_main_rail_1_')
    route_supported=all(any(g.startswith(prefix) for g in case['wheel_contact_geoms_seen']) for case in (coarse,fine) for prefix in required_route_sources)
    checks={'real_contact_on_all_route_parts':route_supported,'wheel_relative_drop_under_2mm':max(coarse['maximum_wheel_vertical_drop_m'],fine['maximum_wheel_vertical_drop_m'])<.002,'all_four_contact_interfaces':len(coarse['interfaces_with_force'])==4 and len(fine['interfaces_with_force'])==4,'wheel_passes_complete_route':len(coarse['route_events'])==4 and len(fine['route_events'])==4,'all_branch_motion':all(coarse['max_body_displacement_m'][n]>1e-9 for n in ('track_assembly','balise_02','balise_03','catenary_50DF4','catenary_51409')),'bounded_penetration_100um':max(coarse['maximum_penetration_m'],fine['maximum_penetration_m'])<.0001,'contact_work_closure_1e_8J':max(coarse['max_contact_generalized_work_closure_error_J'],fine['max_contact_generalized_work_closure_error_J'])<1e-8,'maximum_energy_fraction_under_2percent':max(coarse['maximum_energy_residual_fraction'],fine['maximum_energy_residual_fraction'])<.02,'h2_body_positions_50um':position_error<50e-6,'h2_per_dof_velocity':bool(np.all(dv<=velocity_tolerance)),'h2_normal_impulses_5percent':max(impulses.values())<.05,'h2_five_ms_force_10percent':bool(np.all(force_error/force_scale<.1)),'checkpoint_exact':coarse['complete_checkpoint_exact'] and fine['complete_checkpoint_exact'],'no_native_warnings':not any(coarse['warning_counts']+fine['warning_counts'])}
    now=datetime.now(timezone.utc);result={'schema':'gtsd-operational-protocol-v3','protocol_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'checks':checks,'passed':all(checks.values()),'coarse':coarse,'fine':fine,'convergence':metrics,'finished_utc':now.isoformat(),'finished_beijing':now.astimezone(timezone(timedelta(hours=8))).isoformat()}
    (output/'protocol.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'checks':checks,'finished_utc':result['finished_utc'],'finished_beijing':result['finished_beijing']}),flush=True)
    return result

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();raise SystemExit(0 if run(args.output)['passed'] else 1)
