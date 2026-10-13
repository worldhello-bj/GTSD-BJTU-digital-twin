"""Bounded native-dynamics acceptance for finite mounts, not contact fit."""
import argparse
import gzip
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path
import time
import numpy as np
from plant import Plant

TARGETS={'track_support':'support_03','balise':'balise_02',
         'clamps':'FIT_50DF4_F643_NOMINAL_PIN5_8A2C_CANDIDATE'}


def run_case(target, step, fixed, duration=0.3):
    p=Plant({'gravity_m_s2':[0.,0.,0.],'time_step_s':step,'rig_fixed':fixed})
    sample_stride=round(0.001/step)
    trace=[];samples=[];start=time.monotonic()
    for n in range(round(duration/step)):
        t=(n+0.5)*step
        amplitude=math.sin(math.pi*t/0.05)**2 if t<0.05 else 0.
        receipt=p.step({target:[0.,0.,amplitude]})
        trace.append(receipt)
        if (n+1)%sample_stride==0:samples.append(receipt['state'])
    wall=time.monotonic()-start
    return {'identity_sha256':p.identity,'parameters':p.contract['parameters'],
        'target':target,'wall_time_s':wall,'simulation_time_s':duration,
        'wall_seconds_per_simulation_second':wall/duration,
        'admission':p.admission,'samples':samples,'trace':trace,
        'final_checkpoint':p.checkpoint(),'contract':p.contract}


def displacement(case, body):
    ref=np.array(next(r for r in case['contract']['bodies'] if r['name']==body)['reference_position_m'])
    return max(float(np.linalg.norm(np.array(s['body_poses'][body]['position'])-ref)) for s in case['samples'])


def compare(a,b):
    assert len(a['samples'])==len(b['samples'])
    maxpos=maxangle=maxvel=0.
    for x,y in zip(a['samples'],b['samples']):
        if abs(x['time_s']-y['time_s'])>1e-12:raise AssertionError('Observation clocks differ')
        maxvel=max(maxvel,float(np.max(abs(np.array(x['qvel'])-y['qvel']))))
        for name,px in x['body_poses'].items():
            py=y['body_poses'][name]
            maxpos=max(maxpos,float(np.linalg.norm(np.array(px['position'])-py['position'])))
            dot=abs(float(np.dot(px['quaternion_wxyz'],py['quaternion_wxyz'])))
            maxangle=max(maxangle,2*math.acos(min(1.,dot)))
    ea=max(abs(s['energy_balance_residual_J']) for s in a['samples'])
    eb=max(abs(s['energy_balance_residual_J']) for s in b['samples'])
    return {'maximum_body_position_difference_m':maxpos,'maximum_body_angle_difference_rad':maxangle,
            'maximum_generalized_velocity_difference':maxvel,'mixed_velocity_units':'m/s for slide, rad/s for rotational coordinates',
            'coarse_max_energy_balance_residual_J':ea,'half_max_energy_balance_residual_J':eb,
            'half_to_coarse_energy_residual_ratio':eb/ea if ea else None,
            'position_gate_5um':maxpos<=5e-6,'angle_gate_0_05deg':maxangle<=math.radians(.05),
            'velocity_gate_1e_4':maxvel<=1e-4,
            'energy_residual_gate_1e_5_J':eb<=1e-5,
            'energy_residual_refines':eb<=ea*1.02+1e-12}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    started=datetime.now(timezone.utc)
    report={'schema':'gtsd-mount-only-acceptance-v1','scope':'FINITE_MOUNT_ONLY_NO_CONTACT_ADMISSION',
            'started_utc':started.isoformat(),'contact_acceptance':False,'as_built':False,'cases':{},'all_passed':True}
    for branch,target in TARGETS.items():
        coarse=run_case(target,0.0002,False);half=run_case(target,0.0001,False);fixed=run_case(target,0.0002,True)
        metrics=compare(coarse,half)
        coupling={}
        for other,body in TARGETS.items():
            if other==branch:continue
            moving=displacement(coarse,body);locked=displacement(fixed,body)
            coupling[other]={'max_displacement_m':moving,'fixed_rig_max_displacement_m':locked,
                            'passes':moving>1e-10 and locked<1e-12}
        # Disk-shaped (JSON encode/decode) restoration with all energy ledgers.
        cp=json.loads(json.dumps(half['final_checkpoint']))
        p=Plant(half['parameters']);p.restore(cp)
        for _ in range(20):p.step({target:[0,0,.1]})
        expected=p.snapshot();p.restore(cp)
        for _ in range(20):p.step({target:[0,0,.1]})
        replay=p.snapshot()==expected
        passed=all(v for k,v in metrics.items() if k.endswith(('5um','0_05deg','1e_4','1e_5_J','refines'))) and all(v['passes'] for v in coupling.values()) and replay
        report['cases'][branch]={'refinement':metrics,'cross_branch_coupling':coupling,'checkpoint_exact_replay':replay,
                               'performance':{n:v['wall_seconds_per_simulation_second'] for n,v in [('coarse',coarse),('half',half),('fixed',fixed)]},'passed':passed}
        report['all_passed'] &= passed
        for tag,case in [('coarse',coarse),('half',half),('fixed',fixed)]:
            with gzip.open(a.output/(branch+'_'+tag+'.json.gz'),'wt',encoding='utf8') as f:json.dump(case,f,separators=(',',':'),allow_nan=False)
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'branch':branch,**report['cases'][branch]}),flush=True)
    ended=datetime.now(timezone.utc)
    report['completed_utc']=ended.isoformat()
    report['completed_beijing']=ended.astimezone(timezone(timedelta(hours=8))).isoformat()
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['all_passed'] else 1

if __name__=='__main__':raise SystemExit(main())
