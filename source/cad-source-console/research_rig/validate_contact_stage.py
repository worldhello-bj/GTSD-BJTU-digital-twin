"""Two station pin-core integration; no claim of whole-assembly contact scope."""
import argparse
import gzip
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path
import time
import numpy as np
from plant import Plant


def run(station,dt,variant='nominal_d5'):
    p=Plant({'mode':'pin_core_contact','gravity_m_s2':[0,0,0],'time_step_s':dt,'pin_geometry_variant':variant})
    target='FIT_'+station+'_F643_NOMINAL_PIN5_8A2C_CANDIDATE'
    direction=-1 if station=='50DF4' else 1
    stride=round(.001/dt);samples=[];trace=[];reactions=[];impulse=0.;onset=None;worst_closure=0.;worst_gap=0.;start=time.monotonic()
    for n in range(round(.15/dt)):
        t=(n+.5)*dt;f=math.sin(math.pi*t/.05)**2 if t<.05 else 0.
        r=p.step({target:[0.,direction*f,0.]});trace.append(r)
        reaction=0.
        for c in r['used_contacts']:
            if c['station']!=station:continue
            sign=1 if c['geom2']=='pin_collision_'+station else -1
            reaction+=-direction*sign*c['force_on_geom2_world_N'][1]
            worst_gap=min(worst_gap,c['signed_gap_m'])
        reactions.append(reaction)
        impulse+=reaction*dt
        if onset is None and reaction>.005:onset=(n+1)*dt
        worst_closure=max(worst_closure,abs(r['contact_generalized_work_closure_error_J']))
        if (n+1)%stride==0:samples.append({'state':r['state'],'axial_reaction_N':reaction,'signed_axial_impulse_Ns':impulse})
    cp=json.loads(json.dumps(p.checkpoint()))
    for _ in range(20):p.step()
    expected=p.snapshot();p.restore(cp)
    for _ in range(20):p.step()
    return {'identity_sha256':p.identity,'contract':p.contract,'samples':samples,'trace':trace,
            'time_step_s':dt,'reactions_N':reactions,'checkpoint':cp,'checkpoint_exact_replay':expected==p.snapshot(),
            'onset_threshold_N':.005,'onset_time_s':onset,'minimum_used_gap_m':worst_gap,
            'contact_work_generalized_closure_max_J':worst_closure,'wall_s':time.monotonic()-start}


def relative_difference(a,b):
    return abs(a-b)/max(abs(a),abs(b),1e-12)


def compare(coarse,half):
    if len(coarse['samples'])!=len(half['samples']):raise ValueError('Sampling grids differ')
    max_position=max_angle=0.
    vc=np.array([s['state']['qvel'] for s in coarse['samples']])
    vh=np.array([s['state']['qvel'] for s in half['samples']])
    dv=np.max(abs(vc-vh),axis=0)
    velocity_limits=np.maximum(1e-4,.02*np.maximum(np.max(abs(vc),axis=0),np.max(abs(vh),axis=0)))
    for xs,ys in zip(coarse['samples'],half['samples']):
        x=xs['state'];y=ys['state']
        for body,px in x['body_poses'].items():
            py=y['body_poses'][body]
            max_position=max(max_position,float(np.linalg.norm(np.array(px['position'])-py['position'])))
            max_angle=max(max_angle,2*math.acos(min(1.,abs(float(np.dot(px['quaternion_wxyz'],py['quaternion_wxyz']))))))
    impulses=[c['samples'][-1]['signed_axial_impulse_Ns'] for c in (coarse,half)]
    energies=[max(abs(x['state']['energy_balance_residual_J']) for x in c['samples']) for c in (coarse,half)]
    raw_peaks=[float(np.max(np.abs(c['reactions_N']))) for c in (coarse,half)]
    smooth_peaks=[]
    for c in (coarse,half):
        n=round(.001/c['time_step_s'])
        smooth_peaks.append(float(np.max(abs(np.convolve(c['reactions_N'],np.ones(n)/n,'valid')))))
    onset=[coarse['onset_time_s'],half['onset_time_s']]
    metrics={'maximum_all_body_position_difference_m':max_position,'maximum_all_body_angle_difference_rad':max_angle,
        'maximum_generalized_velocity_difference':float(dv.max()),'per_dof_velocity_difference':dv.tolist(),
        'per_dof_velocity_limits':velocity_limits.tolist(),'failed_velocity_dofs':np.flatnonzero(dv>velocity_limits).tolist(),
        'signed_axial_impulse_Ns':impulses,'signed_axial_impulse_relative_difference':relative_difference(*impulses),
        'raw_absolute_reaction_peak_N':raw_peaks,'raw_peak_relative_difference':relative_difference(*raw_peaks),
        'one_ms_mean_absolute_peak_N':smooth_peaks,'one_ms_peak_relative_difference':relative_difference(*smooth_peaks),
        'energy_balance_max_J':energies,'onset_time_s':onset,
        'minimum_used_gap_m':[coarse['minimum_used_gap_m'],half['minimum_used_gap_m']],
        'exact_checkpoint_replay':coarse['checkpoint_exact_replay'] and half['checkpoint_exact_replay'],
        'maximum_contact_work_closure_J':max(coarse['contact_work_generalized_closure_max_J'],half['contact_work_generalized_closure_max_J']),
        'wall_s':[coarse['wall_s'],half['wall_s']]}
    metrics['gates']={'position_5um':max_position<=5e-6,'angle_0_05deg':max_angle<=math.radians(.05),
        'velocity_per_dof_max_1e_4_or_2percent_peak':bool(np.all(dv<=velocity_limits)),
        'signed_impulse_3percent':metrics['signed_axial_impulse_relative_difference']<=.03,
        'raw_peak_5percent':metrics['raw_peak_relative_difference']<=.05,
        'one_ms_peak_5percent':metrics['one_ms_peak_relative_difference']<=.05,
        'onset_two_coarse_steps':all(t is not None for t in onset) and abs(onset[0]-onset[1])<=2*coarse['time_step_s']+1e-12,
        'half_energy_residual_10uJ':energies[1]<=1e-5,'residual_refines':energies[1]<=energies[0]*1.02+1e-12,
        'checkpoint_exact':metrics['exact_checkpoint_replay'],'work_closure_1e_10_J':metrics['maximum_contact_work_closure_J']<1e-10}
    metrics['passed_declared_gates']=all(metrics['gates'].values())
    return metrics


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--pin-variant',default='research_d4p98',choices=['nominal_d5','research_d4p98'])
    ap.add_argument('--coarse-step',type=float,default=.0001)
    a=ap.parse_args()
    if a.coarse_step not in (.0002,.0001,.00005):ap.error('Use 200, 100 or 50 microseconds')
    a.output.mkdir(parents=True,exist_ok=True)
    started=datetime.now(timezone.utc)
    report={'schema':'gtsd-shared-rig-pin-contact-validation-v2','contact_scope':'two F643 pin/B535/slot interfaces',
        'full_assembly_contact_acceptance':False,'as_built':False,'pin_geometry_variant':a.pin_variant,
        'time_steps_s':[a.coarse_step,a.coarse_step/2],'started_utc':started.isoformat(),
        'protocol_revision_note':'V1 strict absolute 1e-4 velocity failure is retained. V2 explicitly reports a per-DOF absolute-or-relative gate and raw/1ms force peaks; it does not relabel V1 runs.',
        'input':{'gravity_m_s2':[0,0,0],'axial_force_peak_N':1,'pulse_duration_s':.05,'observation_s':.15,'sample_interval_s':.001},
        'cases':{}}
    for station in ('50DF4','51409'):
        coarse=run(station,a.coarse_step,a.pin_variant)
        half=run(station,a.coarse_step/2,a.pin_variant)
        metrics=compare(coarse,half)
        report['cases'][station]=metrics
        for label,case in [('coarse',coarse),('half',half)]:
            with gzip.open(a.output/(station+'_'+label+'.json.gz'),'wt') as f:json.dump(case,f,separators=(',',':'),allow_nan=False)
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({station:metrics}),flush=True)
    ended=datetime.now(timezone.utc)
    report['completed_utc']=ended.isoformat()
    report['completed_beijing']=ended.astimezone(timezone(timedelta(hours=8))).isoformat()
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if all(c['passed_declared_gates'] for c in report['cases'].values()) else 1

if __name__=='__main__':raise SystemExit(main())
