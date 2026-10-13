"""Bounded gravity preparation with exact checkpoint and explicit stop criteria."""
import argparse
from datetime import datetime, timezone, timedelta
import gzip
import json
from pathlib import Path
import time
import numpy as np
from plant import Plant


def run():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    started=datetime.now(timezone.utc)
    p=Plant({'mode':'pin_core_contact','pin_geometry_variant':'research_d4p98',
             'time_step_s':.00005,'gravity_m_s2':[0.,0.,-9.81],
             'rig_stiffness':[400000.,400000.,10000.]})
    start=time.monotonic();held=0;error=None;gap=0.;maximum_residual=0.;samples=[]
    required=round(.1/p.model.opt.timestep)
    try:
        for n in range(round(2./p.model.opt.timestep)):
            row=p.step();s=row['state']
            speed=float(np.max(abs(p.data.qvel)))
            imbalance=float(np.max(abs(p.data.qfrc_passive+p.data.qfrc_constraint-p.data.qfrc_bias)))
            gap=min(gap,s['minimum_contact_gap_m'] or 0.)
            maximum_residual=max(maximum_residual,abs(s['energy_balance_residual_J']))
            held=held+1 if speed<1e-4 and imbalance<.01 else 0
            if (n+1)%20==0:
                samples.append({'time_s':s['time_s'],'max_generalized_velocity':speed,
                                'max_generalized_force_imbalance':imbalance,
                                'energy_balance_residual_J':s['energy_balance_residual_J'],
                                'minimum_contact_gap_m':s['minimum_contact_gap_m']})
            if held>=required:break
    except Exception as exc:error=repr(exc)
    cp=p.checkpoint();expected=None;replay=False
    if not error:
        for _ in range(20):p.step()
        expected=p.snapshot();p.restore(cp)
        for _ in range(20):p.step()
        replay=p.snapshot()==expected;p.restore(cp)
    ended=datetime.now(timezone.utc)
    report={'schema':'gtsd-pin-contact-gravity-v1','identity_sha256':p.identity,
        'started_utc':started.isoformat(),'completed_utc':ended.isoformat(),
        'completed_beijing':ended.astimezone(timezone(timedelta(hours=8))).isoformat(),
        'parameters':p.contract['parameters'],'steps':p.steps,'time_s':p.steps*p.model.opt.timestep,
        'wall_s':time.monotonic()-start,'settled_continuous_0_1s':held>=required,
        'criteria':{'every_dof_velocity_limit':1e-4,'slide_velocity_unit':'m/s','rotational_velocity_unit':'rad/s',
                    'every_dof_force_balance_limit':.01,'slide_force_unit':'N','rotational_force_unit':'N m',
                    'continuous_hold_s':.1,'maximum_simulated_preparation_s':2.},
        'max_generalized_velocity':speed,'max_generalized_force_imbalance':imbalance,
        'minimum_contact_gap_m':gap,'maximum_energy_residual_J':maximum_residual,
        'exact_checkpoint_continuation':replay,'error':error,'as_built':False,
        'calibrated_mass_stiffness_or_friction':False}
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    with gzip.open(a.output/'checkpoint_and_samples.json.gz','wt') as f:
        json.dump({'report':report,'contract':p.contract,'checkpoint':cp,'samples':samples},f,separators=(',',':'),allow_nan=False)
    print(json.dumps(report),flush=True)
    return 0 if report['settled_continuous_0_1s'] and replay and not error else 1

if __name__=='__main__':raise SystemExit(run())
