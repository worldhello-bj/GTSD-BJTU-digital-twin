"""Validate five complete cover bench cases and actual executed source identities."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from simulate import CASES,ROOT


def read_csv(path):
    with path.open(encoding='utf-8',newline='') as f:
        reader=csv.DictReader(f); names=reader.fieldnames
        rows=[tuple(float(row[n]) for n in names) for row in reader]
    return np.array(rows,dtype=[(n,'f8') for n in names])


def main():
    checks=[]; metrics={}; arrays={}
    def check(name,ok,detail):
        checks.append(dict(name=name,passed=bool(ok),detail=detail))
    component=json.loads((ROOT/'component_verification.json').read_text(encoding='utf-8'))
    check('components',component['passed'],component)
    for name in CASES:
        folder=ROOT/'results'; path=folder/(name+'_metrics.json')
        if not path.exists():
            check('complete_'+name,False,'missing full 2-second bench'); continue
        m=json.loads(path.read_text(encoding='utf-8')); metrics[name]=m
        a=read_csv(folder/(name+'.csv')); arrays[name]=a
        check('finite_'+name,all(np.isfinite(a[n]).all() for n in a.dtype.names),'all channels')
        check('full_'+name,m['duration_s']==2 and abs(a['time_s'][-1]-2)<1e-10 and len(a)==401,'complete 200 Hz bench samples')
        check('topology_'+name,m['dofs']==24 and m['configuration']['total_mass_kg']==1,'four 6-DOF .25 kg proxies on a fixed separate bench')
        check('warnings_'+name,not any(m['warning_counts']),m['warning_counts'])
        check('mechanical_'+name,m['maximum_mechanical_residual_J']<.005,dict(maximum=m['maximum_mechanical_residual_J'],limit=.005))
        peak=float(max(abs(a['momentum_balance_residual_Ns'])))
        check('momentum_'+name,peak<1e-6,dict(maximum=peak,limit=1e-6))
        thickness=min(min(r['collision_box_dimensions_m']) for r in m['configuration']['covers'])
        check('penetration_'+name,m['maximum_contact_penetration_m']<.5*thickness,dict(maximum=m['maximum_contact_penetration_m'],limit=.5*thickness,scope='numerical box/floor tolerance; uncalibrated contact'))
        check('release_continuity_'+name,m['release_state_discontinuity']==0,'no qpos/qvel writes during fixture release')
        if name=='fastened':
            for r in m['configuration']['covers']:
                key=r['key']; gap=float(max(a[key+'_fixed_gap_m']))
                check('fixed_'+key,gap<1e-5,dict(maximum_gap_m=gap,limit=1e-5))
        else:
            check('four_release_events_'+name,len(m['release_events'])==4 and all(abs(e['time_s']-.25)<1e-9 for e in m['release_events']),'same physical release time at both step sizes')
            for r in m['configuration']['covers']:
                key=r['key']
                check('landed_'+key+'_'+name,m['final'][key+'_z_m']<.1 and m['final'][key+'_speed_mps']<.02,
                      dict(height_m=m['final'][key+'_z_m'],speed_mps=m['final'][key+'_speed_mps']))
        identity=json.loads((folder/(name+'_identity.json')).read_text(encoding='utf-8'))
        for file,digest in identity['sources_sha256'].items():
            check('source_'+name+'_'+file,hashlib.sha256((ROOT/file).read_bytes()).hexdigest()==digest,'actual executed source')
        for file,digest in identity['outputs_sha256'].items():
            check('output_'+name+'_'+file,hashlib.sha256((folder/file).read_bytes()).hexdigest()==digest,'exact output association')
        replay=json.loads((folder/(name+'_replay.json')).read_text(encoding='utf-8'))
        steps,stride=round(2/m['dt_s']),round((1/60)/m['dt_s'])
        expected=steps//stride+1+int(steps%stride!=0)
        check('replay_'+name,len(replay['frames'])==expected and all(len(f['body_transforms'])==4 for f in replay['frames']),dict(frames=len(replay['frames']),expected=expected))
        check('xml_'+name,hashlib.sha256((folder/(name+'.xml')).read_bytes()).hexdigest()==replay['model_xml_sha256']==m['model_xml_sha256'],'exact model association')
        check('quaternions_'+name,all(abs(sum(v*v for v in state['quaternion_wxyz'])-1)<1e-8 for f in replay['frames'] for state in f['body_transforms'].values()),'normalized solver rotations')
    if len(metrics)==5:
        for main,half in [('drop','drop_half_dt'),('handle_remove','handle_half_dt')]:
            change=abs(metrics[half]['peak_floor_normal_force_N']/metrics[main]['peak_floor_normal_force_N']-1)
            check('halfstep_peak_'+main,change<.05,dict(relative_change=change,limit=.05))
            change=max(abs(metrics[half]['final'][r['key']+'_z_m']-metrics[main]['final'][r['key']+'_z_m']) for r in metrics[main]['configuration']['covers'])
            check('halfstep_final_height_'+main,change<.0001,dict(maximum_difference_m=change,limit=.0001))
        for r in metrics['handle_remove']['configuration']['covers']:
            key=r['key']; a=arrays['handle_remove']
            check('handling_lifts_'+key,max(a[key+'_z_m'])>.42,dict(maximum_height_m=float(max(a[key+'_z_m'])),initial_height_m=.4))
            check('handling_rotates_'+key,max(a[key+'_angular_speed_rad_s'])>1,'applied lever-arm torque produces rotation')
    writes=[]
    for file in ('covers.py','simulate.py'):
        for node in ast.walk(ast.parse((ROOT/file).read_text(encoding='utf-8'))):
            if isinstance(node,(ast.Assign,ast.AugAssign,ast.AnnAssign)):
                targets=node.targets if isinstance(node,ast.Assign) else [node.target]
                writes += [file+':'+ast.unparse(t) for t in targets if any(isinstance(p,ast.Attribute) and p.attr in ('qpos','qvel') for p in ast.walk(t))]
    check('no_state_drivers',not writes,writes)
    passed=len(metrics)==5 and all(c['passed'] for c in checks)
    report=dict(passed=passed,status='PASS_WITH_MODEL_LIMITATIONS' if passed else 'INCOMPLETE_OR_FAILED',cases=len(metrics),checks=checks,
                scope='five full independent maintenance-bench cases, four source-shaped proxies; no moving-vehicle integration',
                limits=['Source dimensions unmeasured; two source lid assembly poses unresolved.',
                        'Four assigned .25 kg masses, uniform box inertias/contact and ideal numerical fixtures are inferred.',
                        'Source fin geometry is visual; bounding-box contact is not literal mesh contact.',
                        'No bolt unscrewing, hand geometry, source hinge or measured fixture compliance.',
                        'Maximum allowed penetration is half the thinnest proxy thickness; no real impact-load accuracy claim.'])
    (ROOT/'results/verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps(dict(status=report['status'],checks=len(checks),failed=[r for r in checks if not r['passed']]),indent=2))
    raise SystemExit(0 if passed else 1)


if __name__=='__main__':
    main()
