"""Force-driven removable-cover workbench; explicit force and constraint work."""
from dataclasses import replace
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from covers import Bench,CoverParameters,ROOT

OUT=ROOT/'results'
CASES=dict(fastened=dict(),drop=dict(release=True),drop_half_dt=dict(release=True,half=True),
           handle_remove=dict(release=True,handle=True),handle_half_dt=dict(release=True,handle=True,half=True))


def run(name,configuration):
    start=time.time(); OUT.mkdir(exist_ok=True)
    sources={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ('covers.py','simulate.py','source_covers.json')}
    params=CoverParameters(timestep_s=.000025 if configuration.get('half') else .00005)
    bench=Bench(params); m,d=bench.model,bench.data; h=float(m.opt.timestep)
    xmlhash=hashlib.sha256(bench.xml.encode('utf-8')).hexdigest()
    (OUT/(name+'.xml')).write_text(bench.xml,encoding='utf-8',newline='\n')
    references={r['key']:dict(position=d.body(r['key']).xpos.tolist(),quaternion_wxyz=d.body(r['key']).xquat.tolist()) for r in bench.spec['covers']}
    initial_energy=float(sum(d.energy)); sums=dict(external=0.,constraint=0.)
    frames=[]; rows=[]; release_change=0.; peak_force=0.; maximum_penetration=0.
    steps=round(2/h); sample=round(.005/h); stride=round((1/60)/h)
    for k in range(steps+1):
        t=k*h
        if configuration.get('release') and k==round(.25/h):
            q,v=d.qpos.copy(),d.qvel.copy()
            for r in bench.spec['covers']:
                bench.release(r['key'])
            release_change=max(float(np.linalg.norm(d.qpos-q)),float(np.linalg.norm(d.qvel-v)))
        d.xfrc_applied[:]=0
        for r in bench.spec['covers']:
            key=r['key']; mass=r['mass_kg']
            if name=='fastened':
                bench.apply_point_force(key,[2,0,0])
            elif configuration.get('handle') and .25<=t<.45:
                # Applied handling force, not a position/speed target. A small
                # world-space lever arm produces actual rotational response.
                bench.apply_point_force(key,[mass*3 if .3<=t<.4 else 0,0,1.5*mass*9.81],[.025,.01,0])
        contact_force=0.; temp=np.zeros(6)
        for ci in range(d.ncon):
            c=d.contact[ci]
            if c.efc_address>=0:
                mujoco.mj_contactForce(m,d,ci,temp)
                if 'bench_floor' in [m.geom(int(g)).name for g in c.geom]:
                    contact_force+=max(0.,float(temp[0]))
                maximum_penetration=max(maximum_penetration,-float(c.dist))
        peak_force=max(peak_force,contact_force)
        if k%sample==0 or k==steps:
            momentum=sum((r['mass_kg']*d.qvel[int(m.joint(r['key']+'_free').dofadr[0]):int(m.joint(r['key']+'_free').dofadr[0])+3] for r in bench.spec['covers']),start=np.zeros(3))
            record=dict(time_s=t,mechanical_energy_delta_J=float(sum(d.energy))-initial_energy,
                        external_work_J=sums['external'],constraint_work_J=sums['constraint'],
                        mechanical_balance_residual_J=float(sum(d.energy))-initial_energy-sum(sums.values()),
                        floor_normal_force_N=contact_force,momentum_balance_residual_Ns=float(np.linalg.norm(momentum-bench.momentum_impulse)))
            for r in bench.spec['covers']:
                key=r['key']; body=d.body(key); index=int(m.joint(key+'_free').dofadr[0])
                record.update({key+'_'+axis+'_m':float(body.xpos[i]) for i,axis in enumerate(('x','y','z'))})
                record[key+'_speed_mps']=float(np.linalg.norm(d.qvel[index:index+3]))
                record[key+'_angular_speed_rad_s']=float(np.linalg.norm(d.qvel[index+3:index+6]))
                record[key+'_fixed_gap_m']=float(np.linalg.norm(body.xpos-np.array(r['initial_bench_center_world_m'])))
                record[key+'_fixture_active']=int(d.eq_active[m.equality(key+'_fixture').id])
            rows.append(record)
        if k%stride==0 or k==steps:
            frames.append(dict(time_s=t,body_transforms={r['key']:dict(position=d.body(r['key']).xpos.tolist(),quaternion_wxyz=d.body(r['key']).xquat.tolist()) for r in bench.spec['covers']}))
        if k==steps:
            break
        work=bench.step()
        for key in sums:
            sums[key]+=work[key]
    for file,digest in sources.items():
        if hashlib.sha256((ROOT/file).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('executed cover source changed')
    replay=dict(schema='B12 inferred maintenance cover force-driven replay v1',model_xml_sha256=xmlhash,
                source_sha256=bench.spec['source_sha256'],reference_body_transforms=references,configuration=bench.spec,frames=frames,
                scope='source-shaped covers on a separate fixed bench; no assembled vehicle pose claim')
    metrics=dict(case=name,duration_s=2.,dt_s=h,configuration=bench.spec,model_xml_sha256=xmlhash,
                 warning_counts=d.warning.number.tolist(),dofs=m.nv,release_events=bench.release_events,
                 release_state_discontinuity=release_change,peak_floor_normal_force_N=peak_force,
                 maximum_contact_penetration_m=maximum_penetration,
                 maximum_mechanical_residual_J=max(abs(r['mechanical_balance_residual_J']) for r in rows),
                 final=rows[-1],runtime_s=time.time()-start)
    with (OUT/(name+'.csv')).open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    (OUT/(name+'_metrics.json')).write_text(json.dumps(metrics,indent=2),encoding='utf-8',newline='\n')
    (OUT/(name+'_replay.json')).write_text(json.dumps(replay,separators=(',',':')),encoding='utf-8',newline='\n')
    files=[OUT/(name+suffix) for suffix in ('.xml','.csv','_metrics.json','_replay.json')]
    identity=dict(solver=mujoco.__version__,numpy=np.__version__,sources_sha256=sources,
                  outputs_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (OUT/(name+'_identity.json')).write_text(json.dumps(identity,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps({k:v for k,v in metrics.items() if k not in ('configuration','final','release_events')},indent=2),flush=True)
    return metrics


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--case',choices=['all',*CASES],default='all')
    name=parser.parse_args().case
    for key in CASES if name=='all' else [name]:
        run(key,CASES[key])
