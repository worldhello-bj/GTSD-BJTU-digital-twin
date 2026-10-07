"""Contact parameter study. Preserves frozen model/results; full force/gas solve.
MuJoCo solref is acceleration-normalized compliance, not directly N/m stiffness.
No geometry, friction coefficient, imposed state, or original peak metric changes.
"""
from pathlib import Path
import sys,json
from xml.etree.ElementTree import fromstring,tostring,indent
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT.parent/'vendor'))
from concurrent.futures import ProcessPoolExecutor,as_completed
CANDIDATES={
 'constant_95':dict(solref='.004 1',solimp='.95 .95 .001'),
 'soft_95':dict(solref='.010 1',solimp='.95 .95 .001'),
 'soft_90':dict(solref='.010 1',solimp='.90 .90 .001'),
 'damped_95':dict(solref='.008 1.5',solimp='.95 .95 .001'),
 'smooth_layer':dict(solref='.008 1',solimp='.1 .95 .0002 .5 2'),
 'impratio_1':dict(impratio=1),
 'friction_tau_50ms':dict(solreffriction='.05 1'),
 'impratio1_friction_tau50ms':dict(impratio=1,solreffriction='.05 1'),
 'tight_solver':dict(solver_iterations=300,solver_tolerance=1e-14),
}
def run(candidate,dt,case="low_adhesion"):
 import simulate_full as sim
 from full_model import model as original
 settings=CANDIDATES[candidate]
 def wrapped(params=None,return_spec=False,return_all_specs=False):
  params=(params or {})|{'include_access_doors':False,'compliant_wheel_rail':False}
  xml,spec,door_spec=original(params,return_all_specs=True);root=fromstring(xml)
  for pair in root.find('contact'):
   if '_flange_' in pair.get('geom1','') or '_flange_' in pair.get('geom2',''):
    pair.set('solref','.004 1');pair.set('solimp','.95 .99 .001')
    for key in ['solref','solimp','solreffriction']:
     if key in settings:pair.set(key,settings[key])
  if 'impratio' in settings:root.find('option').set('impratio',str(settings['impratio']))
  if 'solver_iterations' in settings:
   root.find('option').set('iterations',str(settings['solver_iterations']));root.find('option').set('tolerance',str(settings['solver_tolerance']))
  indent(root);x=tostring(root,encoding='unicode')
  return (x,spec,door_spec) if return_all_specs else (x,spec) if return_spec else x
 sim.model=wrapped;sim.OUT=ROOT/'contact_candidates'/candidate;sim.OUT.mkdir(parents=True,exist_ok=True)
 name=('' if case=='low_adhesion' else case+'_')+f'dt_{round(dt*1e6)}us'
 m=sim.run(name,sim.CASES[case]|dict(timestep_s=dt,include_access_doors=False,compliant_wheel_rail=False),replay=False)
 m['candidate_contact_parameters']=settings;(sim.OUT/f'{name}_metrics.json').write_text(json.dumps(m,indent=2))
 return dict(candidate=candidate,dt=dt,peak_N=m['peak_flange_single_contact_N'],penetration_m=m['max_contact_penetration_m'],stop_m=m['stop_distance_m'],warnings=m['warning_counts'])
if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('--candidates',default='constant_95,soft_95');ap.add_argument('--workers',type=int,default=2);a=ap.parse_args()
 with ProcessPoolExecutor(max_workers=a.workers) as pool:
  tasks=[pool.submit(run,n,dt) for n in a.candidates.split(',') for dt in [.0001,.00005]]
  for task in as_completed(tasks):print('CANDIDATE_RESULT',json.dumps(task.result()),flush=True)
