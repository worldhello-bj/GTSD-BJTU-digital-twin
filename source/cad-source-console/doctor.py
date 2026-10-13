"""Read-only portable admission. No simulation steps, hardware, or downloads."""
from pathlib import Path
import hashlib,json,sys,time
sys.dont_write_bytecode=True
R=Path(__file__).resolve().parent
from prepare_assets import ensure_assets
ensure_assets()
sys.dont_write_bytecode=True;sys.path.insert(0,str(R/'integration'))
def check_manifest(path,root):
 value=json.loads(path.read_text());files=value['files']
 for name,row in files.items():
  p=root/name
  if not p.is_file() or p.stat().st_size!=row['bytes'] or hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:raise RuntimeError('File identity mismatch: '+str(p))
 return len(files)
def main():
 started=time.monotonic();base=R/'baseline_console/GTSD_Assembly_Console'
 n=check_manifest(base/'BUNDLE_MANIFEST.json',base)
 release=check_manifest(R/'PUBLIC_MANIFEST.json',R)
 import numpy as np
 from plant import raw_type
 from seat_plant import raw_type as seat_type
 results=[]
 for mode,fn,folder in [('source_rails',raw_type,'evidence'),('cad_seat_research',seat_type,'evidence_seats')]:
  T,_=fn();p=T();ref=json.loads((R/'integration'/folder/'initial_validation.json').read_text());s=p.snapshot()
  assert s['model_identity_sha256']==ref['model_identity_sha256'];before=p._mechanical_state().copy();cp=p.checkpoint();p.restore(cp)
  assert p.checkpoint()==cp and np.array_equal(before,p._mechanical_state()) and p.snapshot()==s
  results.append({'mode':mode,'identity':s['model_identity_sha256'],'nq':p.model.nq,'nv':p.model.nv,'time_s':p.data.time,'restore_exact':True})
 sys.path.insert(0,str(R/'pantograph'));from candidate_session import CandidateSession,MODE
 c=CandidateSession();assert not c.initialized;c.initialize(MODE);assert c.snapshot()['steps']==0
 print(json.dumps({'ok':True,'frozen_files_checked':n,'release_files_checked':release,'variants':results,'candidate_cold_init':True,'steps_integrated':0,'wall_s':time.monotonic()-started},indent=2))
if __name__=='__main__':main()
