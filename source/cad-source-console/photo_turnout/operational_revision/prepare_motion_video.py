"""Freeze recorded native poses and identical source meshes for a reproducible motion clip."""
import argparse,gzip,hashlib,json,re,sys,time
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;ROOT=H.parents[1];sys.path.insert(0,str(ROOT/'operational_rig'));from build_rig import load_geometry
ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--frames',type=int,default=90);ap.add_argument('--fps',type=int,default=12);a=ap.parse_args();out=H/'animation_v2';out.mkdir(exist_ok=True);case=a.case;summary=json.loads((case/'summary.json').read_text());contract=json.loads((case/'contract.json').read_text());initial=json.loads((case/'initial_snapshot.json').read_text());assert initial['identity_sha256']==summary['identity_sha256'];assert contract['identity_sha256']==summary['assembly_identity_sha256'];geometry=load_geometry(root=ROOT,contract=contract);assert geometry['source_sha256']==contract['profile_sha256']
steps=[int(x)for x in np.rint(np.linspace(0,summary['steps'],a.frames))];wanted=set(steps);by={0:{'step':0,'time_s':0.,'body_poses':initial['body_poses']}};started=time.monotonic()
with gzip.open(case/'poses.jsonl.gz','rt')as f:
 for line in f:
  match=re.match(r'\s*\{\s*"step"\s*:\s*(\d+)',line);assert match
  step=int(match[1])
  if step in wanted:
   row=json.loads(line);by[step]={'step':step,'time_s':row['time_s'],'body_poses':row['body_poses']}
assert set(by)==wanted,(len(by),len(wanted))
frames=[dict(frame=i,**by[n])for i,n in enumerate(steps)];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();payload={'schema':'gtsd-native-motion-video-input-v1','source_protocol':case.parent.name,'source_case':case.name,'runtime_identity_sha256':summary['identity_sha256'],'assembly_identity_sha256':contract['identity_sha256'],'geometry_source_sha256':geometry['source_sha256'],'source_pose_stream_sha256':sha(case/'poses.jsonl.gz'),'source_contract_sha256':sha(case/'contract.json'),'source_summary_sha256':sha(case/'summary.json'),'native_steps_total':summary['steps'],'native_duration_s':summary['duration_s'],'fps':a.fps,'display_duration_s':len(frames)/a.fps,'slow_playback_factor':len(frames)/a.fps/summary['duration_s'],'interpolated_body_poses':False,'geometry':geometry,'bindings':contract['bindings'],'frames':frames}
(out/'motion_inputs.json.gz').write_bytes(gzip.compress(json.dumps(payload,separators=(',',':')).encode(),mtime=0));(out/'frame_samples.json').write_text(json.dumps({k:v for k,v in payload.items()if k not in ['geometry','bindings','frames']}|{'frames':[{'frame':f['frame'],'native_step':f['step'],'simulation_time_s':f['time_s'],'wheel_position_world_m':f['body_poses']['wheel']['position']}for f in frames]},indent=2)+'\n');print('MOTION_INPUTS_FROZEN',len(frames),len(geometry['meshes']),time.monotonic()-started,flush=True)
