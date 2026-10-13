"""Read-only main session and explicit independent candidate on the same server."""
import os
import json,time,threading,uuid
from pathlib import Path
from http.client import HTTPConnection
from source_console import make_server
out=Path(os.environ.get('GTSD_ACCEPTANCE_OUTPUT_DIR',str(Path(__file__).parent/'evidence')));out.mkdir(parents=True,exist_ok=True);server,ctl,ctx=make_server(0,'cad_seat_research');ctl.start();thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();log=[]
def req(path,body=None,status=200,headers=None):
 h=HTTPConnection('127.0.0.1',server.server_port,timeout=25);h.request('POST'if body is not None else'GET',path,None if body is None else json.dumps(body),headers or({'Content-Type':'application/json'}if body is not None else{}));r=h.getresponse();raw=r.read();h.close();log.append({'path':path,'status':r.status,'bytes':len(raw)});assert r.status==status,(r.status,raw[:300]);return json.loads(raw) if 'api/'in path else raw
try:
 for _ in range(200):
  s=req('/api/assembly/runtime')
  if s['phase']=='paused':break
  assert s['phase']!='error',s;time.sleep(.05)
 assert s['steps']==0;cp=req('/api/checkpoint');lease=ctl.last_client
 assert req('/assembly/source-overlays.js');ov=req('/api/assembly/source-overlays');assert len(ov['meshes'])>=14 and len(ov['display_review_contract']['source_review_suppress_exact'])==4
 env=req('/api/assembly/environment-references');assert env['row_count']==157 and env['read_only_semantics']
 assert req('/api/pantograph-candidate/status')['phase']=='disabled'
 for p in ['/pantograph-candidate','/pantograph-candidate/app.js','/pantograph-candidate/core.js','/pantograph-candidate/style.css'] : assert req(p)
 assert req('/api/pantograph-candidate/status')['phase']=='disabled';assert req('/api/checkpoint')==cp;assert ctl.last_client==lease
 req('/api/pantograph-candidate/initialize',{'mode':'guess'},400)
 st=req('/api/pantograph-candidate/initialize',{'mode':'H1_DUAL_FOURBAR_PLANAR'});assert st['initialized']and st['steps']==0
 def cmd(action,**kwargs):
  nonlocal_placeholder=None
  payload={'session_id':st['session_id'],'expected_revision':st['revision'],'request_id':uuid.uuid4().hex,'action':action}|kwargs
  return req('/api/pantograph-candidate/command',payload),payload
 st,_=cmd('checkpoint');saved=st['created_checkpoint_id'];st,payload=cmd('step',steps=10,torque_Nm=.05);end=st;assert st['steps']==10
 assert req('/api/pantograph-candidate/command',payload)==end
 st,_=cmd('restore',checkpoint_id=saved);assert st['steps']==0
 st,_=cmd('step',steps=10,torque_Nm=.05);assert st['qpos']==end['qpos'] and st['qvel']==end['qvel']
 assert req('/api/checkpoint')==cp and req('/api/assembly/runtime')['steps']==0
 req('/api/pantograph-candidate/run',{'anything':True},404)
 req('/api/pantograph-candidate/initialize',{'mode':'H1_DUAL_FOURBAR_PLANAR'},403,{'Content-Type':'application/json','Origin':'http://foreign.example'})
 result={'ok':True,'old_main_checkpoint_unchanged':True,'read_routes_do_not_initialize_candidate':True,'main_read_lease_unchanged':True,'independent_candidate_native_steps':20,'duplicate_request_no_extra_step':True,'candidate_in_memory_restore_and_replay_exact':True,'unknown_run_and_cross_origin_rejected':True,'browser':False,'hardware_connected':False}
 (out/'joint_routes_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
 server.shutdown();server.server_close();thread.join();ctl.close();(out/'joint_routes_http_log.json').write_text(json.dumps(log,indent=2))
