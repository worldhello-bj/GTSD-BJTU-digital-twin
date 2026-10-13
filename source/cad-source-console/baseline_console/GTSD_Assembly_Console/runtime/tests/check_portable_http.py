"""Bounded real loopback regression against relocated, byte-pinned strict82.
No raw class changes. A display-instance wrapper counts native steps and can
hold after one accepted step so the production five-second lease expires
without unbounded integration. This is test pacing, not a numerical change.
"""
from pathlib import Path
import copy,hashlib,json,os,signal,sys,threading,time,uuid
from http.client import HTTPConnection
HERE=Path(__file__).resolve().parents[1]
PORTABLE=HERE/'portable'
EVIDENCE=HERE/'evidence'
opened=set();blocked=[]
allowed=[str(HERE.resolve()),str(Path(sys.prefix).resolve())]
def audit(event,args):
 if event in ('open','os.listdir','os.scandir') and args and isinstance(args[0],(str,bytes)):
  p=str(Path(os.fsdecode(args[0])).resolve())
  if p.startswith('/workspace/shared/') and not any(p==a or p.startswith(a+'/') for a in allowed):
   blocked.append({'event':event,'path':p});raise PermissionError('Portable audit denied old/shared project access: '+p)
  if event=='open':opened.add(p)
sys.addaudithook(audit)
sys.dont_write_bytecode=True
signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(TimeoutError('Acceptance wall-clock cap: 240 seconds')))
signal.alarm(240)
manifest=json.loads((PORTABLE/'PORTABLE_MANIFEST.json').read_text())
def check_files():
 for name,row in manifest['files'].items():
  p=PORTABLE/name;assert not p.is_symlink();assert hashlib.sha256(p.read_bytes()).hexdigest()==row['sha256'],name
check_files()
CONSOLE=PORTABLE/'GTSD_Phase6/console_strict82'
sys.path.insert(0,str(CONSOLE))
import strict82_server as S
c=S.load_config(CONSOLE/'strict82_config.portable.json')
c['base']['child']['base']['port']=0
preflight=S.doctor(c,check_bind=False);assert preflight['ok'],preflight
(EVIDENCE/'relocated_doctor.json').write_text(json.dumps(preflight,ensure_ascii=False,indent=2)+'\n')
server,ctl,ctx=S.make_server(c)
base_factory=ctl.factory
budget={'maximum_native_calls':8,'actual_native_calls':0,'hold_next_step_after_acceptance_s':0.,'records':[]}
def bounded_factory():
 plant=base_factory();native_step=plant.step
 def bounded_step(n):
  assert n==1,n
  if budget['actual_native_calls']>=budget['maximum_native_calls']:raise RuntimeError('Test native integration budget exceeded')
  budget['actual_native_calls']+=1
  with (EVIDENCE/'native_calls.jsonl').open('a') as stream:stream.write(json.dumps({'event':'call_begin','call':budget['actual_native_calls']})+'\n')
  start=time.monotonic();native_step(n)
  budget['records'].append({'call':budget['actual_native_calls'],'steps':plant.steps,'wall_s':time.monotonic()-start})
  with (EVIDENCE/'native_calls.jsonl').open('a') as stream:stream.write(json.dumps({'event':'call_accepted',**budget['records'][-1]})+'\n')
  delay=budget['hold_next_step_after_acceptance_s'];budget['hold_next_step_after_acceptance_s']=0.
  if delay:time.sleep(delay)
 plant.step=bounded_step
 return plant
ctl.factory=bounded_factory
ctl.start();t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
http=[];checks=[];begin=time.monotonic()
def journal(label,detail=None):
 with (EVIDENCE/"progress.jsonl").open("a") as stream:stream.write(json.dumps({"wall_s":time.monotonic()-begin,"label":label,"detail":detail},ensure_ascii=False)+"\n")
journal("startup_in_progress")
def request(path,body=None,expected=(200,202),headers=None,raw=False):
 start=time.monotonic();h=HTTPConnection('127.0.0.1',server.server_port,timeout=80)
 data=body if raw else (None if body is None else json.dumps(body))
 h.request('GET' if body is None else 'POST',path,data,headers or ({} if body is None else {'Content-Type':'application/json'}))
 r=h.getresponse();result=r.read();hs=dict(r.getheaders());h.close()
 row={'method':'GET' if body is None else 'POST','path':path,'status':r.status,'bytes':len(result),'wall_s':time.monotonic()-start}
 if isinstance(body,dict):row.update(action=body.get('action'),request_id=body.get('request_id'))
 if r.status>=400:row['response']=result.decode(errors='replace')
 http.append(row)
 if path!='/api/observation':journal('http',row)
 assert r.status in expected,(r.status,result[:1000]);return result,hs
read=lambda path='/api/observation':json.loads(request(path)[0])
def wait(pred,seconds=90):
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  st=read();assert st['phase']!='error',st.get('error')
  if pred(st):return st
  time.sleep(.04)
 raise AssertionError('Readiness timeout')
def payload(action,args,session=None,rid=None):
 return {'session_id':session or read()['session_id'],'request_id':rid or uuid.uuid4().hex,'action':action,'args':args}
def finish(p,status='applied'):
 st=wait(lambda s:any(e['request_id']==p['request_id'] for e in s['events']))
 e=next(e for e in st['events'] if e['request_id']==p['request_id']);assert e['status']==status,e
 checks.append({'action':p['action'],'request_id':p['request_id'],'result':e});journal('command_finished',checks[-1])
 import gc;gc.collect()
 return st

def command(action,args,status='applied'):
 p=payload(action,args);request('/api/command',p);return finish(p,status)
def resign(value):
 if isinstance(value,dict):
  for k,v in value.items():
   if k!='sha256':resign(v)
  if 'sha256'in value:value['sha256']=S.hashlib.sha256(S.canonical({k:v for k,v in value.items() if k!='sha256'})).hexdigest()
 elif isinstance(value,list):
  for v in value:resign(v)
result={'ok':False}
try:
 initial=wait(lambda s:s['phase']=='paused');p=initial['plant'];assert p['steps']==0 and initial['single_steps_remaining']==0
 assert p['model_identity_sha256']=='9bd3b5a7f26a40c44f1b77f5c71b9ec53488e682170ddfda651a939aa96323db'
 assert p['variant']==S.ACTUAL and p['lineage_observations']['identity_match']
 (EVIDENCE/'relocated_initial_state.json').write_text(json.dumps(initial,ensure_ascii=False,indent=2)+'\n')
 for label,path in [('binding','/api/binding'),('geometry','/geometry.json')]:
  body,hs=request(path);(EVIDENCE/f'relocated_initial_{label}.json').write_bytes(body)
 print('INITIAL_REFERENCE_READY',flush=True)
 for path in ('/','/app.js','/ui-model.js','/contact-sections.js','/internal-systems.js','/style.css','/physics-model.js','/render-model.js','/fan82.js','/fan82.css','/truth-evidence.js','/truth-evidence.css','/truth-evidence-client.js','/api/truth-evidence','/api/internal-registry'):
  body,headers=request(path)
  if path=='/':assert b'four-fan-panel'in body and b'lineage-panel'in body and "default-src 'none'"in headers['Content-Security-Policy']
 panel=read('/api/four-door-fans');cat=read('/api/lineage-registry')
 assert panel['inventory']=={'rotating_axes':6,'sliding_axes':1,'native_added_coordinates':7,'native_nv':82,'native_nq':94}
 default=S.strict((CONSOLE/'console_config.portable.json').read_bytes());assert default['variant']=='internal-coupled-research'
 # Paused observations never renew the lease or alter complete state.
 cp0=read('/api/checkpoint');lease=ctl.last_client
 for _ in range(2):read('/api/four-door-fans');read('/api/phase6-observations');read('/api/lineage-observation')
 assert ctl.last_client==lease and read('/api/checkpoint')==cp0
 # Valid four-fan command, no extra integration until explicit step.
 command('four_door_fan',{'key':'all','voltage_fraction':.2,'power_failed':False})
 invalid=[('unknown',{}),('step',{'steps':0}),('step',{'steps':True}),('step',{'steps':2501}),('pause',{'extra':1}),('four_door_fan',{'key':'bogus','voltage_fraction':.2,'power_failed':False}),('four_door_fan',{'key':'all','voltage_fraction':1.2,'power_failed':False})]
 before_invalid=read('/api/checkpoint')
 for action,args in invalid:request('/api/command',payload(action,args),expected=(400,))
 request('/api/command',payload('pause',{},session='stale_session'),expected=(400,))
 request('/api/command','{"a":1,"a":2}',expected=(400,),raw=True)
 request('/api/command','{"a":NaN}',expected=(400,),raw=True)
 request('/api/command','{}',expected=(415,),headers={'Content-Type':'text/plain'},raw=True)
 request('/api/observation',expected=(403,),headers={'Origin':'https://example.invalid'})
 assert read('/api/checkpoint')==before_invalid
 # Duplicate request is accepted as the same receipt and causes one step only.
 step=payload('step',{'steps':1});request('/api/command',step)
 repeated=json.loads(request('/api/command',step)[0]);assert repeated['status']=='already_received'
 finish(step);accepted=wait(lambda s:s['plant']['steps']==1 and not s['single_steps_remaining']);cp1=read('/api/checkpoint')
 repeat_after=json.loads(request('/api/command',step)[0]);assert repeat_after['status']=='already_received'
 changed=copy.deepcopy(step);changed['args']['steps']=2;request('/api/command',changed,expected=(400,))
 assert read('/api/checkpoint')==cp1
 assert all(x['available']for x in read('/api/four-door-fans')['observations'])
 # Deliberately wrong native identity, re-signed all enclosing checksum fields.
 bad=copy.deepcopy(cp1);bad['plant']['plant']['identity_sha256']='0'*64;resign(bad)
 command('restore_checkpoint',{'checkpoint':bad},'rejected');assert read('/api/checkpoint')==cp1
 assert read('/api/lineage-registry')['catalogue_sha256']==cat['catalogue_sha256']
 # Both strict speed-history attacks remain rejected on this portable path.
 for field in ('air_speed_rad_s','relative_speed_rad_s'):
  bad=copy.deepcopy(cp1);mapping=bad['plant']['four_fans']['last'][field];mapping[next(iter(mapping))]+=.1;resign(bad)
  command('restore_checkpoint',{'checkpoint':bad},'rejected');assert read('/api/checkpoint')==cp1
 command('step',{'steps':1});advanced=wait(lambda s:s['plant']['steps']==2 and not s['single_steps_remaining']);cp2=read('/api/checkpoint')
 old_session=advanced['session_id'];restored=command('restore_checkpoint',{'checkpoint':cp1});assert restored['session_id']!=old_session
 assert read('/api/checkpoint')==cp1
 for key in ('qpos','qvel','telemetry','solver_proxies','lineage_observations'):assert restored['plant'][key]==accepted['plant'][key],key
 request('/api/command',payload('pause',{},session=old_session),expected=(400,))
 command('step',{'steps':1});continued=wait(lambda s:s['plant']['steps']==2 and not s['single_steps_remaining'])
 assert read('/api/checkpoint')==cp2
 for key in ('qpos','qvel','telemetry','solver_proxies','lineage_observations'):assert continued['plant'][key]==advanced['plant'][key],key
 # Queue pause over real HTTP immediately after a real run. The accepted
 # running step completes before pause; paused state has no residual steps.
 start_steps=continued['plant']['steps'];runp=payload('run',{});pausep=payload('pause',{},session=runp['session_id'])
 request('/api/command',runp);request('/api/command',pausep);finish(runp);paused=finish(pausep)
 assert paused['phase']=='paused' and not paused['single_steps_remaining']
 assert paused['plant']['steps']>=start_steps+1
 run_pause_steps=paused['plant']['steps']-start_steps
 # Actual 5s lease, with a test-only post-step hold preventing many native
 # integrations. Read-only endpoints continue while disconnected; no /state.
 assert ctl.lease_s==5.0
 budget['hold_next_step_after_acceptance_s']=5.15
 before=paused['plant']['steps'];lease_begin=time.monotonic();lease_command=payload('run',{});request('/api/command',lease_command);finish(lease_command)
 disconnected=wait(lambda s:s['phase']=='paused' and any(e['action']=='client_lease_expired' for e in s['events']),seconds=20)
 lease_elapsed=time.monotonic()-lease_begin
 assert disconnected['plant']['steps']==before+1
 assert lease_elapsed>=5.0
 cp_end=read('/api/checkpoint');last_client=ctl.last_client
 read('/api/four-door-fans');read('/api/phase6-observations');assert ctl.last_client==last_client and read('/api/checkpoint')==cp_end
 check_files();assert blocked==[],blocked
 (EVIDENCE/'checkpoint_step1.json').write_text(json.dumps(cp1,ensure_ascii=False,indent=2)+'\n')
 (EVIDENCE/'checkpoint_step2.json').write_text(json.dumps(cp2,ensure_ascii=False,indent=2)+'\n')
 result={'ok':True,'scope':'Relocated Python audit + real loopback HTTP short regression; Node DOM logic recorded separately, no browser/GPU or new-assembly physics certification','native_identity':p['model_identity_sha256'],'geometry_sha256':p['source_geometry_sha256'],'interface_identity':p['display_interface']['strict82_console_identity_sha256'],'startup_paused_steps':0,'native_integration_budget':budget,'run_pause_native_steps':run_pause_steps,'disconnect_lease_s':ctl.lease_s,'disconnect_elapsed_s':lease_elapsed,'disconnect_native_steps':1,'lease_test_pacing':'Test-only 5.15s post-acceptance sleep on display instance step, default controller clock/lease untouched; native MRO and numerical methods unchanged.','duplicate_step_once':True,'wrong_native_identity_resigned_rejected':True,'strict_history_attacks_rejected':True,'failed_restore_keeps_checkpoint_and_catalogue':True,'exact_restore':True,'continuation_checkpoint_and_display_exact':True,'post_restore_stale_session_rejected':True,'invalid_http_cases':len([r for r in http if r['status']>=400]),'readonly_endpoints_no_lease_renewal':True,'portable_file_count':len(manifest['files']),'copied_source_files_unchanged':True,'old_project_read_attempts':blocked,'wall_s':time.monotonic()-begin,'browser_GPU':False,'new_assembly_physics':False,'default75_template_unchanged':True}
 print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
finally:
 server.shutdown();server.server_close();t.join();ctl.close();signal.alarm(0)
 (EVIDENCE/'portable_http_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 (EVIDENCE/'http_transcript.json').write_text(json.dumps(http,ensure_ascii=False,indent=2)+'\n')
 (EVIDENCE/'command_results.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n')
 (EVIDENCE/'audit_reads.json').write_text(json.dumps({'schema':'gtsd-portable-python-audit-v1','hook_events':['open','os.listdir','os.scandir'],'denied_scope':'/workspace/shared except this acceptance tree and the active Python venv prefix','allowed_python_prefix':sys.prefix,'early_interpreter_bootstrap_before_hook':'not observed; no project imports performed before hook','native_library_syscalls':'Python audit hook is not an OS-wide native syscall tracer','blocked':blocked,'opened_paths':sorted(opened)},ensure_ascii=False,indent=2)+'\n')
