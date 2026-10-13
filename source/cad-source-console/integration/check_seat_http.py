"""Actual wrapper HTTP / native poses / restore, bounded to three physics steps."""
import os
from pathlib import Path
import sys,json,threading,time,uuid,hashlib,gzip,signal
from http.client import HTTPConnection
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R));import source_console as A
E=Path(os.environ.get('GTSD_ACCEPTANCE_OUTPUT_DIR',str(R/'evidence_seats')));E.mkdir(parents=True,exist_ok=True)
signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(TimeoutError('240s bounded acceptance')));signal.alarm(240)
server,ctl,ctx=A.make_server(0,'cad_seat_research');c=ctx['config'];factory=ctl.factory;calls=[]
def counted_factory():
 p=factory();original=p.step
 def step(n):
  assert n==1 and len(calls)<3
  start=time.monotonic();original(n);calls.append({'wall_s':time.monotonic()-start,'steps':p.steps})
 p.step=step;return p
ctl.factory=counted_factory;ctl.start();thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();journal=[];begin=time.monotonic();result={'ok':False}
def req(path,body=None,expected=200):
 h=HTTPConnection('127.0.0.1',server.server_port,timeout=80);h.request('POST' if body is not None else 'GET',path,None if body is None else json.dumps(body),{} if body is None else {'Content-Type':'application/json'});r=h.getresponse();data=r.read();headers=dict(r.getheaders());h.close();journal.append({'path':path,'status':r.status,'bytes':len(data)});assert r.status==expected,(r.status,data[:400]);return data,headers
read=lambda path='/api/assembly/runtime':json.loads(req(path)[0])
def wait(pred):
 until=time.monotonic()+90
 while time.monotonic()<until:
  s=read();assert s['phase']!='error',s
  if pred(s):return s
  time.sleep(.05)
 raise AssertionError('timeout')
def cmd(action,args,session=None,rid=None):
 p={'session_id':session or read()['session_id'],'request_id':rid or uuid.uuid4().hex,'action':action,'args':args};req('/api/command',p,202)
 s=wait(lambda s:any(e['request_id']==p['request_id']for e in s['events']));e=next(e for e in s['events']if e['request_id']==p['request_id']);assert e['status']=='applied',e;return s
try:
 initial=wait(lambda s:s['phase']=='paused');assert initial['steps']==0 and len(initial['body_poses'])>0;assert initial['variant']=='cad_assembly_1013_source_rails_near_seats_research'
 (E/'actual_runtime_initial.json').write_text(json.dumps(initial,ensure_ascii=False,separators=(',',':')))
 cp0=read('/api/checkpoint');lease=ctl.last_client
 for route in ['/assembly','/assembly/core.js','/assembly/model.js','/assembly/app.js','/assembly/lineage.js','/assembly/style.css','/assembly/registration.js','/assembly/source-overlays.js','/api/assembly/source-overlays','/api/assembly/registration','/api/assembly/bogie-registration','/assembly/bogie-registration.js','/api/assembly/catalog','/api/assembly/mapping','/api/assembly/runtime']:
  data,h=req(route);assert len(data)>0
 for route,expected_hash in [('/assembly/model.glb',c['catalogue']['base_glb']['sha256']),('/assembly/extras.json',c['catalogue']['extras']['sha256'])]:
  data,h=req(route);assert h['Content-Encoding']=='gzip';assert hashlib.sha256(gzip.decompress(data)).hexdigest()==expected_hash
 assert ctl.last_client==lease and cp0==read('/api/checkpoint')
 req('/assembly/../private',expected=404)
 cmd('step',{'steps':1});s1=wait(lambda s:s['steps']==1 and not s['single_steps_remaining']);cp1=read('/api/checkpoint')
 cmd('step',{'steps':1});s2=wait(lambda s:s['steps']==2 and not s['single_steps_remaining']);cp2=read('/api/checkpoint')
 restored=cmd('restore_checkpoint',{'checkpoint':cp1});assert restored['session_id']!=s2['session_id'];assert read('/api/checkpoint')==cp1 and restored['body_poses']==s1['body_poses']
 cmd('step',{'steps':1});s3=wait(lambda s:s['steps']==2 and not s['single_steps_remaining']);assert read('/api/checkpoint')==cp2;assert s3['body_poses']==s2['body_poses']
 for key,s in [('step1',s1),('step2',s2),('restored',restored),('continued',s3)]:(E/f'actual_runtime_{key}.json').write_text(json.dumps(s,ensure_ascii=False,separators=(',',':')))
 for k,v in [('step1',cp1),('step2',cp2)]:(E/f'actual_checkpoint_{k}.json').write_text(json.dumps(v,ensure_ascii=False,separators=(',',':')))
 changed=[b for b in s1['body_poses'] if s1['body_poses'][b]!=initial['body_poses'][b]]
 assert changed
 result={'ok':True,'native_calls':calls,'runtime_pose_count':len(initial['body_poses']),'changed_bodies_after_step1':changed,'readonly_routes_no_mutation_or_lease':True,'complete_resource_hashes':True,'restore_checkpoint_exact_json_structure':True,'restore_body_poses_exact':True,'continued_checkpoint_and_poses_exact':True,'new_physics_scope':'source rail proxy bounds plus 2 near-side spring sites/rest lengths; original mass and DOF retained','wall_s':time.monotonic()-begin,'browser':False,'uncompressed_glb_written':False}
 print(json.dumps(result,indent=2),flush=True)
finally:
 server.shutdown();server.server_close();thread.join();ctl.close();signal.alarm(0);(E/'actual_http_result.json').write_text(json.dumps(result,indent=2)+'\n');(E/'actual_http_requests.json').write_text(json.dumps(journal,indent=2)+'\n')
