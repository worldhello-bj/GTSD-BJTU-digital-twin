"""Small-sample loopback latency, explicitly not browser p95 or real-time throughput."""
from pathlib import Path
import sys,json,time,threading,uuid,statistics
from http.client import HTTPConnection
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import phase7_server as P
config=P.load_config();S=P.phase6_module();cfg=config['base'].copy();cfg.update(variant=S.VARIANT,port=0)
start=time.perf_counter(); server,controller=S.make_server(cfg);thread=threading.Thread(target=server.serve_forever,daemon=True);controller.start();thread.start()
def req(path,payload=None):
 t=time.perf_counter();c=HTTPConnection('127.0.0.1',server.server_port,timeout=60);c.request('GET' if payload is None else 'POST',path,None if payload is None else json.dumps(payload),{} if payload is None else {'Content-Type':'application/json'});r=c.getresponse();b=r.read();c.close();ms=(time.perf_counter()-t)*1000;assert r.status in (200,202),(r.status,b[:300]);return json.loads(b),ms
def state():return req('/api/observation')[0]
def wait(pred):
 deadline=time.perf_counter()+80
 while time.perf_counter()<deadline:
  s=state();assert s['phase']!='error',s.get('error')
  if pred(s):return s
  time.sleep(.01)
 raise RuntimeError('timeout')
try:
 s=wait(lambda s:s['phase']=='paused');startup=(time.perf_counter()-start)*1000
 get=[req('/api/observation')[1] for _ in range(8)]
 commands=[]
 for action,args in [('pause',{}),('pause',{}),('pause',{}),('step',{'steps':1}),('step',{'steps':3})]:
  before=state();ident=uuid.uuid4().hex;p={'session_id':before['session_id'],'request_id':ident,'action':action,'args':args};t=time.perf_counter();_,accepted=req('/api/command',p)
  target=before['plant']['steps']+args.get('steps',0)
  after=wait(lambda s:any(e['request_id']==ident for e in s['events']) and s['plant']['steps']==target and not s['single_steps_remaining'])
  e=next(e for e in after['events'] if e['request_id']==ident);assert e['status']=='applied',e
  commands.append({'action':action,'args':args,'post_accept_ms':accepted,'completion_observed_ms':(time.perf_counter()-t)*1000,'accepted_steps_before':before['plant']['steps'],'accepted_steps_after':after['plant']['steps']})
 out={'schema':'gtsd-phase6-HTTP-latency-small-sample-v1','ok':True,'variant':cfg['variant'],'date_utc':'2026-10-12','measurement':'perf_counter actual loopback HTTP; completion includes 10ms polling, owner scheduling, published snapshot and serialization','startup_to_paused_ms':startup,'get_observation_ms':get,'get_observation_summary_ms':{'min':min(get),'median':statistics.median(get),'max':max(get)},'commands':commands,'integrated_steps':4,'simulated_s':after['plant']['time_s'],'model_identity_sha256':after['plant']['model_identity_sha256'],'limits':['8 GET and 5 command samples; no p95 or sustained throughput estimate','Shared cloud environment with other tasks; not controlled benchmark','No browser/GPU/network-to-user or input-device latency measured','Numerical dt is not promised wall-clock real-time playback','No frozen source or deliverable files changed']}
 (ROOT/'phase6_http_latency.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps(out,ensure_ascii=False,indent=2))
finally:server.shutdown();server.server_close();thread.join();controller.close()
