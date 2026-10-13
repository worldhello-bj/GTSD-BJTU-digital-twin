"""Selected variant starts paused; no commands and no numerical integration."""
from fixture import P,CONFIG
import json,time,threading,copy
from http.client import HTTPConnection
cfg=copy.deepcopy(CONFIG);cfg['base']['port']=0
server,controller,context=P.make_server(cfg);thread=threading.Thread(target=server.serve_forever,daemon=True);controller.start();thread.start()
def get(path):
 c=HTTPConnection('127.0.0.1',server.server_port,timeout=60);c.request('GET',path);r=c.getresponse();b=r.read();c.close();assert r.status==200,(r.status,b[:300]);return json.loads(b)
try:
 deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  state=get('/api/observation');assert state['phase']!='error',state.get('error')
  if state['phase']=='paused':break
  time.sleep(.05)
 else:raise AssertionError('startup timeout')
 observation=get('/api/lineage-observation');catalog=get('/api/lineage-registry');assert observation['identity_match'],observation
 assert state['plant']['steps']==0 and state['plant']['time_s']==0 and not state['client_lease_armed']
 print(json.dumps({'ok':True,'requested_variant':cfg['variant'],'actual_variant':state['plant']['variant'],'native_dof':state['plant']['capabilities']['integrated_dof'],'paused':True,'accepted_steps':0,'lineage_current_identity_match':True,'catalogue_denominators':catalog['denominators']},indent=2))
finally:server.shutdown();server.server_close();thread.join();controller.close()
