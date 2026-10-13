from pathlib import Path
import os,sys,json,time,threading
from http.client import HTTPConnection
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import fan82_server as F
c=F.load_config(os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'console_config.json');assert c['variant']=='internal-coupled-research';c['child']['base']['port']=0
server,ctl,_=F.make_server(c);thread=threading.Thread(target=server.serve_forever,daemon=True);ctl.start();thread.start()
try:
 deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  h=HTTPConnection('127.0.0.1',server.server_port,timeout=30);h.request('GET','/api/observation');r=h.getresponse();s=json.loads(r.read());h.close()
  if s['phase']=='error':raise AssertionError(s['error'])
  if s['phase']=='paused':break
  time.sleep(.1)
 else:raise AssertionError('Default startup timeout')
 assert s['plant']['steps']==0 and len(s['plant']['qvel'])==75 and s['plant']['variant']!='cad_doors_four_fans';print(json.dumps({'ok':True,'default_variant':s['plant']['variant'],'nv':len(s['plant']['qvel']),'steps':0,'actual_HTTP_paused_startup':True}))
finally:server.shutdown();server.server_close();thread.join();ctl.close()
