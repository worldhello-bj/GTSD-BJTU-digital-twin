"""Tiny fake-controller route tests; no numerical model or hardware is constructed."""
from pathlib import Path
import unittest,sys,types,json,tempfile,threading,gzip
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from http.client import HTTPConnection
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import assembly_server as A
class Routes(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();p=Path(self.temp.name);(p/'g.gz').write_bytes(gzip.compress(b'GLB-test-only'));(p/'e.gz').write_bytes(gzip.compress(b'{"meshes":[]}'));self.ctl=types.SimpleNamespace(reads=[],state={'phase':'paused','session_id':'same','snapshot_age_s':0,'plant':{'model_identity_sha256':'model','source_geometry_sha256':'source','variant':'old','time_s':0,'steps':0,'body_poses':{'world':{'position':[0,0,0],'quaternion':[1,0,0,0]}}}})
  def snapshot(*,heartbeat=True):self.ctl.reads.append(heartbeat);return self.ctl.state
  self.ctl.snapshot=snapshot
  class Base(BaseHTTPRequestHandler):
   def _authorized(self):return True
   def _send(self,code,data,content_type='application/json; charset=utf-8'):
    if not isinstance(data,bytes):data=json.dumps(data).encode()
    self.send_response(code);self.send_header('Content-Type',content_type);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
   def do_GET(self):return self._send(200,b'<html><header>old</header>old controller</html>','text/html')
   def log_message(self,*args):pass
  self.old=sys.modules.get('strict82_server');self.stub=types.SimpleNamespace(make_server=lambda c:(ThreadingHTTPServer(('127.0.0.1',0),Base),self.ctl,{}));sys.modules['strict82_server']=self.stub
  self.cfg={'strict_config':{},'catalogue':{'default_scene':'one'},'mapping':{'schema':'test'},'assembly_gzip':str(p/'g.gz'),'extras':str(p/'e.gz')};self.server,_,_=A.make_server(self.cfg);self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
 def tearDown(self):
  self.server.shutdown();self.server.server_close();self.thread.join();self.temp.cleanup()
  if self.old is None:sys.modules.pop('strict82_server',None)
  else:sys.modules['strict82_server']=self.old
 def get(self,path):
  h=HTTPConnection('127.0.0.1',self.server.server_port,timeout=5);h.request('GET',path);r=h.getresponse();body=r.read();headers=dict(r.getheaders());code=r.status;h.close();return code,headers,body
 def test_observation_no_lease_or_state_mutation(self):
  before=json.dumps(self.ctl.state,sort_keys=True);code,_,body=self.get('/api/assembly/runtime');x=json.loads(body);self.assertEqual(code,200);self.assertEqual(self.ctl.reads,[False]);self.assertFalse(x['renews_run_lease']);self.assertFalse(x['assembly_geometry_used_for_solver_collision']);self.assertEqual(json.dumps(self.ctl.state,sort_keys=True),before)
 def test_assets_gzip_fixed_paths(self):
  code,heads,body=self.get('/assembly/model.glb');self.assertEqual(code,200);self.assertEqual(heads['Content-Encoding'],'gzip');self.assertEqual(gzip.decompress(body),b'GLB-test-only');self.assertEqual(heads['Cache-Control'],'no-store')
 def test_catalog_mapping_no_solver_read(self):
  self.assertEqual(json.loads(self.get('/api/assembly/catalog')[2]),self.cfg['catalogue']);self.assertEqual(json.loads(self.get('/api/assembly/mapping')[2]),self.cfg['mapping']);self.assertEqual(self.ctl.reads,[])
 def test_unknown_paths_fail_closed(self):
  for p in ['/assembly/../console_config.json','/assembly/../../etc/passwd','/api/assembly/command','/assembly/no-such-file']:
   self.assertEqual(self.get(p)[0],404)
 def test_old_page_and_handler_preserved(self):
  body=self.get('/')[2];self.assertIn(b'old controller',body);self.assertIn(b'href="/assembly"',body);self.assertIn(b'old controller',self.get('/old-route')[2])
 def test_page_is_explicit_candidate_and_no_import(self):
  code,_,b=self.get('/assembly');self.assertEqual(code,200);s=b.decode();self.assertIn('候选',s);self.assertIn('碰撞未启用',s);self.assertNotIn('type="file"',s);self.assertNotIn('new-run',s)
if __name__=='__main__':unittest.main(verbosity=2)
