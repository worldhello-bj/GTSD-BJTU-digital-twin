import http.client
import json
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from operational_session import RigSession
from unittest.mock import patch
import operational_session as mod
from server import make_server,strict_json
from test_operational_session import FakePlant, PARAMETERS

class ServerTests(unittest.TestCase):
    def setUp(self):
        self.mockdoc=patch.object(mod,'parameter_document',return_value={'defaults':PARAMETERS});self.mockdoc.start()
        self.mockgeometry=patch.object(RigSession,'geometry',lambda s: {'schema':'gtsd-operational-rig-geometry-v1','server_generation_id':s.server_generation_id,'session_id':s.session_id,'units':'m','meshes':[]});self.mockgeometry.start()
        self.server,self.s=make_server(session=RigSession(FakePlant));self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();self.port=self.server.server_port
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join();self.mockdoc.stop();self.mockgeometry.stop()
    def request(self,path,body=None,headers=None,method=None):
        c=http.client.HTTPConnection('127.0.0.1',self.port,timeout=5);h={'Content-Type':'application/json'};h.update(headers or {})
        c.request(method or ('POST' if body is not None else 'GET'),path,body=json.dumps(body) if isinstance(body,dict) else body,headers=h);r=c.getresponse();data=r.read();result=(r.status,dict(r.getheaders()),data);c.close();return result
    def init(self):
        return {'mode':'operational_assembly','parameters':{},'server_generation_id':self.s.server_generation_id,'session_id':None,'expected_revision':0,'request_id':'initialize'}
    def test_generation_reads_and_stale_generation_rejection(self):
        generation = self.s.server_generation_id
        for route in ('status', 'metadata', 'geometry'):
            code, _, body = self.request('/api/operational-rig/' + route)
            self.assertEqual(code, 200)
            self.assertEqual(json.loads(body)['server_generation_id'], generation)
        payload = self.init()
        payload['server_generation_id'] = 'old-server-generation'
        self.assertEqual(self.request('/api/operational-rig/initialize', payload)[0], 409)
        self.assertIsNone(self.s.session_id)
        self.assertEqual(self.request('/api/operational-rig/initialize', self.init())[0], 200)
        before = self.s.snapshot()
        command = {'action': 'step', 'steps': 1, 'server_generation_id': 'old-server-generation',
                   'session_id': self.s.session_id, 'expected_revision': self.s.revision,
                   'request_id': 'stale-generation-step'}
        self.assertEqual(self.request('/api/operational-rig/command', command)[0], 409)
        self.assertEqual(before, self.s.snapshot())

    def test_fixed_reads_never_start_native_session(self):
        for path in ['/operational-rig','/operational-rig/core.js','/operational-rig/model.js','/api/operational-rig/status','/api/operational-rig/metadata','/api/operational-rig/status']:
            status,headers,_=self.request(path);self.assertEqual(status,200,path);self.assertEqual(headers['Cache-Control'],'no-store');self.assertIn("frame-ancestors 'none'",headers['Content-Security-Policy'])
        self.assertIsNone(self.s.session_id);self.assertEqual(self.s.revision,0)
    def test_fixed_allowlist_no_import_or_paths(self):
        for path in ['/../plant.py','/operational-rig/../plant.py','/api/operational-rig/checkpoint-export?id=../../etc/passwd','/api/operational-rig/status?step=1']:
            self.assertIn(self.request(path)[0],(400,404))
        for path in ['/api/operational-rig/import','/api/operational-rig/upload','/api/operational-rig/run','/api/operational-rig/command?path=/tmp/export']:
            self.assertEqual(self.request(path,{})[0],404)
        self.assertIsNone(self.s.session_id)
    def test_exact_host_and_origin(self):
        for h in [{'Host':'evil.test'},{'Host':'127.0.0.1:1'},{'Origin':'https://evil.test'},{'Origin':'null'},{'Origin':f'http://localhost:{self.port}'},{'Sec-Fetch-Site':'cross-site'}]:
            self.assertEqual(self.request('/api/operational-rig/initialize',self.init(),h)[0],403)
        self.assertEqual(self.request('/api/operational-rig/initialize',self.init(),{'Origin':f'http://127.0.0.1:{self.port}'})[0],200)
    def test_json_rejection_never_changes_state(self):
        for raw in ['{"a":1,"a":2}','{"steps":NaN}','[]','null','{broken']:
            self.assertEqual(self.request('/api/operational-rig/initialize',raw)[0],400)
        self.assertEqual(self.request('/api/operational-rig/initialize',self.init(),{'Content-Type':'text/plain'})[0],400)
        self.assertEqual(self.request('/api/operational-rig/initialize','x'*17000)[0],400)
        self.assertIsNone(self.s.session_id)
    def test_control_export_and_stale_http(self):
        code,_,body=self.request('/api/operational-rig/initialize',self.init());self.assertEqual(code,200);state=json.loads(body)
        p={'action':'checkpoint','server_generation_id':state['server_generation_id'],'session_id':state['session_id'],'expected_revision':state['revision'],'request_id':'cp'}
        code,_,body=self.request('/api/operational-rig/command',p);self.assertEqual(code,200);cp=json.loads(body)['created_checkpoint_id']
        code,h,body=self.request('/api/operational-rig/checkpoint-export?id='+cp+'&session_id='+state['session_id']);self.assertEqual(code,200);self.assertIn('attachment;',h['Content-Disposition']);self.assertTrue(json.loads(body)['export_only_no_import_route'])
        p['request_id']='stale';self.assertEqual(self.request('/api/operational-rig/command',p)[0],409)
        self.assertEqual(self.request('/api/operational-rig/checkpoint-export?id='+'a'*32+'&session_id='+state['session_id'])[0],400)
    def test_duplicate_headers_rejected(self):
        for header,values in [('Host',[f'127.0.0.1:{self.port}',f'127.0.0.1:{self.port}']),('Content-Length',['2','2']),('Origin',[f'http://127.0.0.1:{self.port}',f'http://127.0.0.1:{self.port}'])]:
            c=http.client.HTTPConnection('127.0.0.1',self.port);c.putrequest('POST','/api/operational-rig/initialize',skip_host=True);c.putheader('Content-Type','application/json')
            if header!='Host':c.putheader('Host',f'127.0.0.1:{self.port}')
            if header!='Content-Length':c.putheader('Content-Length','2')
            for v in values:c.putheader(header,v)
            c.endheaders(b'{}');r=c.getresponse();r.read();self.assertIn(r.status,(400,403));c.close()


if __name__ == '__main__': unittest.main()
