"""Candidate routes attach to an existing authorized assembly HTTP handler.
Standalone loopback mode reuses baseline assets but never imports strict82.
"""
from pathlib import Path
import argparse,json,sys
from urllib.parse import urlsplit,parse_qs
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from candidate_session import CandidateSession,CandidateError,CandidateConflict,MODE
ROOT=Path(__file__).resolve().parent
BASE=ROOT.parent/'baseline_console/GTSD_Assembly_Console'
PREFIX='/api/pantograph-candidate/'
WEB={'/pantograph-candidate':('index.html','text/html; charset=utf-8'),'/pantograph-candidate/':('index.html','text/html; charset=utf-8'),'/pantograph-candidate/app.js':('app.js','text/javascript; charset=utf-8'),'/pantograph-candidate/core.js':('core.js','text/javascript; charset=utf-8'),'/pantograph-candidate/style.css':('style.css','text/css; charset=utf-8')}

def strict_json(raw):
    def pairs(items):
        out={}
        for key,value in items:
            if key in out:raise CandidateError('Duplicate JSON key rejected: '+key[:80])
            out[key]=value
        return out
    def constant(value):raise CandidateError('Nonfinite JSON value rejected.')
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=constant)

def install_candidate_routes(server,session=None):
    """Decorate RequestHandlerClass. Its existing _authorized/_send/file are reused."""
    session=session or CandidateSession();parent=server.RequestHandlerClass
    class CandidateHandler(parent):
        def do_GET(self):
            parts=urlsplit(self.path);path=parts.path
            if path not in WEB and not path.startswith(PREFIX):return super().do_GET()
            if not self._authorized():return
            try:
                if path in WEB:
                    name,mime=WEB[path];return self.file(ROOT/'web'/name,mime)
                if path==PREFIX+'metadata':return self._send(200,session.metadata())
                if path==PREFIX+'status':return self._send(200,session.snapshot())
                if path==PREFIX+'checkpoint-export':
                    q=parse_qs(parts.query)
                    if set(q)!={'id'} or len(q['id'])!=1:raise CandidateError('One existing checkpoint ID is required.')
                    return self._send(200,session.export_checkpoint(q['id'][0]))
                return self._send(404,{'error':'Unknown fixed candidate route.'})
            except CandidateError as e:return self._send(400,{'error':str(e)})
        def do_POST(self):
            path=urlsplit(self.path).path
            if not path.startswith(PREFIX):return super().do_POST()
            if not self._authorized():return
            if path not in [PREFIX+'initialize',PREFIX+'command']:
                self.close_connection=True
                return self._send(404,{'error':'No candidate run, upload or import route exists.'})
            try:
                if len(self.headers.get_all('Content-Length',[]))!=1:raise CandidateError('Exactly one Content-Length header is required.')
                if self.headers.get('Transfer-Encoding'):raise CandidateError('Chunked candidate commands are not accepted.')
                if self.headers.get('Content-Type','').split(';')[0].strip()!='application/json':raise CandidateError('JSON commands only.')
                origin=self.headers.get('Origin')
                if origin and urlsplit(origin).netloc!=self.headers.get('Host'):raise CandidateError('Cross-origin candidate command rejected.')
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=32768:raise CandidateError('Candidate command size must be 1–32768 bytes.')
                old_timeout=self.connection.gettimeout()
                try:
                    self.connection.settimeout(3.0);raw=self.rfile.read(size)
                finally:self.connection.settimeout(old_timeout)
                if len(raw)!=size:raise CandidateError('Truncated command body.')
                payload=strict_json(raw)
                if path==PREFIX+'initialize':
                    if not isinstance(payload,dict) or set(payload)!={'mode'}:raise CandidateError('Only explicit mode may initialize.')
                    return self._send(200,session.initialize(payload['mode']))
                if path==PREFIX+'command':return self._send(200,session.command(payload))
                return self._send(404,{'error':'No candidate run, upload or import route exists.'})
            except (TimeoutError,OSError):
                self.close_connection=True
                return self._send(408,{'error':'Candidate command body read timed out or failed; no action taken.'})
            except CandidateConflict as e:return self._send(409,{'error':str(e)})
            except (CandidateError,ValueError,TypeError) as e:return self._send(400,{'error':str(e)})
    server.RequestHandlerClass=CandidateHandler
    return session

class LoopbackHandler(BaseHTTPRequestHandler):
    def _authorized(self):
        host=self.headers.get('Host','').split(':')[0]
        if self.client_address[0]!='127.0.0.1' or host not in ['127.0.0.1','localhost']:
            self._send(403,{'error':'Loopback research server only.'});return False
        return True
    def _headers(self,code,mime,size,encoding=None):
        self.send_response(code);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(size));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Content-Security-Policy',"default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'; form-action 'none'")
        if encoding:self.send_header('Content-Encoding',encoding)
        self.end_headers()
    def _send(self,code,data,content_type='application/json; charset=utf-8'):
        body=json.dumps(data,ensure_ascii=False,allow_nan=False).encode() if not isinstance(data,bytes) else data;self._headers(code,content_type,len(body));self.wfile.write(body)
    def file(self,path,mime,encoding=None):
        self._headers(200,mime,path.stat().st_size,encoding)
        try:
            with path.open('rb') as f:
                for b in iter(lambda:f.read(65536),b''):self.wfile.write(b)
        except (BrokenPipeError,ConnectionResetError):pass
    def do_GET(self):
        if not self._authorized():return
        path=urlsplit(self.path).path
        if path=='/api/assembly/catalog':return self._send(200,json.loads((BASE/'app/data/catalog.json').read_text()))
        if path=='/assembly/model.glb':return self.file(BASE/'assets/whole.glb.gz','model/gltf-binary','gzip')
        if path in ['/assembly/core.js','/assembly/model.js','/assembly/lineage.js']:return self.file(BASE/'app/web'/path.rsplit('/',1)[1],'text/javascript; charset=utf-8')
        return self._send(404,{'error':'Unknown route.'})
    def do_POST(self):return self._send(404,{'error':'Unknown route.'})
    def log_message(self,*args):pass

def make_standalone(port=0):
    server=ThreadingHTTPServer(('127.0.0.1',port),LoopbackHandler);session=install_candidate_routes(server);return server,session
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--port',type=int,default=8174);args=ap.parse_args();server,session=make_standalone(args.port);print(f'http://127.0.0.1:{server.server_port}/pantograph-candidate (disabled until explicit H1 click)',flush=True)
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:server.server_close()
