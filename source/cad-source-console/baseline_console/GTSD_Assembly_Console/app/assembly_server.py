#!/usr/bin/env python3
"""Read-only CAD assembly view on the same strict82 virtual console. Frozen solver untouched."""
from pathlib import Path
import json,hashlib,argparse,sys,importlib,os
from urllib.parse import urlsplit
ROOT=Path(__file__).resolve().parent

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def load_config(path=None):
 p=Path(path or ROOT/'console_config.json').resolve();v=json.loads(p.read_text())
 if set(v)!={'schema','strict82_root','strict82_config','strict82_server_sha256','assembly_gzip','catalog','catalog_sha256','extras','mapping_path','mapping_sha256','port'}or v['schema']!='gtsd-assembly-console-config-v1':raise ValueError('Unknown assembly configuration fields')
 if type(v['port'])is not int or not 0<=v['port']<=65535:raise ValueError('Invalid loopback port')
 resolve=lambda s:(p.parent/s).resolve()
 for k in ['strict82_root','strict82_config','assembly_gzip','catalog','extras']:v[k]=str(resolve(v[k]))
 if v['mapping_path']is not None:v['mapping_path']=str(resolve(v['mapping_path']))
 if sha(Path(v['strict82_root'])/'strict82_server.py')!=v['strict82_server_sha256']:raise ValueError('Frozen strict82 console source changed')
 if sha(v['catalog'])!=v['catalog_sha256']:raise ValueError('Unknown assembly catalog')
 catalog=json.loads(Path(v['catalog']).read_text())
 if catalog['schema']!='gtsd-whole-assembly-console-catalog-v1':raise ValueError('Assembly catalog schema')
 if sha(v['assembly_gzip'])!=catalog['base_glb']['gzip_sha256']or sha(v['extras'])!=catalog['extras']['gzip_sha256']:raise ValueError('Assembly geometry resource hash mismatch')
 mapping=None
 if v['mapping_path']is not None:
  if sha(v['mapping_path'])!=v['mapping_sha256']:raise ValueError('Unknown per-mesh mapping')
  mapping=json.loads(Path(v['mapping_path']).read_text())
  if mapping.get('schema')!='gtsd-assembly-console-mapping-v1':raise ValueError('Mapping bundle schema')
  latest=mapping['contract']['latest_assembly_identity']
  names=sorted(x['name']for x in catalog['meshes']);name_hash=hashlib.sha256(json.dumps(names,ensure_ascii=False,separators=(',',':')).encode()).hexdigest()
  if latest['native_sha256']!=catalog['native_sha256']or latest['default_glb_uncompressed_sha256']!=catalog['base_glb']['sha256']or latest['five_scene_union_mesh_names_sha256']!=name_hash or latest['default_scene']!=catalog['default_scene']:raise ValueError('Visual mapping/assembly identity mismatch')
  if sorted(x['name']for x in mapping['meshes'])!=names or any(x['new_collision_allowed']for x in mapping['meshes']):raise ValueError('Unknown mapping rows or unauthorized contact')
 sys.dont_write_bytecode=True;sys.path.insert(0,v['strict82_root']);S=importlib.import_module('strict82_server')
 if Path(S.__file__).resolve()!=Path(v['strict82_root'])/'strict82_server.py':raise ValueError('Strict82 module conflict')
 cfg=S.load_config(v['strict82_config']);cfg['base']['child']['base']['port']=v['port'];v.update(strict_config=cfg,catalogue=catalog,mapping=mapping);return v

def make_server(config):
 S=importlib.import_module('strict82_server');server,ctl,context=S.make_server(config['strict_config']);parent=server.RequestHandlerClass
 class Handler(parent):
  def _send(self,code,data,content_type='application/json; charset=utf-8'):
   if urlsplit(self.path).path=='/'and code==200 and isinstance(data,bytes):
    link=b'<nav style="padding:12px"><a href="/assembly">Open latest CAD assembly / candidate and solver mapping review</a> <span>Static CAD and accepted solver identities are separate.</span></nav>'
    data=data.replace(b'</header>',b'</header>'+link,1)
   return super()._send(code,data,content_type)
  def file(self,path,mime,encoding=None):
   self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(path.stat().st_size));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Content-Security-Policy',"default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'; form-action 'none'")
   if encoding:self.send_header('Content-Encoding',encoding)
   self.end_headers()
   try:
    with path.open('rb')as f:
     for b in iter(lambda:f.read(65536),b''):self.wfile.write(b)
   except (BrokenPipeError,ConnectionResetError):pass
  def do_GET(self):
   path=urlsplit(self.path).path
   if not(path=='/assembly'or path.startswith('/assembly/')or path.startswith('/api/assembly/')):return super().do_GET()
   if not self._authorized():return
   routes={'/assembly':('index.html','text/html; charset=utf-8'),'/assembly/':('index.html','text/html; charset=utf-8'),'/assembly/app.js':('app.js','text/javascript; charset=utf-8'),'/assembly/core.js':('core.js','text/javascript; charset=utf-8'),'/assembly/model.js':('model.js','text/javascript; charset=utf-8'),'/assembly/lineage.js':('lineage.js','text/javascript; charset=utf-8'),'/assembly/style.css':('style.css','text/css; charset=utf-8')}
   if path in routes:name,mime=routes[path];return self.file(ROOT/'web'/name,mime)
   if path=='/assembly/model.glb':return self.file(Path(config['assembly_gzip']),'model/gltf-binary','gzip')
   if path=='/assembly/extras.json':return self.file(Path(config['extras']),'application/json; charset=utf-8','gzip')
   if path=='/api/assembly/catalog':return self._send(200,config['catalogue'])
   if path=='/api/assembly/mapping':
    return self._send(200,config['mapping']or{'schema':'gtsd-assembly-map-pending-v1','ready':False,'rows':[],'dynamic_mapping_allowed':False,'note':'Per-mesh geometry/pose admission not yet complete. All new assembly remains static.'})
   if path=='/api/assembly/runtime':
    state=ctl.snapshot(heartbeat=False);p=state.get('plant')or{}
    value={'schema':'gtsd-assembly-runtime-v1','phase':state.get('phase'),'session_id':state.get('session_id'),'snapshot_age_s':state.get('snapshot_age_s'),'renews_run_lease':False,'model_identity_sha256':p.get('model_identity_sha256'),'source_geometry_sha256':p.get('source_geometry_sha256'),'variant':p.get('variant'),'time_s':p.get('time_s'),'steps':p.get('steps'),'body_poses':p.get('body_poses',{}),'simulation_speed':state.get('performance'),'events':state.get('events',[]),'error':state.get('error'),'single_steps_remaining':state.get('single_steps_remaining',0),'assembly_geometry_used_for_solver_collision':False,'hardware_connected':False}
    return self._send(200,value)
   return self._send(404,{'error':'Unknown fixed assembly route'})
 server.RequestHandlerClass=Handler;return server,ctl,context

def main():
 ap=argparse.ArgumentParser();ap.add_argument('action',choices=['doctor','serve']);ap.add_argument('--config');args=ap.parse_args();c=load_config(args.config);S=importlib.import_module('strict82_server')
 if args.action=='doctor':
  r=S.doctor(c['strict_config'],check_bind=False);r['assembly']={'default_scene':c['catalogue']['default_scene'],'meshes':len(c['catalogue']['scenes'][c['catalogue']['default_scene']]['mesh_names']),'mapping_loaded':c['mapping']is not None,'new_contact_enabled':False};print(json.dumps(r,ensure_ascii=False,indent=2));return 0 if r['ok']else 2
 preflight=S.doctor(c['strict_config'],check_bind=False)
 if not preflight['ok']:
  print(json.dumps(preflight,ensure_ascii=False,indent=2));return 2
 server,ctl,_=make_server(c);ctl.start();print('GTSD CAD assembly + frozen strict82 http://127.0.0.1:'+str(server.server_port)+'/assembly',flush=True)
 try:server.serve_forever(poll_interval=.1)
 except KeyboardInterrupt:pass
 finally:server.server_close();ctl.close()
 return 0
if __name__=='__main__':raise SystemExit(main())
