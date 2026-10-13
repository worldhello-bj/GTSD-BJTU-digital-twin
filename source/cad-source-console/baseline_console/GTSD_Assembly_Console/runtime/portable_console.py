#!/usr/bin/env python3
"""Short explicit launcher for the unchanged portable strict82 console."""
from pathlib import Path
import argparse,hashlib,json,os,sys
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('action',choices=['doctor','serve','verify'])
p.add_argument('--port',type=int,default=None,help='Loopback port, 0 chooses an ephemeral port')
p.add_argument('--audit',action='store_true',help='Deny old source directory access after interpreter bootstrap')
a=p.parse_args()
manifest=json.loads((ROOT/'portable/PORTABLE_MANIFEST.json').read_text())
if a.audit:
 old_roots=tuple('/workspace/shared/'+name for name in manifest['source_root_map'])
 prefix=str(Path(sys.prefix).resolve()); local=str(ROOT.resolve())
 def audit(event,args):
  if event in ('open','os.listdir','os.scandir') and args and isinstance(args[0],(str,bytes)):
   name=str(Path(os.fsdecode(args[0])).resolve())
   if name==local or name.startswith(local+'/') or name==prefix or name.startswith(prefix+'/'):return
   if any(name==r or name.startswith(r+'/') for r in old_roots):raise PermissionError('Old source directory denied by portable audit: '+name)
 sys.addaudithook(audit)
sys.dont_write_bytecode=True
for name,row in manifest['files'].items():
 path=ROOT/'portable'/name
 if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:raise SystemExit('Portable file identity failed: '+name)
if a.action=='verify':
 print(json.dumps({'ok':True,'files':len(manifest['files']),'bytes':manifest['total_bytes'],'unchanged_frozen_files':True}));raise SystemExit(0)
console=ROOT/'portable/GTSD_Phase6/console_strict82'
sys.path.insert(0,str(console))
import strict82_server as S
c=S.load_config(console/'strict82_config.portable.json')
if a.port is not None:
 if not 0<=a.port<=65535:raise SystemExit('port must be 0..65535')
 c['base']['child']['base']['port']=a.port
r=S.doctor(c,check_bind=True)
if a.action=='doctor' or not r['ok']:
 print(json.dumps(r,ensure_ascii=False,indent=2));raise SystemExit(0 if r['ok'] else 2)
server,ctl,_=S.make_server(c);ctl.start()
print('GTSD explicit strict82 http://127.0.0.1:'+str(server.server_port),flush=True)
try:server.serve_forever(poll_interval=.1)
except KeyboardInterrupt:pass
finally:server.server_close();ctl.close()
