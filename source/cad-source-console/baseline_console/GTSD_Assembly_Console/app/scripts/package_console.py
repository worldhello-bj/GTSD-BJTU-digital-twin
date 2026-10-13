"""Build one self-contained software/model ZIP, without copying asset trees."""
from pathlib import Path
import json,hashlib,zipfile
R=Path(__file__).resolve().parents[1];P=R.parent/'gtsd-strict82-portable-acceptance-1012';O=R.parent/'gtsd-console-assembly-release-1012';O.mkdir(exist_ok=True)
Z=O/'GTSD_CAD_Assembly_Strict82_Console_1012.zip';root='GTSD_Assembly_Console/';files={};data={}
for folder in ['web','data','tests','evidence','scripts']:
 for p in (R/folder).rglob('*'):
  if p.is_file() and not any(x in ('__pycache__',)for x in p.parts) and p.suffix not in ('.pyc',):files['app/'+str(p.relative_to(R))]=p
files['app/assembly_server.py']=R/'assembly_server.py';files['app/ACCEPTANCE.json']=R/'ACCEPTANCE.json';files['README_ZH.md']=R/'README_ZH.md'
config=json.loads((R/'console_config.json').read_text());config.update(strict82_root='../runtime/portable/GTSD_Phase6/console_strict82',strict82_config='../runtime/portable/GTSD_Phase6/console_strict82/strict82_config.portable.json',assembly_gzip='../assets/whole.glb.gz');data['app/console_config.json']=(json.dumps(config,indent=2)+'\n').encode()
manifest=json.loads((P/'portable/PORTABLE_MANIFEST.json').read_text())
for rel,row in manifest['files'].items():
 p=P/'portable'/rel;assert p.stat().st_size==row['bytes'] if 'bytes'in row else True
 assert hashlib.sha256(p.read_bytes()).hexdigest()==row['sha256'];files['runtime/portable/'+rel]=p
files['runtime/portable/PORTABLE_MANIFEST.json']=P/'portable/PORTABLE_MANIFEST.json'
for rel in ['portable_console.py','README_PORTABLE_ZH.md','ACCEPTANCE_SUMMARY.json','tests/check_portable_http.py']:
 files['runtime/'+rel]=P/rel
for name in ['portable_http_result.json','http_transcript.json','command_results.json','native_calls.jsonl','dom_result.json']:
 if (P/'evidence'/name).exists():files['runtime/evidence/'+name]=P/'evidence'/name
for p in (R.parent/'gtsd-console-assembly-audit-1012').glob('*'):
 if p.is_file() and p.suffix in ('.json','.md','.log'):files['independent_audit/'+p.name]=p
files['assets/whole.glb.gz']=R.parent/'gtsd-whole-clamp-integration-1012/outputs/GTSD_CAD_clamp_routes_review.glb.gz'
data['verify_bundle.py']=b'''from pathlib import Path\nimport json,hashlib\nr=Path(__file__).resolve().parent;m=json.loads((r/'BUNDLE_MANIFEST.json').read_text());n=0\nfor name,v in m['files'].items():\n p=r/name;assert p.is_file() and not p.is_symlink(),name\n h=hashlib.sha256()\n with p.open('rb')as f:\n  for b in iter(lambda:f.read(1048576),b''):h.update(b)\n assert p.stat().st_size==v['bytes'] and h.hexdigest()==v['sha256'],name\n n+=1\nprint(json.dumps({'ok':True,'files':n,'numerical_or_browser_execution':False}))\n'''
entries={}
with zipfile.ZipFile(Z,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6)as z:
 for rel,p in sorted(files.items()):
  assert p.is_file()and not p.is_symlink(),p
  content=p.read_bytes();entries[rel]={'bytes':len(content),'sha256':hashlib.sha256(content).hexdigest()};z.writestr(root+rel,content,compress_type=zipfile.ZIP_STORED if rel.endswith('.gz')else zipfile.ZIP_DEFLATED)
 for rel,content in sorted(data.items()):entries[rel]={'bytes':len(content),'sha256':hashlib.sha256(content).hexdigest()};z.writestr(root+rel,content)
 m={'schema':'gtsd-assembly-console-bundle-v1','files':entries,'frozen_runtime_files':len(manifest['files']),'new_collision_enabled':False,'browser_acceptance':False,'default_geometry_meshes':1933,'new_library_purpose':'self-contained virtual console with full CAD viewing data and pinned strict82 runtime'};z.writestr(root+'BUNDLE_MANIFEST.json',json.dumps(m,ensure_ascii=False,indent=2)+'\n')
with zipfile.ZipFile(Z)as z:
 assert z.testzip()is None
 for rel,row in entries.items():assert hashlib.sha256(z.read(root+rel)).hexdigest()==row['sha256']
out={'zip':str(Z),'bytes':Z.stat().st_size,'sha256':hashlib.sha256(Z.read_bytes()).hexdigest(),'members':len(entries)+1,'all_member_sha256_verified':True,'zip_crc_pass':True,'logical_bytes':sum(x['bytes']for x in entries.values())};(O/'DELIVERY.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
