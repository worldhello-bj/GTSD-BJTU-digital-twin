from pathlib import Path
import json,hashlib
r=Path(__file__).resolve().parent;m=json.loads((r/'BUNDLE_MANIFEST.json').read_text());n=0
for name,v in m['files'].items():
 p=r/name;assert p.is_file() and not p.is_symlink(),name
 h=hashlib.sha256()
 with p.open('rb')as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 assert p.stat().st_size==v['bytes'] and h.hexdigest()==v['sha256'],name
 n+=1
print(json.dumps({'ok':True,'files':n,'numerical_or_browser_execution':False}))
