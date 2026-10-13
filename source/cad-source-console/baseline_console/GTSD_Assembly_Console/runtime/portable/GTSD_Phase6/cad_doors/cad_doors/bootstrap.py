"""Explicit byte-verified readonly dependencies; no mutable default substitution."""
import json,hashlib,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MANIFEST=json.loads((ROOT/'READONLY_DEPENDENCIES.json').read_text());PATHS={}
for label,row in MANIFEST['dependencies'].items():
 p=Path(os.environ.get('GTSD_DOOR_'+label.upper()+'_ROOT',ROOT.parent/row['default_sibling'])).resolve()
 for name,h in row['source_sha256'].items():
  f=p/name
  if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=h:raise RuntimeError('Door readonly dependency mismatch: '+label+'/'+name)
 PATHS[label]=p
loaded=sys.modules.get('cad_body')
if loaded is not None and getattr(loaded,'__file__',None) and not Path(loaded.__file__).resolve().is_relative_to(PATHS['body']):raise RuntimeError('Conflicting CAD-body module')
if str(PATHS['body']) not in sys.path:sys.path.insert(0,str(PATHS['body']))
