"""All dependencies are read-only and byte-hash verified before import."""
import hashlib,json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
MANIFEST=json.loads((ROOT/'READONLY_DEPENDENCIES.json').read_text())
PATHS={}
for label,row in MANIFEST['dependencies'].items():
    root=Path(os.environ.get('GTSD_CAD_'+label.upper()+'_ROOT',ROOT.parent/row['default_sibling'])).resolve()
    for name,expected in row['source_sha256'].items():
        file=root/name
        if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('CAD-body readonly dependency mismatch: '+label+'/'+name)
    PATHS[label]=root
for module_name,label in [('aux_mount','phase6'),('brake_integrated','phase5')]:
    loaded=sys.modules.get(module_name)
    if loaded is not None and getattr(loaded,'__file__',None) and not Path(loaded.__file__).resolve().is_relative_to(PATHS[label]):
        raise RuntimeError('Conflicting numerical module: '+module_name)
if str(PATHS['phase6']) not in sys.path:sys.path.insert(0,str(PATHS['phase6']))
# Phase6's own bootstrap independently verifies its complete Phase5 numerical tree.
os.environ.setdefault('GTSD_PHASE5_ROOT',str(PATHS['phase5']))
