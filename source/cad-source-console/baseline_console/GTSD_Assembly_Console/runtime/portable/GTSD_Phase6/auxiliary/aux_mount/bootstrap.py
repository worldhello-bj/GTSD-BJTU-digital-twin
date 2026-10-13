"""Portable, hash-verified readonly Phase5 dependency resolution.

Set GTSD_PHASE5_ROOT for an explicit installation. Otherwise Phase6 may be a
sibling directory or its aux_mount overlay may be installed inside Phase5.
No baseline files are written and an unrecognised dependency is rejected.
"""
from pathlib import Path
import hashlib,json,os,sys
ROOT=Path(__file__).resolve().parent.parent
manifest_path=ROOT/'PHASE6_READONLY_BASELINE.json'
if not manifest_path.is_file():raise RuntimeError('Missing Phase6 readonly baseline hash manifest')
_manifest=json.loads(manifest_path.read_text())
explicit=os.environ.get('GTSD_PHASE5_ROOT')
_candidates=[Path(explicit).expanduser()] if explicit else [ROOT.parent/'gtsd-physics-phase5-1012',ROOT,ROOT.parent]
BASELINE=next((p.resolve() for p in _candidates if (p/'brake_integrated/plant.py').is_file() and (p/'support_integrated/combined.py').is_file()),None)
if BASELINE is None:raise RuntimeError('Verified Phase5 dependency not found; install adjacent or set GTSD_PHASE5_ROOT')
BASELINE_HASHES=dict(_manifest['source_sha256'])
for relative,expected in BASELINE_HASHES.items():
    path=BASELINE/relative
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise RuntimeError('Unrecognised or modified Phase5 numerical dependency: '+relative)
for module_name in ('brake_integrated','support_integrated','pneumatic_aux','suspension_3d','doors','cables'):
    loaded=sys.modules.get(module_name)
    if loaded is not None and getattr(loaded,'__file__',None) and not Path(loaded.__file__).resolve().is_relative_to(BASELINE):
        raise RuntimeError('Conflicting already-loaded Phase5 module: '+module_name)
if str(BASELINE) not in sys.path:sys.path.insert(0,str(BASELINE))
BASELINE_IDENTITY_SHA256=hashlib.sha256(json.dumps(BASELINE_HASHES,sort_keys=True,separators=(',',':')).encode()).hexdigest()
