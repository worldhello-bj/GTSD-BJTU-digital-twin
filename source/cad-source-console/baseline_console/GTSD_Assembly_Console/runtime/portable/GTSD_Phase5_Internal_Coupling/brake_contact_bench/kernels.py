"""Locally pinned B8 finite-gas and B9 force-driven balanced spool kernels."""
from pathlib import Path
import hashlib, importlib.util, sys
ROOT=Path(__file__).resolve().parent
VENDOR_HASHES={
 'vendor/b8/pneumatics.py':'a96d81f70e347319ad055c0fea4aa06933f762e01910522377c81a16dc6a1915',
 'vendor/b9/valve_dynamics.py':'acd3d855c74db93793022f4f39c6acec821c1a51f168b23d4c56f0f3b4908a50'}
def verify_vendor():
    for name,expected in VENDOR_HASHES.items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Pinned brake kernel changed: '+name)
def _load(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module);return module
verify_vendor()
_saved_path=list(sys.path);_saved=sys.modules.get('pneumatics')
try:
    b8=_load('_gtsd_contact_brake_b8','vendor/b8/pneumatics.py')
    sys.modules['pneumatics']=b8
    b9=_load('_gtsd_contact_brake_b9','vendor/b9/valve_dynamics.py')
finally:
    sys.path[:]=_saved_path
    if _saved is None:sys.modules.pop('pneumatics',None)
    else:sys.modules['pneumatics']=_saved
