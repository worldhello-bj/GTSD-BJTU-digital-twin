"""Hash-pinned numerical kernels; no legacy mechanical factory is invoked."""
from pathlib import Path
import hashlib
import importlib.util
import sys

ROOT = Path(__file__).resolve().parent
VENDOR_HASHES = {
    'vendor/b8/pneumatics.py': 'a96d81f70e347319ad055c0fea4aa06933f762e01910522377c81a16dc6a1915',
    'vendor/b9/valve_dynamics.py': 'acd3d855c74db93793022f4f39c6acec821c1a51f168b23d4c56f0f3b4908a50',
    'vendor/b10/auxiliaries.py': '03f55f0238a191fed0e8289183248547d51497d56b20c50b2905d143de1e65e4',
}

def verify_vendor():
    for path, expected in VENDOR_HASHES.items():
        if hashlib.sha256((ROOT/path).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Pinned pneumatic auxiliary kernel changed: '+path)

def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

verify_vendor()
# Legacy relative imports are bound only while loading, without replacing the
# integrated plant's separately pinned modules or leaving sys.path mutations.
_saved_path = list(sys.path)
_saved_modules = {key: sys.modules.get(key) for key in ('pneumatics', 'valve_dynamics')}
try:
    b8 = _load('_gtsd_pneumatic_aux_b8', 'vendor/b8/pneumatics.py')
    sys.modules['pneumatics'] = b8
    b9 = _load('_gtsd_pneumatic_aux_b9', 'vendor/b9/valve_dynamics.py')
    sys.modules['valve_dynamics'] = b9
    b10 = _load('_gtsd_pneumatic_aux_b10', 'vendor/b10/auxiliaries.py')
finally:
    sys.path[:] = _saved_path
    for key, value in _saved_modules.items():
        if value is None:
            sys.modules.pop(key, None)
        else:
            sys.modules[key] = value
