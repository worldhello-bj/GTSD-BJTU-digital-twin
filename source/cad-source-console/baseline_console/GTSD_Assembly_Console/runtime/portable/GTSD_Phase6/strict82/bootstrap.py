"""Read-only pinned dependencies for strict82 and scoped rollback composition."""
import hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
BASE=Path('/workspace/shared/gtsd-physics-cad-doors-1012')
TRUSTED=Path('/workspace/shared/gtsd-coupling-performance-1012')
BASE_FREEZE_SHA='b084492e62453f8bd8fdf8bec25c82efd547fc8cd28095aaa0ed1baf30811526'
TRUSTED_SHA='e368f4f19af2e645d1f746036cec40c83c66c1ba1c2a1bad9008124df0cb2c9f'
def sha_file(p):return hashlib.sha256(p.read_bytes()).hexdigest()
if sha_file(BASE/'FAN82_SOURCE_FREEZE.json')!=BASE_FREEZE_SHA:raise RuntimeError('Frozen82 manifest mismatch')
for n,h in json.loads((BASE/'FAN82_SOURCE_FREEZE.json').read_text())['physics_source_sha256'].items():
 if sha_file(BASE/n)!=h:raise RuntimeError('Frozen82 source mismatch '+n)
if sha_file(TRUSTED/'trusted_rollback.py')!=TRUSTED_SHA:raise RuntimeError('Trusted rollback source mismatch')
for label,path in [('fan82',BASE/'fan82/__init__.py'),('trusted_rollback',TRUSTED/'trusted_rollback.py')]:
 module=sys.modules.get(label)
 if module and Path(module.__file__).resolve()!=path.resolve():raise RuntimeError('Conflicting imported dependency '+label)
for path in (BASE,TRUSTED):
 if str(path) not in sys.path:sys.path.insert(0,str(path))
