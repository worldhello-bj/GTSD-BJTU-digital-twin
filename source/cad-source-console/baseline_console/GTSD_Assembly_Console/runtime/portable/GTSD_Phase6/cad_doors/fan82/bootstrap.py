"""Pinned frozen 79 dependency, independently verified for optional 82."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
FROZEN79_SHA256='4a9d61522b2bd05293be8436a79bce1aa62e2ba2070437e466d73d0c76e42046'
p=ROOT/'SOURCE_FREEZE.json'
if hashlib.sha256(p.read_bytes()).hexdigest()!=FROZEN79_SHA256:raise RuntimeError('Frozen 79 manifest identity changed')
for n,h in json.loads(p.read_text())['physics_source_sha256'].items():
 if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h:raise RuntimeError('Frozen 79 dependency changed: '+n)
