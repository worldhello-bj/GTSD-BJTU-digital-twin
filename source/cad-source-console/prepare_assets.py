"""Verify and atomically restore bundled assets. Standard library, no downloads.

All chunk paths and restored paths stay inside this directory. A modified existing
asset is rejected, never overwritten. Missing assets are restored automatically.
"""
from pathlib import Path, PurePosixPath
import argparse
import gzip
import hashlib
import io
import json
import os
import tempfile

ROOT = Path(__file__).resolve().parent
MAX_CHUNK_BYTES = 4 * 1024 * 1024

def sha_file(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def safe_path(root, name):
    rel = PurePosixPath(name)
    if rel.is_absolute() or not rel.parts or '..' in rel.parts or '\\' in name:
        raise ValueError('Unsafe asset path: ' + name)
    path = root.joinpath(*rel.parts)
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('Asset path escapes release: ' + name)
    return path

def ensure_assets(root=ROOT, check_only=False):
    root = Path(root).resolve()
    manifest = json.loads((root / 'ASSET_MANIFEST.json').read_text(encoding='utf-8'))
    if manifest['schema'] != 'gtsd-public-chunked-assets-v1':
        raise ValueError('Unsupported asset manifest')
    restored = []
    for asset in manifest['assets']:
        target = safe_path(root, asset['path'])
        packed = bytearray()
        for chunk in asset['chunks']:
            path = safe_path(root, chunk['path'])
            data = path.read_bytes()
            if len(data) != chunk['bytes'] or len(data) > MAX_CHUNK_BYTES or hashlib.sha256(data).hexdigest() != chunk['sha256']:
                raise ValueError('Chunk identity mismatch: ' + chunk['path'])
            packed.extend(data)
        if len(packed) != asset['packed_bytes'] or hashlib.sha256(packed).hexdigest() != asset['packed_sha256']:
            raise ValueError('Whole payload identity mismatch: ' + asset['path'])
        if target.exists():
            if not target.is_file() or target.stat().st_size != asset['bytes'] or sha_file(target) != asset['sha256']:
                raise ValueError('Existing asset differs; preserve your work and restore a clean copy: ' + asset['path'])
            continue
        if check_only:
            raise FileNotFoundError('Asset not restored: ' + asset['path'])
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.gtsd-restore-', delete=False) as out:
                temporary = Path(out.name)
                source = io.BytesIO(packed)
                if asset['encoding'] == 'gzip':
                    source = gzip.GzipFile(fileobj=source, mode='rb')
                elif asset['encoding'] != 'identity':
                    raise ValueError('Unsupported encoding')
                total = 0
                with source:
                    while True:
                        block = source.read(min(1024 * 1024, asset['bytes'] + 1 - total))
                        if not block:
                            break
                        total += len(block)
                        if total > asset['bytes']:
                            raise ValueError('Oversized decoded asset: ' + asset['path'])
                        out.write(block)
                out.flush()
                os.fsync(out.fileno())
            if temporary.stat().st_size != asset['bytes'] or sha_file(temporary) != asset['sha256']:
                raise ValueError('Restored identity mismatch: ' + asset['path'])
            os.replace(temporary, target)
            restored.append(asset['path'])
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
    return {'ok': True, 'assets_checked': len(manifest['assets']), 'assets_restored': len(restored), 'network_used': False}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-only', action='store_true', help='Verify without creating any missing assets')
    args = parser.parse_args()
    print(json.dumps(ensure_assets(check_only=args.check_only), indent=2))
