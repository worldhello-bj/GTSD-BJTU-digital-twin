"""Verify and atomically restore bundled assets. Standard library, no downloads.

All chunk paths and restored paths stay inside this directory. A modified existing
asset is rejected, never overwritten. Missing assets are restored automatically.
By default every manifest asset is required. --only selects exact manifest paths;
it never declares the whole release complete. --list-assets only lists names.
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
    if not isinstance(name, str) or not name:
        raise ValueError('Unsafe asset path: ' + repr(name))
    rel = PurePosixPath(name)
    if (rel.is_absolute() or not rel.parts or '..' in rel.parts
            or '\\' in name or ':' in name or name != rel.as_posix()
            or any(ord(c) < 32 or ord(c) == 127 for c in name)):
        raise ValueError('Unsafe asset path: ' + name)
    path = root.joinpath(*rel.parts)
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('Asset path escapes release: ' + name)
    return path

def select_assets(root, only=None):
    """Resolve all selectors and paths before any payload reads or writes."""
    requested = None
    if only is not None:
        if isinstance(only, (str, bytes)):
            raise ValueError('only must be a nonempty sequence of exact asset paths')
        requested = list(only)
        if not requested:
            raise ValueError('only must be a nonempty sequence of exact asset paths')
        for name in requested:
            safe_path(root, name)
    manifest = json.loads((root / 'ASSET_MANIFEST.json').read_text(encoding='utf-8'))
    if manifest['schema'] != 'gtsd-public-chunked-assets-v1':
        raise ValueError('Unsupported asset manifest')
    by_name = {}
    for asset in manifest['assets']:
        name = asset['path']
        safe_path(root, name)
        if name in by_name:
            raise ValueError('Duplicate manifest asset path: ' + name)
        by_name[name] = asset
    if requested is None:
        selected = manifest['assets']
    else:
        for name in requested:
            if name not in by_name:
                raise ValueError('Unknown manifest asset path: ' + name)
        names = set(requested)
        # Preserve manifest order and make repeated selectors idempotent.
        selected = [asset for asset in manifest['assets'] if asset['path'] in names]
    for asset in selected:
        for chunk in asset['chunks']:
            safe_path(root, chunk['path'])
    return selected, len(manifest['assets'])

def list_assets(root=ROOT, only=None):
    """List exact asset names without reading chunks or creating any files."""
    selected, _ = select_assets(Path(root).resolve(), only)
    return [asset['path'] for asset in selected]

def ensure_assets(root=ROOT, check_only=False, only=None):
    """Verify/restore all assets, or an explicitly selected nonempty subset."""
    root = Path(root).resolve()
    selected, assets_total = select_assets(root, only)
    restored = []
    for asset in selected:
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
        if target.is_symlink():
            raise ValueError('Existing asset is a symlink; refusing to replace: ' + asset['path'])
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
            # A same-directory hard link publishes the fully verified file
            # atomically and fails if another writer has created the target.
            # Never fall back to an overwrite or a partially visible copy.
            os.link(temporary, target)
            restored.append(asset['path'])
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
    result = {'ok': True, 'assets_checked': len(selected), 'assets_restored': len(restored), 'network_used': False}
    if only is not None:
        result.update(scope='selected', assets_total=assets_total,
                      selected_assets=[asset['path'] for asset in selected])
    if only is None:
        from source_review_assets import source_review_required, ensure_source_review
        if source_review_required(root):
            result['source_review'] = ensure_source_review(root, check_only=check_only)
    return result

def main(argv=None, root=ROOT):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-only', action='store_true', help='Verify without creating any missing assets')
    mode.add_argument('--list-assets', action='store_true', help='List exact manifest paths only; does not verify availability')
    parser.add_argument('--only', action='append', metavar='ASSET_PATH', help='Select this exact, full manifest asset.path; repeat to select more (no globs or basenames)')
    parser.add_argument('--source-review-only', action='store_true', help='Restore/check only the reviewed source JSON files from both pinned containers; no legacy binary assets are required')
    args = parser.parse_args(argv)
    if args.source_review_only:
        if args.only:
            parser.error('--source-review-only cannot be combined with --only')
        from source_review_assets import ensure_source_review, list_source_review
        result = ({'scope': 'source_review', 'files': list_source_review(root), 'network_used': False}
                  if args.list_assets else ensure_source_review(root, check_only=args.check_only))
        print(json.dumps(result, indent=2))
    elif args.list_assets:
        print(json.dumps({'assets': list_assets(root, only=args.only), 'network_used': False}, indent=2))
    else:
        print(json.dumps(ensure_assets(root, check_only=args.check_only, only=args.only), indent=2))

if __name__ == '__main__':
    main()
