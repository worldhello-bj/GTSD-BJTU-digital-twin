"""Restore the pinned, reviewed source JSON bundle; no archive paths or network.

Only the manifest's explicitly approved files can be produced. All files and
all container bytes are verified before publication. Existing files are never
replaced. Directory-relative, no-follow operations close symlink race windows.
"""
from pathlib import Path, PurePosixPath
import hashlib
import json
import lzma
import os
import re
import secrets
import stat

MANIFEST = 'SOURCE_REVIEW_MANIFEST.json'
MANIFEST_SHA256 = '52cffddf6d23c3b92df133eb629ccfe1e73ac3732e47c58fd6551b61d53201d2'
EXTRA_MANIFEST = 'SOURCE_REVIEW_EXTRA_MANIFEST.json'
EXTRA_MANIFEST_SHA256 = '89ce7569251eb9ce14077e70c8d3f8c15364e859f6ef5ca44d31905b76c1992b'
BASE_RELEASE_MANIFEST_SHA256 = '52cffddf6d23c3b92df133eb629ccfe1e73ac3732e47c58fd6551b61d53201d2'
LEGACY_MANIFEST_SHA256 = '357f1af1caadef4964816c9a7b481a07e5b42087652da1b74026d3920313baaa'
MAX_CHUNK_BYTES = 300000
MAX_MANIFEST_BYTES = 256 * 1024
MAX_PACKED_BYTES = 32 * 1024 * 1024
MAX_RAW_BYTES = 64 * 1024 * 1024
MAX_FILE_BYTES = 16 * 1024 * 1024
XZ_MEMORY_LIMIT = 64 * 1024 * 1024
HEX = re.compile(r'[0-9a-f]{64}\Z')
SOURCE_PATH = re.compile(r'integration/data/source_library/[A-Za-z0-9_-]+\.json\Z')
DIRECTORY_OPERATIONS_SUPPORTED = (os.open in os.supports_dir_fd
    and os.link in os.supports_dir_fd and hasattr(os, 'O_NOFOLLOW')
    and hasattr(os, 'O_DIRECTORY'))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate source-review JSON key: ' + key)
        result[key] = value
    return result


def _parts(name):
    if not isinstance(name, str) or not name:
        raise ValueError('Unsafe source-review path')
    rel = PurePosixPath(name)
    if (rel.is_absolute() or '..' in rel.parts or '\\' in name or ':' in name
            or name != rel.as_posix() or not rel.parts
            or any(ord(c) < 32 or ord(c) == 127 for c in name)):
        raise ValueError('Unsafe source-review path: ' + name)
    return rel.parts


def _parent(root, name, create=False):
    """Return a pinned parent fd; never follow a directory symlink."""
    parts = _parts(name)
    if not DIRECTORY_OPERATIONS_SUPPORTED:
        raise OSError('Source review restore requires no-follow directory-relative file operations; use Linux/macOS or WSL')
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    fd = os.open(root, flags)
    try:
        for part in parts[:-1]:
            if create:
                try:
                    os.mkdir(part, dir_fd=fd)
                except FileExistsError:
                    pass
            next_fd = os.open(part, flags, dir_fd=fd)
            os.close(fd)
            fd = next_fd
        return fd, parts[-1]
    except BaseException:
        os.close(fd)
        raise


def _read(root, name, limit):
    fd, leaf = _parent(root, name)
    try:
        file_fd = os.open(leaf, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=fd)
        try:
            info = os.fstat(file_fd)
            if not stat.S_ISREG(info.st_mode) or info.st_size > limit:
                raise ValueError('Invalid or oversized source-review file: ' + name)
            stream = os.fdopen(file_fd, 'rb')
            file_fd = None
            with stream:
                data = stream.read(limit + 1)
                if len(data) > limit:
                    raise ValueError('Oversized source-review file: ' + name)
                return data
        finally:
            if file_fd is not None:
                os.close(file_fd)
    finally:
        os.close(fd)


def _integer(value, lower, upper, label):
    if type(value) is not int or not lower <= value <= upper:
        raise ValueError('Invalid source-review ' + label)


def _hash(value):
    if not isinstance(value, str) or not HEX.fullmatch(value):
        raise ValueError('Invalid source-review SHA-256')


def _validate(manifest, chunk_prefix='payloads/source-review-v1'):
    if chunk_prefix not in ('payloads/source-review-v1', 'payloads/source-review-extra-v1'):
        raise ValueError('Unsupported source-review chunk namespace')
    if (not isinstance(manifest, dict)
            or manifest.get('schema') != 'gtsd-reviewed-source-bundle-v1'
            or manifest.get('encoding') != 'xz-concatenated-json-v1'
            or manifest.get('max_chunk_bytes') != MAX_CHUNK_BYTES):
        raise ValueError('Unsupported source-review manifest')
    _integer(manifest.get('file_count'), 1, 1024, 'file count')
    _integer(manifest.get('raw_bytes'), 1, MAX_RAW_BYTES, 'decoded length')
    _integer(manifest.get('packed_bytes'), 1, MAX_PACKED_BYTES, 'packed length')
    _hash(manifest.get('raw_sha256'))
    _hash(manifest.get('packed_sha256'))
    files = manifest.get('files')
    if not isinstance(files, list) or len(files) != manifest['file_count']:
        raise ValueError('Source-review file count mismatch')
    offset = 0
    seen = set()
    for row in files:
        if not isinstance(row, dict) or set(row) != {'path', 'offset', 'bytes', 'sha256'}:
            raise ValueError('Invalid source-review file record')
        name = row['path']
        _parts(name)
        if not SOURCE_PATH.fullmatch(name) or name.endswith('/catalog.json'):
            raise ValueError('Source-review path not allowed: ' + name)
        if name in seen:
            raise ValueError('Duplicate source-review path: ' + name)
        seen.add(name)
        _integer(row['bytes'], 1, MAX_FILE_BYTES, 'file length')
        _integer(row['offset'], 0, MAX_RAW_BYTES, 'file offset')
        _hash(row['sha256'])
        if row['offset'] != offset or offset + row['bytes'] > manifest['raw_bytes']:
            raise ValueError('Invalid source-review file boundary: ' + name)
        offset += row['bytes']
    if offset != manifest['raw_bytes']:
        raise ValueError('Source-review final boundary mismatch')
    if [row['path'] for row in files] != sorted(seen):
        raise ValueError('Source-review files must have canonical order')
    chunks = manifest.get('chunks')
    count = (manifest['packed_bytes'] + MAX_CHUNK_BYTES - 1) // MAX_CHUNK_BYTES
    if not isinstance(chunks, list) or len(chunks) != count:
        raise ValueError('Source-review chunk count mismatch')
    remaining = manifest['packed_bytes']
    for index, row in enumerate(chunks):
        if not isinstance(row, dict) or set(row) != {'path', 'bytes', 'sha256'}:
            raise ValueError('Invalid source-review chunk record')
        expected = f"{chunk_prefix}/{manifest['packed_sha256'][:20]}.{index:03d}.bin"
        if row['path'] != expected:
            raise ValueError('Source-review chunk path not allowed')
        _integer(row['bytes'], 1, MAX_CHUNK_BYTES, 'chunk length')
        _hash(row['sha256'])
        if row['bytes'] != min(MAX_CHUNK_BYTES, remaining):
            raise ValueError('Source-review chunk boundary mismatch')
        remaining -= row['bytes']
    return manifest


def load_manifest(root):
    raw = _read(root, MANIFEST, MAX_MANIFEST_BYTES)
    if digest(raw) != MANIFEST_SHA256:
        raise ValueError('Pinned source-review manifest identity mismatch')
    return _validate(json.loads(raw, object_pairs_hook=unique_object))



def load_manifests(root):
    manifests = [load_manifest(root)]
    # A genuine first-release index requires its new companion in this version.
    # Legacy synthetic manifests remain supported for the original API tests.
    requires_extra = (os.path.lexists(Path(root) / EXTRA_MANIFEST)
        or digest(_read(root, MANIFEST, MAX_MANIFEST_BYTES)) == BASE_RELEASE_MANIFEST_SHA256)
    if requires_extra:
        raw = _read(root, EXTRA_MANIFEST, MAX_MANIFEST_BYTES)
        if digest(raw) != EXTRA_MANIFEST_SHA256:
            raise ValueError('Pinned additional source-review manifest identity mismatch')
        extra = _validate(json.loads(raw, object_pairs_hook=unique_object),
                          'payloads/source-review-extra-v1')
        manifests.append(extra)
    paths = [row['path'] for manifest in manifests for row in manifest['files']]
    if len(paths) != len(set(paths)):
        raise ValueError('Duplicate source-review path across containers')
    if (sum(manifest['raw_bytes'] for manifest in manifests) > MAX_RAW_BYTES
            or sum(manifest['packed_bytes'] for manifest in manifests) > MAX_PACKED_BYTES):
        raise ValueError('Source-review combined container limit exceeded')
    return manifests


def source_review_required(root):
    """Recognize this release while retaining legacy-only API fixture support."""
    root = Path(root).resolve()
    if root == Path(__file__).resolve().parent or os.path.lexists(root / MANIFEST):
        return True
    return digest(_read(root, 'ASSET_MANIFEST.json', 256 * 1024)) == LEGACY_MANIFEST_SHA256


def list_source_review(root):
    return [row['path'] for manifest in load_manifests(Path(root).resolve()) for row in manifest['files']]


def _existing(root, row):
    try:
        data = _read(root, row['path'], row['bytes'])
    except FileNotFoundError:
        return False
    if len(data) != row['bytes'] or digest(data) != row['sha256']:
        raise ValueError('Existing source-review file differs; refusing to overwrite: ' + row['path'])
    return True


def _decode(root, manifest):
    chunks = []
    for row in manifest['chunks']:
        data = _read(root, row['path'], MAX_CHUNK_BYTES)
        if len(data) != row['bytes'] or digest(data) != row['sha256']:
            raise ValueError('Source-review chunk identity mismatch: ' + row['path'])
        chunks.append(data)
    packed = b''.join(chunks)
    if len(packed) != manifest['packed_bytes'] or digest(packed) != manifest['packed_sha256']:
        raise ValueError('Source-review packed identity mismatch')
    decoder = lzma.LZMADecompressor(format=lzma.FORMAT_XZ, memlimit=XZ_MEMORY_LIMIT)
    raw = decoder.decompress(packed, max_length=manifest['raw_bytes'] + 1)
    if len(raw) != manifest['raw_bytes'] or not decoder.eof or decoder.unused_data:
        raise ValueError('Source-review decoded length, trailing data or stream mismatch')
    if digest(raw) != manifest['raw_sha256']:
        raise ValueError('Source-review decoded identity mismatch')
    for row in manifest['files']:
        data = raw[row['offset']:row['offset'] + row['bytes']]
        if digest(data) != row['sha256']:
            raise ValueError('Source-review restored identity mismatch: ' + row['path'])
        # JSON is inspected only, never reformatted, so the exact original bytes survive.
        json.loads(data.decode('utf-8'), object_pairs_hook=unique_object)
    return raw


def _publish(root, row, data):
    fd, leaf = _parent(root, row['path'], create=True)
    temporary = '.gtsd-source-review-' + secrets.token_hex(16)
    created = False
    try:
        out_fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                         0o644, dir_fd=fd)
        created = True
        with os.fdopen(out_fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        # A verified whole file appears atomically; a competing target always wins.
        os.link(temporary, leaf, src_dir_fd=fd, dst_dir_fd=fd, follow_symlinks=False)
        os.fsync(fd)
    finally:
        if created:
            os.unlink(temporary, dir_fd=fd)
        os.close(fd)


def ensure_source_review(root, check_only=False):
    root = Path(root).resolve()
    manifests = load_manifests(root)
    files = [row for manifest in manifests for row in manifest['files']]
    # Preflight both containers and verify all outputs before the first write.
    present = {row['path']: _existing(root, row) for row in files}
    decoded = [_decode(root, manifest) for manifest in manifests]
    missing = [row for row in files if not present[row['path']]]
    if check_only and missing:
        raise FileNotFoundError('Source-review file not restored: ' + missing[0]['path'])
    restored = 0
    for manifest, raw in zip(manifests, decoded):
        for row in manifest['files']:
            if present[row['path']] or _existing(root, row):
                continue
            data = raw[row['offset']:row['offset'] + row['bytes']]
            _publish(root, row, data)
            restored += 1
    return {'ok': True, 'scope': 'source_review', 'files_checked': len(files),
            'files_restored': restored, 'network_used': False}
