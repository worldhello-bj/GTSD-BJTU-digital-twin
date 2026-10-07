#!/usr/bin/env python3
"""Verify B8 backups and restore losslessly split original ZIPs. Offline only."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent

def checked_path(relative):
    p = (ROOT / relative).resolve()
    if not p.is_relative_to(ROOT):
        raise ValueError('Manifest path escapes repository')
    return p

def check(path, record):
    data = path.read_bytes()
    if len(data) != record['size'] or hashlib.sha256(data).hexdigest() != record['sha256']:
        raise ValueError(f'Integrity mismatch: {path.name}')
    return data

def main():
    manifest = json.loads((ROOT / 'BACKUP_MANIFEST.json').read_text(encoding='utf-8'))
    for record in manifest['stored_files']:
        check(checked_path(record['path']), record)
    for archive in manifest['split_archives']:
        target = checked_path(archive['path'])
        if target.exists():
            check(target, archive)
        else:
            temporary = target.with_name(target.name + '.restoring')
            with temporary.open('xb') as output:
                for part in archive['parts']:
                    output.write(check(checked_path(part['path']), part))
            check(temporary, archive)
            with zipfile.ZipFile(temporary) as z:
                if z.testzip() is not None:
                    raise ValueError(f'ZIP CRC failure: {target.name}')
            temporary.rename(target)
        print(f'Restored and verified: {target.relative_to(ROOT)}')
    for record in manifest['original_artifacts']:
        path = checked_path(record['path'])
        check(path, record)
        if path.suffix == '.zip':
            with zipfile.ZipFile(path) as z:
                if z.testzip() is not None:
                    raise ValueError(f'ZIP CRC failure: {path.name}')
    print('All four original B8 ZIPs and both media files verified.')

if __name__ == '__main__':
    main()
