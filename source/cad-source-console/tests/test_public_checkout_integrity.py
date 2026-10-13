"""Read-only regression gates for the public checkout (standard library only).

Install this file under source/cad-source-console/tests and run unittest discover.
For an external audit: python test_public_checkout_integrity.py --console PATH.
Missing binary transport is reported as one explicit skip by default; use
--require-complete (or GTSD_REQUIRE_COMPLETE_ASSETS=1) to make it a failure.
A successful default run with that skip is NOT full release/runtime acceptance.
Packed source JSON has its own pinned transport classification when present.
No assets are restored, downloaded, or decoded; no simulation is initialized.
"""
from functools import lru_cache
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import unittest

sys.dont_write_bytecode = True
CONSOLE = Path(os.environ.get('GTSD_CHECKOUT_CONSOLE_ROOT', Path(__file__).resolve().parents[1])).resolve()
REQUIRE_COMPLETE = os.environ.get('GTSD_REQUIRE_COMPLETE_ASSETS') == '1'
BINARY_SUFFIXES = {'.bin', '.png', '.blend', '.glb', '.gz', '.jpg', '.jpeg', '.webp', '.zip', '.mp4'}
HEX256 = re.compile(r'^[0-9a-f]{64}$')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON key: ' + key)
        result[key] = value
    return result


def load_json(path):
    try:
        return json.loads(path.read_bytes(), object_pairs_hook=unique_object)
    except (ValueError, UnicodeError, OSError) as exc:
        raise AssertionError(f'{path.name} is not readable valid JSON: {exc}') from exc


def checked_path(root, name):
    if not isinstance(name, str) or not name:
        raise AssertionError('Empty or non-string manifest path')
    rel = PurePosixPath(name)
    if rel.is_absolute() or '..' in rel.parts or '\\' in name or ':' in name or rel.as_posix() != name:
        raise AssertionError('Unsafe manifest path: ' + repr(name))
    path = root.joinpath(*rel.parts)
    if not path.resolve().is_relative_to(root.resolve()):
        raise AssertionError('Manifest path escapes checkout: ' + name)
    return path


@lru_cache(maxsize=1)
def public_manifest():
    value = load_json(CONSOLE / 'PUBLIC_MANIFEST.json')
    if value.get('schema') != 'gtsd-public-console-manifest-v1' or not isinstance(value.get('files'), dict):
        raise AssertionError('Unexpected PUBLIC_MANIFEST schema or files mapping')
    for name, row in value['files'].items():
        checked_path(CONSOLE, name)
        if not isinstance(row, dict) or type(row.get('bytes')) is not int or row['bytes'] < 0 or not HEX256.fullmatch(str(row.get('sha256'))):
            raise AssertionError('Invalid file identity record: ' + name)
    return value


@lru_cache(maxsize=1)
def asset_manifest():
    value = load_json(CONSOLE / 'ASSET_MANIFEST.json')
    if value.get('schema') != 'gtsd-public-chunked-assets-v1' or not isinstance(value.get('assets'), list):
        raise AssertionError('Unexpected ASSET_MANIFEST schema or assets list')
    paths = [row['path'] for row in value['assets']]
    if len(set(paths)) != len(paths):
        raise AssertionError('Duplicate asset path')
    for asset in value['assets']:
        checked_path(CONSOLE, asset['path'])
        for chunk in asset['chunks']:
            checked_path(CONSOLE, chunk['path'])
    return value



@lru_cache(maxsize=1)
def source_review_manifest():
    """Optional new transport; no decoded files or container reads required."""
    path = CONSOLE / 'SOURCE_REVIEW_MANIFEST.json'
    if not path.exists():
        return None
    value = load_json(path)
    # Importing this standard-library helper performs no reads/writes or restore.
    import importlib.util
    spec = importlib.util.spec_from_file_location('gtsd_source_review_gate', CONSOLE / 'source_review_assets.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    try:
        if identity(path) != (path.stat().st_size, helper.MANIFEST_SHA256):
            raise ValueError('Pinned source-review manifest identity mismatch')
        helper._validate(value)
    except (ValueError, KeyError, TypeError) as exc:
        raise AssertionError('Invalid source-review transport manifest: ' + str(exc)) from exc
    extra_path = CONSOLE / 'SOURCE_REVIEW_EXTRA_MANIFEST.json'
    if extra_path.exists():
        extra = load_json(extra_path)
        try:
            if identity(extra_path) != (extra_path.stat().st_size, helper.EXTRA_MANIFEST_SHA256):
                raise ValueError('Pinned additional source-review manifest identity mismatch')
            helper._validate(extra, 'payloads/source-review-extra-v1')
        except (ValueError, KeyError, TypeError) as exc:
            raise AssertionError('Invalid additional source-review manifest: ' + str(exc)) from exc
        if {row['path'] for row in value['files']} & {row['path'] for row in extra['files']}:
            raise AssertionError('Duplicate source-review path across containers')
        value = {**value, 'files': value['files'] + extra['files'],
                 'chunks': value['chunks'] + extra['chunks']}
    old_targets = {row['path'] for row in asset_manifest()['assets']}
    old_chunks = {chunk['path'] for row in asset_manifest()['assets'] for chunk in row['chunks']}
    review_targets = {row['path'] for row in value['files']}
    review_chunks = {row['path'] for row in value['chunks']}
    if (review_targets & (old_targets | old_chunks) or review_chunks & (old_targets | old_chunks)
            or review_targets & review_chunks):
        raise AssertionError('Source-review paths overlap the original asset transport')
    public = public_manifest()['files']
    for row in value['files'] + value['chunks']:
        checked_path(CONSOLE, row['path'])
        if public.get(row['path']) != {key: row[key] for key in ('bytes', 'sha256')}:
            raise AssertionError('Source-review/public identity mismatch: ' + row['path'])
    return value


def identity(path):
    h = hashlib.sha256()
    count = 0
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            count += len(data)
            h.update(data)
    return count, h.hexdigest()


def availability():
    assets = asset_manifest()['assets']
    targets = {a['path'] for a in assets}
    chunks = {c['path'] for a in assets for c in a['chunks']}
    review = source_review_manifest()
    review_targets = {row['path'] for row in review['files']} if review else set()
    review_chunks = {row['path'] for row in review['chunks']} if review else set()
    all_targets = targets | review_targets
    chunks |= review_chunks
    missing_chunks = sorted(p for p in chunks if not checked_path(CONSOLE, p).is_file())
    absent = [p for p in public_manifest()['files'] if not checked_path(CONSOLE, p).is_file()]
    missing_static_binary = sorted(p for p in absent if p not in all_targets and p not in chunks and PurePosixPath(p).suffix.lower() in BINARY_SUFFIXES)
    missing_source = sorted(p for p in absent if p not in all_targets and p not in chunks and p not in missing_static_binary)
    absent_targets = sorted(p for p in targets if not checked_path(CONSOLE, p).is_file())
    incomplete_assets = [a['path'] for a in assets if any(c['path'] in missing_chunks for c in a['chunks'])]
    result = {'planned_assets': len(assets), 'present_chunksets': len(assets) - len(incomplete_assets),
            'incomplete_assets': incomplete_assets, 'missing_chunks': missing_chunks,
            'missing_static_binary': missing_static_binary, 'missing_source_files': missing_source,
            'unrestored_asset_targets': absent_targets,
            'note': 'Chunk presence alone is not decoded-asset or runtime acceptance.'}
    if review:
        result.update(reviewed_source_files=len(review_targets),
            reviewed_source_chunks=len(review_chunks),
            missing_source_review_chunks=sorted(review_chunks & set(missing_chunks)),
            unrestored_source_review_targets=sorted(p for p in review_targets if not checked_path(CONSOLE, p).is_file()))
    return result


class PublicCheckoutIntegrityTests(unittest.TestCase):
    def test_public_manifest_json_schema_and_records(self):
        self.assertGreater(len(public_manifest()['files']), 0)

    def test_present_public_files_match_exact_bytes(self):
        mismatches = []
        for name, row in public_manifest()['files'].items():
            path = checked_path(CONSOLE, name)
            if path.exists() and (not path.is_file() or identity(path) != (row['bytes'], row['sha256'])):
                mismatches.append(name)
        self.assertEqual(mismatches, [], 'Present files differ from PUBLIC_MANIFEST: ' + ', '.join(mismatches))

    def test_missing_source_files_are_not_hidden_as_binary_transport(self):
        missing = availability()['missing_source_files']
        self.assertEqual(missing, [], 'Missing source/metadata text: ' + ', '.join(missing))

    def test_present_chunks_match_asset_manifest(self):
        bad = []
        manifest = asset_manifest()
        for asset in manifest['assets']:
            for chunk in asset['chunks']:
                path = checked_path(CONSOLE, chunk['path'])
                if path.exists() and (not path.is_file() or chunk['bytes'] > manifest['max_chunk_bytes'] or identity(path) != (chunk['bytes'], chunk['sha256'])):
                    bad.append(chunk['path'])
        review = source_review_manifest()
        if review:
            for chunk in review['chunks']:
                path = checked_path(CONSOLE, chunk['path'])
                if path.exists() and (not path.is_file() or chunk['bytes'] > review['max_chunk_bytes'] or identity(path) != (chunk['bytes'], chunk['sha256'])):
                    bad.append(chunk['path'])
        self.assertEqual(bad, [], 'Present binary chunks have wrong bytes/hash: ' + ', '.join(bad))

    def test_binary_transport_completeness_is_explicit(self):
        state = availability()
        if not state['missing_chunks'] and not state['missing_static_binary']:
            return
        missing = state['missing_chunks'] + state['missing_static_binary']
        preview = ', '.join(missing[:8])
        if len(missing) > 8:
            preview += f'; {len(missing) - 8} more (run this file with --report for all paths)'
        message = (f"PARTIAL CHECKOUT: {state['present_chunksets']}/{state['planned_assets']} chunksets present; "
                   f"{len(state['missing_chunks'])} chunks and {len(state['missing_static_binary'])} other binary files missing. "
                   'Full console/runtime acceptance is blocked. Missing paths: ' +
                   preview)
        if REQUIRE_COMPLETE:
            self.fail(message)
        self.skipTest(message)

    def test_frozen_crlf_csv_has_no_text_normalization(self):
        if shutil.which('git') is None:
            self.skipTest('Git unavailable; frozen CSV attribute gate requires a Git checkout')
        command = subprocess.run(['git', '-C', str(CONSOLE), 'rev-parse', '--show-toplevel'], capture_output=True, text=True)
        if command.returncode:
            self.skipTest('No Git checkout metadata; CSV attributes must be tested in a true checkout')
        repo = Path(command.stdout.strip()).resolve()
        baseline = CONSOLE / 'baseline_console/GTSD_Assembly_Console'
        frozen = load_json(baseline / 'BUNDLE_MANIFEST.json')['files']
        conflicts = []
        for name, expected in frozen.items():
            if not name.endswith('.csv'):
                continue
            path = checked_path(baseline, name)
            if not path.is_file():
                self.fail('Frozen CSV missing: ' + name)
            data = path.read_bytes()
            self.assertEqual((len(data), hashlib.sha256(data).hexdigest()), (expected['bytes'], expected['sha256']), 'Frozen CSV byte identity changed: ' + name)
            if b'\r\n' not in data:
                continue
            rel = path.relative_to(repo).as_posix()
            output = subprocess.check_output(['git', '-C', str(repo), 'check-attr', '-z', 'text', '--', rel])
            fields = output.split(b'\0')
            if len(fields) < 3 or fields[2] != b'unset':
                conflicts.append(rel)
        self.assertEqual(conflicts, [], 'Frozen CRLF CSV requires an exact -text/binary attribute rule, preserving original bytes: ' + ', '.join(conflicts))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--console', type=Path, default=CONSOLE)
    parser.add_argument('--require-complete', action='store_true', default=REQUIRE_COMPLETE)
    parser.add_argument('--report', type=Path, help='Optional availability JSON; must be outside checkout')
    args, remaining = parser.parse_known_args()
    CONSOLE = args.console.resolve()
    REQUIRE_COMPLETE = args.require_complete
    if args.report:
        destination = args.report.resolve()
        git_root = subprocess.run(['git', '-C', str(CONSOLE), 'rev-parse', '--show-toplevel'], capture_output=True, text=True)
        protected = Path(git_root.stdout.strip()).resolve() if git_root.returncode == 0 else CONSOLE
        if destination.is_relative_to(protected):
            parser.error('--report must be outside the audited checkout')
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            state = availability()
        except AssertionError as exc:
            state = {'error': str(exc)}
        destination.write_text(json.dumps(state, indent=2) + '\n', encoding='utf-8')
    unittest.main(argv=[sys.argv[0], *remaining])
