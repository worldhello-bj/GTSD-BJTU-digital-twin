"""Small standard-library tests for explicit partial-release restoration."""
from contextlib import redirect_stdout
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch
import gzip
import hashlib
import io
import json
import os
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from prepare_assets import ensure_assets, list_assets, main, safe_path


def digest(data):
    return hashlib.sha256(data).hexdigest()


class SelectiveRestoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.native_data = b'authorized native fixture\n' * 30
        self.native = self.asset('native/model.blend', self.native_data, 'gzip')
        self.other = self.asset('renders/preview.png', b'preview fixture', 'identity')
        self.missing = self.asset('assembly/whole.glb', b'missing geometry', 'gzip')
        self.missing_chunk = self.root / self.missing['chunks'][0]['path']
        self.missing_chunk.unlink()
        self.assets = [self.missing, self.native, self.other]
        self.write_manifest()

    def tearDown(self):
        self.tmp.cleanup()

    def asset(self, name, raw, encoding):
        packed = gzip.compress(raw, mtime=0) if encoding == 'gzip' else raw
        chunks = []
        for index, start in enumerate(range(0, len(packed), 17)):
            data = packed[start:start + 17]
            chunk = f'payloads/{digest(raw)[:12]}.{index:03d}.bin'
            path = self.root / chunk
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            chunks.append({'path': chunk, 'bytes': len(data), 'sha256': digest(data)})
        return {'path': name, 'bytes': len(raw), 'sha256': digest(raw),
                'encoding': encoding, 'packed_bytes': len(packed),
                'packed_sha256': digest(packed), 'chunks': chunks}

    def write_manifest(self):
        (self.root / 'ASSET_MANIFEST.json').write_text(json.dumps({
            'schema': 'gtsd-public-chunked-assets-v1', 'assets': self.assets}),
            encoding='utf-8')

    def restore_native(self, **kwargs):
        return ensure_assets(self.root, only=[self.native['path']], **kwargs)

    def assert_no_targets(self):
        for asset in self.assets:
            self.assertFalse((self.root / asset['path']).exists())
        self.assertFalse(list(self.root.rglob('.gtsd-restore-*')))

    def test_selected_asset_succeeds_with_unrelated_chunk_missing(self):
        before = {str(p): (p.read_bytes(), p.stat().st_mtime_ns)
                  for p in self.root.rglob('*') if p.is_file()}
        report = self.restore_native()
        self.assertEqual(report, {'ok': True, 'assets_checked': 1,
            'assets_restored': 1, 'network_used': False, 'scope': 'selected',
            'assets_total': 3, 'selected_assets': [self.native['path']]})
        self.assertEqual((self.root / self.native['path']).read_bytes(), self.native_data)
        for path, expected in before.items():
            p = Path(path)
            self.assertEqual((p.read_bytes(), p.stat().st_mtime_ns), expected)
        self.assertFalse((self.root / 'assembly').exists())
        self.assertFalse((self.root / 'renders').exists())
        self.assertFalse(list(self.root.rglob('.gtsd-restore-*')))

    def test_multiple_selectors_and_duplicates_restore_only_unique_assets(self):
        result = ensure_assets(self.root, only=[self.other['path'],
                              self.native['path'], self.other['path']])
        self.assertEqual(result['assets_checked'], 2)
        self.assertEqual(result['assets_restored'], 2)
        self.assertEqual(result['selected_assets'], [self.native['path'], self.other['path']])
        self.assertFalse((self.root / 'assembly').exists())

    def test_unknown_selector_rejected_before_valid_asset_is_written(self):
        with self.assertRaisesRegex(ValueError, 'Unknown manifest asset path'):
            ensure_assets(self.root, only=[self.native['path'], 'unknown.blend'])
        self.assert_no_targets()

    def test_basename_glob_and_prefix_are_not_selectors(self):
        for name in ['model.blend', 'native/*', 'native', 'native/model']:
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'Unknown'):
                ensure_assets(self.root, only=[name])
        self.assert_no_targets()

    def test_dangerous_selectors_rejected_before_payload_reads_or_writes(self):
        names = ['../escape', '/tmp/escape', 'a/../../escape', 'a\\escape',
                 '', '.', './native/model.blend', 'native//model.blend',
                 'native/model.blend/', 'C:escape', 'C:/escape',
                 'native/model.blend:stream', 'native/\x00bad', 'native/\nbad']
        for name in names:
            with self.subTest(name=repr(name)), patch.object(Path, 'read_bytes',
                    side_effect=AssertionError('payload read before validation')):
                with self.assertRaisesRegex(ValueError, 'Unsafe asset path'):
                    ensure_assets(self.root, only=[self.native['path'], name])
        self.assert_no_targets()

    def test_empty_api_selection_cannot_silently_restore_all(self):
        for names in [[], (), '', self.native['path']]:
            with self.subTest(names=names), self.assertRaises(ValueError):
                ensure_assets(self.root, only=names)
        self.assert_no_targets()

    def test_unsafe_selected_chunk_rejected_before_any_target_is_written(self):
        self.other['chunks'][0]['path'] = '../escaped-chunk'
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'Unsafe asset path'):
            ensure_assets(self.root, only=[self.native['path'], self.other['path']])
        self.assert_no_targets()

    def test_unsafe_manifest_target_rejected_before_any_write(self):
        self.other['path'] = '../escaped-target'
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'Unsafe asset path'):
            self.restore_native()
        self.assertFalse((self.root / 'native').exists())

    def test_duplicate_manifest_asset_names_are_rejected(self):
        self.assets.append(deepcopy(self.native))
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'Duplicate manifest asset path'):
            self.restore_native()
        self.assert_no_targets()

    def test_selected_idempotence_and_readonly_verification(self):
        self.restore_native()
        target = self.root / self.native['path']
        before = (target.stat().st_mtime_ns, target.read_bytes())
        for kwargs in [{}, {'check_only': True}]:
            self.assertEqual(self.restore_native(**kwargs)['assets_restored'], 0)
            self.assertEqual((target.stat().st_mtime_ns, target.read_bytes()), before)

    def test_selected_check_only_does_not_create_missing_asset(self):
        with self.assertRaisesRegex(FileNotFoundError, 'Asset not restored'):
            self.restore_native(check_only=True)
        self.assert_no_targets()
        self.assertFalse((self.root / 'native').exists())

    def test_selected_missing_chunk_fails_without_creating_target(self):
        (self.root / self.native['chunks'][0]['path']).unlink()
        with self.assertRaises(FileNotFoundError):
            self.restore_native()
        self.assert_no_targets()

    def test_selected_corrupt_chunk_hash_rejected_before_write(self):
        chunk = self.root / self.native['chunks'][0]['path']
        raw = chunk.read_bytes()
        chunk.write_bytes(bytes([raw[0] ^ 1]) + raw[1:])
        with self.assertRaisesRegex(ValueError, 'Chunk identity mismatch'):
            self.restore_native()
        self.assert_no_targets()
        self.assertFalse((self.root / 'native').exists())

    def test_selected_corrupt_chunk_still_fails_if_restored_target_exists(self):
        self.restore_native()
        chunk = self.root / self.native['chunks'][0]['path']
        chunk.write_bytes(chunk.read_bytes() + b'!')
        with self.assertRaisesRegex(ValueError, 'Chunk identity mismatch'):
            self.restore_native(check_only=True)
        self.assertEqual((self.root / self.native['path']).read_bytes(), self.native_data)

    def test_selected_packed_hash_rejected_before_write(self):
        self.native['packed_sha256'] = '0' * 64
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'Whole payload identity mismatch'):
            self.restore_native()
        self.assert_no_targets()

    def test_selected_decoded_hash_rejected_and_temporary_cleaned(self):
        self.native['sha256'] = '0' * 64
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'Restored identity mismatch'):
            self.restore_native()
        self.assert_no_targets()

    def test_modified_selected_target_is_preserved(self):
        self.restore_native()
        target = self.root / self.native['path']
        target.write_bytes(b'user changes')
        with self.assertRaisesRegex(ValueError, 'Existing asset differs'):
            self.restore_native()
        self.assertEqual(target.read_bytes(), b'user changes')

    def test_default_restore_still_requires_missing_unselected_asset(self):
        self.restore_native()
        with self.assertRaises(FileNotFoundError):
            ensure_assets(self.root)
        self.assertFalse((self.root / 'renders').exists())

    def test_default_check_still_requires_all_assets(self):
        self.restore_native()
        with self.assertRaises(FileNotFoundError):
            ensure_assets(self.root, check_only=True)

    def test_full_default_keeps_original_success_report(self):
        self.assets = [self.native, self.other]
        self.write_manifest()
        self.assertEqual(ensure_assets(self.root), {'ok': True,
            'assets_checked': 2, 'assets_restored': 2, 'network_used': False})

    def test_list_assets_is_readonly_with_missing_payloads(self):
        for path in (self.root / 'payloads').glob('*'):
            path.unlink()
        with patch.object(Path, 'read_bytes', side_effect=AssertionError('payload read')):
            self.assertEqual(list_assets(self.root), [a['path'] for a in self.assets])
            self.assertEqual(list_assets(self.root, only=[self.native['path']]), [self.native['path']])
        self.assert_no_targets()

    def test_cli_repeated_only_and_check_only(self):
        for mode in [[], ['--check-only']]:
            output = io.StringIO()
            with redirect_stdout(output):
                main(mode + ['--only', self.native['path'], '--only', self.other['path']], root=self.root)
            self.assertEqual(json.loads(output.getvalue())['assets_checked'], 2)
        self.assertFalse((self.root / 'assembly').exists())

    def test_cli_list_assets_reports_names_without_restoring(self):
        output = io.StringIO()
        with redirect_stdout(output):
            main(['--list-assets'], root=self.root)
        self.assertEqual(json.loads(output.getvalue()), {
            'assets': [a['path'] for a in self.assets], 'network_used': False})
        self.assert_no_targets()

    def test_cli_default_does_not_hide_missing_full_release(self):
        output = io.StringIO()
        with redirect_stdout(output), self.assertRaises(FileNotFoundError):
            main([], root=self.root)
        self.assertEqual(output.getvalue(), '')
        self.assert_no_targets()

    def test_target_symlink_outside_release_is_rejected(self):
        with tempfile.TemporaryDirectory() as outside:
            (self.root / 'native').symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'escapes release'):
                self.restore_native()
            self.assertEqual(list(Path(outside).iterdir()), [])

    def test_chunk_symlink_outside_release_is_rejected(self):
        chunk = self.root / self.native['chunks'][0]['path']
        with tempfile.TemporaryDirectory() as outside:
            external = Path(outside) / 'chunk'
            external.write_bytes(chunk.read_bytes())
            chunk.unlink()
            chunk.symlink_to(external)
            with self.assertRaisesRegex(ValueError, 'escapes release'):
                self.restore_native()
        self.assert_no_targets()

    def test_dangling_target_symlink_is_preserved(self):
        target = self.root / self.native['path']
        target.parent.mkdir()
        target.symlink_to(target.parent / 'missing-user-file')
        with self.assertRaisesRegex(ValueError, 'Existing asset is a symlink'):
            self.restore_native()
        self.assertTrue(target.is_symlink())
        self.assertFalse(target.exists())

    def test_publish_is_atomic_and_new_target_race_never_overwrites(self):
        original_link = os.link
        target = self.root / self.native['path']
        def competing_writer(source, destination):
            self.assertFalse(target.exists())
            self.assertEqual(Path(source).read_bytes(), self.native_data)
            target.write_bytes(b'concurrent user changes')
            return original_link(source, destination)
        with patch('prepare_assets.os.link', side_effect=competing_writer):
            with self.assertRaises(FileExistsError):
                self.restore_native()
        self.assertEqual(target.read_bytes(), b'concurrent user changes')
        self.assertFalse(list(self.root.rglob('.gtsd-restore-*')))

    def test_unsupported_atomic_publish_fails_closed_and_cleans_temporary(self):
        with patch('prepare_assets.os.link', side_effect=OSError('hard links unavailable')):
            with self.assertRaisesRegex(OSError, 'hard links unavailable'):
                self.restore_native()
        self.assert_no_targets()


if __name__ == '__main__':
    unittest.main()
