"""Small offline tests of portable restoration safety and idempotence."""
from pathlib import Path
import gzip
import hashlib
import json
import sys
import tempfile
import unittest
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from prepare_assets import ensure_assets, safe_path

class RestoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.raw = b'public generated mesh data\n' * 20
        packed = gzip.compress(self.raw, mtime=0)
        self.chunk = self.root / 'payload.bin'
        self.chunk.write_bytes(packed)
        digest = lambda b: hashlib.sha256(b).hexdigest()
        self.asset = {'path': 'data/model.json', 'bytes': len(self.raw), 'sha256': digest(self.raw), 'encoding': 'gzip', 'packed_bytes': len(packed), 'packed_sha256': digest(packed), 'chunks': [{'path': 'payload.bin', 'bytes': len(packed), 'sha256': digest(packed)}]}
        self.write_manifest()
    def tearDown(self):
        self.tmp.cleanup()
    def write_manifest(self):
        (self.root / 'ASSET_MANIFEST.json').write_text(json.dumps({'schema': 'gtsd-public-chunked-assets-v1', 'assets': [self.asset]}))
    def test_restore_and_idempotent_readonly_check(self):
        self.assertEqual(ensure_assets(self.root)['assets_restored'], 1)
        p = self.root / self.asset['path']
        before = p.stat().st_mtime_ns
        self.assertEqual(p.read_bytes(), self.raw)
        self.assertEqual(ensure_assets(self.root, check_only=True)['assets_restored'], 0)
        self.assertEqual(p.stat().st_mtime_ns, before)
    def test_missing_check_only_does_not_write(self):
        with self.assertRaises(FileNotFoundError): ensure_assets(self.root, check_only=True)
        self.assertFalse((self.root / 'data').exists())
    def test_corrupt_chunk_rejected_before_write(self):
        self.chunk.write_bytes(self.chunk.read_bytes() + b'!')
        with self.assertRaises(ValueError): ensure_assets(self.root)
        self.assertFalse((self.root / 'data').exists())
    def test_modified_existing_file_preserved(self):
        ensure_assets(self.root)
        p = self.root / self.asset['path']; p.write_bytes(b'user changes')
        with self.assertRaises(ValueError): ensure_assets(self.root)
        self.assertEqual(p.read_bytes(), b'user changes')
    def test_path_traversal_rejected(self):
        for name in ['../escape', '/tmp/escape', 'a/../../escape', 'a\\escape']:
            with self.assertRaises(ValueError): safe_path(self.root, name)
    def test_decoded_length_mismatch_cleans_temporary(self):
        self.asset['bytes'] -= 1; self.write_manifest()
        with self.assertRaises(ValueError): ensure_assets(self.root)
        self.assertFalse((self.root / self.asset['path']).exists())
        self.assertFalse(list(self.root.rglob('.gtsd-restore-*')))

if __name__ == '__main__': unittest.main()
