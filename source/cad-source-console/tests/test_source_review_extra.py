"""Two independent source containers: atomic boundaries and compatibility."""
from contextlib import redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
import lzma
from pathlib import Path
from unittest.mock import patch
import unittest
import source_review_assets as source
import prepare_assets
import test_source_review_assets as fixtures


def digest(data):return hashlib.sha256(data).hexdigest()


class AdditionalSourceContainerTests(unittest.TestCase):
    def setUp(self):
        self.base=fixtures.SourceReviewRestoreTests();self.base.setUp()
        self.addCleanup(self.base.doCleanups);self.root=self.base.root
        self.pin=patch.object(source,'EXTRA_MANIFEST_SHA256','');self.pin.start();self.addCleanup(self.pin.stop)
        self.raw=b'{"additional":3}\n';packed=lzma.compress(self.raw,preset=6,check=lzma.CHECK_SHA256)
        self.path='integration/data/source_library/extra_fixture.json'
        self.chunk=f'payloads/source-review-extra-v1/{digest(packed)[:20]}.000.bin'
        file=self.root/self.chunk;file.parent.mkdir(parents=True);file.write_bytes(packed)
        self.extra={'schema':'gtsd-reviewed-source-bundle-v1','encoding':'xz-concatenated-json-v1',
            'max_chunk_bytes':300000,'file_count':1,'raw_bytes':len(self.raw),'raw_sha256':digest(self.raw),
            'packed_bytes':len(packed),'packed_sha256':digest(packed),
            'files':[{'path':self.path,'offset':0,'bytes':len(self.raw),'sha256':digest(self.raw)}],
            'chunks':[{'path':self.chunk,'bytes':len(packed),'sha256':digest(packed)}]}
        self.save()

    def save(self,pin=True):
        raw=json.dumps(self.extra).encode();(self.root/source.EXTRA_MANIFEST).write_bytes(raw)
        if pin:source.EXTRA_MANIFEST_SHA256=digest(raw)
    def restore(self,**kwargs):return source.ensure_source_review(self.root,**kwargs)
    def assert_absent(self):self.base.assert_absent();self.assertFalse((self.root/self.path).exists())

    def test_both_containers_restore_original_bytes(self):
        result=self.restore();self.assertEqual(result['files_checked'],3);self.assertEqual(result['files_restored'],3)
        self.assertEqual((self.root/self.path).read_bytes(),self.raw)
        self.assertEqual([p.read_bytes() for p in self.base.targets()],self.base.raws)

    def test_extra_corrupt_chunk_prevents_first_container_publication(self):
        p=self.root/self.chunk;p.write_bytes(p.read_bytes()+b'!')
        with self.assertRaises(ValueError):self.restore()
        self.assert_absent()

    def test_extra_missing_chunk_prevents_first_container_publication(self):
        (self.root/self.chunk).unlink()
        with self.assertRaises(FileNotFoundError):self.restore()
        self.assert_absent()

    def test_extra_manifest_pin_prevents_first_container_publication(self):
        self.extra['files'][0]['sha256']='0'*64;self.save(pin=False)
        with self.assertRaisesRegex(ValueError,'Pinned additional'):self.restore()
        self.assert_absent()

    def test_duplicate_path_across_containers_rejected(self):
        self.extra['files'][0]['path']=self.base.manifest['files'][0]['path'];self.save()
        with self.assertRaisesRegex(ValueError,'across containers'):self.restore()
        self.assert_absent()

    def test_extra_cannot_reuse_first_namespace(self):
        self.extra['chunks'][0]['path']=self.extra['chunks'][0]['path'].replace('source-review-extra-v1','source-review-v1');self.save()
        with self.assertRaisesRegex(ValueError,'chunk path'):self.restore()
        self.assert_absent()

    def test_modified_extra_target_blocks_all_new_outputs(self):
        p=self.root/self.path;p.parent.mkdir(parents=True);p.write_bytes(b'edit')
        with self.assertRaises(ValueError):self.restore()
        self.base.assert_absent();self.assertEqual(p.read_bytes(),b'edit')

    def test_known_first_release_requires_extra_manifest(self):
        (self.root/source.EXTRA_MANIFEST).unlink()
        with patch.object(source,'BASE_RELEASE_MANIFEST_SHA256',source.MANIFEST_SHA256):
            with self.assertRaises(FileNotFoundError):self.restore()
        self.assert_absent()

    def test_combined_raw_limit_rejected(self):
        total=self.base.manifest['raw_bytes']+self.extra['raw_bytes']
        with patch.object(source,'MAX_RAW_BYTES',total-1):
            with self.assertRaisesRegex(ValueError,'combined'):self.restore()
        self.assert_absent()

    def test_readonly_check_applies_to_both_containers(self):
        with self.assertRaises(FileNotFoundError):self.restore(check_only=True)
        self.assert_absent();self.restore()
        times={p:p.stat().st_mtime_ns for p in self.base.targets()+[self.root/self.path]}
        self.assertEqual(self.restore(check_only=True)['files_restored'],0)
        self.assertEqual(times,{p:p.stat().st_mtime_ns for p in times})

    def test_list_both_without_payload_reads(self):
        for r in self.base.manifest['chunks']:(self.root/r['path']).unlink()
        (self.root/self.chunk).unlink()
        self.assertEqual(len(source.list_source_review(self.root)),3);self.assert_absent()

    def test_cli_restores_both_without_legacy_assets(self):
        output=io.StringIO()
        with redirect_stdout(output):prepare_assets.main(['--source-review-only'],root=self.root)
        self.assertEqual(json.loads(output.getvalue())['files_restored'],3)

    def test_second_container_interruption_then_retry(self):
        original=source._publish
        def stop(root,row,data):
            if row['path']==self.path:raise KeyboardInterrupt()
            return original(root,row,data)
        with patch.object(source,'_publish',side_effect=stop):
            with self.assertRaises(KeyboardInterrupt):self.restore()
        self.assertTrue(all(p.exists() for p in self.base.targets()));self.assertFalse((self.root/self.path).exists())
        self.assertEqual(self.restore()['files_restored'],1)

    def test_extra_decoded_sha_failure_prevents_first_container_publication(self):
        self.extra['files'][0]['sha256']='0'*64;self.save()
        with self.assertRaisesRegex(ValueError,'restored identity'):self.restore()
        self.assert_absent()


if __name__=='__main__':unittest.main()
