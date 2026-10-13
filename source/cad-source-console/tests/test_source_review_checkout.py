"""Read-only checkout classification for optional reviewed-source transport."""
from pathlib import Path
from unittest.mock import patch
import hashlib
import importlib.util
import json
import lzma
import tempfile
import unittest
import source_review_assets


def digest(data): return hashlib.sha256(data).hexdigest()


class SourceReviewCheckoutTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        path=Path(__file__).with_name('test_public_checkout_integrity.py')
        spec=importlib.util.spec_from_file_location('source_review_checkout_test_gate',path)
        self.gate=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.gate)
        self.gate.CONSOLE=self.root
        self.gate.REQUIRE_COMPLETE=False
        raw=b'{"new":true}\n';packed=lzma.compress(raw,preset=6,check=lzma.CHECK_SHA256)
        self.target='integration/data/source_library/new_fixture.json'
        self.chunk=f'payloads/source-review-v1/{digest(packed)[:20]}.000.bin'
        self.review={'schema':'gtsd-reviewed-source-bundle-v1','encoding':'xz-concatenated-json-v1',
            'max_chunk_bytes':300000,'file_count':1,'raw_bytes':len(raw),'raw_sha256':digest(raw),
            'packed_bytes':len(packed),'packed_sha256':digest(packed),
            'files':[{'path':self.target,'offset':0,'bytes':len(raw),'sha256':digest(raw)}],
            'chunks':[{'path':self.chunk,'bytes':len(packed),'sha256':digest(packed)}]}
        self.write(self.chunk,packed)
        self.assets={'schema':'gtsd-public-chunked-assets-v1','max_chunk_bytes':4194304,'assets':[]}
        self.write('ASSET_MANIFEST.json',json.dumps(self.assets).encode())
        self.write('SOURCE_REVIEW_MANIFEST.json',json.dumps(self.review).encode())
        helper=Path(source_review_assets.__file__).read_text()
        helper=helper.replace(source_review_assets.MANIFEST_SHA256,digest((self.root/'SOURCE_REVIEW_MANIFEST.json').read_bytes()))
        self.write('source_review_assets.py',helper.encode())
        self.public={'schema':'gtsd-public-console-manifest-v1','files':{}}
        for path in self.root.rglob('*'):
            if path.is_file():
                data=path.read_bytes();self.public['files'][path.relative_to(self.root).as_posix()]={'bytes':len(data),'sha256':digest(data)}
        self.public['files'][self.target]={'bytes':len(raw),'sha256':digest(raw)}
        self.save_public()
        self.case=self.gate.PublicCheckoutIntegrityTests()

    def write(self,name,data):
        path=self.root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    def save_public(self):
        self.write('PUBLIC_MANIFEST.json',json.dumps(self.public).encode())
        for fn in (self.gate.public_manifest,self.gate.asset_manifest,self.gate.source_review_manifest):fn.cache_clear()

    def test_packed_checkout_missing_decoded_targets_is_classified(self):
        state=self.gate.availability()
        self.assertEqual(state['missing_source_files'],[])
        self.assertEqual(state['reviewed_source_files'],1)
        self.assertEqual(state['unrestored_source_review_targets'],[self.target])
        self.case.test_missing_source_files_are_not_hidden_as_binary_transport()
        self.case.test_binary_transport_completeness_is_explicit()
        self.assertFalse((self.root/self.target).exists())

    def test_optional_layer_does_not_decode_or_restore(self):
        with patch.object(lzma,'LZMADecompressor',side_effect=AssertionError('must not decode')):
            self.case.test_present_chunks_match_asset_manifest()
            self.case.test_missing_source_files_are_not_hidden_as_binary_transport()
        self.assertFalse((self.root/self.target).exists())

    def test_missing_new_chunk_is_an_explicit_transport_blocker(self):
        (self.root/self.chunk).unlink()
        state=self.gate.availability()
        self.assertEqual(state['missing_chunks'],[self.chunk])
        self.assertEqual(state['missing_source_review_chunks'],[self.chunk])
        self.assertEqual(state['missing_source_files'],[])
        with self.assertRaises(unittest.SkipTest):self.case.test_binary_transport_completeness_is_explicit()
        self.gate.REQUIRE_COMPLETE=True
        with self.assertRaises(AssertionError):self.case.test_binary_transport_completeness_is_explicit()

    def test_corrupt_new_chunk_fails_readonly_identity_gate(self):
        self.write(self.chunk,b'corrupt')
        with self.assertRaises(AssertionError):self.case.test_present_chunks_match_asset_manifest()

    def test_corrupt_restored_target_fails_public_identity_gate(self):
        self.write(self.target,b'custom')
        with self.assertRaises(AssertionError):self.case.test_present_public_files_match_exact_bytes()
        self.assertEqual((self.root/self.target).read_bytes(),b'custom')

    def test_tampered_new_manifest_fails_pinned_gate(self):
        self.review['files'][0]['sha256']='0'*64
        self.write('SOURCE_REVIEW_MANIFEST.json',json.dumps(self.review).encode())
        with self.assertRaisesRegex(AssertionError,'Pinned'):self.gate.availability()

    def test_public_and_source_index_cannot_disagree(self):
        self.public['files'][self.target]['sha256']='0'*64;self.save_public()
        with self.assertRaisesRegex(AssertionError,'public identity'):self.gate.availability()

    def test_missing_source_manifest_remains_visible_as_missing_metadata(self):
        (self.root/'SOURCE_REVIEW_MANIFEST.json').unlink()
        state=self.gate.availability()
        self.assertIn('SOURCE_REVIEW_MANIFEST.json',state['missing_source_files'])
        with self.assertRaises(AssertionError):self.case.test_missing_source_files_are_not_hidden_as_binary_transport()

    def test_absent_extension_preserves_legacy_availability_shape(self):
        (self.root/'SOURCE_REVIEW_MANIFEST.json').unlink()
        self.public['files']={};self.save_public()
        self.assertEqual(self.gate.availability(),{'planned_assets':0,'present_chunksets':0,
            'incomplete_assets':[],'missing_chunks':[],'missing_static_binary':[],
            'missing_source_files':[],'unrestored_asset_targets':[],
            'note':'Chunk presence alone is not decoded-asset or runtime acceptance.'})

    def test_absent_extension_does_not_hide_missing_ordinary_source(self):
        (self.root/'SOURCE_REVIEW_MANIFEST.json').unlink()
        self.public['files']={'main.py':{'bytes':2,'sha256':digest(b'hi')}};self.save_public()
        self.assertEqual(self.gate.availability()['missing_source_files'],['main.py'])
        with self.assertRaises(AssertionError):self.case.test_missing_source_files_are_not_hidden_as_binary_transport()


if __name__=='__main__':unittest.main()
