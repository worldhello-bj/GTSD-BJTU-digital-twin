"""Bounded offline source-transport tests; no runtime or network execution."""
from contextlib import redirect_stdout
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch
import hashlib
import io
import json
import lzma
import os
import tempfile
import unittest
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import source_review_assets as source
import prepare_assets


def digest(data): return hashlib.sha256(data).hexdigest()


class SourceReviewRestoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.raws = [b'{"a": 1}\n', '{"b": "中文"}\n'.encode()]
        self.raw = b''.join(self.raws)
        self.manifest = {'schema':'gtsd-reviewed-source-bundle-v1',
            'encoding':'xz-concatenated-json-v1', 'max_chunk_bytes':300000,
            'file_count':2, 'raw_bytes':len(self.raw), 'raw_sha256':digest(self.raw),
            'files':[]}
        offset=0
        for index, data in enumerate(self.raws):
            self.manifest['files'].append({'path':f'integration/data/source_library/new_{index}.json',
                'offset':offset, 'bytes':len(data), 'sha256':digest(data)})
            offset+=len(data)
        self.pin = patch.object(source, 'MANIFEST_SHA256', '')
        self.pin.start()
        self.addCleanup(self.pin.stop)
        self.addCleanup(self.temp.cleanup)
        self.pack(self.raw)

    def save(self, repin=True):
        data=(json.dumps(self.manifest,sort_keys=True)+'\n').encode()
        (self.root/source.MANIFEST).write_bytes(data)
        if repin:source.MANIFEST_SHA256=digest(data)

    def pack(self, raw, suffix=b'', preset=6):
        packed=lzma.compress(raw,format=lzma.FORMAT_XZ,check=lzma.CHECK_SHA256,preset=preset)+suffix
        self.manifest.update(packed_bytes=len(packed),packed_sha256=digest(packed))
        self.manifest['chunks']=[]
        for index,start in enumerate(range(0,len(packed),300000)):
            data=packed[start:start+300000]
            name=f'payloads/source-review-v1/{digest(packed)[:20]}.{index:03d}.bin'
            path=self.root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
            self.manifest['chunks'].append({'path':name,'bytes':len(data),'sha256':digest(data)})
        self.save()

    def targets(self): return [self.root/row['path'] for row in self.manifest['files']]
    def assert_absent(self):
        self.assertTrue(all(not p.exists() for p in self.targets()))
        self.assertFalse(list(self.root.rglob('.gtsd-source-review-*')))
    def restore(self, **kwargs): return source.ensure_source_review(self.root, **kwargs)
    def invalid(self):
        self.save()
        with self.assertRaises((ValueError,OSError,lzma.LZMAError)):
            self.restore()
        self.assert_absent()

    def test_exact_restore_and_idempotent_check(self):
        self.assertEqual(self.restore()['files_restored'],2)
        before=[(p.read_bytes(),p.stat().st_mtime_ns) for p in self.targets()]
        self.assertEqual([x[0] for x in before],self.raws)
        self.assertEqual(self.restore()['files_restored'],0)
        self.assertEqual(self.restore(check_only=True)['files_restored'],0)
        self.assertEqual(before,[(p.read_bytes(),p.stat().st_mtime_ns) for p in self.targets()])

    def test_readonly_missing_never_writes(self):
        with self.assertRaises(FileNotFoundError):self.restore(check_only=True)
        self.assert_absent()
        self.assertFalse((self.root/'integration').exists())

    def test_list_does_not_read_payload_or_create_files(self):
        for row in self.manifest['chunks']:(self.root/row['path']).unlink()
        self.assertEqual(source.list_source_review(self.root),[row['path'] for row in self.manifest['files']])
        self.assert_absent()

    def test_manifest_tamper_fails_pinned_identity(self):
        self.manifest['files'][0]['sha256']='0'*64
        self.save(repin=False)
        with self.assertRaisesRegex(ValueError,'Pinned'):self.restore()
        self.assert_absent()

    def test_duplicate_json_keys_rejected_even_if_pinned(self):
        data=(self.root/source.MANIFEST).read_bytes().replace(b'"file_count": 2', b'"file_count": 2, "file_count": 2')
        (self.root/source.MANIFEST).write_bytes(data);source.MANIFEST_SHA256=digest(data)
        with self.assertRaisesRegex(ValueError,'Duplicate'):self.restore()
        self.assert_absent()

    def test_duplicate_target_rejected(self):
        self.manifest['files'][1]['path']=self.manifest['files'][0]['path'];self.invalid()

    def test_unsafe_target_and_nonwhitelist_paths_rejected(self):
        original=self.manifest['files'][0]['path']
        for path in ['../escape.json','/tmp/escape.json','integration/data/source_library/../x.json',
                     'integration/data/source_library/catalog.json','integration/data/source_library/new.json/child',
                     'integration/data/source_library/a\\b.json','README.md','integration/data/source_library/a:stream.json']:
            with self.subTest(path=path):
                self.manifest['files'][0]['path']=path;self.save()
                with self.assertRaises(ValueError):self.restore()
        self.manifest['files'][0]['path']=original;self.assert_absent()

    def test_hole_overlap_and_negative_offsets_rejected(self):
        for offset in [-1,0,1,1000000,True]:
            with self.subTest(offset=offset):
                self.manifest['files'][1]['offset']=offset;self.invalid()

    def test_out_of_bounds_last_file_rejected(self):
        self.manifest['files'][-1]['bytes']+=1;self.invalid()

    def test_uncovered_tail_rejected(self):
        self.manifest['raw_bytes']+=1;self.invalid()

    def test_declared_limits_rejected(self):
        for key,value in [('raw_bytes',source.MAX_RAW_BYTES+1),('packed_bytes',source.MAX_PACKED_BYTES+1),('file_count',True)]:
            old=self.manifest[key];self.manifest[key]=value
            with self.subTest(key=key):self.invalid()
            self.manifest[key]=old

    def test_oversized_file_limit_rejected(self):
        self.manifest['files'][0]['bytes']=source.MAX_FILE_BYTES+1;self.invalid()

    def test_unsafe_chunk_path_rejected(self):
        self.manifest['chunks'][0]['path']='../payload';self.invalid()

    def test_duplicate_chunk_rejected(self):
        self.manifest['chunks'].append(deepcopy(self.manifest['chunks'][0]));self.invalid()

    def test_chunk_byte_limit_rejected(self):
        self.manifest['chunks'][0]['bytes']=300001;self.invalid()

    def test_missing_chunk_fails_without_writes(self):
        (self.root/self.manifest['chunks'][0]['path']).unlink()
        with self.assertRaises(FileNotFoundError):self.restore()
        self.assert_absent()

    def test_tampered_chunk_rejected_even_if_targets_exist(self):
        self.restore()
        chunk=self.root/self.manifest['chunks'][0]['path'];data=chunk.read_bytes()
        chunk.write_bytes(bytes([data[0]^1])+data[1:])
        with self.assertRaisesRegex(ValueError,'chunk identity'):self.restore(check_only=True)
        self.assertEqual([p.read_bytes() for p in self.targets()],self.raws)

    def test_packed_sha_mismatch_rejected(self):
        # Valid shape uses the declared whole hash in the chunk name.
        old=self.manifest['chunks'][0]['path'];self.manifest['packed_sha256']='0'*64
        new='payloads/source-review-v1/'+('0'*20)+'.000.bin'
        (self.root/old).rename(self.root/new);self.manifest['chunks'][0]['path']=new
        self.invalid()

    def test_raw_sha_mismatch_rejected_before_any_writes(self):
        self.manifest['raw_sha256']='0'*64;self.invalid()

    def test_last_file_sha_mismatch_rejects_first_file_too(self):
        self.manifest['files'][-1]['sha256']='0'*64;self.invalid()

    def test_bounded_expansion_rejected(self):
        self.pack(self.raw+b' ' * 1000000)
        with self.assertRaisesRegex(ValueError,'decoded length'):self.restore()
        self.assert_absent()

    def test_truncated_xz_rejected(self):
        row=self.manifest['chunks'][0];p=self.root/row['path'];packed=p.read_bytes()[:-10]
        self.manifest['packed_bytes']=len(packed);self.manifest['packed_sha256']=digest(packed)
        name=f'payloads/source-review-v1/{digest(packed)[:20]}.000.bin';(self.root/name).write_bytes(packed)
        self.manifest['chunks']=[{'path':name,'bytes':len(packed),'sha256':digest(packed)}];self.invalid()

    def test_trailing_bytes_and_concatenated_stream_rejected(self):
        for suffix in [b'junk',lzma.compress(b'{}')]:
            with self.subTest(suffix=suffix[:8]):
                self.pack(self.raw,suffix=suffix)
                with self.assertRaisesRegex(ValueError,'trailing'):self.restore()
                self.assert_absent()

    def test_decoder_memory_limit_rejected(self):
        self.pack(self.raw,preset=9)
        with self.assertRaises(lzma.LZMAError):self.restore()
        self.assert_absent()

    def test_invalid_utf8_or_json_never_published(self):
        for data in [b'\xff'*len(self.raw),b'x'*len(self.raw)]:
            self.manifest['raw_sha256']=digest(data)
            for row in self.manifest['files']:
                row['sha256']=digest(data[row['offset']:row['offset']+row['bytes']])
            self.pack(data)
            with self.assertRaises((UnicodeError,ValueError)):self.restore()
            self.assert_absent()

    def test_modified_last_target_preserved_before_first_publish(self):
        target=self.targets()[-1];target.parent.mkdir(parents=True);target.write_bytes(b'custom')
        with self.assertRaises(ValueError):self.restore()
        self.assertEqual(target.read_bytes(),b'custom');self.assertFalse(self.targets()[0].exists())

    def test_target_symlink_rejected(self):
        target=self.targets()[0];target.parent.mkdir(parents=True)
        other=self.root/'other';other.write_bytes(self.raws[0]);target.symlink_to(other)
        with self.assertRaises(OSError):self.restore()
        self.assertTrue(target.is_symlink());self.assertEqual(other.read_bytes(),self.raws[0])

    def test_directory_symlink_rejected_inside_and_outside_root(self):
        for destination in [self.root/'inside',self.root.parent]:
            destination.mkdir(exist_ok=True)
            link=self.root/'integration';link.symlink_to(destination,target_is_directory=True)
            with self.assertRaises(OSError):self.restore()
            link.unlink()
        self.assert_absent()

    def test_chunk_symlink_rejected(self):
        chunk=self.root/self.manifest['chunks'][0]['path'];other=self.root/'chunk-data';chunk.rename(other);chunk.symlink_to(other)
        with self.assertRaises(OSError):self.restore()
        self.assert_absent()

    def test_manifest_symlink_rejected(self):
        manifest=self.root/source.MANIFEST;other=self.root/'manifest-data';manifest.rename(other);manifest.symlink_to(other)
        with self.assertRaises(OSError):self.restore()
        self.assert_absent()

    def test_nonregular_target_rejected(self):
        target=self.targets()[0];target.mkdir(parents=True)
        with self.assertRaises(ValueError):self.restore()
        self.assertTrue(target.is_dir())

    def test_atomic_publish_competing_target_never_overwritten(self):
        original=os.link
        def competing(src,dst,**kwargs):
            fd=os.open(dst,os.O_WRONLY|os.O_CREAT|os.O_EXCL,dir_fd=kwargs['dst_dir_fd'])
            os.write(fd,b'concurrent user edit');os.close(fd)
            return original(src,dst,**kwargs)
        with patch.object(source.os,'link',side_effect=competing):
            with self.assertRaises(FileExistsError):self.restore()
        self.assertEqual(self.targets()[0].read_bytes(),b'concurrent user edit')
        self.assertFalse(list(self.root.rglob('.gtsd-source-review-*')))

    def test_hardlink_unavailable_fails_closed(self):
        with patch.object(source.os,'link',side_effect=OSError('hardlinks unavailable')):
            with self.assertRaisesRegex(OSError,'hardlinks'):self.restore()
        self.assert_absent()

    def test_interrupted_second_file_then_retry(self):
        original=source._publish;calls=0
        def interrupted(*args):
            nonlocal calls
            calls+=1
            if calls==2:raise KeyboardInterrupt()
            return original(*args)
        with patch.object(source,'_publish',side_effect=interrupted):
            with self.assertRaises(KeyboardInterrupt):self.restore()
        before=self.targets()[0].stat().st_mtime_ns
        self.assertEqual(self.restore()['files_restored'],1)
        self.assertEqual(self.targets()[0].stat().st_mtime_ns,before)
        self.assertEqual([p.read_bytes() for p in self.targets()],self.raws)

    def test_interruption_during_publish_cleans_staging(self):
        with patch.object(source.os,'link',side_effect=KeyboardInterrupt()):
            with self.assertRaises(KeyboardInterrupt):self.restore()
        self.assert_absent()
        self.assertEqual(self.restore()['files_restored'],2)

    def test_stale_crash_staging_does_not_block_retry(self):
        directory=self.targets()[0].parent;directory.mkdir(parents=True)
        temporary=directory/'.gtsd-source-review-prior-crash';temporary.write_bytes(b'partial')
        self.assertEqual(self.restore()['files_restored'],2)
        self.assertEqual(temporary.read_bytes(),b'partial')

    def test_cli_source_only_works_without_legacy_manifest(self):
        for mode in [[],['--check-only']]:
            output=io.StringIO()
            with redirect_stdout(output):prepare_assets.main(['--source-review-only']+mode,root=self.root)
            self.assertEqual(json.loads(output.getvalue())['files_checked'],2)

    def test_cli_source_list_does_not_restore(self):
        output=io.StringIO()
        with redirect_stdout(output):prepare_assets.main(['--source-review-only','--list-assets'],root=self.root)
        self.assertEqual(len(json.loads(output.getvalue())['files']),2);self.assert_absent()

    def test_cli_source_and_legacy_selectors_conflict(self):
        with redirect_stdout(io.StringIO()),patch('sys.stderr',new=io.StringIO()):
            with self.assertRaises(SystemExit):prepare_assets.main(['--source-review-only','--only','x'],root=self.root)
        self.assert_absent()

    def test_default_automatically_restores_source_layer(self):
        (self.root/'ASSET_MANIFEST.json').write_text(json.dumps({'schema':'gtsd-public-chunked-assets-v1','assets':[]}))
        result=prepare_assets.ensure_assets(self.root)
        self.assertEqual(result['source_review']['files_restored'],2)
        self.assertEqual(result['assets_checked'],0)

    def test_known_release_cannot_hide_missing_source_manifest(self):
        raw=json.dumps({'schema':'gtsd-public-chunked-assets-v1','assets':[]}).encode()
        (self.root/'ASSET_MANIFEST.json').write_bytes(raw)
        (self.root/source.MANIFEST).unlink()
        with patch.object(source,'LEGACY_MANIFEST_SHA256',digest(raw)):
            with self.assertRaises(FileNotFoundError):prepare_assets.ensure_assets(self.root)
        self.assert_absent()

    def test_legacy_only_does_not_require_source_transport(self):
        (self.root/'ASSET_MANIFEST.json').write_text(json.dumps({'schema':'gtsd-public-chunked-assets-v1','assets':[]}))
        (self.root/source.MANIFEST).unlink()
        self.assertEqual(prepare_assets.ensure_assets(self.root),{'ok':True,'assets_checked':0,'assets_restored':0,'network_used':False})


if __name__=='__main__': unittest.main()
