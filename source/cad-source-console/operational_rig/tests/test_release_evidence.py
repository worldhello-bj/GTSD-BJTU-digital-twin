import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'verify_release_evidence.py'
spec = importlib.util.spec_from_file_location('operational_release_evidence', MODULE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReleaseEvidence(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.patcher = patch.object(module, 'ROOT', self.root)
        self.patcher.start()
        (self.root / 'web').mkdir()
        (self.root / 'web/app.js').write_text('test fixture')
        self.records = {
            'protocol': {'passed': True, 'checks': {'fixture_gate': True},
                         'coarse': {'identity_sha256': 'native', 'assembly_identity_sha256': 'assembly'}},
            'http': {'passed': True, 'checks': {'complete_http_demo_and_terminal_checkpoint_restore': True},
                     'complete_demo': {'steps': 10}, 'identity_sha256': 'native'},
            'ui': {'result': 'pass', 'checks': {'fixture_dom': True}, 'identity_sha256': 'native',
                   'http_evidence': {}, 'web_sha256': {'web/app.js': hashlib.sha256(b'test fixture').hexdigest()}},
            'render': {'runtime_identity_sha256': 'native', 'assembly_identity_sha256': 'assembly',
                       'native_reopened': True, 'all_indices_and_membership_verified': True,
                       'displacement_scale': 1., 'maximum_world_vertex_error_mm': .001, 'tolerance_mm': .002}}

    def tearDown(self):
        self.patcher.stop()
        self.tmp.cleanup()

    def verify(self):
        for key in ('protocol', 'http', 'render'):
            (self.root / (key + '.json')).write_text(json.dumps(self.records[key]))
        self.records['ui']['http_evidence']['report_sha256'] = hashlib.sha256((self.root / 'http.json').read_bytes()).hexdigest()
        (self.root / 'ui.json').write_text(json.dumps(self.records['ui']))
        return module.verify(*(self.root / (key + '.json') for key in ('protocol', 'http', 'ui', 'render', 'output')))

    def test_matching_evidence(self):
        result = self.verify()
        self.assertTrue(result['passed'])
        self.assertFalse(result['webgl_browser_visual_tested'])
        self.assertFalse(result['browser_download_completion_tested'])

    def test_failed_protocol_rejected(self):
        self.records['protocol']['passed'] = False
        with self.assertRaises(ValueError): self.verify()

    def test_a_failed_gate_rejected(self):
        self.records['protocol']['checks']['fixture_gate'] = False
        with self.assertRaises(ValueError): self.verify()

    def test_only_short_http_rejected(self):
        self.records['http']['complete_demo'] = None
        with self.assertRaises(ValueError): self.verify()

    def test_old_render_identity_rejected(self):
        self.records['render']['runtime_identity_sha256'] = 'old'
        with self.assertRaises(ValueError): self.verify()

    def test_scaled_motion_rejected(self):
        self.records['render']['displacement_scale'] = 10.
        with self.assertRaises(ValueError): self.verify()

    def test_changed_ui_rejected(self):
        (self.root / 'web/app.js').write_text('changed fixture')
        with self.assertRaises(ValueError): self.verify()


if __name__ == '__main__': unittest.main()
