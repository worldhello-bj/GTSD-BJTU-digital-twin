import copy
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import console_launcher as L
import truth_overlay as T
import phase6_server as S

CONFIG = S.load_config(ROOT/'phase6_config.json')
EVIDENCE = Path(CONFIG['evidence_root'])
SOURCE = Path(CONFIG['source_root'])
ENCODER = Path(CONFIG['encoder_candidate_root']) if CONFIG.get('encoder_candidate_root') else None


def fixture_matrix():
    rows=[]
    targets=["DTB-C0020", "DTB-C0073", "DTB-C0074", "DTB-C0013", "DTB-C0067"]
    for i in range(109):
        rows.append({"id": targets[i] if i<len(targets) else f"TEST-{i}", "name": "test",
                     "fields": {key: {"status": "MISSING", "truth": None} for key in T.FIELDS},
                     "identity_status": "CANDIDATE_IDENTITY", "model_mapping": {"geometry_status_from_registry": "HAS_DIRECT_GEOMETRY"},
                     "as_built_1to1_verified": False})
    return {"schema": T.FILES['matrix'][1], "source_geometry_sha256": "a"*64, "rows": rows}


class TruthTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix="truth-overlay-")
        self.root=Path(self.temp.name)
        for key,(filename,schema) in T.FILES.items():
            value=fixture_matrix() if key=='matrix' else {"schema":schema, {"proposals":"proposals","constraints":"constraints","findings":"findings"}[key]:[]}
            (self.root/filename).write_text(json.dumps(value))
        self.store=T.EvidenceStore(self.root)

    def tearDown(self):self.temp.cleanup()

    def test_109_by_7_without_mesh_coverage_percentage(self):
        result=self.store.read('a'*64)
        self.assertEqual(result['counts']['field_entries'],763)
        self.assertIsNone(result['counts']['as_built_coverage_percentage'])
        self.assertIs(result['geometry_identity_match'],True)
        self.assertIs(result['physical_model_changed'],False)
        self.assertIs(result['checkpoint_identity_changed'],False)

    def test_distinct_identity_mapping_connection_and_parameter_fields(self):
        row=self.store.read()['rows'][0]
        self.assertEqual(row['model_mapping']['geometry_status_from_registry'],'HAS_DIRECT_GEOMETRY')
        self.assertEqual(row['display_identity_status'],'CANDIDATE_IDENTITY')
        self.assertEqual(row['fields']['connection']['status'],'MISSING')
        self.assertEqual(row['fields']['mass']['status'],'MISSING')
        self.assertIn('不证明',row['semantic_guards'][0]['text'])

    def test_encoder_speed_candidate_has_no_angle_or_decoder_mapping(self):
        for row in self.store.read()['rows'][1:3]:
            guard=row['semantic_guards'][0]
            self.assertEqual(guard['observed_label'],'3000RPM')
            self.assertIs(guard['angle_conversion_available'],False)
            self.assertIs(guard['hardware_decoder_available'],False)
            self.assertIsNone(guard['calibrated_range_rpm'])
            self.assertIsNone(guard['physical_axle_binding'])

    def test_center_distance_example_and_radii_have_explicit_semantics(self):
        for row in self.store.read()['rows'][3:5]:
            text=row['semantic_guards'][0]['text']
            for term in ('517.833','轨轮廓中心距','478.333','39.5','算例','162.25/152.35','半径','不是直径'):
                self.assertIn(term,text)

    def test_revision_changes_without_touching_physics(self):
        before=self.store.read()
        path=self.root/'independent_findings.json'
        value=json.loads(path.read_text());value['findings'].append({'id':'new','text':'new evidence'})
        path.write_text(json.dumps(value))
        after=self.store.read()
        self.assertNotEqual(before['evidence_revision_sha256'],after['evidence_revision_sha256'])
        self.assertEqual(before['source_geometry_sha256'],after['source_geometry_sha256'])
        self.assertFalse(after['checkpoint_identity_changed'])

    def test_export_is_detached(self):
        first=self.store.read();first['rows'][0]['fields']['mass']['status']='tampered'
        self.assertEqual(self.store.read()['rows'][0]['fields']['mass']['status'],'MISSING')

    def test_geometry_mismatch_and_absent_solver_are_distinct(self):
        self.assertIsNone(self.store.read()['geometry_identity_match'])
        self.assertIs(self.store.read('b'*64)['geometry_identity_match'],False)

    def test_wrong_counts_fields_duplicate_ids_and_schema_rejected(self):
        for mutate,code in [(lambda m:m['rows'].pop(),'truth_rows'),
                            (lambda m:m['rows'][0]['fields'].pop('mass'),'truth_fields'),
                            (lambda m:m['rows'][1].update(id=m['rows'][0]['id']),'truth_ids'),
                            (lambda m:m.update(schema='other'),'truth_schema')]:
            value=fixture_matrix();mutate(value);(self.root/'truth_matrix_109.json').write_text(json.dumps(value))
            with self.assertRaises(L.ConfigError) as caught:self.store.read()
            self.assertEqual(caught.exception.code,code)

    def test_missing_file_and_invalid_json_fail_instead_of_zero_truth(self):
        path=self.root/'independent_findings.json';path.write_text('{')
        with self.assertRaises(L.ConfigError):self.store.read()
        path.unlink()
        with self.assertRaises(L.ConfigError) as caught:self.store.read()
        self.assertEqual(caught.exception.code,'truth_source')

    @unittest.skipUnless(EVIDENCE.exists(),'independent evidence directory is not present')
    def test_actual_independent_evidence_matrix(self):
        result=T.EvidenceStore(EVIDENCE).read()
        self.assertEqual(len(result['rows']),109)
        self.assertEqual(sum(result['counts']['field_status_counts']['mass'].values()),109)
        proposal_ids={row['id'] for row in result['difference_proposals']}
        self.assertTrue({'DIFF-C0020','DIFF-GAUGE-LABEL','DIFF-ENCODER-SIGNAL-SEMANTICS'} <= proposal_ids)
        self.assertTrue(all(not row['as_built_1to1_verified'] for row in result['rows']))


class AuxiliaryNamedTests(unittest.TestCase):
    def plant(self):
        return {'variant':'mounted_auxiliary','steps':0,'time_s':0,'qpos':[999]*91,'qvel':[999]*79,
                'capabilities':{'integrated_dof':79},
                'telemetry':{'auxiliary_mount':{'state_owner':'MUJOCO_QPOS_QVEL_ONLY','native':{
                 'angle_rad':{'compressor_rotor':0,'compressor_fan_rotor':1,'bay_fan_rotor':2},
                 'omega_rad_s':{'compressor_rotor':3,'compressor_fan_rotor':4,'bay_fan_rotor':5},
                 'piston_displacement_m':0,'piston_velocity_m_s':6},'last_sensor':{}}}}

    def test_named_zero_values_not_numeric_layout(self):
        p=self.plant();before=copy.deepcopy(p);result=T.auxiliary_observations(p)
        self.assertEqual(p,before);self.assertEqual(len(result['observations']),8)
        self.assertTrue(all(row['available'] for row in result['observations']))
        self.assertTrue(all(row['coordinate_index_used'] is None for row in result['observations']))
        self.assertEqual(result['observations'][0]['value'],0)
        self.assertNotIn(999,[r['value'] for r in result['observations']])

    def test_missing_nonfinite_boolean_or_wrong_variant_never_coerced(self):
        for value in (None,True,float('nan'),float('inf'),'0'):
            p=self.plant();p['telemetry']['auxiliary_mount']['native']['angle_rad']['compressor_rotor']=value
            row=T.auxiliary_observations(p)['observations'][0];self.assertFalse(row['available']);self.assertIsNone(row['value'])
        p=self.plant();p['variant']='old75'
        self.assertTrue(all(not row['available'] for row in T.auxiliary_observations(p)['observations']))

    def test_sensor_interval_zero_missing_and_detached(self):
        p=self.plant();p['telemetry']['auxiliary_mount']['last_sensor']={'compressor':{'force_local_N':[0,1,2]}}
        self.assertEqual(T.auxiliary_observations(p)['mount_sensor_records'],[])
        p['steps']=1;result=T.auxiliary_observations(p);result['mount_sensor_records'][0]['record']['force_local_N'][0]=999
        self.assertEqual(p['telemetry']['auxiliary_mount']['last_sensor']['compressor']['force_local_N'][0],0)

    @unittest.skipUnless(SOURCE.exists(),'frozen registry source not present')
    def test_old_coordinate_layout_guard_is_retained_at_79dof(self):
        spec=importlib.util.spec_from_file_location('gtsd_frozen_registry_test',SOURCE/'internal_registry/registry.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        registry=module.build_registry();p=self.plant();p['source_geometry_sha256']=registry['source_geometry_sha256'];p['steps']=1
        result=module.observe(p)
        expected={row['id'] for row in registry['signals'] if row.get('required_coordinate_lengths') and any(len(p.get(k,[]))!=n for k,n in row['required_coordinate_lengths'].items()) and not row.get('hardware_only')}
        observed={row['id']:row for row in result['observations']}
        self.assertTrue(expected)
        for key in expected:
            self.assertEqual(observed[key]['availability'],'COORDINATE_LAYOUT_MISMATCH')
            self.assertIsNone(observed[key]['value'])


class Phase6ConfigTests(unittest.TestCase):
    def test_default_stays_75_and_new_variant_is_explicit(self):
        config=S.load_config(ROOT/'phase6_config.json')
        self.assertEqual(config['variant'],'internal-coupled-research')
        self.assertIn('mounted-auxiliary-research',S.ALLOWED)

    def test_reuses_server_handler_and_pure_module_has_no_aux_import(self):
        tree=ast.parse(Path(T.__file__).read_text())
        modules=[node.module for node in ast.walk(tree) if isinstance(node,ast.ImportFrom)]
        modules += [alias.name for node in ast.walk(tree) if isinstance(node,ast.Import) for alias in node.names]
        self.assertFalse(any(name and (name.startswith('aux_mount') or name=='mujoco') for name in modules))
        self.assertNotIn('MountedAuxiliaryPlant',T.__dict__)


class DesignCandidateTests(unittest.TestCase):
    def test_catalog_cannot_enable_current_physics_binding(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'catalog.json'
            catalog=json.loads((ROOT/'design_candidates.json').read_text())
            catalog['candidates'][0]['current_solver_binding']=True
            path.write_text(json.dumps(catalog))
            with self.assertRaises(L.ConfigError) as caught:T.EvidenceStore(EVIDENCE,path).read()
            self.assertEqual(caught.exception.code,'candidate_auto_binding')

    def test_current_source_identity_and_new_design_are_separate(self):
        result=T.EvidenceStore(EVIDENCE,ROOT/'design_candidates.json').read('f'*64)
        self.assertEqual(result['current_running_geometry_identity'],'f'*64)
        self.assertFalse(result['geometry_identity_match'])
        self.assertTrue(all(row['current_solver_binding'] is False for row in result['independent_design_candidates']))
        body=next(row for row in result['independent_design_candidates'] if row['id']=='BODY-CAD-CROSS-SECTION-CANDIDATE')
        self.assertFalse(body['changes_mass_inertia_collision'])

    def test_frozen_encoder_is_default_off_and_independent(self):
        from candidate_review import EncoderReview
        result=EncoderReview(ENCODER).metadata()
        self.assertTrue(result['available']);self.assertFalse(result['current_solver_binding'])
        self.assertFalse(result['enabled_by_default']);self.assertFalse(result['physical_geometry_replaced'])
        self.assertEqual(result['mesh_count_for_asset_inventory_only'],10)
        self.assertFalse(result['contract']['analog_angle_mapping_allowed'])
        self.assertFalse(result['contract']['hardware_transport_allowed'])

    def test_optional_review_absence_does_not_create_automatic_asset(self):
        from candidate_review import EncoderReview
        self.assertFalse(EncoderReview(None).metadata()['available'])
        with self.assertRaises(L.ConfigError):EncoderReview(ENCODER).read_verified('../console/server.py')

    def test_unknown_dependency_manifest_hash_is_rejected(self):
        import dependency_admission as A
        with self.assertRaises(L.ConfigError) as caught:A.manifest('auxiliary',b'new unreviewed bytes')
        self.assertEqual(caught.exception.code,'unreviewed_dependency_hash')

    def test_portable_template_has_no_absolute_runtime_path_and_default_stays_75(self):
        template=json.loads((ROOT/'phase6_config.portable.json').read_text())
        self.assertEqual(template['variant'],'internal-coupled-research')
        self.assertTrue(all(not Path(value).is_absolute() for key,value in template.items() if key.endswith('_root')))

    def test_pantograph_is_local_xy_only_without_dynamics_or_auto_placement(self):
        from candidate_review import PantographReview
        value=PantographReview(CONFIG.get('pantograph_candidate_root')).metadata()
        self.assertTrue(value['available']);self.assertFalse(value['allow_dynamics'])
        self.assertIsNone(value['automatic_vehicle_placement'])
        self.assertEqual(value['contract']['hide_frozen_meshes'],[])
        self.assertEqual(value['cross_view_registration_status'],'CANDIDATE_CORRESPONDENCE_NOT_REGISTERED')

    def test_proxy_axes_use_declared_directions_and_are_orthonormal(self):
        import numpy as np
        from mounted_display import axis_frame
        for axis in ([1,0,0],[0,0,1],[0,1,0],[-1,2,3]):
            frame=axis_frame(axis);unit=np.asarray(axis)/np.linalg.norm(axis)
            np.testing.assert_allclose(frame[:,2],unit,atol=1e-14)
            np.testing.assert_allclose(frame.T@frame,np.eye(3),atol=1e-14)
            self.assertAlmostEqual(np.linalg.det(frame),1)
        for axis in ([0,0,0],[1,2],[float('nan'),0,1]):
            with self.assertRaises(ValueError):axis_frame(axis)


if __name__=='__main__':unittest.main(verbosity=2)
