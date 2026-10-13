import copy,json,unittest
from fixture import registry,CONFIG
from lineage_registry import verify_stage,digest
import phase7_server as P
class RegistryTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.reg,cls.snap,cls.stage_args=registry()
 def test_all_1940_retained_and26_new_are_distinct(self):
  c=self.reg.catalogue();self.assertEqual(c['denominators']['retained_baseline_mesh_records'],1940);self.assertEqual(len(c['meshes']),1966);self.assertEqual(sum(m['lineage']=='NEW_REVIEWED_DESIGN_MESH' for m in c['meshes']),26);self.assertFalse(c['legacy_expected_hash_modified']);self.assertNotEqual(c['baseline_registry_source_geometry_sha256'],c['current_geometry_sha256'])
 def test_one_retained_vertex_mutation_rejected(self):
  args=list(self.stage_args);r=json.loads(args[1]);r['meshes'][0]['vertices'][0]+=.001;args[1]=json.dumps(r).encode();args[2]=copy.deepcopy(args[2]);import hashlib;args[2]['composed_geometry_sha256']=hashlib.sha256(args[1]).hexdigest()
  with self.assertRaisesRegex(ValueError,'content mismatch'):verify_stage(*args)
 def test_forged_count_rejected(self):
  args=list(self.stage_args);args[2]=copy.deepcopy(args[2]);args[2]['count']+=1
  with self.assertRaisesRegex(ValueError,'denominator'):verify_stage(*args)
 def test_replacement_omission_rejected(self):
  args=list(self.stage_args);args[3]=args[3][:-1]
  with self.assertRaises(ValueError):verify_stage(*args)
 def test_named_available_counts_no_legacy_array_relabel(self):
  o=self.reg.observe(self.snap);d=o['denominators']['legacy'];self.assertEqual(d['total'],1886);self.assertEqual(d['reasons']['UNMIGRATED_GENERALIZED_ARRAY_SEMANTICS'],687);self.assertEqual(d['reasons']['UNAVAILABLE_HARDWARE_UNBOUND'],31);self.assertGreater(d['available'],850);self.assertEqual(sum(d['reasons'].values()),1886)
 def test_available_pressure_current_and_native_pose(self):
  rows={r['id']:r for r in self.reg.observe(self.snap)['observations']}
  self.assertTrue(rows['telemetry.pneumatic.pressure_Pa.reservoir']['available']);self.assertTrue(rows['pose.payload_secondary.quaternion']['available']);self.assertTrue(any(r['available'] and r['unit']=='A' for r in rows.values()));self.assertTrue(any(r['available'] and 'compressor_rotor' in r['id'] and r['unit']=='rad/s' for r in rows.values()))
 def test_wrong_model_identity_unavailable(self):
  s=copy.deepcopy(self.snap);s['model_identity_sha256']='0'*64;o=self.reg.observe(s);self.assertFalse(o['identity_match']);self.assertFalse(any(r['available'] for r in o['observations']))
 def test_native_timestamp_and_unit_guard(self):
  s=copy.deepcopy(self.snap);s['ideal_observations']['sample_time_s']+=1;o=self.reg.observe(s);self.assertTrue(any(r['availability']=='NAMED_OBSERVATION_IDENTITY_OR_TIME_MISMATCH' for r in o['observations']))
  s=copy.deepcopy(self.snap);row=next(r for r in s['ideal_observations']['observations'] if r['signal_id']=='joint.wheel_spin_1.velocity');row['unit']='V';o=self.reg.observe(s);self.assertTrue(any(r['availability']=='NATIVE_UNIT_MISMATCH' for r in o['observations']))
 def test_zero_is_value_bool_is_invalid_pressure(self):
  s=copy.deepcopy(self.snap);s['telemetry']['pneumatic']['pressure_Pa']['reservoir']=0;r=next(r for r in self.reg.observe(s)['observations'] if r['id']=='telemetry.pneumatic.pressure_Pa.reservoir');self.assertTrue(r['available']);self.assertEqual(r['value'],0)
  s['telemetry']['pneumatic']['pressure_Pa']['reservoir']=False;r=next(r for r in self.reg.observe(s)['observations'] if r['id']=='telemetry.pneumatic.pressure_Pa.reservoir');self.assertFalse(r['available'])
 def test_detached_and_no_state_mutation(self):
  before=json.dumps(self.snap,sort_keys=True);o=self.reg.observe(self.snap);o['observations'][0]['value']='bad';c=self.reg.catalogue();c['meshes'][0]['name']='bad';self.assertEqual(json.dumps(self.snap,sort_keys=True),before);self.assertNotEqual(self.reg.meshes[0]['name'],'bad')
 def test_new_CAD_does_not_invent_109_identity(self):
  rows=[m for m in self.reg.meshes if m['lineage']=='NEW_REVIEWED_DESIGN_MESH'];self.assertTrue(all(m['registered_component_id'] is None and not m['physical_installation_verified'] for m in rows));self.assertTrue(all(m['source_refs'] for m in rows))
 def test_numerical_manifest_identity_cannot_be_resigned_to_unknown_sources(self):
  from pathlib import Path
  base=CONFIG['base'];freeze=P.phase6_module().verify_auxiliary_freeze(base['auxiliary_root']);body=json.loads((Path(base['auxiliary_root'])/'PHASE6_READONLY_BASELINE.json').read_text());body['source_sha256'][next(iter(body['source_sha256']))]='0'*64
  with self.assertRaisesRegex(ValueError,'manifest identity'):P.verify_baseline_bytes(freeze,base['source_root'],json.dumps(body))
 def test_nonfinite_config_and_duplicate_keys_rejected(self):
  for value in ('{"a":1e999}','{"a":0,"a":1}','{"a":NaN}'):
   with self.assertRaises(ValueError):P.strict(value)
 def test_readonly_app_bridge_no_solver_command_export(self):
  from pathlib import Path
  src=(Path(CONFIG['base']['source_root'])/'console/web/app.js').read_bytes();out=P.bridge_app(src).decode();bridge=out[out.index('window.GTSDLineageView='):out.index('controls();poll();\n})();')];self.assertNotIn('command(',bridge);self.assertIn('gtsd-lineage-selection',out);self.assertIn('pick:',bridge)
  import subprocess
  self.assertEqual(subprocess.run(['node','--check'],input=out.encode(),capture_output=True).returncode,0)
if __name__=='__main__':unittest.main()
