from pathlib import Path
import sys,os,json,unittest,copy,hashlib,tempfile
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import fan82_server as F
from fan82_view import normalize_command_registry,panel_payload
CONFIG=F.load_config(os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'console82_config.json')

def byte(v):return json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()
def sha(v):return hashlib.sha256(byte(v)).hexdigest()
def fixture():
 old={'meshes':[{'id':'1','name':'retained','vertices':[[0,0,0]],'indices':[0],'physics_body_name':'parent'},{'id':'2','name':'shroud','vertices':[[0,1,0]],'indices':[0],'physics_body_name':'door'}]};new=copy.deepcopy(old);new['meshes'][1]['physics_body_name']='stator';spec={'added_mounts':{'key':{'source_shroud_mesh':'shroud','parent':'door','stator':'stator'}}};proof={'prior_geometry_sha256':sha(old),'geometry_sha256':sha(new),'retained_exact_objects':[{'mesh_name':'retained','prior_canonical_sha256':sha(old['meshes'][0]),'new_canonical_sha256':sha(new['meshes'][0])}],'rebound_only_objects':[{'mesh_name':'shroud','prior_canonical_sha256':sha(old['meshes'][1]),'new_canonical_sha256':sha(new['meshes'][1])}]};return old,new,proof,spec
class Tests(unittest.TestCase):
 def test_config_explicit(self):self.assertEqual(CONFIG['variant'],F.VARIANT)
 def test_default_not82(self):
  with tempfile.TemporaryDirectory(dir=ROOT/'tests') as d:
   p=Path(d)/'default.json';v=F.strict(Path(CONFIG['path']).read_bytes());source=Path(CONFIG['path']).parent
   for name in ('phase7_console_root','phase7_config','fan82_root'):v[name]=str((source/v[name]).resolve())
   v['variant']='internal-coupled-research';p.write_text(json.dumps(v));self.assertEqual(F.load_config(p)['variant'],'internal-coupled-research')
 def test_freeze_reviewed(self):self.assertEqual(F.verify_fan82(CONFIG)['nv'],82)
 def test_duplicate_json(self):
  with self.assertRaises(ValueError):F.strict('{"a":1,"a":2}')
 def test_nonfinite(self):
  for s in ('NaN','Infinity','1e999'):
   with self.assertRaises(ValueError):F.strict(s)
 def test_unknown_key(self):
  with tempfile.TemporaryDirectory(dir=ROOT/'tests') as d:
   p=Path(d)/'bad.json';v=F.strict((ROOT/'console82_config.json').read_bytes());v['hardware_transport']=True;p.write_text(json.dumps(v))
   with self.assertRaises(ValueError):F.load_config(p)
 def test_detached_schema(self):
  raw={'variant':F.ACTUAL,'command_registry':{'four_door_fan':{'args':{'key':{'type':'enum','values':['all','a']},'power_failed':{'type':'boolean'}}}},'nested':{'a':[0]}};before=copy.deepcopy(raw);shown=normalize_command_registry(raw);self.assertEqual(raw,before);self.assertEqual(shown['command_registry']['four_door_fan']['args']['key'],{'type':'string','enum':['all','a']});shown['nested']['a'][0]=2;self.assertEqual(raw,before)
 def test_unexpected_schema_reject(self):
  with self.assertRaises(ValueError):normalize_command_registry({'variant':F.ACTUAL,'command_registry':{'four_door_fan':{'args':{'key':{'type':'string','enum':['all']}}}}})
 def test_unicode_canonical(self):
  old,new,p,s=fixture();old['meshes'][0]['name']=new['meshes'][0]['name']='保留';p['retained_exact_objects'][0].update(mesh_name='保留',prior_canonical_sha256=sha(old['meshes'][0]),new_canonical_sha256=sha(new['meshes'][0]));p.update(prior_geometry_sha256=sha(old),geometry_sha256=sha(new));self.assertEqual(F.verify_rebind(byte(old),byte(new),p,s)['proof_rows'],1)
 def test_rebind_shape_unchanged(self):
  old,new,p,s=fixture();self.assertTrue(F.verify_rebind(byte(old),byte(new),p,s)['all_vertices_indices_exact_unchanged'])
 def test_rebind_shape_tamper_reject_even_resigned(self):
  old,new,p,s=fixture();new['meshes'][1]['vertices'][0][0]=1;p['geometry_sha256']=sha(new);p['rebound_only_objects'][0]['new_canonical_sha256']=sha(new['meshes'][1])
  with self.assertRaises(ValueError):F.verify_rebind(byte(old),byte(new),p,s)
 def test_rebind_extra_field_reject(self):
  old,new,p,s=fixture();new['meshes'][1]['fake']='changed';p['geometry_sha256']=sha(new);p['rebound_only_objects'][0]['new_canonical_sha256']=sha(new['meshes'][1])
  with self.assertRaises(ValueError):F.verify_rebind(byte(old),byte(new),p,s)
 def test_duplicate_proof_reject(self):
  old,new,p,s=fixture();p['retained_exact_objects']*=2
  with self.assertRaises(ValueError):F.verify_rebind(byte(old),byte(new),p,s)
 def test_wrong_native_identity_clears(self):
  s={'variant':F.ACTUAL,'model_identity_sha256':'wrong','source_geometry_sha256':'x','telemetry':{}};out=panel_payload(s,{'added_mounts':{}},'expected','x');self.assertFalse(out['identity_match']);self.assertTrue(all(r['value'] is None and not r['available'] for r in out['observations']))
if __name__=='__main__':unittest.main()
