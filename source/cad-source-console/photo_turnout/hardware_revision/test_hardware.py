import sys,json,gzip,unittest,hashlib,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;sys.path.insert(0,str(H.parent))
import cap_geometry as C
class Tests(unittest.TestCase):
 def test_meshes_and_datums(self):
  m=json.loads((H/'change_manifest.json').read_text());p=json.load(gzip.open(H/'candidate_meshes.json.gz','rt'));self.assertEqual(len(p['meshes']),111);self.assertEqual(len(m['mounting_datums']),12);self.assertEqual(len(m['remove']),33)
  for x in p['meshes']:self.assertTrue(C.topology(x)['closed']);self.assertTrue(C.topology(x)['positive_volume'])
  for d in m['mounting_datums']:self.assertEqual(d['rail_foot_envelope_section_z_mm'],-62.);self.assertIn(d['source_station'],range(6))
 def test_rail_core_unchanged(self):
  a={m['name']:m for m in json.load(gzip.open(H.parent/'build/candidate_meshes.json.gz','rt'))['meshes']};b={m['name']:m for m in json.load(gzip.open(H/'candidate_meshes.json.gz','rt'))['meshes']}
  for n in a:
   if n.startswith(('PHOTO_FROG','PHOTO_WING','PHOTO_RETAIN')):
    for k in ['vertices','indices']:self.assertEqual(a[n][k],b[n][k])
 def test_deterministic(self):
  paths=[H/'candidate_meshes.json.gz',H/'change_manifest.json'];before=[p.read_bytes()for p in paths];subprocess.run([sys.executable,str(H/'build_hardware.py')],check=True)
  self.assertEqual(before,[p.read_bytes()for p in paths])
 def test_audit_scope(self):
  a=json.loads((H/'material_audit.json').read_text());self.assertEqual(a['missing_from_exact_check'],[]);self.assertFalse(a['strict_whole_scene_clearance_pass']);self.assertTrue(a['no_solver_executed'])
if __name__=='__main__':unittest.main(verbosity=2)
