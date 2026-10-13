"""Fresh-checkout tests: python test_portable_rebuild.py --project-root ROOT."""
import argparse,copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
import portable_rebuild as p
import fit_profile as api
import mesh_geometry as g
H=Path(__file__).resolve().parent
class PortableTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.inputs=p.read(H/'rebuild_inputs.json');cls.profiles=[p.read(H/f'profile_{m}.json')for m in ['conforming_hull','flat_top_blank']];cls.first=cls.profiles[0];cls.sha=cls.first['requirements']['source_replacements_sha256'];cls.scene={n:{'name':n}for n in cls.first['remove_names']};cls.scene['retained']={'name':'retained'}
 def test_metadata_and_group_completeness(self):
  for profile in self.profiles:
   self.assertTrue(api.validate_profile(profile,self.sha));self.assertEqual(len(profile['remove_names']),13);self.assertEqual(len(profile['add_meshes']),9);self.assertTrue(set(sum(self.inputs['groups'].values(),[])).issubset(profile['source_baseline_remove_names']))
 def test_atomic_apply_and_original_unchanged(self):
  before=dict(self.scene);after=api.compose_fit_scene(self.scene,self.first,self.sha);self.assertEqual(self.scene,before);self.assertEqual(len(after),10)
 def test_missing_member_and_wrong_revision_rejected(self):
  bad=dict(self.scene);del bad[self.first['remove_names'][0]]
  with self.assertRaises(ValueError):api.compose_fit_scene(bad,self.first,self.sha)
  with self.assertRaises(ValueError):api.compose_fit_scene(self.scene,self.first,'wrong')
 def test_repeated_apply_and_option_stacking_rejected(self):
  after=api.compose_fit_scene(self.scene,self.first,self.sha)
  with self.assertRaises(ValueError):api.compose_fit_scene(after,self.first,self.sha)
  with self.assertRaises(ValueError):api.compose_fit_scene(after,self.profiles[1],self.sha)
 def test_collision_or_default_promotion_rejected(self):
  a=copy.deepcopy(self.first);a['add_meshes'][0]['collision_enabled']=True
  with self.assertRaises(ValueError):api.validate_profile(a,self.sha)
  a=copy.deepcopy(self.first);a['default_enabled']=True
  with self.assertRaises(ValueError):api.validate_profile(a,self.sha)
 def test_malformed_geometry_rejected(self):
  for val in [float('nan'),float('inf')]:
   a=copy.deepcopy(self.first);a['add_meshes'][0]['vertices'][0]=val
   with self.assertRaises(ValueError):api.validate_profile(a,self.sha)
  a=copy.deepcopy(self.first);a['add_meshes'][0]['indices'][0]=10**9
  with self.assertRaises(ValueError):api.validate_profile(a,self.sha)
 def test_all_solids_closed_positive(self):
  for profile in self.profiles:
   for m in profile['add_meshes']:
    v=np.array(m['vertices']).reshape(-1,3);f=np.array(m['indices']).reshape(-1,3);self.assertTrue(g.closed(v,f));self.assertGreater(g.volume(v,f),0)
 def test_public_only_rebuild_bit_exact_vertices_and_indices(self):
  with tempfile.TemporaryDirectory(prefix='gtsd-track-fit-')as temp:
   reports=p.rebuild(PROJECT_ROOT,Path(temp));self.assertEqual(len(reports),2)
   for mode,original in zip(['conforming_hull','flat_top_blank'],self.profiles):
    rebuilt=p.read(Path(temp)/f'profile_{mode}.json');self.assertEqual(original['remove_names'],rebuilt['remove_names'])
    for a,b in zip(original['add_meshes'],rebuilt['add_meshes']):self.assertEqual(a['vertices'],b['vertices']);self.assertEqual(a['indices'],b['indices'])
   self.assertTrue(all(r['pass']and r['source_default_mesh_count']==1936 and r['after_mesh_count']==1932 for r in reports))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--project-root',type=Path,required=True);args,rest=ap.parse_known_args();PROJECT_ROOT=args.project_root;unittest.main(argv=['test_portable_rebuild.py']+rest,verbosity=2)
