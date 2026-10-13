"""Fast regression tests for source retention, topology and integration guards."""
import hashlib,json,unittest
from pathlib import Path
import numpy as np
from geometry import *
R=Path(__file__).resolve().parent
load=lambda n:json.loads((R/n).read_text())
class ClampProfiles(unittest.TestCase):
 def test_frozen_profiles(self):
  for line in (R/'PROFILE_SHA256SUMS').read_text().splitlines():
   expected,name=line.split();self.assertEqual(hashlib.sha256((R/name).read_bytes()).hexdigest(),expected)
 def test_three_exclusive_profiles(self):
  for fn,count in [('clamp_fit_profile.json',6),('clamp_messenger_datum_profile.json',7),('clamp_b535_review_profile.json',4)]:
   p=load(fn);self.assertEqual(len(p['remove_names']),count);self.assertEqual(len(p['add_meshes']),count);self.assertEqual(p['supported_scene_ids'],['20_FRONT_ROUTE_NOMINAL_FIT']);self.assertTrue(p['atomic_application']);self.assertEqual(len(set(p['remove_names'])),count)
 def test_main_wire_untouched(self):
  p=load('clamp_fit_profile.json');self.assertNotIn('ENV_messenger_wire',p['remove_names']);self.assertTrue(p['unresolved_messenger_groove_overlap_retained'])
 def test_each_new_mesh_oriented_watertight(self):
  for m in load('clamp_messenger_datum_profile.json')['add_meshes']:
   v=verts(m);f=faces(m);_,inv=np.unique(v,axis=0,return_inverse=True);f=inv[f];directed=np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]);edges={tuple(x)for x in directed};self.assertEqual(len(edges),len(directed));self.assertTrue(all((b,a)in edges for a,b in edges));self.assertTrue(topology(m)['positive_volume'])
 def test_retain_front_projection(self):
  old={m['name']:m for m in load('published_clamp_meshes.json')['meshes']};new={m['name']:m for m in load('clamp_fit_profile.json')['add_meshes']}
  for side in ['518DF','518E0']:
   self.assertTrue(np.array_equal(verts(old[f'CAD_{side}_4079_vertical_link'])[:,1:],verts(new[f'FIT_{side}_4079_DEPTH_UNRESOLVED_ROUND5_CANDIDATE'])[:,1:]))
 def test_cross_integral_translation_invariance(self):
  a=cylinder('a',0,[0,0,0],-.02,.02,.0025,64);b=cylinder('b',2,[0,0,0],-.01,.01,.0025,64);v=cross_extrusion_overlap(a,0,b,2)['volume_mm3'];self.assertGreater(v,0)
  for shift in [[4,-5,2],[-6,3,1]]:
   self.assertAlmostEqual(v,cross_extrusion_overlap(translate(a,shift),0,translate(b,shift),2)['volume_mm3'],places=7)
 def test_uncertainties_stay_unresolved(self):
  p=load('clamp_fit_profile.json');self.assertIsNone(p['configuration']['long_link']['actual_depth_mm']);self.assertIsNone(p['configuration']['long_link']['upper_fixing_method']);self.assertIsNone(p['configuration']['lower_pin']['confirmed_bolt_identity']);self.assertIsNone(p['configuration']['lower_pin']['axial_retention_method'])
 def test_optional_endpoint_interference_not_hidden(self):
  d=load('clamp_fit_validation.json');self.assertTrue(d['all_tests_passed']);self.assertEqual(d['profile_mesh_counts'],{'main':6,'optional_messenger_datum':7,'b535_review':4});sh=[q for q in d['connection_interfaces']if 'shackle' in q['interface']];self.assertEqual(len(sh),2)
  for q in sh:self.assertGreater(q['optional_messenger_datum_after']['volume_mm3'],q['before']['volume_mm3']);self.assertEqual(q['after']['volume_mm3'],q['before']['volume_mm3'])
 def test_profile_no_machine_paths_or_physics(self):
  for fn in ['clamp_fit_profile.json','clamp_messenger_datum_profile.json','clamp_b535_review_profile.json']:
   s=(R/fn).read_text();self.assertNotIn(chr(47)+'workspace'+chr(47),s);p=json.loads(s);self.assertFalse(p['as_built_confirmed']);self.assertFalse(p['physics_new_collision']);self.assertEqual(p['physics_added_mass_kg'],0)
if __name__=='__main__':unittest.main(verbosity=2)
