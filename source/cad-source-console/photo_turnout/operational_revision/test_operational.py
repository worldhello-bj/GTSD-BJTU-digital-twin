import gzip,json,sys,unittest
from pathlib import Path
import numpy as np
import warnings
warnings.filterwarnings("ignore", message="Arrays of 2-dimensional vectors are deprecated", category=DeprecationWarning)
H=Path(__file__).resolve().parent;sys.path.insert(0,str(H.parent));sys.path.insert(0,str(H))
import cap_geometry as C
from build_candidate import clip_mesh_plane
from build_operational import witness
class OperationalTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.profile=json.loads((H/'operational_source_profile.json').read_text());cls.p=cls.profile['profiles'][0];cls.mesh={m['name']:m for m in cls.p['groups'][0]['add_meshes']};cls.interface=json.loads((H/'mesh_interface.json').read_text());cls.cfg=json.loads((H/'parameters.json').read_text())
 def test_counts_and_source_identity(self):
  self.assertEqual(len(self.p['groups'][0]['remove']),67);self.assertEqual(len(self.mesh),141);self.assertEqual(self.interface['full_scene_mesh_count'],2006);self.assertEqual(len(self.interface['meshes']),2006);self.assertEqual(len(self.interface['geometry_changes']),3)
 def test_interface_witnesses(self):
  by={m['name']:m for m in self.interface['meshes']}
  for n,m in self.mesh.items():self.assertEqual(witness(m),by[n]['geometry_witness_sha256'])
 def test_complete_hardware_and_relations(self):
  regional=json.load(gzip.open(H/'regional_meshes.json.gz','rt'));by={m['name']:m for m in self.interface['meshes']}
  self.assertEqual(len(regional['meshes']),205)
  for m in regional['meshes']:self.assertEqual(witness(m),by[m['name']]['geometry_witness_sha256'])
  r=self.interface['mechanical_relations'];self.assertTrue(all(x['relation_scope']=='assembly_design' and x['current_solver_model']=='rigid_group_finite_mount' and x['independent_preload_or_pad_contact_solved'] is False for x in r));self.assertEqual(sum(x['joint_model']=='preloaded_bolt_nut_toe_clamp'for x in r),12);self.assertEqual(sum(x['joint_model']=='pad_compression_between_plate_and_sleeper'for x in r),6);self.assertEqual(sum(n.startswith('PHOTO_CLAMP')for n in self.mesh),96)
 def test_default_actual_gap_and_topology(self):
  n=np.array(self.cfg['joint']['miter_normal_world']);f=C.verts(self.mesh['PHOTO_FROG_source_finite_V'])@n
  for name in ['PHOTO_RETAIN_left_ENV_main_rail_1','PHOTO_RETAIN_left_ENV_main_rail_top_1']:
   self.assertAlmostEqual((f.min()-(C.verts(self.mesh[name])@n).max())*1000,.5,places=8)
  for m in self.mesh.values():self.assertTrue(C.topology(m)['closed']);self.assertGreater(C.signed_volume(m),0)
 def test_adjustable_joint_gaps(self):
  normal=np.array(self.cfg['joint']['miter_normal_world']);origin=np.array(self.cfg['joint']['reference_point_mm'])/1000
  refs={m['name']:m for m in json.load(gzip.open(H/'joint_reference_meshes.json.gz','rt'))['meshes']}
  for gap in [.25,.5,1.,2.]:
   out=[]
   for name,positive in [('PHOTO_FROG_source_finite_V',True),('PHOTO_RETAIN_left_ENV_main_rail_1',False),('PHOTO_RETAIN_left_ENV_main_rail_top_1',False)]:
    m=clip_mesh_plane(refs[name],normal,origin+normal*((gap-min(gap/2,.25))/1000 if positive else -min(gap/2,.25)/1000),positive,name);self.assertTrue(C.topology(m)['closed']);self.assertGreater(C.signed_volume(m),0);out.append(m)
   for tail in out[1:]:self.assertAlmostEqual(((C.verts(out[0])@normal).min()-(C.verts(tail)@normal).max())*1000,gap,places=8)
 def test_actual_material_audit(self):
  a=json.loads((H/'geometry_audit.json').read_text());self.assertTrue(a['pass']);self.assertEqual(a['positive_material_pairs'],[]);self.assertAlmostEqual(a['joint']['before']['volume_mm3'],.48824768531221413,places=6);self.assertEqual(a['joint']['after']['volume_mm3'],0.)
if __name__=='__main__':
 with warnings.catch_warnings():
  warnings.simplefilter('ignore',DeprecationWarning)
  unittest.main(verbosity=2,warnings="ignore")
