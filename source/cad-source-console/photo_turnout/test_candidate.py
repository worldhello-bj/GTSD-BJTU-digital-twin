import copy,gzip,hashlib,json,tempfile,unittest
from pathlib import Path
import numpy as np
import build_candidate as B
import cap_geometry as C
import geos_kernel as K
H=Path(__file__).resolve().parent

class CandidateTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.manifest=json.loads((H/'build/change_manifest.json').read_text());cls.payload=json.load(gzip.open(H/'build/candidate_meshes.json.gz','rt'));cls.mesh={m['name']:m for m in cls.payload['meshes']};cls.graph=json.loads((H/'native_edge_graph.json').read_text());cls.curves={r['handle']:np.array(r['points'])for r in cls.graph['source_curves']};cls.seed=json.load(gzip.open(H/'source_seed.json.gz','rt'));cls.source={m['name']:m for m in cls.seed['meshes']}
 def test_native_identity_and_finite_endpoint_chains(self):
  c=self.curves;self.assertEqual(len(self.graph['source_curves']),57)
  for h1,i,h2,j in [('4ECED',0,'4ECFE',0),('4ED5C',0,'4ECEE',1),('4ECEE',0,'4ECFA',1),('4ED5D',0,'4ED02',1),('4ED02',0,'4ECEC',1)]:self.assertLess(np.linalg.norm(c[h1][i]-c[h2][j]),1e-5)
  self.assertAlmostEqual(c['4ED17'][0,1]-c['4ECED'][0,1],39.5,7)
 def test_frog_points_right_and_stays_between_finite_faces(self):
  m=self.mesh['PHOTO_FROG_source_finite_V'];v=C.verts(m)*1000;v=v[np.abs(v[:,2])<1e-6];nose=self.curves['4ECED'][0];self.assertAlmostEqual(v[:,0].max(),nose[0],7);self.assertGreaterEqual(v[:,0].min(),self.curves['4ECED'][1,0]-1e-3)
  self.assertGreaterEqual(v[:,1].min(),nose[1]-1e-7)
  a,b=self.curves['4ECFE'];t=(b-a)/np.linalg.norm(b-a)
  cross=t[0]*(v[:,1]-a[1])-t[1]*(v[:,0]-a[0]);self.assertGreaterEqual(cross.min(),-1e-3)
 def test_all_output_closed_positive_and_unique(self):
  self.assertEqual(len(self.mesh),14)
  for m in self.mesh.values():
   t=C.topology(m);self.assertTrue(t['closed'],m['name']);self.assertTrue(t['positive_volume'],m['name']);self.assertGreater(t['min_area_mm2'],0,m['name'])
 def test_independent_rail_faces_have_zero_material_overlap(self):
  audit=json.loads((H/'build/material_audit.json').read_text())
  self.assertTrue(audit['all_candidate_pairs_material_separated'])
  for r in audit['core_pairs_full_triangle_projection_bound']:self.assertLess(r['volume_mm3_upper_bound'],1e-6)
 def test_inherited_I_section_retains_all_source_Z_events_and_widths(self):
  evidence=json.loads((H/'build/inherited_section_evidence.json').read_text())
  self.assertTrue(np.allclose(evidence['Z_events_mm'],[-65,-58,-52,-19,-13,0],atol=1e-5))
  for key,value in [('head_width_mm',39.5),('web_width_mm',12.),('foot_width_mm',73.)]:self.assertAlmostEqual(evidence[key],value,4)
  self.assertEqual(evidence['source_section_status'],'INHERITED_SOURCE_RENDER_I_SECTION_NOT_AS_BUILT_CONFIRMED')
 def test_parallel_slot_widths_are_source_association_not_photo_measurements(self):
  f=self.manifest['facts'];self.assertAlmostEqual(f['lower_slot_horizontal_mm'],46/3,8);self.assertAlmostEqual(f['upper_slot_perpendicular_mm'],46/3,8);self.assertEqual(f['photo_metric_dimensions'],[]);self.assertEqual(f['photo_to_cad_registration'],'REGION_LEVEL_CANDIDATE_NOT_CALIBRATED')
 def test_separate_stack_and_source_sleepers_unchanged(self):
  f=self.manifest['facts'];self.assertTrue(np.allclose(f['rail_z_mm'],[-65,0],atol=1e-5));self.assertEqual(f['long_plate_z_mm'],[-69,-65]);self.assertEqual(f['pad_z_mm'],[-72,-69]);self.assertEqual(len(self.manifest['separate_pads']),5)
  removed={r['name']for r in self.manifest['remove']};self.assertFalse(any('sleeper' in n for n in removed));self.assertFalse(any('fastener'in n or'clip'in n for n in removed))
 def test_remainder_vertices_and_far_sections_preserved(self):
  for m in self.mesh.values():
   if m['metadata']['role']!='unchanged_source_remainder':continue
   original=self.source[m['metadata']['source_name']];v=C.verts(m);sv=C.verts(original);p=np.asarray(m['metadata']['explicit_join_plane']['point_m']);n=np.asarray(m['metadata']['explicit_join_plane']['normal']);far=np.abs((v-p)@n)>1e-8
   s={tuple(q)for q in sv}
   self.assertTrue(all(tuple(q)in s for q in v[far]),m['name'])
  # Named whole source rail removals always have explicit retained remainder(s).
  names=[m['metadata'].get('source_name')for m in self.mesh.values()if m['metadata']['role']=='unchanged_source_remainder']
  for n in ['ENV_main_rail_1','ENV_main_rail_top_1','CAD_TRACK_REV_inner_right_native_xy']:self.assertIn(n,names)
 def test_deterministic_rebuild_byte_identity(self):
  with tempfile.TemporaryDirectory(dir=H)as td:
   out=Path(td);B.build(H/'config.json',out)
   for n in ['candidate_meshes.json.gz','change_manifest.json','plan_geometry.json']:self.assertEqual((out/n).read_bytes(),(H/'build'/n).read_bytes(),n)
 def test_configuration_changes_only_explicit_unknown_section(self):
  c=json.loads((H/'config.json').read_text());c['slot_depth_mm']=60;c['long_plate_thickness_mm']=5
  with tempfile.TemporaryDirectory(dir=H)as td:
   td=Path(td);p=td/'config.json';B.write(p,c);m,_=B.build(p,td/'out');self.assertEqual(m['facts']['nose_world_mm'],self.manifest['facts']['nose_world_mm']);self.assertTrue(np.allclose(m['facts']['rail_z_mm'],[-65,0],atol=1e-5));self.assertEqual(m['facts']['long_plate_z_mm'],[-70,-65]);self.assertEqual(m['facts']['pad_z_mm'],[-72,-70]);self.assertEqual(json.loads((td/'out/inherited_section_evidence.json').read_text())['slot_depth_engineering_choice_mm'],60)
   parts=json.load(gzip.open(td/'out/candidate_meshes.json.gz','rt'))['meshes'];a=C.actual_section(parts[0],2,-.062,np.zeros(3));b=C.actual_section(parts[1],2,-.062,np.zeros(3));self.assertGreater(K.area(K.G.GEOSIntersection(a,b)),1.,'Shallower slot must not silently claim foot clearance')
 def test_invalid_or_hidden_section_modes_rejected(self):
  c=json.loads((H/'config.json').read_text());c['cross_section_mode']='automatic_collision_trim'
  with tempfile.TemporaryDirectory(dir=H)as td:
   p=Path(td)/'bad.json';B.write(p,c)
   with self.assertRaises(AssertionError):B.build(p,Path(td)/'out')
 def test_full_scene_material_audit_reports_blockers_honestly(self):
  a=json.loads((H/'build/material_audit.json').read_text());self.assertTrue(a['geometry_validation_pass']);self.assertTrue(a['all_candidate_pairs_material_separated']);self.assertFalse(a['all_candidate_vs_retained_clearance_pass']);self.assertFalse(a['strict_whole_scene_clearance_pass']);self.assertEqual(a['missing_from_material_check'],[]);self.assertGreaterEqual(len(a['positive_material_pairs']),17)
  self.assertGreater(a['baseline_rail_crossing_actual_material']['volume_mm3'],500000)
  for r in a['positive_material_pairs']:self.assertGreater(min(abs(n)for n in r['positive_volume_witness']['winding_numbers']),.99)
 def test_no_new_physics_or_as_built_claim(self):
  self.assertFalse(self.payload['new_physics']);self.assertFalse(self.payload['as_built']);self.assertFalse(self.manifest['facts']['source_track_axes_changed'])
  for m in self.mesh.values():self.assertFalse(m['metadata']['as_built_confirmed'])

if __name__=='__main__':unittest.main(verbosity=2)
