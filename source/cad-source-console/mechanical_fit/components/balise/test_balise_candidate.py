#!/usr/bin/env python3
"""Small independent regression tests; no Blender, hardware, or source mutation."""
import argparse,copy,json,tempfile,unittest,sys
from pathlib import Path
import numpy as np
from balise_geometry import classify,inside,closed,triangle_hash,load_candidate_geometry,overlap_chords,apply_profiles,validate_changed_meshes
from build_balise_candidate import assembly_names,make,HERE
parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--project-root',type=Path,required=True);options,remaining=parser.parse_known_args();ROOT=options.project_root;sys.argv=[sys.argv[0]]+remaining

F=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[3,7,6],[3,6,2],[0,4,7],[0,7,3],[1,2,6],[1,6,5]])
def cube(lo,hi):
    lo=np.array(lo);hi=np.array(hi);v=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],float)
    return lo+v*(hi-lo),F

class GeometryTests(unittest.TestCase):
    def test_triangle_overlap_and_ray_chord(self):
        a=cube([0,0,0],[1,1,1]);b=cube([.5,.5,.5],[1.5,1.5,1.5])
        self.assertTrue(closed(*a));self.assertEqual(classify(a,b)['status'],'POSITIVE_MATERIAL_OVERLAP')
        for c in overlap_chords(a,b,[.75,.75,.75]):self.assertAlmostEqual(c['shared_material_chord_mm'],500)
    def test_exact_contact_is_not_material_penetration(self):
        a=cube([0,0,0],[1,1,1]);b=cube([1,0,0],[2,1,1])
        self.assertEqual(classify(a,b)['status'],'NO_POSITIVE_VOLUME_AABB')
    def test_mesh_proof_when_aabb_overlaps(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],float)
        f=np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])
        r=classify((v,f),(1-v,f))
        self.assertEqual(r['status'],'TRIANGULATED_MESHES_SEPARATED')
        self.assertEqual(r['surface_crossing_count'],0);self.assertEqual(r['containment_witnesses'],[])
    def test_cross_profile_interference_is_not_hidden(self):
        before={'left':cube([0,0,0],[1,1,1]),'right':cube([3,0,0],[4,1,1])}
        def profile(old,new,geo):
            v,f=geo
            return {'remove_names':[old],'add_meshes':[{'name':new,'vertices':v.reshape(-1).tolist(),'indices':f.reshape(-1).tolist(),'metadata':{'replaces_mesh_name':old}}]}
        profiles=[profile('left','new_left',cube([1,0,0],[2,1,1])),profile('right','new_right',cube([1.5,0,0],[2.5,1,1]))]
        after,ids=apply_profiles(before,profiles);r=validate_changed_meshes(before,after,ids)
        self.assertFalse(r['passes_no_new_interference_gate']);self.assertEqual(len(r['new_positive_or_uncleared_interference_pairs']),1)
        with self.assertRaises(ValueError):apply_profiles(after,profiles)
    def test_split_predecessor_does_not_excuse_new_overlap(self):
        before={'old':cube([0,0,0],[1,1,1])};after={'a':cube([0,0,0],[1,1,1]),'b':cube([.5,0,0],[1.5,1,1])}
        r=validate_changed_meshes(before,after,{'a':'old','b':'old'})
        self.assertFalse(r['passes_no_new_interference_gate']);self.assertEqual(r['after_positive_aabb_pairs'][0]['before']['status'],'NO_DISTINCT_BEFORE_PAIR')
    def test_containment_is_detected(self):
        self.assertEqual(classify(cube([0,0,0],[3,3,3]),cube([1,1,1],[2,2,2]))['status'],'POSITIVE_MATERIAL_OVERLAP')

class ArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile=json.loads((HERE/'balise_fit_profile.json').read_text());cls.report=json.loads((HERE/'balise_fit_validation.json').read_text());cls.geo,_,_=load_candidate_geometry(ROOT,json.loads((HERE/'balise_source_seed.json').read_text()))
    def test_atomic_18_for_18_and_no_external_source_modification(self):
        p=self.profile;self.assertEqual(len(p['remove_names']),18);self.assertEqual(len(set(p['remove_names'])),18)
        self.assertEqual(len(p['add_meshes']),18);self.assertEqual(len({m['name'] for m in p['add_meshes']}),18)
        self.assertFalse(p['as_built_confirmed']);self.assertFalse(p['default_enabled']);self.assertFalse(p['physics_new_collision'])
        for i,s in [(2,12),(3,17)]:
            expected=set(assembly_names(i)+[f'DETAIL_BENCH_branch_chair_{s:02}_2',f'DETAIL_BENCH_branch_fastener_{s:02}_2_-1',f'DETAIL_BENCH_branch_fastener_{s:02}_2_1'])
            self.assertTrue(expected.issubset(p['remove_names']))
        self.assertFalse(any('sleeper' in n or 'rail' in n for n in p['remove_names']))
    def test_actual_mesh_transforms_and_unchanged_shape(self):
        for m in self.profile['add_meshes']:
            v=np.array(m['vertices']).reshape(-1,3);f=np.array(m['indices']).reshape(-1,3);meta=m['metadata'];old=meta['replaces_mesh_name']
            self.assertEqual(triangle_hash(v,f),meta['triangle_sha256']);self.assertEqual(len(f),len(self.geo[old][1]));self.assertTrue(closed(v,f))
            if old.startswith('ENV_balise'):
                ov,of=self.geo[old];t=np.array(meta['old_baked_world_to_new_baked_world_matrix'])
                np.testing.assert_allclose(ov@t[:3,:3].T+t[:3,3],v,rtol=0,atol=1e-12)
                np.testing.assert_array_equal(of,f);np.testing.assert_array_equal(v[:,2],ov[:,2])
                np.testing.assert_allclose(t[:3,:3].T@t[:3,:3],np.eye(3),rtol=0,atol=1e-14)
    def test_three_original_internal_witnesses_and_full_scene_gate(self):
        r=self.report;self.assertTrue(r['passes_geometric_candidate_gate']);self.assertEqual(len(r['targets']),3)
        for t in r['targets']:
            a,b=t['pair'].split('|');p=np.array(t['before']['witness_world_m']);self.assertTrue(inside(*self.geo[a],p));self.assertTrue(inside(*self.geo[b],p))
            self.assertIn(t['after']['status'],['NO_POSITIVE_VOLUME_AABB','TRIANGULATED_MESHES_SEPARATED'])
        self.assertEqual(r['remaining_after_review_pairs'],[])
        self.assertGreaterEqual(len(r['all_moved_vs_all_pairs_with_positive_AABB_before_or_after']),18)
    def test_clip_seating_uses_real_source_rail_and_retains_shape(self):
        for group in self.profile['groups']:
            self.assertEqual(len(group['clip_seating_candidates']),2)
            for clip in group['clip_seating_candidates']:
                self.assertTrue(clip['every_clip_XY_vertex_inside_actual_chair_material_midsection'])
                self.assertGreaterEqual(clip['actual_rail_side_gap_mm'],.5-1e-8)
                self.assertGreaterEqual(clip['minimum_chair_plan_AABB_edge_margin_mm'],.5-1e-8)
                old=clip['name'];row=next(m for m in self.profile['add_meshes'] if m['metadata']['replaces_mesh_name']==old)
                v=np.array(row['vertices']).reshape(-1,3);ov,of=self.geo[old]
                expected=ov.copy();expected[:,1]+=clip['world_Y_delta_from_source_group_mm']/1000
                np.testing.assert_allclose(v,expected,rtol=0,atol=1e-12)
                np.testing.assert_array_equal(np.array(row['indices']).reshape(-1,3),of)
    def test_current_main_five_scenes_and_atomic_resource_counts(self):
        x=json.loads((HERE/'current_main_effective_scene_validation.json').read_text());self.assertTrue(x['pass'])
        self.assertEqual(len(x['all_five_scene_results']),5)
        for r in x['all_five_scene_results']:
            self.assertTrue(r['all_18_remove_names_present']);self.assertTrue(r['source_pending_new_names_not_duplicated'])
            self.assertEqual(r['source_replacement_removed_count'],323);self.assertEqual(r['source_replacement_added_count'],326)
            self.assertEqual(r['effective_before_count'],r['effective_after_count'])
            self.assertEqual(r['new_positive_or_uncleared_interference_pairs'],[])
    def test_invalid_unrotated_body_fails_parameter_gate(self):
        c=json.loads((HERE/'fit_config.json').read_text());c['assemblies'][0]['yaw_deg']=0
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            with self.assertRaisesRegex(ValueError,'will not fit source bay'):make(ROOT,c,Path(d))
    def test_disallowed_z_lift_and_unsafe_lateral_offset_rejected(self):
        c=json.loads((HERE/'fit_config.json').read_text());c['deck_z_offset_mm']=27.6
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            with self.assertRaisesRegex(ValueError,'unchanged inherited deck Z'):make(ROOT,c,Path(d))
        c['deck_z_offset_mm']=0;c['assemblies'][0]['lateral_center_m']=0
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            with self.assertRaisesRegex(ValueError,'violates finite rail triangle corridor'):make(ROOT,c,Path(d))

if __name__=='__main__':unittest.main(verbosity=2)
