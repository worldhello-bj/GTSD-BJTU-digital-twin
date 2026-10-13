import json,unittest,html.parser
from pathlib import Path
H=Path(__file__).resolve().parent
class Tags(html.parser.HTMLParser):
 def __init__(self):super().__init__();self.paths=[];self.bands=[]
 def handle_starttag(self,tag,attrs):
  a=dict(attrs)
  if tag=='polyline':self.paths.append(a)
  if tag=='rect'and a.get('class')=='band':self.bands.append(a)
class SourceReviewTests(unittest.TestCase):
 def test_actual_curve_count_and_identity(self):
  g=json.loads((H/'native_edge_graph.json').read_text());self.assertEqual(len(g['source_curves']),57);self.assertEqual(len({r['handle']for r in g['source_curves']}),57);w=json.loads((H/'model_identity_witness.json').read_text());self.assertTrue(w['all_are_distinct_direct_Model_entities']);self.assertEqual(w['exact_equal_or_reversed_curve_duplicates'],[])
 def test_five_plan_intervals_preserved(self):
  c=json.loads((H/'complete_connection_review.json').read_text());self.assertEqual(len(c['paired_head_plan_intervals']),5);self.assertTrue(all(abs(r['plan_head_band_width_mm']-39.5)<1e-6 for r in c['paired_head_plan_intervals']))
 def test_endpoint_and_interior_relations_distinguished(self):
  c=json.loads((H/'complete_connection_review.json').read_text());targets=[r for r in c['detail_endpoints']if r['handle']in ['4ED5C','4ED5D']and r['endpoint']==1];self.assertEqual(len(targets),2);self.assertTrue(all(r['exact_finite_edge_connections'][0]['relation']=='INTERIOR_OF_FINITE_SOURCE_EDGE'for r in targets))
 def test_open_chain_checked_beyond_237_subset(self):
  c=json.loads((H/'whole_model_connection_witness.json').read_text());self.assertEqual(sum(c['entity_type_counts'].values()),2738);a=json.loads((H/'native_curve_interior_witness.json').read_text());self.assertEqual(a['open_4ED2C_other_geometric_continuations_found'],0);self.assertEqual(a['spline_control_bbox_candidates'],[])
 def test_viewer_has_all_curves_no_external_dependencies(self):
  text=(H/'rail_source_review.html').read_text();t=Tags();t.feed(text);self.assertEqual(len(t.paths),57);self.assertEqual(len(t.bands),5);self.assertNotIn('src="http',text);self.assertNotIn('fetch(',text)
 def test_modes_do_not_delete_material_or_move_axes(self):
  c=json.loads((H/'topology_candidate_fields.json').read_text());self.assertEqual(c['default_mode'],'source_only');self.assertFalse(c['ready_to_generate_solid']);self.assertFalse(c['allow_automatic_material_cut']);self.assertFalse(c['allow_axis_translation']);self.assertTrue(all(v is None for v in c['fields_requiring_explicit_evidence_or_engineering_choice'].values()))
 def test_real_layer_findings(self):
  d=json.loads((H/'rail_layer_overlap.json').read_text());a,b=d['pairs'];self.assertTrue(all(r['positive_overlap_found']for r in a['height_layer_results']));self.assertTrue(b['height_layer_results'][-1]['positive_overlap_found']);self.assertFalse(b['height_layer_results'][0]['positive_overlap_found'])
if __name__=='__main__':unittest.main(verbosity=2)
