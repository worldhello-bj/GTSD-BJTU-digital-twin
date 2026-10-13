import unittest
import numpy as np
from audit_combined import winding,surface_distance,topo
from triangle_classifier import classify

def box(lo=(0,0,0),hi=(1,1,1)):
 v=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],float);v=np.array(lo)+v*(np.array(hi)-lo)
 f=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
 return v,f
class AuditTests(unittest.TestCase):
 def test_closed_consistent_cube(self):
  t=topo(box());self.assertTrue(t['closed_edge_two_incidence']);self.assertTrue(t['consistent_directed_edges']);self.assertEqual(t['degenerate_triangle_count'],0)
 def test_winding_and_distance(self):
  g=box();self.assertAlmostEqual(winding(g,np.array([.5,.5,.5])),1);self.assertAlmostEqual(winding(g,np.array([2,.5,.5])),0);self.assertAlmostEqual(surface_distance(g,np.array([.5,.5,.5])),.5);self.assertAlmostEqual(surface_distance(g,np.array([2,.5,.5])),1)
 def test_disjoint_boxes(self):self.assertEqual(classify(box(),box((2,0,0),(3,1,1)))['status'],'NO_POSITIVE_VOLUME_AABB')
 def test_touch_is_not_volume_overlap(self):self.assertEqual(classify(box(),box((1,0,0),(2,1,1)))['status'],'NO_POSITIVE_VOLUME_AABB')
 def test_containment_detected(self):self.assertEqual(classify(box(),box((.2,.2,.2),(.8,.8,.8)))['status'],'POSITIVE_MATERIAL_OVERLAP')
 def test_volume_witness_independent(self):
  a=box();b=box((.75,.1,.1),(1.75,.9,.9));r=classify(a,b);self.assertEqual(r['status'],'POSITIVE_MATERIAL_OVERLAP');p=np.array(r['witness_world_m']);self.assertAlmostEqual(abs(winding(a,p)),1);self.assertAlmostEqual(abs(winding(b,p)),1);self.assertGreater(min(surface_distance(g,p) for g in [a,b]),0)
 def test_open_surface_not_called_closed(self):
  v,f=box();self.assertFalse(topo((v,f[:-1]))['closed_edge_two_incidence'])
if __name__=='__main__':unittest.main()
