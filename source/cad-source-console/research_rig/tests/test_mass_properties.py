import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mass_properties import integrate, combine


def box(name='box', dimensions=(2., 3., 4.), position=(0., 0., 0.)):
    v = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                  [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]], float)
    v = v * np.array(dimensions) / 2 + position
    f = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
         [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    return {'name': name, 'vertices': v.ravel().tolist(), 'indices': np.array(f).ravel().tolist()}


class MassTest(unittest.TestCase):
    def test_box_analytic(self):
        p = integrate(box(), mass_kg=12.)
        self.assertAlmostEqual(p['volume_m3'], 24.)
        np.testing.assert_allclose(p['inertia_com_world_kg_m2'], np.diag([25.,20.,13.]))

    def test_translation_rotation(self):
        m = box(position=(234.,-189.,312.))
        q = np.array([[0,-1,0],[1,0,0],[0,0,1.]])
        m['vertices'] = (np.array(m['vertices']).reshape(-1,3) @ q.T).ravel().tolist()
        p = integrate(m, mass_kg=12.)
        np.testing.assert_allclose(p['center_of_mass_world_m'], [189.,234.,312.])
        np.testing.assert_allclose(p['inertia_com_world_kg_m2'], np.diag([20.,25.,13.]), atol=1e-12)

    def test_density_and_explicit_mass_agree(self):
        a = integrate(box(), density_kg_m3=2.)
        b = integrate(box(), mass_kg=48.)
        np.testing.assert_allclose(a['inertia_com_world_kg_m2'], b['inertia_com_world_kg_m2'])

    def test_parallel_axis(self):
        a = integrate(box('a', position=(-2,0,0)), mass_kg=1.)
        b = integrate(box('b', position=(2,0,0)), mass_kg=1.)
        p = combine([a,b], hypothesis_id='test')
        np.testing.assert_allclose(p['center_of_mass_world_m'], [0,0,0])
        np.testing.assert_allclose(p['inertia_com_world_kg_m2'], np.diag([50/12,40/12+8,26/12+8]))
        with self.assertRaises(ValueError): combine([a,a], hypothesis_id='test')

    def test_fail_closed(self):
        for k in [0.,-1.,float('nan')]:
            with self.assertRaises(ValueError): integrate(box(), mass_kg=k)
        with self.assertRaises(ValueError): integrate(box(), mass_kg=1.,density_kg_m3=1.)
        m = box();m['indices'] = m['indices'][:-3]
        with self.assertRaises(ValueError): integrate(m,mass_kg=1.)
        m = box();m['indices'] = np.array(m['indices']).reshape(-1,3)[:,::-1].ravel().tolist()
        with self.assertRaises(ValueError): integrate(m,mass_kg=1.)

if __name__ == '__main__': unittest.main()
