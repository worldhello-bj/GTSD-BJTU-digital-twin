"""Analytic freefall, causal force/fixture controls, contact and inertia checks."""
import json
import unittest
import numpy as np
from covers import Bench,CoverParameters,ROOT


class CoverTests(unittest.TestCase):
    def test_source_dimensions_and_uniform_proxy_inertia(self):
        bench=Bench()
        self.assertEqual(bench.model.nv,24)
        self.assertEqual(bench.spec['total_mass_kg'],1.)
        for record in bench.spec['covers']:
            np.testing.assert_allclose(bench.model.body(record['key']).inertia,record['diagonal_inertia_kg_m2'],rtol=1e-10)
            self.assertGreater(min(record['collision_box_dimensions_m']),0)

    def test_freefall_matches_analytic_gravity_and_release_preserves_state(self):
        bench=Bench(with_floor=False)
        q,v=bench.data.qpos.copy(),bench.data.qvel.copy()
        for record in bench.spec['covers']:
            bench.release(record['key'])
        np.testing.assert_array_equal(q,bench.data.qpos); np.testing.assert_array_equal(v,bench.data.qvel)
        for _ in range(2000):
            bench.step()
        t=bench.data.time; h=bench.spec['parameters']['timestep_s']
        for record in bench.spec['covers']:
            index=int(bench.model.joint(record['key']+'_free').qposadr[0])
            # MuJoCo semi-implicit position update has the known O(h) offset.
            expected=.4-.5*9.81*t*(t+h)
            self.assertAlmostEqual(bench.data.qpos[index+2],expected,places=10)

    def test_fastened_fixtures_react_to_force(self):
        bench=Bench(); start=bench.data.qpos.copy()
        bench.apply_point_force('cover_0',[2,0,0])
        for _ in range(1000):
            bench.step()
        self.assertLess(np.max(abs(bench.data.qpos-start)),1e-5)
        self.assertGreater(np.linalg.norm(bench.data.qfrc_constraint),2)

    def test_point_force_changes_rotation_without_coordinate_assignment(self):
        bench=Bench(with_floor=False); bench.release('cover_0')
        state=bench.data.qpos.copy()
        bench.apply_point_force('cover_0',[0,0,4],[.05,0,0])
        np.testing.assert_array_equal(state,bench.data.qpos)
        for _ in range(200):
            bench.step()
        index=int(bench.model.joint('cover_0_free').dofadr[0])
        self.assertGreater(abs(bench.data.qvel[index+4]),.1)

    def test_invalid_input_preserves_state_and_force(self):
        bench=Bench(); force=bench.data.xfrc_applied.copy(); q=bench.data.qpos.copy()
        with self.assertRaises(ValueError):bench.release('bad')
        for values in ([float('nan'),0,0],[1,2]):
            with self.assertRaises(ValueError):bench.apply_point_force('cover_0',values)
        np.testing.assert_array_equal(force,bench.data.xfrc_applied); np.testing.assert_array_equal(q,bench.data.qpos)
        with self.assertRaises(ValueError):CoverParameters(proxy_mass_kg=-1)


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CoverTests))
    record=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,
                failures=[str(t)+'\n'+msg for t,msg in result.failures],errors=[str(t)+'\n'+msg for t,msg in result.errors])
    (ROOT/'component_verification.json').write_text(json.dumps(record,indent=2),encoding='utf-8',newline='\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
