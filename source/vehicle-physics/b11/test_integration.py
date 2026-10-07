"""Run frozen control/contact/gas regressions and B11 operator controls."""
import json
import hashlib
import unittest
import numpy as np
import live
from compat import load_b8
from cables import ROOT


class Controls(unittest.TestCase):
    def test_operator_commands_change_no_mechanical_state(self):
        vehicle = live.LiveVehicle(settle_s=0)
        q, v = vehicle.data.qpos.copy(), vehicle.data.qvel.copy()
        vehicle.set_auxiliary_voltage('compressor', 1)
        vehicle.set_auxiliary_power_failure('bay_fan', True)
        vehicle.release_cable_attachment('supply_1')
        np.testing.assert_array_equal(q, vehicle.data.qpos)
        np.testing.assert_array_equal(v, vehicle.data.qvel)
        vehicle.step(20)
        self.assertGreater(vehicle.network.pump.angle_rad, 0)
        self.assertEqual(sum(vehicle.data.warning.number), 0)
        self.assertFalse(vehicle.network.automatic_compressor)

    def test_bad_input_is_atomic(self):
        vehicle = live.LiveVehicle(settle_s=0)
        q, state = vehicle.data.qpos.copy(), vehicle.network.snapshot()
        for name, value in [('wrong', True), ('bay_fan', 1)]:
            with self.assertRaises(ValueError):
                vehicle.set_auxiliary_power_failure(name, value)
        with self.assertRaises(ValueError):
            vehicle.set_auxiliary_voltage('compressor', -1)
        with self.assertRaises(ValueError):
            vehicle.release_cable_attachment('wrong')
        np.testing.assert_array_equal(q, vehicle.data.qpos)
        self.assertEqual(state, vehicle.network.snapshot())


if __name__ == '__main__':
    suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(Controls)])
    for name in ('test_compliant_contact', 'test_live_controls', 'test_pneumatics'):
        module = load_b8(name, 'b11_regression_'+name)
        if hasattr(module, 'OUT'):
            module.OUT = ROOT/'component_results'
            module.OUT.mkdir(exist_ok=True)
        suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(tests_run=result.testsRun, passed=result.wasSuccessful(),
                  failures=[str(t)+'\n'+msg for t,msg in result.failures], errors=[str(t)+'\n'+msg for t,msg in result.errors])
    report['sources_sha256'] = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                for name in ('test_integration.py','cables.py','live.py','source_cables.json')}
    (ROOT/'regression_verification.json').write_text(json.dumps(report,indent=2), encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
