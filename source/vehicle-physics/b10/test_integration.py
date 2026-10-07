"""B10 wiring plus frozen B8 regressions; no kinematic reset on operator input."""
import json
from pathlib import Path
import unittest
import numpy as np

from auxiliaries import ROOT
from compat import load_b8
import live

OUT = ROOT/"component_results"


class Controls(unittest.TestCase):
    def test_auxiliary_input_does_not_teleport_states(self):
        v = live.LiveVehicle(settle_s=0)
        q, dq = v.data.qpos.copy(), v.data.qvel.copy()
        state = v.network.pump.snapshot()
        v.set_auxiliary_voltage("compressor", 1)
        np.testing.assert_array_equal(q, v.data.qpos)
        np.testing.assert_array_equal(dq, v.data.qvel)
        self.assertEqual(state, v.network.pump.snapshot())
        v.step(1)
        self.assertGreater(v.network.pump.angle_rad, 0)
        self.assertGreater(v.network.fans["bay_fan"].omega_rad_s, 0)
        self.assertEqual(len(v.network.volumes), 27)
        self.assertEqual(v.model.nv, 77)
        self.assertEqual(len(v.network.spools), 51)

    def test_power_failure_preserves_momentum_and_batching(self):
        a, b = live.LiveVehicle(settle_s=0), live.LiveVehicle(settle_s=0)
        for v in (a,b):
            v.set_auxiliary_voltage("compressor", 1)
            v.step(100)
            state = v.network.pump.snapshot()
            v.set_auxiliary_power_failure("compressor", True)
            self.assertEqual(state["momentum_kg_m2_s"], v.network.pump.momentum_kg_m2_s)
            v.set_brake("apply")
        a.step(30); b.step(10); b.step(20)
        np.testing.assert_array_equal(a.data.qpos, b.data.qpos)
        self.assertEqual(a.network.snapshot(), b.network.snapshot())
        self.assertEqual(sum(a.telemetry()["warning_counts"]), 0)

    def test_invalid_fault_and_voltage_leave_state_unchanged(self):
        v = live.LiveVehicle(settle_s=0)
        state = v.network.snapshot()
        for name, failed in (("unknown", True), ("compressor", 1)):
            with self.assertRaises(ValueError):
                v.set_auxiliary_power_failure(name, failed)
        with self.assertRaises(ValueError):
            v.set_auxiliary_voltage("compressor", 2)
        self.assertEqual(state, v.network.snapshot())


if __name__ == "__main__":
    import test_auxiliaries
    OUT.mkdir(exist_ok=True)
    reports = {}
    for name, modules in (("components", [test_auxiliaries]),
                          ("regression", [load_b8(n, "b10_regression_"+n) for n in
                           ("test_compliant_contact", "test_live_controls", "test_pneumatics")])):
        suite = unittest.TestSuite()
        for module in modules:
            if hasattr(module, "OUT"):
                module.OUT = OUT
            suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
        if name == "regression":
            suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(Controls))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        report = dict(tests_run=result.testsRun, passed=result.wasSuccessful(),
                      failures=[str(t)+"\n"+msg for t,msg in result.failures],
                      errors=[str(t)+"\n"+msg for t,msg in result.errors])
        (OUT/(name+".json")).write_text(json.dumps(report, indent=2), encoding="utf-8")
        reports[name] = report
    raise SystemExit(0 if all(r["passed"] for r in reports.values()) else 1)
