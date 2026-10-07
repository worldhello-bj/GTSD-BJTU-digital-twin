"""Short control-wiring regressions, separate from full settled acceptance cases."""
import json
from pathlib import Path
import unittest
import numpy as np

from compat import load_b8
import live

OUT = Path(__file__).parent/"component_results"


class B9Controls(unittest.TestCase):
    def test_no_command_state_teleport_and_actual_valve_causality(self):
        v = live.LiveVehicle(settle_s=0)
        q, dq = v.data.qpos.copy(), v.data.qvel.copy()
        pad = next(iter(v.pads))
        initial = {n:s.snapshot() for n,s in v.network.spools.items()}
        v.set_brake("apply")
        np.testing.assert_array_equal(v.data.qpos, q)
        np.testing.assert_array_equal(v.data.qvel, dq)
        self.assertEqual({n:s.snapshot() for n,s in v.network.spools.items()}, initial)
        v.step(1)
        spool = v.network.spools[pad+"_supply"]
        self.assertGreater(spool.position_m, 0)
        self.assertEqual(spool.opening, 0)
        v.step(v.steps_for_duration(.05))
        self.assertGreater(v.network.pressure_Pa(pad), live.base.AMBIENT_PA+1000)
        self.assertEqual(v.model.nv, 77)
        self.assertEqual(len(v.network.spools), 51)
        self.assertEqual(sum(v.telemetry()["warning_counts"]), 0)

    def test_live_failure_and_batching(self):
        a, b = live.LiveVehicle(settle_s=0), live.LiveVehicle(settle_s=0)
        for v in (a, b):
            v.set_brake("apply")
            for pad in v.pads:
                v.set_valve_coil_failure(pad+"_supply", True)
        a.step(30); b.step(10); b.step(20)
        np.testing.assert_array_equal(a.data.qpos, b.data.qpos)
        self.assertEqual(a.network.snapshot(), b.network.snapshot())
        self.assertTrue(all(v.opening == 0 for n,v in a.network.spools.items() if n.endswith("_pad_supply")))


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    suite = unittest.TestSuite()
    # Frozen B8 tests continue to test B8 unchanged, and B9 controls separately.
    for script in ("test_compliant_contact", "test_live_controls", "test_pneumatics"):
        module = load_b8(script, "b9_regression_"+script)
        if hasattr(module, "OUT"):
            module.OUT = OUT
        suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(B9Controls))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(schema="B9 short integration and unchanged B8 regressions v1", tests_run=result.testsRun,
                  passed=result.wasSuccessful(), failures=[str(t)+"\n"+msg for t,msg in result.failures],
                  errors=[str(t)+"\n"+msg for t,msg in result.errors])
    (OUT/"regression.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)
