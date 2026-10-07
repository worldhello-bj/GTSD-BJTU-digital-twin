"""Reproducible pneumatic physics tests; no third-party packages required.

Run: python b8/test_pneumatics.py
Writes pneumatic_results/test_report.json and convergence.csv (synthetic tests).
"""
from __future__ import annotations
import csv
import json
import math
from pathlib import Path
import time
import unittest

from pneumatics import (GasVolume, Orifice, Compressor, PneumaticNetwork,
                        make_vehicle_network, orifice_mass_flow_kg_s,
                        AMBIENT_PA, TEMPERATURE_K, AIR_R, PROVENANCE)

OUT = Path(__file__).parent / 'pneumatic_results'
RESULTS = {}


def advance(n, duration, dt, x=None, commands=None):
    count = max(1, math.ceil(duration / dt))
    h = duration / count
    for k in range(count):
        pos = x((k + 1) * h) if callable(x) else x
        n.update(h, pos, commands if k == 0 else None, snapshot=False)


class PneumaticTests(unittest.TestCase):
    def assert_audit(self, n, mass_tol=2e-13, energy_tol=1e-7):
        a = n.audit()
        self.assertLess(abs(a['mass_balance_residual_kg']), mass_tol)
        self.assertLess(abs(a['gas_energy_balance_residual_J']), energy_tol)
        self.assertLess(abs(a['combined_energy_balance_residual_J']), energy_tol)
        self.assertLess(abs(a['exergy_balance_residual_J']), energy_tol)
        self.assertGreaterEqual(a['exergy_destroyed_J'], -1e-12)
        for p in n.pressures_Pa().values():
            self.assertTrue(math.isfinite(p) and p > 0)

    def test_01_orifice_choking_and_limits(self):
        f = lambda pd: orifice_mass_flow_kg_s(600000, pd, TEMPERATURE_K, 1e-7)
        self.assertEqual(f(50000), f(200000))
        critical = 600000 * (2 / 2.4)**(1.4 / .4)
        self.assertAlmostEqual(f(critical * (1-1e-7)), f(critical * (1+1e-7)), delta=1e-15)
        self.assertEqual(f(600000), 0)
        self.assertGreater(f(599999.999), 0)
        self.assertLess(f(500000), f(200000))
        with self.assertRaises(ValueError):
            f(600001)
        RESULTS['choked_flow'] = {'mass_flow_kg_s': f(200000), 'critical_pressure_ratio': critical / 600000,
                                  'near_equal_pressure_flow_kg_s': f(599999.999)}

    def test_02_closed_equalization(self):
        n = PneumaticNetwork([GasVolume('tank', 2e-4, 600000), GasVolume('cylinder', 1e-4, 100000)],
                              [Orifice('valve', 'tank', 'cylinder', 3e-6, opening=1)])
        p_exact = (2e-4 * 600000 + 1e-4 * 100000) / 3e-4
        initial_B = n.exergy_J()
        advance(n, 1, .001)
        for p in n.pressures_Pa().values():
            self.assertAlmostEqual(p, p_exact, delta=1e-7)
        self.assertLess(n.exergy_J(), initial_B)
        self.assertEqual(n.audit()['external_mass_in_kg'], 0)
        self.assertEqual(n.audit()['external_mass_out_kg'], 0)
        self.assert_audit(n)
        RESULTS['closed_equalization'] = {'analytic_final_pressure_Pa': p_exact, **n.snapshot()}

    def test_03_closed_cylinder_work_and_compression(self):
        n = PneumaticNetwork([GasVolume('piston', 1e-4, 500000, .001)])
        m0 = n.total_mass_kg()
        advance(n, .2, .002, x=lambda t: {'piston': .1 * t / .2})
        expected_work = 500000 * 1e-4 * math.log(2)
        a = n.audit()
        self.assertAlmostEqual(n.pressure_Pa('piston'), 250000, delta=1e-8)
        self.assertAlmostEqual(a['gas_boundary_work_J'], expected_work, delta=1e-10)
        self.assertAlmostEqual(a['gas_heat_from_bath_J'], expected_work, delta=1e-10)
        self.assertAlmostEqual(a['useful_mechanical_work_J'], expected_work-AMBIENT_PA*1e-4, delta=1e-10)
        self.assertAlmostEqual(n.total_mass_kg(), m0, delta=1e-16)
        self.assert_audit(n)
        RESULTS['closed_cylinder_expansion'] = n.snapshot()
        advance(n, .2, .002, x=lambda t: {'piston': .1 * (1-t/.2)})
        self.assertAlmostEqual(n.pressure_Pa('piston'), 500000, delta=1e-8)
        self.assertAlmostEqual(n.audit()['useful_mechanical_work_J'], 0, delta=1e-10)
        self.assert_audit(n)

    def test_04_exhaust_leak_and_atmospheric_intake(self):
        summaries = {}
        for label, area in [('normal', 2e-8), ('quick', 2e-7), ('leak', 2e-9)]:
            n = PneumaticNetwork([GasVolume('c', 4e-5, 500000, .0002)],
                                  [Orifice(label, 'c', None, area, opening=1)])
            advance(n, 1.0, .001)
            summaries[label] = n.snapshot()
            self.assertGreater(n.audit()['external_mass_out_kg'], 0)
            self.assertGreater(n.audit()['gas_heat_from_bath_J'], 0)
            self.assert_audit(n)
        ps = {k: v['pressure_Pa']['c'] for k,v in summaries.items()}
        self.assertLess(ps['quick'], ps['normal'])
        self.assertLess(ps['normal'], ps['leak'])
        n = PneumaticNetwork([GasVolume('c', 1e-4, AMBIENT_PA, .001)],
                              [Orifice('ambient', 'c', None, 2e-7, opening=1)])
        advance(n, .1, .001, x=lambda t: {'c': t})
        advance(n, 12, .001)
        self.assertAlmostEqual(n.pressure_Pa('c'), AMBIENT_PA, delta=1e-7)
        self.assertGreater(n.audit()['external_mass_in_kg'], 0)
        self.assert_audit(n)
        summaries['atmospheric_intake'] = n.snapshot()
        RESULTS['exhaust_and_leak'] = summaries

    def test_05_reservoir_loss_and_independent_valves(self):
        n = make_vehicle_network(brake_names=['brake'], include_pantograph=False)
        advance(n, 30, .001, commands={'reservoir_leak': 1})
        self.assertLess(n.pressure_Pa('reservoir'), AMBIENT_PA + 100)
        advance(n, .5, .001, commands={'reservoir_leak': 0, 'brake_supply': 1})
        self.assertLess(n.forces_N()['brake'], .006)
        self.assert_audit(n)
        RESULTS['loss_of_supply'] = n.snapshot()
        # A charged isolated chamber keeps its mass after supply loss; a leak
        # or exhaust command is required for automatic pressure release.
        n = make_vehicle_network(brake_names=['brake'], include_pantograph=False)
        advance(n, 1, .001, commands={'brake_supply': 1})
        p_before = n.pressure_Pa('brake')
        advance(n, 30, .001, commands={'reservoir_leak': 1, 'brake_supply': 0})
        self.assertAlmostEqual(n.pressure_Pa('brake'), p_before, delta=1e-6)
        self.assert_audit(n)
        RESULTS['check_valve_retained_pressure_Pa'] = p_before

    def test_06_finite_mass_reservoir_pressure_drops(self):
        names = [f'b{i}' for i in range(16)]
        n = make_vehicle_network(names, reservoir_volume_m3=2e-4, include_pantograph=False)
        p0 = n.pressure_Pa('reservoir')
        advance(n, 2, .001, commands={name+'_supply': 1 for name in names})
        self.assertLess(n.pressure_Pa('reservoir'), p0-20000)
        self.assertEqual(n.audit()['external_mass_in_kg'], 0)
        self.assert_audit(n)
        RESULTS['finite_supply'] = n.snapshot()

    def test_07_compressor_mass_and_work_are_external(self):
        n = PneumaticNetwork([GasVolume('r', .001, AMBIENT_PA)], compressors=[
            Compressor('compressor', 'r', .002, 500000, .65, 1)])
        advance(n, 4, .001)
        a = n.audit()
        self.assertAlmostEqual(n.pressure_Pa('r'), 500000, delta=1e-7)
        self.assertGreater(a['external_mass_in_kg'], 0)
        self.assertGreater(a['compressor_work_in_J'], 0)
        self.assertAlmostEqual(a['compressor_work_in_J']*.65, a['exergy_delta_J'], delta=1e-8)
        self.assertGreater(a['compressor_heat_to_bath_J'], 0)
        self.assert_audit(n)
        RESULTS['compressor'] = n.snapshot()
        # A large step beginning below atmosphere must charge work for the
        # entire above-ambient part, rather than netting against initial vacuum.
        n = PneumaticNetwork([GasVolume('r', .001, 50000)], compressors=[
            Compressor('compressor', 'r', 1, 500000, 1, 1)], max_substep_s=1)
        n.update(1)
        self.assertAlmostEqual(n.audit()['compressor_work_in_J'], n.exergy_J(), delta=1e-8)
        self.assert_audit(n)

    def test_08_large_step_and_invalid_input(self):
        n = PneumaticNetwork([GasVolume('a', 1e-8, 1e8), GasVolume('b', 1e-3, 10000)],
                             [Orifice('v', 'a', 'b', 1, opening=1)], max_substep_s=10)
        n.update(10)
        self.assertGreater(n.equalization_caps, 0)
        self.assert_audit(n)
        old = n.snapshot()
        with self.assertRaises(ValueError):
            n.update(.1, valve_commands={'v': 0, 'unknown': 1})
        self.assertEqual(n.snapshot(), old)
        with self.assertRaises(ValueError):
            PneumaticNetwork([GasVolume('bad', 1e-4)], gamma=float('nan'))
        c = PneumaticNetwork([GasVolume('c', 1e-4, piston_area_m2=.001)])
        with self.assertRaises(ValueError):
            c.update(.1, {'c': -.1})
        with self.assertRaises(ValueError):
            c.update(0, {'c': .01})

    def test_09_gas_spring_force_is_volume_dependent(self):
        extras = [GasVolume(f'air_{b}_{s}', .00025, 200000, .003) for b in 'AB' for s in 'LR']
        n = make_vehicle_network([], include_pantograph=False, extra_chambers=extras)
        f0 = n.forces_N()['air_A_L']
        n.update(.001, {'air_A_L': -.02, 'air_A_R': .02})
        f = n.forces_N()
        self.assertGreater(f['air_A_L'], f0)
        self.assertLess(f['air_A_R'], f0)
        self.assertEqual(f['air_B_L'], f0)
        self.assertAlmostEqual(n.pressure_Pa('air_A_L'), 200000*.00025/(.00025-.003*.02), delta=1e-8)
        self.assert_audit(n)
        RESULTS['airspring_volume_force'] = n.snapshot()

    def test_10_timestep_convergence_with_observed_motion(self):
        def run(dt):
            n = PneumaticNetwork([GasVolume('r', 1e-4, 600000), GasVolume('c', 2e-5, AMBIENT_PA, .0002)],
                                  [Orifice('supply', 'r', 'c', 8e-8, opening=1),
                                   Orifice('leak', 'c', None, 5e-9, opening=1)], max_substep_s=dt)
            # Analytic geometry is solely a numerical-validation input, not a
            # mechanical animation/servo used by the integrated vehicle model.
            advance(n, .2, dt, x=lambda t: {'c': .04*math.sin(2*math.pi*t)})
            self.assert_audit(n)
            return n
        reference = run(.0000625)
        refp = reference.pressure_Pa('c')
        rows = []
        for dt in [.004,.002,.001,.0005,.00025,.000125]:
            n = run(dt)
            rows.append(dict(dt_s=dt, chamber_pressure_Pa=n.pressure_Pa('c'),
                             reference_pressure_Pa=refp,
                             pressure_error_Pa=abs(n.pressure_Pa('c')-refp),
                             useful_work_J=n.audit()['useful_mechanical_work_J'],
                             mass_residual_kg=n.audit()['mass_balance_residual_kg']))
        for coarse, fine in zip(rows, rows[1:]):
            self.assertLess(fine['pressure_error_Pa'], coarse['pressure_error_Pa'])
        self.assertLess(rows[-1]['pressure_error_Pa']/refp, .001)
        OUT.mkdir(exist_ok=True)
        with (OUT/'convergence.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
        RESULTS['timestep_convergence'] = rows

    def test_11_pressure_force_moves_free_mass_without_trajectory(self):
        n = PneumaticNetwork([GasVolume('c', 4e-5, AMBIENT_PA+300000, .0002)])
        mass=.5; stiffness=2000.; damping=30.; x=v=0.; dt=.0001
        initial_force=n.forces_N()['c']
        friction_loss=0.
        for _ in range(10000):
            force=n.forces_N()['c']
            v += dt*(force-stiffness*x-damping*v)/mass
            friction_loss += damping*v*v*dt
            x += dt*v
            n.update(dt, {'c': x}, snapshot=False)
        final_force=n.forces_N()['c']
        self.assertGreater(x, .015)
        self.assertLess(x, .030)
        self.assertLess(final_force, initial_force)
        self.assertLess(abs(v), 1e-6)
        mechanical_energy=.5*mass*v*v+.5*stiffness*x*x
        mech_residual=mechanical_energy+friction_loss-n.audit()['useful_mechanical_work_J']
        self.assertLess(abs(mech_residual), .003)
        self.assert_audit(n)
        RESULTS['free_mass_coupling'] = {'position_m':x,'velocity_mps':v,
            'initial_force_N':initial_force,'final_force_N':final_force,
            'mechanical_energy_plus_loss_minus_gas_work_J':mech_residual, **n.snapshot()}


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    started=time.perf_counter()
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(PneumaticTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report=dict(schema='B8 isothermal pneumatic validation v1', provenance=PROVENANCE,
                tests_run=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                passed=result.wasSuccessful(), elapsed_wall_s=time.perf_counter()-started,
                findings=RESULTS)
    (OUT/'test_report.json').write_text(json.dumps(report,indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)
