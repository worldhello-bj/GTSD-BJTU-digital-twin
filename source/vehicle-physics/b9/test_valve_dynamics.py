"""Analytic, energetic, causal, fault and gas-coupling checks for B9 ports."""
from dataclasses import replace
import json
import math
from pathlib import Path
import unittest

from valve_dynamics import (Spool, SpoolParameters, DynamicValveNetwork,
                            make_vehicle_network, PROVENANCE)
from pneumatics import GasVolume, Orifice, PneumaticNetwork, AMBIENT_PA

OUT = Path(__file__).parent / "component_results"
RESULTS = {}


def integrate(spool, duration, dt, command):
    for _ in range(round(duration/dt)):
        spool.advance(dt, command)


def sample_network(parameters=None):
    gas = PneumaticNetwork([GasVolume("reservoir", .002, AMBIENT_PA+500000),
                            GasVolume("cylinder", 2e-6)],
                           [Orifice("cylinder_supply", "reservoir", "cylinder", 8e-9, one_way=True),
                            Orifice("cylinder_exhaust", "cylinder", None, 1.6e-8),
                            Orifice("cylinder_leak", "cylinder", None, 3e-9)])
    return DynamicValveNetwork(gas, parameters=parameters)


class ValveTests(unittest.TestCase):
    def check_ledger(self, spool):
        self.assertLess(abs(spool.audit()["balance_residual_J"]), 2e-10)
        self.assertGreaterEqual(spool.damping_loss_J, 0)

    def test_command_is_force_and_zero_dt_holds_state(self):
        s = Spool()
        state = s.position_m, s.velocity_m_s
        s.advance(0, 1)
        self.assertEqual((s.position_m, s.velocity_m_s), state)
        self.assertEqual(s.opening, 0)
        s.advance(.00005)
        self.assertGreater(s.position_m, 0)
        self.assertEqual(s.opening, 0)  # finite land before flow begins

    def test_closed_without_force(self):
        s = Spool()
        integrate(s, .1, .00005, 0)
        self.assertEqual((s.position_m, s.velocity_m_s, s.opening), (0, 0, 0))
        self.check_ledger(s)

    def test_interior_step_matches_analytic_second_order_response(self):
        p = SpoolParameters(damping_N_s_m=8.0)
        s = Spool(p)
        u, duration = .2, .01
        integrate(s, duration, .000025, u)
        alpha = p.damping_N_s_m/(2*p.mass_kg)
        omega = math.sqrt(p.spring_N_m/p.mass_kg-alpha*alpha)
        equilibrium = p.coil_force_N*u/p.spring_N_m
        exact = equilibrium*(1-math.exp(-alpha*duration)*(math.cos(omega*duration)+alpha/omega*math.sin(omega*duration)))
        self.assertAlmostEqual(s.position_m, exact, delta=1e-8)
        self.check_ledger(s)
        RESULTS["analytic_response"] = dict(solved_m=s.position_m, analytic_m=exact, error_m=abs(s.position_m-exact))

    def test_end_stop_equilibrium_and_work(self):
        s = Spool()
        integrate(s, .2, .00005, 1)
        p = s.parameters
        equilibrium = (p.coil_force_N+p.stop_stiffness_N_m*p.travel_m)/(p.spring_N_m+p.stop_stiffness_N_m)
        self.assertAlmostEqual(s.position_m, equilibrium, delta=1e-10)
        self.assertEqual(s.opening, 1)
        self.assertLess(s.max_stop_penetration_m, .00003)
        self.check_ledger(s)
        RESULTS["end_stop"] = s.snapshot()

    def test_switch_off_does_not_reset_and_spring_closes(self):
        s = Spool()
        integrate(s, .08, .00005, 1)
        before = s.position_m, s.velocity_m_s
        energy = s.energy_J()
        s.advance(0, 0)
        self.assertEqual((s.position_m, s.velocity_m_s), before)
        integrate(s, .12, .00005, 0)
        self.assertEqual(s.opening, 0)
        self.assertLess(s.energy_J(), energy*1e-7)
        self.check_ledger(s)

    def test_coil_failure_retains_mechanics_and_removes_work(self):
        s = Spool()
        integrate(s, .05, .00005, 1)
        before = s.position_m, s.velocity_m_s
        work = s.coil_work_J
        s.coil_failed = True
        self.assertEqual((s.position_m, s.velocity_m_s), before)
        integrate(s, .1, .00005, 1)
        self.assertEqual(s.opening, 0)
        self.assertEqual(s.coil_work_J, work)
        self.check_ledger(s)

    def test_passive_initial_velocity_and_stop_contacts(self):
        s = Spool(position_m=.0019, velocity_m_s=2)
        initial = s.energy_J()
        for _ in range(4000):
            s.advance(.00005, 0)
            self.assertLessEqual(s.energy_J(), initial+1e-10)
        self.assertGreater(s.max_stop_penetration_m, 0)
        self.check_ledger(s)

    def test_parameter_and_command_validation(self):
        for field in SpoolParameters.__dataclass_fields__:
            for value in (0, -1, float("inf"), float("nan")):
                with self.assertRaises(ValueError):
                    SpoolParameters(**{field: value})
        with self.assertRaises(ValueError):
            SpoolParameters(land_m=.003)
        s = Spool()
        for dt, u in ((-.1, 0), (float("nan"), 0), (.1, 2), (.1, float("nan"))):
            with self.assertRaises(ValueError):
                s.advance(dt, u)

    def test_network_initial_command_does_not_open_port(self):
        n = sample_network()
        initial = n.snapshot()
        n.update(0, valve_commands={"cylinder_supply": 1})
        self.assertEqual(n.gas.snapshot(), {k:v for k,v in initial.items() if k not in ("spools", "spool_audit")})
        self.assertEqual(n.spools["cylinder_supply"].command, 1)

    def test_invalid_network_update_is_atomic(self):
        n = sample_network()
        before = n.snapshot(), n.configuration()
        for cmd in ({"cylinder_supply": 1, "unknown": 0}, {"cylinder_supply": float("nan")}):
            with self.assertRaises(ValueError):
                n.update(.001, valve_commands=cmd)
            self.assertEqual((n.snapshot(), n.configuration()), before)
        with self.assertRaises(ValueError):
            n.update(.001, {"cylinder": 1}, {"cylinder_supply": 1})
        self.assertEqual((n.snapshot(), n.configuration()), before)

    def test_actuation_delay_and_gas_conservation(self):
        n = sample_network()
        n.update(.00005, valve_commands={"cylinder_supply": 1})
        self.assertAlmostEqual(n.pressure_Pa("cylinder"), AMBIENT_PA, delta=1e-9)
        for _ in range(10000):
            n.update(.00005, snapshot=False)
        self.assertGreater(n.pressure_Pa("cylinder"), AMBIENT_PA+100000)
        audit = n.audit()
        self.assertLess(abs(audit["mass_balance_residual_kg"]), 2e-13)
        self.assertLess(abs(audit["gas_energy_balance_residual_J"]), 1e-7)
        self.assertLess(abs(audit["exergy_balance_residual_J"]), 1e-7)
        self.assertLess(abs(n.spool_audit()["balance_residual_J"]), 2e-10)
        RESULTS["gas_coupling"] = n.snapshot()

    def test_slow_spool_delays_pressure(self):
        a = sample_network()
        b = sample_network(replace(SpoolParameters(), damping_N_s_m=80))
        for _ in range(2000):
            a.update(.00005, valve_commands={"cylinder_supply": 1}, snapshot=False)
            b.update(.00005, valve_commands={"cylinder_supply": 1}, snapshot=False)
        self.assertGreater(a.pressure_Pa("cylinder"), b.pressure_Pa("cylinder")+5000)
        RESULTS["slow_valve"] = dict(normal_pressure_Pa=a.pressure_Pa("cylinder"), slow_pressure_Pa=b.pressure_Pa("cylinder"))

    def test_failed_supply_does_not_create_pressure(self):
        n = sample_network()
        n.set_coil_failure("cylinder_supply", True)
        for _ in range(2000):
            n.update(.00005, valve_commands={"cylinder_supply": 1}, snapshot=False)
        self.assertAlmostEqual(n.pressure_Pa("cylinder"), AMBIENT_PA, delta=1e-9)
        self.assertEqual(n.spools["cylinder_supply"].position_m, 0)
        n.set_coil_failure("cylinder_supply", False)
        for _ in range(2000):
            n.update(.00005, snapshot=False)
        self.assertGreater(n.pressure_Pa("cylinder"), AMBIENT_PA)

    def test_close_command_keeps_open_spool_and_then_closes(self):
        n = sample_network()
        for _ in range(1000):
            n.update(.00005, valve_commands={"cylinder_supply": 1}, snapshot=False)
        before = n.spools["cylinder_supply"].snapshot()
        n.close_all_valves()
        self.assertEqual(n.spools["cylinder_supply"].position_m, before["position_m"])
        self.assertEqual(n.orifices["cylinder_supply"].opening, before["opening"])
        for _ in range(2400):
            n.update(.00005, snapshot=False)
        self.assertEqual(n.orifices["cylinder_supply"].opening, 0)

    def test_leaks_remain_apertures_not_coil_commands(self):
        n = sample_network()
        self.assertNotIn("cylinder_leak", n.spools)
        n.update(.001, valve_commands={"cylinder_leak": .5})
        self.assertEqual(n.orifices["cylinder_leak"].opening, .5)

    def test_substeps_report_whole_interval_mean_flow(self):
        n = sample_network()
        initial = n.edge_mass_kg["cylinder_supply"]
        n.update(.01, valve_commands={"cylinder_supply": 1})
        self.assertAlmostEqual(n.last_flows_kg_s["cylinder_supply"], (n.edge_mass_kg["cylinder_supply"]-initial)/.01)

    def test_three_step_sizes_converge_in_pressure_and_spool(self):
        traces = []
        for dt in (.0001, .00005, .000025):
            n = sample_network()
            values = []
            for i in range(round(.1/dt)):
                n.update(dt, valve_commands={"cylinder_supply": 1}, snapshot=False)
                if (i+1) % round(.001/dt) == 0:
                    values.append((n.pressure_Pa("cylinder"), n.spools["cylinder_supply"].position_m))
            traces.append(values)
        pressure_deltas = [max(abs(x[0]-y[0]) for x,y in zip(traces[i], traces[i+1])) for i in (0,1)]
        position_deltas = [max(abs(x[1]-y[1]) for x,y in zip(traces[i], traces[i+1])) for i in (0,1)]
        self.assertLess(pressure_deltas[1], pressure_deltas[0])
        self.assertLess(pressure_deltas[1], 100)
        self.assertLess(position_deltas[1], 1e-7)
        RESULTS["three_step_convergence"] = dict(dt_s=[.0001,.00005,.000025], max_pressure_difference_Pa=pressure_deltas, max_position_difference_m=position_deltas)

    def test_vehicle_topology_and_configuration(self):
        n = make_vehicle_network([f"pad_{i}" for i in range(16)],
                                 extra_chambers=[GasVolume(f"air_{i}", .000125, 200000, .0015) for i in range(8)])
        self.assertEqual(len(n.volumes), 26)
        self.assertEqual(len(n.spools), 51)
        self.assertEqual(n.spool_audit()["mechanical_state_count"], 102)
        self.assertEqual(n.configuration()["dynamic_metering_ports"]["pressure_balance_area_m2"], 0)
        json.dumps(n.configuration())
        with self.assertRaises(ValueError):
            sample_network().set_coil_failure("unknown", True)

    def test_damping_sensitivity_keeps_energy_passive(self):
        for damping in (3, 8, 12, 80):
            s = Spool(SpoolParameters(damping_N_s_m=damping))
            integrate(s, .1, .00005, 1)
            initial = s.energy_J()
            integrate(s, .1, .00005, 0)
            self.assertLess(s.energy_J(), initial)
            self.check_ledger(s)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ValveTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(schema="B9 dynamic metering-port component validation v1", provenance=PROVENANCE,
                  tests_run=result.testsRun, passed=result.wasSuccessful(),
                  failures=[str(t)+"\n"+msg for t,msg in result.failures],
                  errors=[str(t)+"\n"+msg for t,msg in result.errors], results=RESULTS)
    (OUT/"verification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)
