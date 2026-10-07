"""Analytic, conservation, causality and refinement checks for the B10 bench."""
from dataclasses import replace
import math
import unittest

from auxiliaries import (AMBIENT_PA, Fan, FanParameters, MotorParameters,
                         PumpParameters, ThermalLump, make_vehicle_network)


def bench(seconds=1, dt=.0002, **kwargs):
    net = make_vehicle_network([], include_pantograph=False,
                               reservoir_initial_pressure_Pa=AMBIENT_PA, **kwargs)
    net.automatic_compressor = False
    for _ in range(round(seconds/dt)):
        net.update(dt, valve_commands={"compressor": 1}, snapshot=False)
    return net


class AuxiliaryTests(unittest.TestCase):
    def test_fan_matches_analytic_linear_motor_response(self):
        # With negligible air drag: w(t)=tau/b*(1-exp(-b*t/J)).
        fan = Fan(replace(FanParameters(), aerodynamic_drag_Nm_s2=1e-20))
        p, mp = fan.parameters, fan.motor.parameters
        drive = mp.torque_back_emf_constant*mp.voltage_V/mp.resistance_ohm
        damping = p.bearing_drag_Nm_s+mp.torque_back_emf_constant**2/mp.resistance_ohm
        for _ in range(1000):
            fan.advance(1e-5, 1)
        exact = drive/damping*(1-math.exp(-damping*.01/p.inertia_kg_m2))
        self.assertLess(abs(fan.omega_rad_s-exact)/exact, 1e-7)
        self.assertLess(abs(fan.audit()["mechanical_balance_residual_J"]), 1e-12)
        self.assertLess(abs(fan.motor.audit()["electrical_balance_residual_J"]), 1e-12)

    def test_fan_power_failure_is_continuous_and_passive(self):
        fan = Fan()
        for _ in range(1000):
            fan.advance(.0002, 1)
        omega = fan.omega_rad_s
        energy = fan.audit()["kinetic_energy_J"]
        fan.motor.power_failed = True
        self.assertEqual(omega, fan.omega_rad_s)
        fan.advance(.0002, 1)
        self.assertGreater(fan.omega_rad_s, 0)
        self.assertLess(fan.omega_rad_s, omega)
        self.assertLess(fan.audit()["kinetic_energy_J"], energy)
        self.assertEqual(fan.motor.current_A, 0)
        self.assertLess(abs(fan.audit()["mechanical_balance_residual_J"]), 1e-12)

    def test_fan_reverse_motion_dissipates(self):
        fan = Fan()
        fan.omega_rad_s = -10
        initial = fan.audit()["kinetic_energy_J"]
        fan.motor.power_failed = True
        fan.advance(.001, 0)
        self.assertGreater(fan.omega_rad_s, -10)
        self.assertLess(fan.omega_rad_s, 0)
        self.assertAlmostEqual(fan.audit()["mechanical_balance_residual_J"], initial, places=12)

    def test_thermal_matches_analytic_response(self):
        t = ThermalLump(100, 2, .01)
        for _ in range(10000):
            t.advance(.01, .5, 100)
        exact = 293.15+50/3*(1-math.exp(-3))
        self.assertAlmostEqual(t.temperature_K, exact, places=7)
        self.assertLess(abs(t.snapshot()["balance_residual_J"]), 1e-7)

    def test_pump_mass_and_all_energy_ledgers(self):
        net = bench()
        audit = net.audit()
        self.assertGreater(net.gas.edge_mass_kg["compressor_discharge"], 1e-6)
        self.assertGreater(net.pressure_Pa("reservoir"), AMBIENT_PA+1000)
        self.assertEqual(net.gas.compressors, {})
        self.assertEqual(len(net.gas.volumes), 2)
        self.assertLess(abs(audit["mass_balance_residual_kg"]), 1e-13)
        self.assertLess(abs(audit["gas_energy_balance_residual_J"]), 1e-8)
        self.assertLess(abs(audit["exergy_balance_residual_J"]), 1e-7)
        self.assertLess(abs(net.pump.audit()["mechanical_balance_residual_J"]), 1e-8)
        self.assertLess(abs(net.pump.audit()["electrical_balance_residual_J"]), 1e-8)
        self.assertLess(abs(net.pump.audit()["gas_geometry_work_discrepancy_J"]), 1e-8)
        for f in net.fans.values():
            self.assertLess(abs(f.audit()["mechanical_balance_residual_J"]), 1e-8)
        for t in net.thermal.values():
            self.assertLess(abs(t.snapshot()["balance_residual_J"]), 1e-7)

    def test_power_failure_preserves_state_and_stops_intake(self):
        net = bench(.2)
        pump = net.pump
        state = (pump.angle_rad, pump.momentum_kg_m2_s)
        pump.motor.power_failed = True
        self.assertEqual(state, (pump.angle_rad, pump.momentum_kg_m2_s))
        work = pump.motor.electrical_work_J
        net.update(.01, snapshot=False)
        self.assertEqual(work, pump.motor.electrical_work_J)
        self.assertNotEqual(state, (pump.angle_rad, pump.momentum_kg_m2_s))
        self.assertLess(abs(pump.audit()["mechanical_balance_residual_J"]), 1e-8)

    def test_start_without_power_cannot_compress(self):
        net = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=AMBIENT_PA)
        net.pump.motor.power_failed = True
        for _ in range(100):
            net.update(.0002, snapshot=False)
        self.assertEqual(net.pump.angle_rad, 0)
        self.assertEqual(net.pressure_Pa("reservoir"), AMBIENT_PA)
        self.assertEqual(net.edge_mass_kg["compressor_discharge"], 0)

    def test_closed_intake_does_not_create_air(self):
        net = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=AMBIENT_PA)
        net.automatic_compressor = False
        net.pump.intake_failed_closed = True
        for _ in range(2500):
            net.update(.0002, valve_commands={"compressor": 1}, snapshot=False)
        self.assertEqual(net.audit()["external_mass_in_kg"], 0)
        self.assertLess(net.edge_mass_kg["compressor_discharge"], 3e-7)
        self.assertLess(abs(net.audit()["mass_balance_residual_kg"]), 1e-13)

    def test_closed_discharge_cannot_supply_reservoir(self):
        net = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=AMBIENT_PA)
        net.pump.discharge_failed_closed = True
        for _ in range(2500):
            net.update(.0002, snapshot=False)
        self.assertEqual(net.edge_mass_kg["compressor_discharge"], 0)
        self.assertEqual(net.pressure_Pa("reservoir"), AMBIENT_PA)

    def test_piston_mass_contributes_to_inertia(self):
        net = make_vehicle_network([], include_pantograph=False)
        p = net.pump.parameters
        self.assertAlmostEqual(net.pump.mass_matrix(0), p.flywheel_inertia_kg_m2)
        self.assertAlmostEqual(net.pump.mass_matrix(math.pi/2), p.flywheel_inertia_kg_m2+p.piston_mass_kg*p.crank_radius_m**2)

    def test_hysteresis_does_not_overwrite_state(self):
        net = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=560000)
        self.assertEqual(net.pump.switch(), 0)
        net.pump.pressure_switch_on = True
        self.assertEqual(net.pump.switch(), 1)
        net.pump.motor.power_failed = True
        net.update(.001, snapshot=False)
        self.assertEqual(net.pump.momentum_kg_m2_s, 0)

    def test_half_step_converges_delivered_mass(self):
        main, half, quarter = bench(.5, .0002), bench(.5, .0001), bench(.5, .00005)
        a, b, c = (n.edge_mass_kg["compressor_discharge"] for n in (main, half, quarter))
        self.assertLess(abs(a-b)/abs(b), .025)
        self.assertLess(abs(b-c), abs(a-b))

    def test_invalid_commands_and_displacements_are_atomic(self):
        net = make_vehicle_network([], include_pantograph=False)
        before = net.snapshot()
        for arguments in (dict(dt=-1), dict(dt=0, valve_commands={"compressor": 2}),
                          dict(dt=.01, valve_commands={"bay_fan": math.nan}),
                          dict(dt=.01, displacements_m={"compressor_cylinder": 0}),
                          dict(dt=.01, valve_commands={"compressor_intake": 1})):
            with self.assertRaises(ValueError):
                net.update(**arguments)
            self.assertEqual(before, net.snapshot())

    def test_close_valves_preserves_fan_commands_and_momentum(self):
        net = bench(.02)
        state = net.pump.momentum_kg_m2_s
        net.close_all_valves()
        self.assertEqual(state, net.pump.momentum_kg_m2_s)
        self.assertEqual(net.commands["bay_fan"], 1)

    def test_invalid_parameters(self):
        for obj, kwargs in ((PumpParameters, {"bore_m": 0}), (MotorParameters, {"resistance_ohm": math.inf}),
                            (FanParameters, {"inertia_kg_m2": -1}),
                            (PumpParameters, {"cut_in_pressure_Pa": 700000})):
            with self.assertRaises(ValueError):
                obj(**kwargs)

    def test_large_unwrapped_telemetry_angle_does_not_affect_phase_solve(self):
        a = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=AMBIENT_PA)
        b = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=AMBIENT_PA)
        b.pump.angle_rad = 1e12
        for _ in range(500):
            a.update(.0001, snapshot=False); b.update(.0001, snapshot=False)
        self.assertEqual(a.pump.phase_rad, b.pump.phase_rad)
        self.assertEqual(a.pump.momentum_kg_m2_s, b.pump.momentum_kg_m2_s)
        self.assertEqual(a.pressures_Pa(), b.pressures_Pa())


if __name__ == "__main__":
    unittest.main()
