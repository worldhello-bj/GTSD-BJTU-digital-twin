"""Focused live-controller tests; no GUI and no official full-case suite rerun.

From vehicle-physics: python b8/test_live_controls.py
These deliberately short un-settled integrations test control wiring/causality,
not vehicle stopping distance, contact accuracy, or mechanical calibration.
"""
from dataclasses import asdict
import math
import unittest

import live_viewer_full as live
import mujoco
import numpy as np


class LiveControlTests(unittest.TestCase):
    def vehicle(self):
        return live.LiveVehicle(settle_s=0.0)

    def test_source_topology_and_sign_are_unchanged(self):
        v = self.vehicle()
        motor_names = [v.model.actuator(i).name for i in range(v.model.nu)
                       if v.model.actuator(i).name.endswith("_motor_torque")]
        self.assertEqual(motor_names, ["A_motor_torque"])
        self.assertEqual(v.parameters["powered_bogies"], ["A"])
        self.assertEqual(v.parameters["motor_power_cap_W"], 250.0)
        self.assertEqual(len(v.airs), 8)
        self.assertEqual(len(v.pads), 16)
        self.assertEqual(len(v.network.volumes), 26)
        for car in ("A", "B"):
            self.assertEqual(v.model.joint(car + "_car_free").type[0],
                             mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(v.model.actuator_gear[v.panto.actuator_id, 0], -1.0)
        self.assertEqual(v.dt_s, live.PARAMS["timestep_s"])
        expected, _ = live.build_model(live.PARAMS | {"include_pantograph": True}, return_spec=True)
        self.assertEqual(v.xml, expected)

    def test_commands_do_not_assign_mechanical_or_gas_state(self):
        v = self.vehicle()
        qpos, qvel = v.data.qpos.copy(), v.data.qvel.copy()
        masses = {n: x.mass_kg for n, x in v.network.volumes.items()}
        for action in live.KEY_ACTIONS.values():
            v.apply_action(action)
        np.testing.assert_array_equal(v.data.qpos, qpos)
        np.testing.assert_array_equal(v.data.qvel, qvel)
        self.assertEqual(masses, {n: x.mass_kg for n, x in v.network.volumes.items()})
        self.assertEqual(v.data.time, 0.0)

    def test_drive_changes_force_and_solver_state_then_coast_cuts_torque(self):
        v = self.vehicle()
        q0 = v.data.qpos.copy()
        v.set_drive(True)
        v.step(v.steps_for_duration(0.03))
        self.assertLess(v.data.ctrl[v.motor_actuator_id], 0.0)
        self.assertGreater(abs(v.data.qvel[v.motor_dof_id]), 0.001)
        self.assertGreater(float(np.linalg.norm(v.data.qpos - q0)), 0.0)
        self.assertLessEqual(abs(v.data.ctrl[v.motor_actuator_id]), 4.0)
        self.assertLessEqual(abs(v.last_motor_start_power_W), 250.0 + 1e-10)
        v.apply_action("coast")
        v.step()
        self.assertEqual(v.data.ctrl[v.motor_actuator_id], 0.0)
        self.assertEqual(v.last_motor_start_power_W, 0.0)

    def test_brake_apply_release_pressure_and_force(self):
        v = self.vehicle()
        name = next(iter(v.pads))
        p0 = v.network.pressure_Pa(name)
        v.apply_action("brake_apply")
        v.step(v.steps_for_duration(0.08))
        filled = v.network.pressure_Pa(name)
        self.assertGreater(filled, p0 + 1000.0)
        self.assertGreater(v.data.ctrl[v.pads[name][1]], 0.0)
        self.assertEqual(v.network.orifices[name + "_supply"].opening, 1.0)
        self.assertEqual(v.network.orifices[name + "_exhaust"].opening, 0.0)
        v.apply_action("brake_release")
        v.step(v.steps_for_duration(0.08))
        self.assertLess(v.network.pressure_Pa(name), filled)
        self.assertEqual(v.network.orifices[name + "_supply"].opening, 0.0)
        self.assertEqual(v.network.orifices[name + "_exhaust"].opening, 1.0)

    def test_pressure_forces_and_endpoint_volumes_are_the_actual_coupling(self):
        v = self.vehicle()
        v.set_brake("apply")
        v.set_pantograph("raise")
        v.step(v.steps_for_duration(0.03))
        forces_at_start = v.network.forces_N()
        v.step()
        for name, (_, actuator_id) in (v.pads | v.airs).items():
            self.assertEqual(v.data.ctrl[actuator_id], forces_at_start[name])
        self.assertEqual(v.data.ctrl[v.panto.actuator_id], forces_at_start["pantograph"])
        for name, actual in v.displacements_m().items():
            node = v.network.volumes[name]
            self.assertAlmostEqual(node.displacement_m, actual, places=14)
            self.assertAlmostEqual(node.volume_m3,
                                   node.dead_volume_m3 + node.piston_area_m2 * actual, places=16)
        self.assertGreater(max(abs(v.network.volumes[n].displacement_m) for n in v.airs), 1e-6)
        self.assertAlmostEqual(v.panto.travel(v.data),
                               v.pantograph_spec.initial_cylinder_length_m
                               - float(v.data.ten_length[v.panto.tendon_id]), places=15)
        self.assertLess(abs(v.network.audit()["mass_balance_residual_kg"]), 1e-12)

    def test_panto_raise_normal_and_quick_exhaust(self):
        normal, quick = self.vehicle(), self.vehicle()
        for v in (normal, quick):
            v.set_pantograph("raise")
            v.step(v.steps_for_duration(0.10))
            self.assertGreater(v.network.pressure_Pa("pantograph"), live.AMBIENT_PA + 1000.0)
            self.assertGreater(v.data.ctrl[v.panto.actuator_id], 0.0)
        filled = normal.network.pressure_Pa("pantograph")
        normal.apply_action("panto_normal_exhaust")
        quick.apply_action("panto_quick_exhaust")
        for v in (normal, quick):
            v.step(v.steps_for_duration(0.05))
            self.assertEqual(v.network.orifices["pantograph_supply"].opening, 0.0)
        self.assertLess(normal.network.pressure_Pa("pantograph"), filled)
        self.assertLess(quick.network.pressure_Pa("pantograph"), normal.network.pressure_Pa("pantograph"))
        self.assertEqual(normal.network.orifices["pantograph_quick_exhaust"].opening, 0.0)
        self.assertEqual(quick.network.orifices["pantograph_exhaust"].opening, 0.0)
        quick.apply_action("panto_hold")
        self.assertFalse(any(quick.valve_commands()[n] for n in
                             ("pantograph_supply", "pantograph_exhaust", "pantograph_quick_exhaust")))

    def test_leaks_change_real_flow_and_close_on_second_toggle(self):
        v = self.vehicle()
        v.set_brake("apply")
        v.set_pantograph("raise")
        v.step(v.steps_for_duration(0.03))
        v.set_brake("hold")
        v.set_pantograph("hold")
        pad = next(iter(v.pads))
        edges = [pad + "_leak", "reservoir_leak", "air_A_FL_exhaust", "pantograph_leak"]
        previous = {name: v.network.edge_mass_kg[name] for name in edges}
        for leak in ("brake_line", "reservoir", "air_A_FL", "pantograph"):
            v.apply_action("toggle_" + leak + "_leak")
        v.step(v.steps_for_duration(0.005))
        for name in edges:
            self.assertEqual(v.network.orifices[name].opening, 1.0)
            self.assertGreater(v.network.edge_mass_kg[name], previous[name])
        for leak in ("brake_line", "reservoir", "air_A_FL", "pantograph"):
            v.apply_action("toggle_" + leak + "_leak")
        v.step()
        for name in edges:
            self.assertEqual(v.network.orifices[name].opening, 0.0)
        self.assertLess(abs(v.network.audit()["mass_balance_residual_kg"]), 1e-12)

    def test_external_collector_force_expires_in_simulation_time(self):
        v = self.vehicle()
        v.pulse_collector_force(-12.0, duration_s=5 * v.dt_s)
        v.step()
        self.assertEqual(v.last_collector_force_z_N, -12.0)
        self.assertEqual(v.data.xfrc_applied[v.panto.head_body_id, 2], -12.0)
        v.step(6)
        self.assertEqual(v.last_collector_force_z_N, 0.0)
        self.assertEqual(v.data.xfrc_applied[v.panto.head_body_id, 2], 0.0)
        self.assertEqual(np.count_nonzero(v.data.xfrc_applied), 0)

    def test_keyboard_map_and_input_validation(self):
        v = self.vehicle()
        for key, action in live.KEY_ACTIONS.items():
            self.assertEqual(live.action_for_key(ord(key)), action)
            self.assertEqual(live.action_for_key(ord(key.lower())), action)
        self.assertIsNone(live.action_for_key(256))
        self.assertIsNone(live.action_for_key(ord("Z")))
        for operation in (lambda: v.set_brake("bad"), lambda: v.set_pantograph("bad"),
                          lambda: v.set_leak("unknown", True), lambda: v.apply_action("unknown"),
                          lambda: v.step(-1), lambda: v.step(0.5),
                          lambda: v.pulse_collector_force(math.nan),
                          lambda: v.pulse_collector_force(duration_s=0),
                          lambda: v.steps_for_duration(-1),
                          lambda: live.presentation_steps(v.dt_s, 0, 1)):
            with self.assertRaises(ValueError):
                operation()

    def test_display_mirror_cannot_reset_the_physical_plant(self):
        v = self.vehicle()
        v.set_drive(True)
        v.step(20)
        display_model = mujoco.MjModel.from_xml_string(v.xml)
        display_data = mujoco.MjData(display_model)
        state = np.empty(mujoco.mj_stateSize(v.model, mujoco.mjtState.mjSTATE_INTEGRATION))
        live.copy_solver_state_for_display(v, display_model, display_data, state)
        np.testing.assert_array_equal(display_data.qpos, v.data.qpos)
        np.testing.assert_array_equal(display_data.qvel, v.data.qvel)
        real_qpos = v.data.qpos.copy()
        mujoco.mj_resetData(display_model, display_data)  # Models a GUI-only reset.
        display_model.opt.timestep = 0.2
        np.testing.assert_array_equal(v.data.qpos, real_qpos)
        self.assertEqual(v.model.opt.timestep, live.PARAMS["timestep_s"])
        live.copy_solver_state_for_display(v, display_model, display_data, state)
        np.testing.assert_array_equal(display_data.qpos, real_qpos)
        with self.assertRaises(ValueError):
            live.copy_solver_state_for_display(v, v.model, v.data, state)

    def test_six_doors_use_torque_without_pose_override(self):
        v = self.vehicle()
        q, dq = v.data.qpos.copy(), v.data.qvel.copy()
        v.apply_action("doors_open")
        np.testing.assert_array_equal(q, v.data.qpos)
        np.testing.assert_array_equal(dq, v.data.qvel)
        self.assertEqual(len(v.access_doors.angles(v.data)), 6)
        self.assertFalse(any(v.access_doors.latched(v.data).values()))
        v.step(v.steps_for_duration(.05))
        self.assertGreater(max(abs(x) for x in v.access_doors.angles(v.data).values()), .001)
        q, dq = v.data.qpos.copy(), v.data.qvel.copy()
        v.apply_action("doors_close")
        np.testing.assert_array_equal(q, v.data.qpos)
        np.testing.assert_array_equal(dq, v.data.qvel)
        v.step(1)
        self.assertEqual(sum(v.telemetry()["warning_counts"]), 0)

    def test_presentation_batches_do_not_change_physics(self):
        a, b = self.vehicle(), self.vehicle()
        self.assertEqual(live.presentation_steps(a.dt_s, 30, 1), round(1 / (30 * a.dt_s)))
        self.assertEqual(live.presentation_steps(a.dt_s, 60, 0.5), round(0.5 / (60 * a.dt_s)))
        a.set_drive(True)
        b.set_drive(True)
        a.step(30)
        b.step(10)
        b.step(20)
        np.testing.assert_array_equal(a.data.qpos, b.data.qpos)
        np.testing.assert_array_equal(a.data.qvel, b.data.qvel)
        self.assertEqual(a.network.pressures_Pa(), b.network.pressures_Pa())
        self.assertEqual(a.dt_s, b.dt_s)
        self.assertEqual(asdict(a.commands), asdict(b.commands))
        self.assertEqual(sum(a.telemetry()["warning_counts"]), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
