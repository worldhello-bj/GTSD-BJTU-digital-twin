"""B8 live forward dynamics. Keyboard commands never prescribe mechanical state.

Run ``python b8/live_viewer_full.py --help`` from vehicle-physics. The headless
LiveVehicle API has no dependency on a window or hardware connection.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import queue
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "vendor"))
import mujoco
import numpy as np

from full_model import PARAMS, model as build_model
from pneumatics import AMBIENT_PA, GasVolume, make_vehicle_network


@dataclass
class OperatorCommands:
    drive: bool = False
    brake: str = "release"
    pantograph: str = "normal_exhaust"
    brake_cylinder_branch_leak: bool = False
    doors: str = "close"
    reservoir_leak: bool = False
    air_A_FL_leak: bool = False
    pantograph_leak: bool = False


KEY_ACTIONS = {
    "W": "drive", "S": "coast", "B": "brake_apply", "N": "brake_release",
    "R": "panto_raise", "H": "panto_hold", "E": "panto_normal_exhaust",
    "Q": "panto_quick_exhaust", "1": "toggle_brake_cylinder_branch_leak",
    "2": "toggle_reservoir_leak", "3": "toggle_air_A_FL_leak",
    "4": "toggle_pantograph_leak", "T": "collector_pulse",
    "O": "doors_open", "K": "doors_close",
}


def action_for_key(keycode: int) -> str | None:
    """MuJoCo/GLFW letter key codes; unknown keys keep viewer defaults."""
    if 0 <= keycode < 128:
        return KEY_ACTIONS.get(chr(keycode).upper())
    return None


class LiveVehicle:
    """Single-threaded, controllable, finite-volume/mechanical forward solver.

    The model and pneumatic parameters match simulate_full.py's nominal build.
    No geometry, inertia, stiffness, contact, or integration settings are tuned
    here. ``settle_s`` integrates passive dynamics; it never sets a settled pose.
    """

    def __init__(self, *, settle_s: float = 6.0):
        if not math.isfinite(settle_s) or settle_s < 0:
            raise ValueError("settle_s must be finite and nonnegative")
        self.parameters = PARAMS | {"include_pantograph": True}
        self.xml, self.pantograph_spec, self.door_spec = build_model(self.parameters, return_all_specs=True)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.dt_s = float(self.model.opt.timestep)
        mujoco.mj_forward(self.model, self.data)
        self.contact_law = None
        if self.parameters.get("compliant_wheel_rail"):
            from compliant_contact import CompliantWheelRail, ContactParameters
            p = self.parameters
            self.contact_law = CompliantWheelRail(self.model,ContactParameters(p["contact_stiffness_Npm"],p["contact_dissipation_spm"],p["contact_regularization_mps"],p["wheel_rail_mu"],p["contact_distance_tolerance"]))
        self.panto = self.pantograph_spec.bind(self.model)
        self.access_doors = self.door_spec.bind(self.model) if self.door_spec else None
        self.pads = {
            self.model.body(i).name: (
                int(self.model.joint(self.model.body(i).name + "_slide").qposadr[0]),
                int(self.model.actuator(self.model.body(i).name + "_force").id),
            )
            for i in range(1, self.model.nbody)
            if self.model.body(i).name.endswith("_pad")
        }
        self.airs = {
            f"air_{car}_{side}": (
                int(self.model.tendon(f"air_{car}_{side}").id),
                int(self.model.actuator(f"air_{car}_{side}_force").id),
            )
            for car in ("A", "B") for side in ("FL", "FR", "RL", "RR")
        }
        self.air_reference_length_m = {
            name: float(self.data.ten_length[ids[0]]) for name, ids in self.airs.items()
        }
        self.motor_actuator_id = int(self.model.actuator("A_motor_torque").id)
        self.motor_dof_id = int(self.model.joint("A_motor_z").dofadr[0])
        self.car_body_ids = {car: int(self.model.body(car + "_carbody").id)
                             for car in ("A", "B")}
        bellows = [GasVolume(name, self.parameters["airspring_dead_volume_m3"],
                            self.parameters["airspring_initial_pressure_Pa"],
                            self.parameters["airspring_area_m2"], 0.0)
                   for name in self.airs]
        self.network = make_vehicle_network(
            self.pads, extra_chambers=bellows,
            brake_dead_volume_m3=self.parameters["brake_dead_volume_m3"],
            brake_area_m2=self.parameters["brake_piston_area_m2"],
            pantograph_area_m2=self.pantograph_spec.params["cylinder_area_m2"],
            initial_displacements_m=self.displacements_m(),
            reservoir_initial_pressure_Pa=AMBIENT_PA + 500000.0,
        )
        self.commands = OperatorCommands()
        self.drive_fraction = 0.0
        self.drive_ramp_s = 0.15
        self.collector_force_z_N = 0.0
        self.collector_force_until_s = 0.0
        self.last_applied_forces_N = self.network.forces_N()
        self.last_motor_start_power_W = 0.0
        self.last_collector_force_z_N = 0.0
        self.step_count = 0
        self.step(self.steps_for_duration(settle_s))
        self.settled_time_s = float(self.data.time)
        self.initial_A_x_m = float(self.data.body("A_carbody").xpos[0])

    def steps_for_duration(self, duration_s: float) -> int:
        """Round requested duration to whole fixed physics steps; never change dt."""
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError("duration_s must be finite and nonnegative")
        return int(round(duration_s / self.dt_s))

    def displacements_m(self) -> dict[str, float]:
        """Read only actual solver endpoints, including signed panto retraction."""
        return ({name: float(self.data.qpos[ids[0]]) for name, ids in self.pads.items()}
                | {name: float(self.data.ten_length[ids[0]]) - self.air_reference_length_m[name]
                   for name, ids in self.airs.items()}
                | {"pantograph": self.panto.travel(self.data)})

    def set_drive(self, enabled: bool) -> None:
        self.commands.drive = bool(enabled)

    def set_brake(self, mode: str) -> None:
        if mode not in ("apply", "release", "hold"):
            raise ValueError("brake must be apply, release, or hold")
        self.commands.brake = mode

    def set_pantograph(self, mode: str) -> None:
        if mode not in ("raise", "hold", "normal_exhaust", "quick_exhaust"):
            raise ValueError("pantograph must be raise, hold, normal_exhaust, or quick_exhaust")
        self.commands.pantograph = mode

    def set_leak(self, name: str, enabled: bool) -> None:
        if name == "brake_line": name = "brake_cylinder_branch"  # Legacy alias, never a train brake pipe.
        if name not in ("brake_cylinder_branch", "reservoir", "air_A_FL", "pantograph"):
            raise ValueError("unknown leak; choose brake_cylinder_branch, reservoir, air_A_FL, pantograph")
        setattr(self.commands, name + "_leak", bool(enabled))

    def pulse_collector_force(self, force_z_N: float = -50.0,
                              duration_s: float = 0.2) -> None:
        """World-z force at collector COM. Repeated commands replace the pulse."""
        if not math.isfinite(force_z_N) or not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("finite force and positive finite duration_s required")
        self.collector_force_z_N = float(force_z_N)
        self.collector_force_until_s = float(self.data.time) + duration_s

    def apply_action(self, action: str) -> None:
        if action in ("drive", "coast"):
            self.set_drive(action == "drive")
        elif action in ("brake_apply", "brake_release"):
            self.set_brake(action.removeprefix("brake_"))
        elif action.startswith("panto_"):
            self.set_pantograph(action.removeprefix("panto_"))
        elif action.startswith("toggle_"):
            name = action.removeprefix("toggle_").removesuffix("_leak")
            if name == "brake_line": name = "brake_cylinder_branch"
            if name not in ("brake_cylinder_branch", "reservoir", "air_A_FL", "pantograph"):
                raise ValueError("unknown action: " + action)
            self.set_leak(name, not getattr(self.commands, name + "_leak"))
        elif action in ("doors_open", "doors_close"):
            if self.access_doors is None: raise ValueError("Door mechanisms are not in this model")
            if action == "doors_open": self.access_doors.release_latch(self.data)
            self.commands.doors = action.removeprefix("doors_")
        elif action == "collector_pulse":
            self.pulse_collector_force()
        else:
            raise ValueError("unknown action: " + action)

    def valve_commands(self) -> dict[str, float]:
        """Set every path explicitly so an earlier supply/exhaust cannot persist."""
        cmd = {name: 0.0 for name in self.network.orifices}
        for name in self.pads:
            cmd[name + "_supply"] = float(self.commands.brake == "apply")
            cmd[name + "_exhaust"] = float(self.commands.brake == "release")
            cmd[name + "_leak"] = float(self.commands.brake_cylinder_branch_leak)
        cmd["pantograph_supply"] = float(self.commands.pantograph == "raise")
        cmd["pantograph_exhaust"] = float(self.commands.pantograph == "normal_exhaust")
        cmd["pantograph_quick_exhaust"] = float(self.commands.pantograph == "quick_exhaust")
        cmd["pantograph_leak"] = float(self.commands.pantograph_leak)
        cmd["reservoir_leak"] = float(self.commands.reservoir_leak)
        cmd["air_A_FL_exhaust"] = float(self.commands.air_A_FL_leak)
        return cmd

    def step(self, count: int = 1) -> None:
        """Pressure force -> mj_step -> measured volumes -> gas flow, as B8 suite.

        MuJoCo alone advances qpos/qvel. Operator input changes only controls,
        valve openings, and the collector external force. The pneumatic update
        follows the same first-order partitioned coupling as simulate_full.py.
        """
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError("step count must be a nonnegative integer")
        valves = self.valve_commands()
        for _ in range(count):
            self.last_applied_forces_N = self.network.forces_N()
            if self.access_doors:
                if self.commands.doors == "open":
                    self.access_doors.set_opening_torques(self.data, {d.key:d.torque_limit_Nm for d in self.door_spec.doors})
                else:
                    self.access_doors.release(self.data)
                    self.access_doors.try_latch(self.data)
            for name, ids in self.pads.items():
                self.data.ctrl[ids[1]] = self.last_applied_forces_N[name]
            for name, ids in self.airs.items():
                self.data.ctrl[ids[1]] = self.last_applied_forces_N[name]
            self.panto.set_force(self.data, self.last_applied_forces_N["pantograph"])
            # A's one motor has a 0.15 s command ramp; coast cuts torque next step.
            self.drive_fraction = (min(1.0, self.drive_fraction + self.dt_s / self.drive_ramp_s)
                                   if self.commands.drive else 0.0)
            omega = float(self.data.qvel[self.motor_dof_id])
            torque = -min(self.parameters["motor_torque_cap_Nm"],
                          self.parameters["motor_power_cap_W"] / max(abs(omega), 1e-9))
            torque *= self.drive_fraction
            self.data.ctrl[self.motor_actuator_id] = torque
            self.last_motor_start_power_W = torque * omega
            self.data.xfrc_applied[self.panto.head_body_id, :] = 0.0
            self.last_collector_force_z_N = (
                self.collector_force_z_N if self.data.time < self.collector_force_until_s else 0.0)
            self.data.xfrc_applied[self.panto.head_body_id, 2] = self.last_collector_force_z_N
            if self.contact_law is not None:
                self.data.qfrc_applied[:] = self.contact_law.evaluate(self.data)["qforce"]
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.network.update(self.dt_s, self.displacements_m(), valves, snapshot=False)
            self.step_count += 1
            if not (np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()):
                raise RuntimeError("non-finite mechanical state; stop simulation")
            if np.any(self.data.warning.number):
                raise RuntimeError("MuJoCo warning; stop simulation: " + str(self.data.warning.number.tolist()))

    def telemetry(self) -> dict:
        """SI values; pressures absolute, applied forces refer to the last step."""
        pressures = self.network.pressures_Pa()
        first_pad = next(iter(self.pads))
        return {
            "simulation_time_s": float(self.data.time),
            "operator_time_s": float(self.data.time) - self.settled_time_s,
            "physics_dt_s": self.dt_s,
            "A_position_m": self.data.body("A_carbody").xpos.tolist(),
            "B_position_m": self.data.body("B_carbody").xpos.tolist(),
            "A_travel_m": float(self.data.body("A_carbody").xpos[0]) - self.initial_A_x_m,
            "A_world_x_speed_mps": float(self.data.joint("A_car_free").qvel[0]),
            "B_world_x_speed_mps": float(self.data.joint("B_car_free").qvel[0]),
            "motor_A_torque_Nm": float(self.data.ctrl[self.motor_actuator_id]),
            "motor_A_rotor_omega_radps": float(self.data.qvel[self.motor_dof_id]),
            "motor_A_power_at_step_start_W": self.last_motor_start_power_W,
            "motor_power_cap_W": self.parameters["motor_power_cap_W"],
            "reservoir_absolute_pressure_Pa": pressures["reservoir"],
            "representative_brake_chamber": first_pad,
            "representative_brake_absolute_pressure_Pa": pressures[first_pad],
            "pantograph_absolute_pressure_Pa": pressures["pantograph"],
            "pantograph_retraction_m": self.panto.travel(self.data),
            "pantograph_applied_retraction_force_N": self.last_applied_forces_N["pantograph"],
            "collector_pivot_height_m": float(self.data.body("panto_collector").xpos[2]),
            "collector_contact_normal_N": self.panto.contact_force(self.model, self.data),
            "collector_last_applied_world_z_force_N": self.last_collector_force_z_N,
            "coupler_length_m": float(self.data.ten_length[self.model.tendon("coupler").id]),
            "all_absolute_pressures_Pa": pressures,
            "all_volumes_m3": {n: v.volume_m3 for n, v in self.network.volumes.items()},
            "commands": asdict(self.commands),
            "access_doors": self.access_doors.telemetry(self.data) if self.access_doors else {},
            "warning_counts": self.data.warning.number.tolist(),
            "air_mass_balance_residual_kg": self.network.audit()["mass_balance_residual_kg"],
        }


def presentation_steps(physics_dt_s: float, fps: float, speed: float) -> int:
    """Presentation pacing changes batch size, never the mechanical timestep."""
    if not all(math.isfinite(x) and x > 0 for x in (physics_dt_s, fps, speed)):
        raise ValueError("physics_dt_s, fps, and speed must be finite and positive")
    return max(1, round(speed / (fps * physics_dt_s)))


def copy_solver_state_for_display(vehicle: LiveVehicle, display_model, display_data,
                                  state_buffer: np.ndarray) -> None:
    """Copy the current solved state into a separate presentation-only model.

    Passive-viewer reset/drag/sliders must not rewrite the physical plant. This
    one-way copy never targets vehicle.data, never steps the display model, and
    never supplies a trajectory to the real forward solver.
    """
    if display_data is vehicle.data or display_model is vehicle.model:
        raise ValueError("the viewer must use its own model and data")
    state_kind = mujoco.mjtState.mjSTATE_INTEGRATION
    mujoco.mj_getState(vehicle.model, vehicle.data, state_buffer, state_kind)
    mujoco.mj_setState(display_model, display_data, state_buffer, state_kind)
    mujoco.mj_forward(display_model, display_data)


def run_viewer(vehicle: LiveVehicle, *, fps: float = 30.0, speed: float = 1.0) -> None:
    # Import only here: headless use never initializes GLFW/a display connection.
    import mujoco.viewer
    pending: queue.SimpleQueue[str] = queue.SimpleQueue()
    paused = False

    def key_callback(keycode: int) -> None:
        action = "pause" if keycode == ord("P") else action_for_key(keycode)
        if action:
            pending.put(action)

    batch = presentation_steps(vehicle.dt_s, fps, speed)
    display_model = mujoco.MjModel.from_xml_string(vehicle.xml)
    display_data = mujoco.MjData(display_model)
    state_buffer = np.empty(mujoco.mj_stateSize(vehicle.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    copy_solver_state_for_display(vehicle, display_model, display_data, state_buffer)
    print("SIMULATION ONLY, no hardware connection. W drive / S coast; B brake / N release;\n"
          "R panto raise / H hold / E normal exhaust / Q quick exhaust;\n"
          "1 brake-cylinder branch leak / 2 reservoir leak / 3 A-FL air-spring leak / 4 panto leak;\n"
          "O unlock/open access doors / K release/close/guarded latch;\nT collector -50 N for 0.2 simulated seconds; P pause; Esc close.\n"
          "Physics dt remains fixed. Console telemetry once per wall second.", flush=True)
    wall_start = time.monotonic()
    sim_start = float(vehicle.data.time)
    next_report = wall_start
    with mujoco.viewer.launch_passive(display_model, display_data,
                                      key_callback=key_callback) as viewer:
        with viewer.lock():
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -15.0
            viewer.cam.distance = 8.0
            viewer.cam.lookat[:] = [3.0, 0.0, 0.8]
        while viewer.is_running():
            frame_start = time.monotonic()
            while not pending.empty():
                action = pending.get()
                if action == "pause":
                    paused = not paused
                else:
                    vehicle.apply_action(action)
                print("command:", action, "paused:", paused, flush=True)
            with viewer.lock():
                if not paused:
                    vehicle.step(batch)
                copy_solver_state_for_display(vehicle, display_model, display_data, state_buffer)
                viewer.cam.lookat[0] = (vehicle.data.body("A_carbody").xpos[0]
                                        + vehicle.data.body("B_carbody").xpos[0]) / 2
            viewer.sync()
            now = time.monotonic()
            if now >= next_report:
                report = vehicle.telemetry()
                report["achieved_simulated_seconds_per_wall_second"] = (
                    (vehicle.data.time - sim_start) / max(now - wall_start, 1e-9))
                print(json.dumps(report, sort_keys=True), flush=True)
                next_report = now + 1.0
            # A slow machine runs slower rather than changing dt or skipping poses.
            time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - frame_start)))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run controls without opening a window")
    parser.add_argument("--settle-s", type=float, default=6.0, help="passive solver settling, simulated seconds")
    parser.add_argument("--duration-s", type=float, default=1.0, help="headless controlled duration, simulated seconds")
    parser.add_argument("--action", action="append", choices=sorted(set(KEY_ACTIONS.values())), default=[],
                        help="initial operator action; repeat to combine (default: coast, vented)")
    parser.add_argument("--fps", type=float, default=30.0, help="viewer target frames per wall second")
    parser.add_argument("--speed", type=float, default=1.0, help="target simulated seconds per wall second")
    args = parser.parse_args(argv)
    try:
        presentation_steps(float(PARAMS["timestep_s"]), args.fps, args.speed)
        if not args.headless and sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            parser.error("no GUI display detected; use --headless, or launch on a graphical desktop")
        print("Building the frozen B8 model; passive settling is forward integration.", file=sys.stderr, flush=True)
        vehicle = LiveVehicle(settle_s=args.settle_s)
        for action in args.action:
            vehicle.apply_action(action)
        if args.headless:
            vehicle.step(vehicle.steps_for_duration(args.duration_s))
            print(json.dumps(vehicle.telemetry(), indent=2, sort_keys=True))
        else:
            run_viewer(vehicle, fps=args.fps, speed=args.speed)
    except (ValueError, RuntimeError) as exc:
        parser.exit(1, f"Simulation stopped: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
