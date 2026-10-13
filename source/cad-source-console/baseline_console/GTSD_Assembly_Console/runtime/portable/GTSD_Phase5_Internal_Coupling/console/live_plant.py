"""Interactive adapter of the frozen v5 integrated experiment, in SI units.

Commands select electrical, valve and external laboratory-force ports only.
There is no qpos animation, historical vehicle import or device/network I/O.
"""
from __future__ import annotations

import copy
from dataclasses import asdict, is_dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent
INTEGRATED = SOURCE / "integrated"
if str(INTEGRATED) not in sys.path:
    sys.path.insert(0, str(INTEGRATED))
import integrated_model as im
from run_integrated import apply_head, WHEELS

SCHEMA = "gtsd-v5-live-plant-checkpoint-v1"
STATE_KIND = mujoco.mjtState.mjSTATE_INTEGRATION
COMMANDS = {
    "drive": {"label": "牵引电源", "args": {"enabled": {"type": "boolean"}}},
    "brake": {"label": "等效气制动阀", "args": {"mode": {"type": "string", "enum": ["apply", "hold", "release"]}}},
    "head_load": {"label": "弓头外部试验力（世界竖直，非升弓气缸）", "args": {"force_N": {"type": "number", "minimum": -30., "maximum": 30.}}},
}
CAPABILITIES = {
    "mechanical": True, "new_geometry": True, "integrated_dof": 15,
    "traction_RL_motor": True, "fixed_volume_air_brake": True,
    "dynamic_valve_spools": True, "pantograph_mechanics": True,
    "external_head_test_force": True, "pantograph_pneumatic_actuator": False,
    "doors": False, "auxiliaries": False, "cables": False,
    "calibrated_hardware_sensors": False, "hardware_control": False,
}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def sha(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def pack(value):
    """Data-only typed state. Class names cannot instantiate anything on load."""
    if isinstance(value, np.ndarray):
        return {"array": value.tolist(), "dtype": str(value.dtype), "shape": list(value.shape)}
    if isinstance(value, np.generic):
        return value.item()
    if value is None or type(value) in (bool, str, int, float):
        return value
    if type(value) is dict:
        return {k: pack(v) for k, v in value.items()}
    if type(value) is list:
        return [pack(v) for v in value]
    if hasattr(value, "__dict__"):
        return {"record": type(value).__module__ + "." + type(value).__qualname__,
                "fields": {k: pack(v) for k, v in vars(value).items()}}
    raise TypeError("unsupported checkpoint state type " + str(type(value)))


def unpack_like(value, template, path="state"):
    """Rehydrate only the already-constructed local schema, with exact fields."""
    if isinstance(template, np.ndarray):
        if type(value) is not dict or set(value) != {"array", "dtype", "shape"} or value["shape"] != list(template.shape) or value["dtype"] != str(template.dtype):
            raise ValueError(path + " array schema mismatch")
        arr = np.asarray(value["array"], dtype=template.dtype)
        if arr.shape != template.shape or not np.isfinite(arr).all():
            raise ValueError(path + " invalid array")
        return arr
    if isinstance(template, np.generic):
        template = template.item()
    if template is None:
        if value is not None:
            raise ValueError(path + " must be null")
        return value
    if type(template) in (bool, str, int, float):
        expected = type(template)
        if expected is float:
            valid = type(value) in (float, int) and math.isfinite(value)
        else:
            valid = type(value) is expected
        if not valid:
            raise ValueError(path + " scalar type mismatch")
        return float(value) if expected is float else value
    if type(template) is list:
        if type(value) is not list or len(value) != len(template):
            raise ValueError(path + " list schema mismatch")
        return [unpack_like(v, t, path + "[]") for v, t in zip(value, template)]
    if type(template) is dict:
        if type(value) is not dict or set(value) != set(template):
            raise ValueError(path + " keys mismatch")
        return {k: unpack_like(value[k], t, path + "." + k) for k, t in template.items()}
    record = type(template).__module__ + "." + type(template).__qualname__
    if type(value) is not dict or set(value) != {"record", "fields"} or value["record"] != record:
        raise ValueError(path + " record schema mismatch")
    if is_dataclass(template) and template.__dataclass_params__.frozen:
        if value != pack(template):
            raise ValueError(path + " immutable parameters changed")
        return template
    fields = unpack_like(value["fields"], vars(template), path + ".fields")
    for key, field in fields.items():
        setattr(template, key, field)
    return template


def pneumatic_configuration(brake):
    c = copy.deepcopy(brake.network.configuration())
    for value in c["orifices"].values():
        value.pop("opening")
    for value in c["compressors"].values():
        value.pop("opening")
    dynamic = c["dynamic_metering_ports"]
    for key in ("requested_coil_force_fractions", "coil_failures", "audit"):
        dynamic.pop(key)
    # These derived gas constants must not be silently altered by a checkpoint.
    c["derived_constants"] = {k: getattr(brake.network.gas, k) for k in ("RT", "cv", "cp")}
    return c


class LivePlant:
    def __init__(self, dt=im.DEFAULT_DT, *, builder=None, binding_builder=None, identity_extra=None):
        self.model, xml, self.bp, self.pp, self.mp, geometry = (builder or im.build_model)(dt)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.dt = float(self.model.opt.timestep)
        self.brake = im.coupling.FixedVolumeBrake(im.coupling.BrakeParameters(gas_substep_s=min(.0005, dt)))
        self.motors = [im.coupling.Motor(self.mp), im.coupling.Motor(self.mp)]
        self.wheel_dofs = np.array([self.model.jnt_dofadr[self.model.joint(f"wheel_spin_{i}").id] for i in (1, 2)])
        self.axle_actuators = np.array([self.model.actuator(f"axle_torque_{i}").id for i in (1, 2)])
        self.commands = {"drive_enabled": False, "brake_mode": "hold", "head_force_N": 0.}
        self.steps = 0
        self.work = np.zeros(5)
        self.last_step_work = np.zeros(5)
        self.constraint_work_parts = np.zeros(2)
        self.last_motor = [motor.preview(dt, 0., False) for motor in self.motors]
        self.last_brake = [self.brake.force_torque(i, 0.) for i in (1, 2)]
        self.last_head_force = 0.
        self.last_generalized_head_force = np.zeros(self.model.nv)
        self.last_constraint_generalized = np.zeros(self.model.nv)
        self.last_limit_generalized = np.zeros(self.model.nv)
        self.last_mid_velocity = np.zeros(self.model.nv)
        self.last_corrector_error = np.zeros(2)
        self.warnings = np.zeros(len(self.data.warning), dtype=np.int64)
        self.energy0 = float(sum(self.data.energy))
        self.last_guard = {"feedback_residual_rad_s": 0., "speed_tolerance_rad_s": 1e-7,
                           "per_axle_brake_work_J": [0., 0.], "positive_brake_work_tolerance_J": 1e-12,
                           "feedback_converged": True, "discrete_brake_passive": True}
        self.binding = (binding_builder or im.binding)(self.model, geometry)
        self.identity = self._identity(xml)
        self.identity["extensions"] = copy.deepcopy(identity_extra or {})
        self.capabilities = CAPABILITIES | {"integrated_dof": self.model.nv}
        self.command_registry = copy.deepcopy(COMMANDS)
        self.failed = False

    def _identity(self, xml):
        files = [INTEGRATED / "integrated_model.py", INTEGRATED / "run_integrated.py", ROOT / "live_plant.py"]
        files += sorted(p for p in (INTEGRATED / "vendor").rglob("*")
                        if p.is_file() and "__pycache__" not in str(p))
        files += [SOURCE / p for p in ("assembly_contract.json", "scripts/component_bogie.py",
                                       "scripts/component_pantograph.py", "components/pantograph_manifest.json")]
        return {"schema": "gtsd-v5-model-identity-v1", "mujoco_version": mujoco.__version__,
                "xml_sha256": hashlib.sha256(xml.encode()).hexdigest(),
                "source_geometry_sha256": im.FROZEN_GEOMETRY_SHA,
                "source_sha256": {str(p.relative_to(SOURCE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                "dimensions": {"nq": self.model.nq, "nv": self.model.nv, "nu": self.model.nu, "nbody": self.model.nbody},
                "dt_s": self.dt, "parameters": {"bogie": self.bp, "pantograph": self.pp, "motor": asdict(self.mp),
                                                "brake": asdict(self.brake.p)},
                "pneumatic_configuration": pneumatic_configuration(self.brake),
                "state_kind": int(STATE_KIND), "state_size": mujoco.mj_stateSize(self.model, STATE_KIND)}

    def command(self, name, args):
        # Share exact schemas with HTTP/controller; direct callers are validated too.
        from controller import validate_command
        validate_command({"session_id": "plant", "request_id": "direct", "action": name, "args": args}, COMMANDS)
        if self.failed:
            raise ValueError("plant stopped after a rejected numerical interval")
        if name == "drive":
            self.commands["drive_enabled"] = args["enabled"]
        elif name == "brake":
            self.commands["brake_mode"] = args["mode"]
        elif name == "head_load":
            self.commands["head_force_N"] = float(args["force_N"])
        else:
            raise ValueError("unsupported live plant command")

    def step(self, n=1):
        if type(n) is not int or not 1 <= n <= 2500:
            raise ValueError("invalid step count")
        if self.failed:
            raise RuntimeError("plant stopped after rejected numerical interval")
        for _ in range(n):
            try:
                self._step()
            except Exception:
                # The controller retains its last accepted immutable state. A failed
                # gas/mechanical trial is never published/exported or advanced again.
                self.failed = True
                raise

    def _step(self):
        m, d, h, brake, motors = self.model, self.data, self.dt, self.brake, self.motors
        mujoco.mj_forward(m, d)
        mujoco.mj_rnePostConstraint(m, d)
        mujoco.mj_subtreeVel(m, d)
        old_p = [brake.network.pressure_Pa(f"brake_axle_{i}") for i in (1, 2)]
        brake.advance(h, self.commands["brake_mode"] == "apply", self.commands["brake_mode"] == "release")
        mid_p = [.5 * (old_p[i-1] + brake.network.pressure_Pa(f"brake_axle_{i}")) for i in (1, 2)]
        v0, ten0 = d.qvel.copy(), d.ten_velocity.copy()
        omega0 = v0[self.wheel_dofs].copy()
        guess = omega0.copy()
        state = np.zeros(mujoco.mj_stateSize(m, STATE_KIND))
        mujoco.mj_getState(m, d, state, STATE_KIND)
        for iteration in range(4):
            used_feedback = guess.copy()
            ms = [motors[i].preview(h, float(guess[i]), self.commands["drive_enabled"]) for i in range(2)]
            bs = [brake.force_torque(i+1, float(guess[i]), mid_p[i]) for i in range(2)]
            mujoco.mj_setState(m, d, state, STATE_KIND)
            mujoco.mj_forward(m, d)
            # Set this module's named force ports only. Future modules own theirs.
            d.ctrl[self.axle_actuators] = [ms[i]["torque_Nm"] + bs[i]["torque_Nm"] for i in range(2)]
            qext = apply_head(m, d, self.commands["head_force_N"])
            mujoco.mj_forward(m, d)
            mujoco.mj_step(m, d)
            actual_constraint = d.qfrc_constraint.copy()
            actual_limit = np.zeros(m.nv)
            is_limit = np.isin(d.efc_type, [int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT), int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)])
            mujoco.mj_mulJacTVec(m, d, actual_limit, np.where(is_limit, d.efc_force, 0.))
            observed = .5 * (omega0 + d.qvel[self.wheel_dofs])
            if np.max(abs(observed - guess)) < 1e-10:
                break
            guess = observed
        vmid = .5 * (v0 + d.qvel)
        wmid = vmid[self.wheel_dofs]
        self.last_guard = im.coupling.validate_discrete_coupling_step(h, used_feedback, wmid, [b["torque_Nm"] for b in bs])
        self.last_corrector_error = wmid - used_feedback
        self.last_constraint_generalized, self.last_limit_generalized = actual_constraint.copy(), actual_limit.copy()
        self.last_mid_velocity = vmid.copy()
        self.constraint_work_parts += h * np.array([float(np.dot(actual_limit, vmid)), float(np.dot(actual_constraint - actual_limit, vmid))])
        mujoco.mj_forward(m, d)
        self.last_generalized_head_force = apply_head(m, d, self.commands["head_force_N"])
        mujoco.mj_forward(m, d)
        ten1 = d.ten_velocity.copy()
        damping = -h * float(np.dot(m.dof_damping, vmid * vmid) + np.dot(m.tendon_damping, (.5 * (ten0 + ten1)) ** 2))
        head_work = h * float(np.dot(qext, vmid))
        constraint_work = h * float(np.dot(actual_constraint, vmid))
        motor_work = sum(ms[i]["torque_Nm"] * wmid[i] * h for i in range(2))
        brake_work = sum(bs[i]["torque_Nm"] * wmid[i] * h for i in range(2))
        for i in range(2):
            motors[i].commit(ms[i], ms[i]["torque_Nm"] * float(wmid[i]) * h)
        self.last_step_work = np.array([motor_work, brake_work, head_work, damping, constraint_work])
        self.work += self.last_step_work
        brake.brake_heat_J = -self.work[1]
        self.last_motor, self.last_brake = ms, bs
        self.last_head_force = self.commands["head_force_N"]
        self.steps += 1
        self.warnings = np.maximum(self.warnings, [w.number for w in d.warning])
        if not np.isfinite(np.r_[d.qpos, d.qvel, d.qacc]).all():
            raise ArithmeticError("nonfinite mechanical state")

    def snapshot(self):
        if self.failed:
            raise ValueError("cannot publish a rejected numerical interval")
        m, d = self.model, self.data
        mujoco.mj_rnePostConstraint(m, d)
        mujoco.mj_subtreeVel(m, d)
        cn, cf, _ = im.panto.contacts(m, d)
        base = m.body("panto_base").id
        telemetry = im.helpers.telemetry(m, d, self.bp, d.ctrl[self.axle_actuators], 0., float(sum(self.work[:3])), float(-self.work[3]))
        telemetry.update({"panto_contact_normal_N": cn, "panto_contact_world_force_N": cf.tolist(),
                          "panto_reaction_on_payload_world_N": (-d.cfrc_int[base, 3:]).tolist(),
                          "head_external_test_force_N": self.last_head_force,
                          "pneumatic": self.brake.snapshot(), "motors": [v.audit() for v in self.motors],
                          "last_motor": copy.deepcopy(self.last_motor), "last_brake": copy.deepcopy(self.last_brake),
                          "energy_work_columns_J": self.work.tolist(),
                          "energy_residual_after_reported_work_J": float(sum(d.energy)) - self.energy0 - float(sum(self.work)),
                          "energy_note": "Signed soft-contact/limit work included; residual is reported, not claimed as complete whole-system energy closure.",
                          "constraint_limit_and_other_work_J": self.constraint_work_parts.tolist(),
                          "discrete_acceptance": copy.deepcopy(self.last_guard),
                          "signal_status": "IDEAL_SOLVER_SI_OBSERVATIONS_NOT_CALIBRATED_HARDWARE_SENSOR", "hardware_raw": None})
        return {"schema": "gtsd-v5-live-plant-state-v1", "time_s": float(d.time), "steps": self.steps,
                "dt_s": self.dt, "body_poses": im.poses(m, d), "qpos": d.qpos.tolist(), "qvel": d.qvel.tolist(),
                "commands": copy.deepcopy(self.commands), "telemetry": telemetry,
                "capabilities": copy.deepcopy(self.capabilities), "command_registry": copy.deepcopy(self.command_registry),
                "model_identity_sha256": sha(self.identity), "source_geometry_sha256": im.FROZEN_GEOMETRY_SHA,
                "warning_counts": self.warnings.tolist(), "calibration_status": "UNCALIBRATED_ENGINEERING_EQUIVALENT"}

    def _dynamic(self):
        names = ("commands", "steps", "work", "last_step_work", "constraint_work_parts", "last_motor", "last_brake",
                 "last_head_force", "last_generalized_head_force", "last_constraint_generalized", "last_limit_generalized",
                 "last_mid_velocity", "last_corrector_error", "warnings", "energy0", "last_guard")
        return {name: getattr(self, name) for name in names}

    def checkpoint(self):
        if self.failed:
            raise ValueError("cannot checkpoint a rejected numerical interval")
        state = np.zeros(mujoco.mj_stateSize(self.model, STATE_KIND))
        mujoco.mj_getState(self.model, self.data, state, STATE_KIND)
        return {"schema": SCHEMA, "identity": copy.deepcopy(self.identity), "identity_sha256": sha(self.identity),
                "mujoco_integration_state": state.tolist(),
                "mujoco_warning_state": [{"number": w.number, "lastinfo": w.lastinfo} for w in self.data.warning],
                "brake": pack(self.brake), "motors": pack(self.motors), "dynamic": pack(self._dynamic())}

    def restore(self, checkpoint):
        expected = {"schema", "identity", "identity_sha256", "mujoco_integration_state", "mujoco_warning_state", "brake", "motors", "dynamic"}
        if type(checkpoint) is not dict or set(checkpoint) != expected or checkpoint["schema"] != SCHEMA:
            raise ValueError("live checkpoint schema mismatch")
        canonical(checkpoint)
        if checkpoint["identity"] != self.identity or checkpoint["identity_sha256"] != sha(self.identity):
            raise ValueError("model/source/parameter/MuJoCo identity mismatch")
        state = np.asarray(checkpoint["mujoco_integration_state"], dtype=float)
        if state.shape != (mujoco.mj_stateSize(self.model, STATE_KIND),) or not np.isfinite(state).all() or state[0] < 0:
            raise ValueError("invalid complete MuJoCo state")
        # Restore is intended only on an unpublished fresh candidate. Controller
        # swaps it in after ALL fields and invariants pass, keeping old state intact.
        self.brake = unpack_like(checkpoint["brake"], self.brake, "brake")
        self.motors = unpack_like(checkpoint["motors"], self.motors, "motors")
        values = unpack_like(checkpoint["dynamic"], self._dynamic(), "dynamic")
        for key, value in values.items():
            setattr(self, key, value)
        if pneumatic_configuration(self.brake) != self.identity["pneumatic_configuration"]:
            raise ValueError("pneumatic topology/parameters changed")
        if any(v.mass_kg <= 0 or v.volume_m3 <= 0 for v in self.brake.network.volumes.values()):
            raise ValueError("nonphysical gas mass or volume")
        if self.steps < 0 or abs(state[0] - self.steps * self.dt) > 1e-7 or abs(self.brake.network.time_s - state[0]) > 1e-7:
            raise ValueError("inconsistent mechanical/gas/step time")
        for name, args in (("drive", {"enabled": self.commands["drive_enabled"]}),
                           ("brake", {"mode": self.commands["brake_mode"]}),
                           ("head_load", {"force_N": self.commands["head_force_N"]})):
            self.command(name, args)
        mujoco.mj_setState(self.model, self.data, state, STATE_KIND)
        warmstart = self.data.qacc_warmstart.copy()
        mujoco.mj_forward(self.model, self.data)
        self.data.qacc_warmstart[:] = warmstart
        warning_state = checkpoint["mujoco_warning_state"]
        if type(warning_state) is not list or len(warning_state) != len(self.data.warning):
            raise ValueError("invalid MuJoCo warning state")
        for warning, value in zip(self.data.warning, warning_state):
            if type(value) is not dict or set(value) != {"number", "lastinfo"} or any(type(v) is not int or v < 0 for v in value.values()):
                raise ValueError("invalid MuJoCo warning counter")
            warning.number, warning.lastinfo = value["number"], value["lastinfo"]
        if not np.isfinite(np.r_[self.data.qpos, self.data.qvel, self.data.qacc]).all():
            raise ValueError("restored mechanics are not finite")
        canonical(self.snapshot())
