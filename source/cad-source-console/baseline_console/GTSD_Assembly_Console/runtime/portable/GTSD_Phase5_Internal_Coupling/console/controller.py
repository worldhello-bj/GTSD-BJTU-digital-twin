"""Single-owner, loopback-console controller for the new-geometry v5 plant.

No historical vehicle factory, startup cache, hardware or outbound network.
Only the solver worker may construct, advance, command or restore a plant.
"""
from __future__ import annotations

from collections import OrderedDict, deque
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import queue
import re
import secrets
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))
CHECKPOINT_SCHEMA = "gtsd-v5-console-checkpoint-v1"
LIMITS = [
    "仅作用于本机回环软件求解器，无实机、硬件或外发网络控制。",
    "新几何工程等效模型；质量、惯量、接触、电机与气路参数尚未实测标定。",
    "冷启动从新模型初态开始，未跳过沉降，也未加载旧版准备缓存。",
    "车门、辅机和线缆尚未迁移；固定源网格不代表对应机构已仿真。",
    "失联暂停只冻结数值积分，不代表实车制动或安全停车。",
]
CONTROL_SCHEMAS = {
    "run": {}, "pause": {}, "new_run": {}, "restore_run": {},
    "step": {"steps": {"type": "integer", "minimum": 1, "maximum": 2500}},
    "restore_checkpoint": {"checkpoint": {"type": "object"}},
}


def make_plant():
    """Import the additive live adapter, never the historical B11 factory."""
    from live_plant import LivePlant
    return LivePlant()


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def validate_value(value, spec, name):
    kind = spec.get("type")
    if kind == "boolean" and type(value) is not bool:
        raise ValueError(name + " must be boolean")
    if kind == "integer" and type(value) is not int:
        raise ValueError(name + " must be an integer")
    if kind == "number" and (type(value) not in (float, int) or not math.isfinite(value)):
        raise ValueError(name + " must be finite")
    if kind == "string" and type(value) is not str:
        raise ValueError(name + " must be a string")
    if kind == "object" and type(value) is not dict:
        raise ValueError(name + " must be an object")
    if kind not in ("boolean", "integer", "number", "string", "object"):
        raise ValueError("unsupported command argument schema")
    if "enum" in spec and value not in spec["enum"]:
        raise ValueError(name + " is not an accepted mode")
    if "minimum" in spec and value < spec["minimum"]:
        raise ValueError(name + " below minimum")
    if "maximum" in spec and value > spec["maximum"]:
        raise ValueError(name + " above maximum")


def validate_command(payload, registry):
    if type(payload) is not dict or set(payload) != {"session_id", "request_id", "action", "args"}:
        raise ValueError("expected session_id, request_id, action, args only")
    for key in ("session_id", "request_id"):
        if type(payload[key]) is not str or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", payload[key]):
            raise ValueError("invalid " + key)
    action, args = payload["action"], payload["args"]
    if type(action) is not str or type(args) is not dict:
        raise ValueError("action must be a string and args an object")
    schemas = CONTROL_SCHEMAS | {name: value["args"] for name, value in registry.items()}
    if action not in schemas or set(args) != set(schemas[action]):
        raise ValueError("unsupported action or unexpected arguments")
    for key, spec in schemas[action].items():
        validate_value(args[key], spec, key)
    canonical(payload)  # Reject nonfinite values anywhere, including checkpoints.
    return copy.deepcopy(payload)


class Controller:
    def __init__(self, *, factory=make_plant, lease_s=5.0, publish_s=.1,
                 clock=time.monotonic, resume=None):
        if not math.isfinite(lease_s) or lease_s <= 0 or not math.isfinite(publish_s) or publish_s <= 0:
            raise ValueError("lease and publication intervals must be finite and positive")
        self.factory, self.clock, self.lease_s, self.publish_s = factory, clock, lease_s, publish_s
        self.resume = copy.deepcopy(resume)
        self.lock = threading.RLock()
        self.pending = queue.Queue(maxsize=64)
        self.stop_event = threading.Event()
        self.thread = None
        self.plant = None
        self.binding = None
        self.visual_resource_bytes = None
        self.previous = None
        self.registry = {}
        self.phase = "building"
        self.error = None
        self.session_id = secrets.token_hex(16)
        self.sequence = 0
        self.events = deque(maxlen=80)
        self.receipts = OrderedDict()
        self.last_client = clock()
        self.published_wall = clock()
        self.started_wall = clock()
        self.started_sim = 0.
        self.lease_armed = False
        self.single_steps = 0
        self.startup_wall_s = None
        self.state = self._state(None)

    def _state(self, snapshot):
        wall = max(1e-12, self.clock() - self.started_wall)
        return {
            "schema": "gtsd-v5-console-state-v1", "model_version": getattr(self.plant, "model_version", "v5-new-geometry-live"),
            "session_id": self.session_id, "sequence": self.sequence, "phase": self.phase,
            "error": self.error, "plant": snapshot,
            "capabilities": copy.deepcopy(snapshot.get("capabilities", {}) if snapshot else {}),
            "command_registry": copy.deepcopy(self.registry), "limits": copy.deepcopy(getattr(self.plant, "limits", LIMITS)),
            "single_steps_remaining": self.single_steps,
            "previous_run_available": self.previous is not None,
            "events": list(self.events), "client_lease_armed": self.lease_armed,
            "client_lease_s": self.lease_s, "startup_wall_s": self.startup_wall_s,
            "checkpoint": {"schema": CHECKPOINT_SCHEMA, "scope": getattr(self.plant, "checkpoint_scope", "complete configured plant including MuJoCo integration and every active subsystem's state, controls and ledgers"), "automatic_cache": False},
            "performance": {"simulated_seconds_per_wall_second": max(0., ((snapshot or {}).get("time_s", 0.) - self.started_sim) / wall)},
        }

    def _publish(self):
        if self.plant is not None and getattr(self.plant, "failed", False):
            # Preserve the last published accepted observation; never display a
            # rejected gas/mechanical trial as a valid integrated endpoint.
            snap = copy.deepcopy(self.state.get("plant"))
        else:
            snap = self.plant.snapshot() if self.plant is not None else None
            if snap is not None and "ideal_observations" not in snap:
                from sensors.observe import observe
                snap["ideal_observations"] = observe(self.plant.model, self.plant.data,
                                                     snap["telemetry"], snap["source_geometry_sha256"])
        value = self._state(snap)
        canonical(value)  # Never publish NaN/Infinity or partial mutable buffers.
        with self.lock:
            self.state = copy.deepcopy(value)
            self.published_wall = self.clock()

    def start(self):
        if self.thread is not None:
            raise RuntimeError("controller already started")
        self.thread = threading.Thread(target=self._loop, name="gtsd-v5-solver", daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=15)
            if self.thread.is_alive():
                raise RuntimeError("solver worker did not stop")

    def snapshot(self, *, heartbeat=True):
        with self.lock:
            if heartbeat:
                self.last_client = self.clock()
            value = copy.deepcopy(self.state)
            value["snapshot_age_s"] = max(0., self.clock() - self.published_wall)
            return value

    def submit(self, payload):
        with self.lock:
            command = validate_command(payload, self.registry)
            key = (command["session_id"], command["request_id"])
            content = digest(command)
            if key in self.receipts:
                receipt = self.receipts[key]
                if receipt["digest"] != content:
                    raise ValueError("request_id already used for different content")
                return {"status": "already_received", "request_id": key[1],
                        "result": receipt["status"], "error": receipt.get("error")}
            if command["session_id"] != self.session_id:
                raise ValueError("stale session; refresh before commanding")
            recovery = command["action"] in ("new_run", "restore_run", "restore_checkpoint")
            if self.stop_event.is_set() or self.phase in ("building", "closed") or self.phase == "error" and not recovery:
                raise ValueError("solver is not command-ready")
            try:
                self.pending.put_nowait(("command", command))
            except queue.Full as exc:
                raise ValueError("command queue full") from exc
            self.receipts[key] = {"digest": content, "status": "queued"}
            # Never evict requests from the current session: replaying a very old
            # command in that session must not execute twice. Older-session
            # receipts can be bounded; once evicted their session is rejected.
            while len(self.receipts) > 512:
                candidate = next((k for k, v in self.receipts.items() if k[0] != self.session_id and v["status"] != "queued"), None)
                if candidate is None:
                    break
                del self.receipts[candidate]
        return {"status": "queued", "request_id": command["request_id"]}

    def export_checkpoint(self, timeout=15.):
        """A coherent export is captured by the owner between accepted steps."""
        event, box = threading.Event(), {}
        with self.lock:
            if self.phase in ("building", "error", "closed") or self.stop_event.is_set():
                raise ValueError("solver has no exportable accepted state")
            try:
                self.pending.put_nowait(("export", (event, box)))
            except queue.Full as exc:
                raise ValueError("command queue full") from exc
        if not event.wait(timeout):
            raise TimeoutError("checkpoint export not acknowledged; no automatic retry")
        if "error" in box:
            raise ValueError(box["error"])
        return box["checkpoint"]

    def _checkpoint(self):
        payload = self.plant.checkpoint()
        content = {"schema": CHECKPOINT_SCHEMA, "plant": payload}
        return content | {"sha256": digest(content)}

    def _candidate(self, checkpoint):
        if type(checkpoint) is not dict or set(checkpoint) != {"schema", "plant", "sha256"}:
            raise ValueError("invalid console checkpoint shape")
        content = {"schema": checkpoint["schema"], "plant": checkpoint["plant"]}
        if checkpoint["schema"] != CHECKPOINT_SCHEMA or checkpoint["sha256"] != digest(content):
            raise ValueError("console checkpoint version or digest mismatch")
        candidate = self.factory()
        candidate.restore(copy.deepcopy(checkpoint["plant"]))
        canonical(candidate.snapshot())
        return candidate

    def _install(self, candidate, *, new_session):
        self.plant = candidate
        self.binding = copy.deepcopy(candidate.binding)
        self.visual_resource_bytes = getattr(candidate, "visual_resource_bytes", None)
        snap = candidate.snapshot()
        self.registry = copy.deepcopy(snap["command_registry"])
        if set(self.registry) & set(CONTROL_SCHEMAS):
            raise ValueError("plant command conflicts with console control")
        for entry in self.registry.values():
            if type(entry) is not dict or type(entry.get("args")) is not dict:
                raise ValueError("plant command registry must contain argument schemas")
        self.phase, self.error, self.single_steps, self.lease_armed = "paused", None, 0, False
        self.started_wall, self.started_sim = self.clock(), snap["time_s"]
        if new_session:
            self.session_id = secrets.token_hex(16)

    def _finish(self, command, status, error=None):
        with self.lock:
            key = (command["session_id"], command["request_id"])
            self.receipts[key].update(status=status, error=error)
            self.sequence += 1
            self.events.append({"sequence": self.sequence, "request_id": command["request_id"],
                                "session_id": command["session_id"], "action": command["action"],
                                "status": status, "error": error})

    def _apply(self, command):
        action, args = command["action"], command["args"]
        if command["session_id"] != self.session_id:
            raise ValueError("session changed before queued command was applied")
        if action == "run":
            self.phase, self.single_steps, self.lease_armed = "running", 0, True
            self.last_client = self.clock()
        elif action == "pause":
            self.phase, self.single_steps = "paused", 0
        elif action == "step":
            if self.phase != "paused" or self.single_steps:
                raise ValueError("single-step requires an idle paused solver")
            self.single_steps = args["steps"]
            self.lease_armed = True
            self.last_client = self.clock()
        elif action in ("new_run", "restore_run", "restore_checkpoint"):
            if action == "restore_run" and self.previous is None:
                raise ValueError("no previous run available")
            candidate = self.factory() if action == "new_run" else self._candidate(
                self.previous if action == "restore_run" else args["checkpoint"])
            previous = self.previous if getattr(self.plant, "failed", False) else self._checkpoint()
            self._install(candidate, new_session=True)
            self.previous = previous
        else:
            self.plant.command(action, copy.deepcopy(args))

    def _process(self, item):
        kind, value = item
        if kind == "export":
            event, box = value
            try:
                box["checkpoint"] = self._checkpoint()
            except Exception as exc:
                box["error"] = str(exc)
            finally:
                event.set()
        else:
            try:
                self._apply(value)
                self._finish(value, "applied")
            except Exception as exc:
                # Candidate construction/restore is isolated. A dependency or
                # solver validation failure must reject this request rather than
                # silently kill the owner thread with more commands queued.
                self._finish(value, "rejected", str(exc))
            self._publish()

    def _loop(self):
        try:
            begin = self.clock()
            candidate = self._candidate(self.resume) if self.resume is not None else self.factory()
            self._install(candidate, new_session=False)
            self.startup_wall_s = self.clock() - begin
            self._publish()
            while not self.stop_event.is_set():
                try:
                    item = self.pending.get(timeout=0. if self.phase == "running" or self.single_steps else .03)
                except queue.Empty:
                    item = None
                if item is not None:
                    self._process(item)
                with self.lock:
                    expired = self.lease_armed and self.clock() - self.last_client > self.lease_s
                if expired and (self.phase == "running" or self.single_steps):
                    self.phase, self.single_steps = "paused", 0
                    self.sequence += 1
                    self.events.append({"sequence": self.sequence, "action": "client_lease_expired",
                                        "request_id": None, "session_id": self.session_id,
                                        "status": "applied", "error": None})
                    self._publish()
                if self.phase == "running" or self.single_steps:
                    try:
                        self.plant.step(1)
                    except Exception as exc:
                        self.phase, self.error, self.single_steps = "error", type(exc).__name__ + ": " + str(exc), 0
                        self._publish()
                        continue  # Explicit reset/validated recovery stays available.
                    if self.single_steps:
                        self.single_steps -= 1
                    if not self.single_steps and self.phase == "paused" or self.clock() - self.published_wall >= self.publish_s:
                        self._publish()
                elif self.clock() - self.published_wall >= self.publish_s:
                    self._publish()
        except Exception as exc:
            self.phase, self.error, self.single_steps = "error", type(exc).__name__ + ": " + str(exc), 0
            # A failed integrator must never publish its possibly partial state.
            with self.lock:
                self.state.update(phase="error", error=self.error, single_steps_remaining=0)
                self.published_wall = self.clock()
        finally:
            while True:
                try:
                    kind, value = self.pending.get_nowait()
                except queue.Empty:
                    break
                if kind == "export":
                    event, box = value
                    box["error"] = "solver stopped before export"
                    event.set()
                else:
                    self._finish(value, "rejected", "solver stopped before command")
            if self.phase != "error":
                self.phase = "closed"
                with self.lock:
                    self.state["phase"] = "closed"
