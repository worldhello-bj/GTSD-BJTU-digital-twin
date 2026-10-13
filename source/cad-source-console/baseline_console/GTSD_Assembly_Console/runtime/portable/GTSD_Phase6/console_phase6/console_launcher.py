#!/usr/bin/env python3
"""Strict, loopback-only configuration adapter for the frozen GTSD console.

No solver changes, physical parameter overrides, installation, browser launch,
hardware I/O, external requests or automatic checkpoint recovery.
"""
from __future__ import annotations

import argparse
import errno
import hashlib
from http.client import HTTPConnection, HTTPException
import importlib.metadata
import json
import math
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time

SCHEMA = "gtsd-console-launch-config-v1"
MAX_CONFIG = 64 * 1024
MAX_CHECKPOINT = 2 * 1024 * 1024
DEPENDENCIES = {"mujoco": "3.3.7", "numpy": "2.2.6"}
VARIANTS = {
    "base": 15, "extended": 32, "extended-cables": 71,
    "cover-bench": 18, "brake-contact-bench": 3,
    "cable-support-bench": 25, "legacy-drawbar": 12,
    "legacy-closed-pantograph": 6, "contact-sections-research": 32,
    "internal-brake-research": 36, "internal-coupled-research": 75,
}
REQUIRED_FILES = (
    "requirements.txt", "console/server.py", "console/controller.py", "console/phase5_controller.py",
    "console/live_plant.py", "console/web/index.html", "console/web/app.js",
    "console/web/ui-model.js", "console/web/style.css",
    "console/web/contact-sections.js", "console/web/internal-systems.js",
    "web/physics-model.js", "web/render-model.js", "web/geometry.json",
    "internal_registry/registry.py",
)
VARIANT_FILES = {
    "extended": ("console/extended_plant.py", "console/export_isolation.py"),
    "extended-cables": ("console/extended_plant.py", "console/export_isolation.py"),
    "cover-bench": ("console/bench_adapter.py",),
    "brake-contact-bench": ("console/research_bench_adapter.py",),
    "cable-support-bench": ("console/research_bench_adapter.py",),
    "contact-sections-research": ("console/sections_export_isolation.py",),
    "internal-brake-research": ("console/phase5_display.py", "brake_integrated/plant.py"),
    "internal-coupled-research": ("console/phase5_display.py", "support_integrated/combined.py"),
}


class ConfigError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def strict_json(data):
    def pairs(rows):
        result = {}
        for key, value in rows:
            if key in result:
                raise ConfigError("duplicate_key", f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    def invalid_constant(value):
        raise ConfigError("nonfinite_json", f"Non-finite JSON value: {value}")

    def finite_float(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            invalid_constant(value)
        return parsed

    try:
        return json.loads(data, object_pairs_hook=pairs, parse_constant=invalid_constant,
                          parse_float=finite_float)
    except ConfigError:
        raise
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise ConfigError("invalid_json", f"Invalid UTF-8 JSON: {exc}") from exc


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def read_bounded(path, limit, label):
    try:
        if not path.is_file():
            raise ConfigError("missing_file", f"{label} is not a readable file: {path}")
        # Bound the read itself, not just a racy stat before an unbounded read.
        with path.open("rb") as stream:
            value = stream.read(limit + 1)
    except OSError as exc:
        raise ConfigError("file_access", f"Cannot read {label}: {path}: {exc}") from exc
    if len(value) > limit:
        raise ConfigError("file_too_large", f"{label} exceeds {limit} bytes: {path}")
    return value


def resolve_path(value, base, label):
    if type(value) is not str or not value.strip() or "\x00" in value:
        raise ConfigError("invalid_path", f"{label} must be a nonempty path string")
    try:
        path = Path(value).expanduser()
        return (path if path.is_absolute() else base / path).resolve()
    except (OSError, RuntimeError) as exc:
        raise ConfigError("invalid_path", f"Cannot resolve {label}: {exc}") from exc


def checkpoint_header(path):
    checkpoint = strict_json(read_bounded(path, MAX_CHECKPOINT, "checkpoint"))
    if type(checkpoint) is not dict or set(checkpoint) != {"schema", "plant", "sha256"}:
        raise ConfigError("checkpoint_shape", "Expected a complete console checkpoint, not an observation export")
    if checkpoint["schema"] != "gtsd-v5-console-checkpoint-v1" or type(checkpoint["plant"]) is not dict:
        raise ConfigError("checkpoint_schema", "Unsupported complete checkpoint schema")
    try:
        digest = hashlib.sha256(canonical({"schema": checkpoint["schema"], "plant": checkpoint["plant"]})).hexdigest()
    except (ValueError, OverflowError) as exc:
        raise ConfigError("checkpoint_value", f"Invalid checkpoint value: {exc}") from exc
    if checkpoint["sha256"] != digest:
        raise ConfigError("checkpoint_digest", "Checkpoint SHA-256 does not match its canonical content")
    return {"path": str(path), "envelope_sha256": digest,
            "validation": "envelope_only; full physical identity and state are checked by the source solver"}


def load_config(path):
    path = Path(path).expanduser().resolve()
    value = strict_json(read_bounded(path, MAX_CONFIG, "configuration"))
    expected = {"schema", "source_root", "variant", "port", "resume"}
    if type(value) is not dict or set(value) != expected:
        supplied = set(value) if type(value) is dict else set()
        raise ConfigError("config_fields", f"Configuration requires exactly {sorted(expected)}; missing={sorted(expected-supplied)}, unknown={sorted(supplied-expected)}")
    if value["schema"] != SCHEMA:
        raise ConfigError("config_schema", f"Expected schema {SCHEMA}")
    if type(value["variant"]) is not str or value["variant"] not in VARIANTS:
        raise ConfigError("invalid_variant", "Unknown or unreviewed variant; choose one of: " + ", ".join(VARIANTS))
    if type(value["port"]) is not int or not 0 <= value["port"] <= 65535:
        raise ConfigError("invalid_port", "port must be an integer from 0 to 65535; 0 requests an ephemeral loopback port")
    root = resolve_path(value["source_root"], path.parent, "source_root")
    if not root.is_dir():
        raise ConfigError("source_missing", f"Source directory does not exist: {root}; paths are relative to the configuration file")
    value = dict(value, source_root=str(root), config_path=str(path))
    if value["resume"] is not None:
        value["resume"] = str(resolve_path(value["resume"], path.parent, "resume"))
        checkpoint_header(Path(value["resume"]))
    return value


def issue(code, message):
    return {"code": code, "message": message}


def check_port(port):
    """Check availability without connecting to or modifying another service."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", port))
    except OSError as exc:
        code = "port_in_use" if exc.errno == errno.EADDRINUSE else "port_unavailable"
        raise ConfigError(code, f"Cannot bind 127.0.0.1:{port}: {exc}. Choose a free port; do not stop an unknown service.") from exc


def doctor(config, *, check_bind=True):
    root = Path(config["source_root"])
    errors, warnings = [], []
    if sys.version_info < (3, 10):
        errors.append(issue("python_version", "Python >=3.10 is required; this delivery was verified with Python 3.12.14"))
    elif sys.version_info[:2] != (3, 12):
        warnings.append(issue("python_not_verified", "This Python minor version was not used for this delivery's recorded verification; use Python 3.12 for the reproduced setup"))
    versions = {}
    for name, expected in DEPENDENCIES.items():
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
        if versions[name] != expected:
            errors.append(issue("dependency_version", f"{name}=={expected} required, found {versions[name]!r} under {sys.executable}"))
    hashes = {}
    for relative in REQUIRED_FILES + VARIANT_FILES.get(config["variant"], ()):
        path = root / relative
        try:
            if not path.is_file():
                raise FileNotFoundError(str(path))
            # Streaming hash avoids loading the large source geometry in memory.
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
            hashes[relative] = digest.hexdigest()
        except OSError as exc:
            errors.append(issue("source_resource", f"Missing/unreadable required source asset {path}: {exc}"))
    if check_bind:
        try:
            check_port(config["port"])
        except ConfigError as exc:
            errors.append(issue(exc.code, str(exc)))
    if config["resume"]:
        warnings.append(issue("resume_requires_solver", "Envelope verified only; full source/model/parameter identity and state validation occur before the resumed plant is installed. Invalid restoration enters an error state without stepping."))
    return {
        "schema": "gtsd-console-preflight-v1", "ok": not errors,
        "config": config, "python": sys.version.split()[0], "python_executable": sys.executable,
        "dependencies": versions, "source_file_sha256": hashes,
        "bind": "127.0.0.1", "port_checked": check_bind,
        "port_check_is_reservation": False, "expected_dof": VARIANTS[config["variant"]],
        "errors": errors, "warnings": warnings,
        "boundaries": ["software-only", "no hardware I/O", "no external network requests",
                       "no installation", "no automatic resume", "no browser opened",
                       "hashes are observations, not authenticity certificates",
                       "engineering assumptions are not 1:1 hardware calibration"],
    }


def server_arguments(config, *, ephemeral=False):
    result = ["--variant", config["variant"], "--port", str(0 if ephemeral else config["port"])]
    if config["resume"]:
        result += ["--resume", config["resume"]]
    return result


def inspect_port(port):
    if type(port) is not int or not 1 <= port <= 65535:
        raise ConfigError("inspect_port", "Inspect requires the actual listening port from 1 to 65535")
    connection = HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        # Unlike /api/state this read intentionally does not renew the run lease.
        connection.request("GET", "/api/observation")
        response = connection.getresponse()
        body = response.read(8 * 1024 * 1024 + 1)
        if response.status != 200:
            raise ConfigError("inspect_http", f"Loopback observation returned HTTP {response.status}")
        if len(body) > 8 * 1024 * 1024:
            raise ConfigError("inspect_size", "Observation exceeds the diagnostic response limit")
        state = strict_json(body)
        if type(state) is not dict or state.get("schema") != "gtsd-v5-console-state-v1":
            raise ConfigError("inspect_schema", "The selected loopback port is not a recognized GTSD observation service")
        return state
    except (OSError, ValueError, HTTPException) as exc:
        if isinstance(exc, ConfigError):
            raise
        raise ConfigError("inspect_connection", f"Cannot read GTSD at 127.0.0.1:{port}: {exc}") from exc
    finally:
        connection.close()


def describe_state(state):
    plant = state.get("plant") or {}
    observations = plant.get("ideal_observations") or {}
    signals = observations.get("observations") or []
    return {
        "schema": "gtsd-console-diagnostic-v1", "phase": state.get("phase"),
        "error": state.get("error"), "session_id": state.get("session_id"),
        "snapshot_age_s": state.get("snapshot_age_s"),
        "time_s": plant.get("time_s"), "steps": plant.get("steps"), "dt_s": plant.get("dt_s"),
        "model_identity_sha256": plant.get("model_identity_sha256"),
        "source_geometry_sha256": plant.get("source_geometry_sha256"),
        "capabilities": state.get("capabilities"),
        "command_registry": state.get("command_registry"),
        "limits": state.get("limits"), "solver_proxy_count": len(plant.get("solver_proxies") or []),
        "observation_summary": {
            "sample_time_s": observations.get("sample_time_s"), "signal_count": len(signals),
            "units": sorted({row["unit"] for row in signals if type(row) is dict and type(row.get("unit")) is str}),
            "quality_labels": sorted({row["quality"] for row in signals if type(row) is dict and type(row.get("quality")) is str}),
            "hardware_decode": "not supplied; no raw-frame mapping inferred",
        },
        "client_lease_s": state.get("client_lease_s"), "renews_run_lease": False,
        "meaning": "live read-only observation; not a checkpoint, hardware certificate, or browser/GPU test",
    }


def smoke(config, timeout=30):
    """Cold-build the configured source through its existing CLI; never step it."""
    command = [sys.executable, str(Path(config["source_root"]) / "console/server.py")] + server_arguments(config, ephemeral=True)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    process = subprocess.Popen(command, cwd=config["source_root"], env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = queue.Queue(), queue.Queue()
    def drain(stream, target):
        for line in stream:
            target.put(line)
    threads = [threading.Thread(target=drain, args=(process.stdout, stdout), daemon=True),
               threading.Thread(target=drain, args=(process.stderr, stderr), daemon=True)]
    for thread in threads:
        thread.start()
    deadline, port, state = time.monotonic() + timeout, None, None
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                details = "".join(list(stderr.queue))[-3000:]
                raise ConfigError("startup_exit", f"Source server exited with {process.returncode}: {details}")
            while not stdout.empty():
                line = stdout.get_nowait()
                marker = "http://127.0.0.1:"
                if marker in line:
                    try:
                        port = int(line.split(marker, 1)[1].strip().rstrip("/"))
                    except ValueError:
                        pass
            if port:
                state = inspect_port(port)
                if state["phase"] == "error":
                    raise ConfigError("startup_plant", "Source solver rejected startup: " + str(state.get("error")))
                if state["phase"] == "paused":
                    plant = state["plant"]
                    if plant["capabilities"].get("integrated_dof") != VARIANTS[config["variant"]]:
                        raise ConfigError("startup_dof", "Actual DOF differs from the reviewed variant contract")
                    if plant["capabilities"].get("hardware_control") is not False:
                        raise ConfigError("startup_hardware", "Software-only capability is not explicitly false")
                    if not config["resume"] and (plant["steps"] != 0 or plant["time_s"] != 0):
                        raise ConfigError("startup_advanced", "Fresh startup unexpectedly advanced the solver")
                    second = inspect_port(port)
                    if second["plant"]["steps"] != plant["steps"] or second["plant"]["time_s"] != plant["time_s"]:
                        raise ConfigError("startup_unstable", "Read-only observation unexpectedly advanced the solver")
                    return {"ok": True, "scope": "actual ephemeral loopback HTTP, configured source CLI, paused model construction, zero additional integration steps",
                            "variant": config["variant"], "diagnostic": describe_state(second),
                            "browser": "not tested", "gpu": "not tested", "downloads": "not tested"}
            time.sleep(.03)
        raise ConfigError("startup_timeout", f"Source server did not reach paused within {timeout} seconds")
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        for thread in threads:
            thread.join(timeout=1)
        process.stdout.close()
        process.stderr.close()


def serve(config):
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(Path(config["source_root"]) / "console"))
    # Reuse the source CLI and exact existing variant factories without rewriting
    # any numerical loop or adding physical configuration overrides.
    try:
        from server import main as server_main
        return server_main(server_arguments(config))
    except OSError as exc:
        code = "port_in_use" if exc.errno == errno.EADDRINUSE else "source_io"
        raise ConfigError(code, f"Source startup failed: {exc}. The preflight port check does not reserve the port.") from exc
    except (ImportError, ValueError) as exc:
        raise ConfigError("source_startup", f"Source console could not start: {type(exc).__name__}: {exc}") from exc


def emit(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("doctor", "command", "serve", "smoke", "inspect"))
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("console_config.json"))
    parser.add_argument("--port", type=int, help="inspect only: actual already-listening loopback port")
    args = parser.parse_args(argv)
    try:
        if args.action == "inspect":
            config = load_config(args.config)
            emit(describe_state(inspect_port(config["port"] if args.port is None else args.port)))
            return 0
        if args.port is not None:
            raise ConfigError("unexpected_port", "--port is only for read-only inspect; edit the configuration for launch")
        config = load_config(args.config)
        report = doctor(config, check_bind=args.action in ("doctor", "serve"))
        if args.action == "doctor" or not report["ok"]:
            emit(report)
            return 0 if report["ok"] else 2
        if args.action == "command":
            emit({"argv": [sys.executable, str(Path(config["source_root"]) / "console/server.py")] + server_arguments(config),
                  "cwd": config["source_root"], "environment": {"PYTHONDONTWRITEBYTECODE": "1"},
                  "instruction": "argument array, not a shell command; no shell expansion is used"})
        elif args.action == "smoke":
            emit(smoke(config))
        else:
            return serve(config)
        return 0
    except ConfigError as exc:
        emit({"ok": False, "error": issue(exc.code, str(exc))})
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
