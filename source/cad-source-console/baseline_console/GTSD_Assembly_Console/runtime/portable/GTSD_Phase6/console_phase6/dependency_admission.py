"""Reviewed release hash admission; no absolute runtime paths or source mutation."""
import hashlib
from pathlib import Path
from console_launcher import ConfigError, read_bounded, strict_json


def contract():
    value = strict_json(read_bounded(Path(__file__).with_name("accepted_dependencies.json"), 128 * 1024, "reviewed dependency hashes"))
    if value.get("schema") != "gtsd-console-reviewed-dependency-hashes-v1":
        raise ConfigError("dependency_admission", "Unsupported reviewed dependency contract")
    return value


def manifest(name, body):
    expected = contract()["source_manifests"].get(name)
    if expected is None or hashlib.sha256(body).hexdigest() != expected:
        raise ConfigError("unreviewed_dependency_hash", f"Unreviewed {name} manifest hash; use the reviewed release or obtain an explicit new acceptance bundle")


def phase5_hash_errors(observed):
    expected = contract()["phase5_files"]
    return [name for name, digest in expected.items() if observed.get(name) != digest]


def observer_file(root, relative="aux_mount/observations.py"):
    expected = contract().get("readonly_observer_files", {}).get(relative)
    body = read_bounded(Path(root) / relative, 128 * 1024, "read-only observer source")
    if expected is None or hashlib.sha256(body).hexdigest() != expected:
        raise ConfigError("unreviewed_observer_hash", "Read-only observer source differs from this reviewed release; it will not be imported or executed")
