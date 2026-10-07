"""Create/check a dedicated immutable-by-convention executed source snapshot."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
SNAPSHOT = REPO/"execution-snapshots/b10-v1"


def main():
    global SNAPSHOT
    parser = argparse.ArgumentParser()
    parser.add_argument('--version', default='b10-v1', help='fresh b10-* snapshot name after any source change')
    version = parser.parse_args().version
    if not version.startswith('b10-') or not all(c.isalnum() or c == '-' for c in version):
        raise ValueError('snapshot version must be a simple b10-* name')
    SNAPSHOT = REPO/'execution-snapshots'/version
    SNAPSHOT.mkdir(parents=True, exist_ok=True)
    files = {}
    for version in ("b8", "b9", "b10"):
        for path in (ROOT.parent/version).glob("*.py"):
            files[path.relative_to(REPO)] = path
    for relative in ("source/vehicle-physics/build_model.py", "source/vehicle-physics/b8/parameters.json", "source/vehicle-physics/b8/results/physics_run_identity.json",
                     "source/vehicle-physics/b9/results/service_brake_identity.json"):
        files[Path(relative)] = REPO/relative
    manifest = {}
    for relative, path in files.items():
        destination = SNAPSHOT/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists() and destination.read_bytes() != path.read_bytes():
            raise RuntimeError("snapshot exists with different source; use a new explicit snapshot version")
        if not destination.exists():
            shutil.copyfile(path, destination)
        manifest[relative.as_posix()] = hashlib.sha256(destination.read_bytes()).hexdigest()
    (SNAPSHOT/"EXECUTION_SOURCES.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(dict(snapshot=str(SNAPSHOT), source_files=len(files), sha256=manifest["source/vehicle-physics/b10/auxiliaries.py"])))


if __name__ == "__main__":
    main()
