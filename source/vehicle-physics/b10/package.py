"""Explicit B10 reproducible package with byte/CRC checks, no local environment."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]


def main():
    acceptance = json.loads((ROOT/"results/verification.json").read_text(encoding="utf-8"))
    readback = json.loads((ROOT/"replay/reports/b10_readback.json").read_text(encoding="utf-8"))
    if not acceptance["passed"] or not readback["passed"]:
        raise RuntimeError("physics and saved replay verification required")
    files = {}
    for folder in (ROOT, ROOT.parent/"b8", REPO/"source/GTSD_BJTU_B8/scripts"):
        for p in folder.rglob("*"):
            if not p.is_file() or "__pycache__" in p.parts or p.suffix in (".pyc", ".blend1", ".blend2", ".log"):
                continue
            if p.name == "GTSD_BJTU_B8_source_dynamics.blend":
                continue
            files[p.relative_to(REPO).as_posix()] = p
    for relative in ("source/vehicle-physics/build_model.py", "source/vehicle-physics/b9/compat.py",
                     "source/vehicle-physics/b9/valve_dynamics.py", "source/vehicle-physics/b9/simulate.py",
                     "source/vehicle-physics/b9/results/service_brake_identity.json", "source/vehicle-physics/b9/requirements.txt",
                     "local-replay-base/GTSD_BJTU_B8/source/GTSD_BJTU_B7_detailed.blend",
                     "local-replay-base/GTSD_BJTU_B8/reports/source_door_closed_rest.json"):
        files[relative] = REPO/relative
    manifest = dict(schema="B10 auxiliary equivalent incremental package v1", acceptance=acceptance["status"],
                    vehicle_cases=3, bench_cases=6,
                    contents={n:dict(size=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for n,p in sorted(files.items())},
                    exclusions=["credentials, runtime environments, home files, full frozen B8 archives, intermediate display save"],
                    scope="inferred B10 fixed auxiliary bench + B9 valves + frozen B8 vehicle, complete new numeric outputs and replay")
    destination = REPO/"artifacts/B10"; destination.mkdir(parents=True, exist_ok=True)
    archive = destination/"GTSD_BJTU_B10_auxiliary_dynamics.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for name,p in sorted(files.items()):
            z.write(p,name)
        z.writestr("MANIFEST_B10.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        z.writestr("START_HERE.txt", "B10 inferred auxiliary equivalents\nRead source/vehicle-physics/b10/README.md and RESULTS.md.\nOpen source/vehicle-physics/b10/replay/assets/GTSD_BJTU_B10_source_dynamics.blend with Blender 5.2.\nSelect B10_INFERRED_AUXILIARY_BENCH for the labelled 33.3x slow startup.\nInstall source/vehicle-physics/b9/requirements.txt to rerun Python simulations.\nAll new device topology/parameters are inferred and uncalibrated.\n")
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:
            raise RuntimeError("ZIP CRC failure")
        for name, record in manifest["contents"].items():
            data = z.read(name)
            if len(data) != record["size"] or hashlib.sha256(data).hexdigest() != record["sha256"]:
                raise RuntimeError("package identity mismatch: "+name)
    report = dict(passed=True, path=archive.relative_to(REPO).as_posix(), files=len(files),
                  size=archive.stat().st_size, sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                  zip_crc="passed", file_hashes="all passed", manifest=manifest)
    (destination/"PACKAGE_VERIFICATION.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k != "manifest"}, indent=2))


if __name__ == "__main__":
    main()
