"""Package an explicit B9 reproducible subset; no home files or dependencies."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]


def main():
    acceptance = json.loads((ROOT/"results/verification.json").read_text(encoding="utf-8"))
    readback = json.loads((ROOT/"replay/reports/b9_readback.json").read_text(encoding="utf-8"))
    if not acceptance["passed"] or not readback["passed"]:
        raise RuntimeError("physics acceptance and saved Blender checks must pass")
    files = {}
    # Keep original repository-relative paths so package and repo commands match.
    for folder in (ROOT, ROOT.parent/"b8", REPO/"source/GTSD_BJTU_B8/scripts"):
        for p in folder.rglob("*"):
            if not p.is_file() or "__pycache__" in p.parts or p.suffix in (".pyc", ".blend1", ".blend2"):
                continue
            if p.name == "GTSD_BJTU_B8_source_dynamics.blend":
                continue  # temporary adapter save; the B9 asset is verified
            files[p.relative_to(REPO).as_posix()] = p
    for relative in ("source/vehicle-physics/build_model.py",
                     "local-replay-base/GTSD_BJTU_B8/source/GTSD_BJTU_B7_detailed.blend",
                     "local-replay-base/GTSD_BJTU_B8/reports/source_door_closed_rest.json"):
        files[relative] = REPO/relative
    manifest = dict(schema="B9 incremental reproducible package v1", acceptance=acceptance["status"],
                    physics_cases=acceptance["integrated_cases"],
                    contents={n:dict(size=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for n,p in sorted(files.items())},
                    exclusions=["Third-party binary dependencies, credentials, home directories, full B8 frozen archives, intermediate B8-labelled asset"],
                    scope="B9 metering-port prototype; frozen B8 solver/source needed by B9; full B9 numeric outputs and source-mesh replay")
    destination = REPO/"artifacts/B9"
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination/"GTSD_BJTU_B9_valve_dynamics.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for name,p in sorted(files.items()):
            z.write(p,name)
        z.writestr("MANIFEST_B9.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        z.writestr("START_HERE.txt", "B9 dynamic metering-port prototype\nRead source/vehicle-physics/b9/README.md and RESULTS.md.\nOpen source/vehicle-physics/b9/replay/assets/GTSD_BJTU_B9_source_dynamics.blend with Blender 5.2.\nReplay is display only. Python simulation requires installing b9/requirements.txt.\nAll added parameters are uncalibrated.\n")
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:
            raise RuntimeError("package CRC failure")
        for name, record in manifest["contents"].items():
            data = z.read(name)
            if len(data) != record["size"] or hashlib.sha256(data).hexdigest() != record["sha256"]:
                raise RuntimeError("packaged byte identity mismatch: "+name)
    report = dict(passed=True, path=archive.relative_to(REPO).as_posix(), files=len(files),
                  size=archive.stat().st_size, sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                  zip_crc="passed", file_hashes="all passed", manifest=manifest)
    (destination/"PACKAGE_VERIFICATION.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k != "manifest"}, indent=2))

if __name__ == "__main__":
    main()
