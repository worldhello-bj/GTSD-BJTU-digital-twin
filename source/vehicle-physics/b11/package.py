"""Explicit independently runnable B11 package, with complete accepted trajectories."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]


def main():
    acceptance=json.loads((ROOT/'results/verification.json').read_text(encoding='utf-8'))
    readback=json.loads((ROOT/'replay/reports/b11_readback.json').read_text(encoding='utf-8'))
    if not acceptance['passed'] or not readback['passed']:
        raise RuntimeError('complete physics and saved-file acceptance required')
    asset = ROOT/'replay/assets/GTSD_BJTU_B11_source_cable_dynamics.blend'
    if hashlib.sha256(asset.read_bytes()).hexdigest()!=readback['blend_sha256']:
        raise RuntimeError('saved Blender asset differs from accepted readback')
    files={}
    def include(path):
        files[path.relative_to(REPO).as_posix()]=path
    for folder in (ROOT.parent/'b8',REPO/'source/GTSD_BJTU_B8/scripts'):
        for path in folder.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc','.log'):
                include(path)
    required_source = ('cables.py','simulate.py','live.py','extract_source_cables.py','derive_b8_solver.py',
                       'verify.py','report.py','test_cables.py','test_integration.py','build_blender_replay.py',
                       'validate_blender_replay.py','render_blender_replay.py','package.py','smoke_package.py',
                       'import_results.py','replay_geometry.py','cable_contact_audit.py','README.md','RESULTS.md','source_cables.json',
                       'component_verification.json','regression_verification.json')
    for name in required_source:
        include(ROOT/name)
    for name in ('v2_rejection.json','v2_4of6_verification.json','v3_constraint_probe.json'):
        include(ROOT/'history'/name)
    for case in ('service_brake','half_dt','spatial_refinement','doors_open_close','doors_half_dt','tether_release'):
        for suffix in ('.xml','.csv','_metrics.json','_identity.json','_auxiliaries.csv','_replay.json'):
            include(ROOT/'results'/(case+suffix))
    for name in ('verification.json','b11_cable_response.png','b11_cable_response.pdf'):
        include(ROOT/'results'/name)
    for path in (ROOT/'replay').rglob('*'):
        if path.is_file() and path.suffix not in ('.blend1','.blend2','.log') and path.name not in ('B11_door_intermediate.blend','GTSD_BJTU_B8_source_dynamics.blend'):
            include(path)
    for path in (ROOT.parent/'b10').glob('*.py'):
        include(path)
    for relative in ('source/vehicle-physics/build_model.py','source/vehicle-physics/b9/compat.py',
                     'source/vehicle-physics/b9/valve_dynamics.py','source/vehicle-physics/b9/simulate.py',
                     'source/vehicle-physics/b9/results/service_brake_identity.json','source/vehicle-physics/b9/requirements.txt',
                     'source/vehicle-physics/b10/README.md','source/vehicle-physics/b10/RESULTS.md',
                     'source/vehicle-physics/b10/replay/inputs/auxiliary_startup.csv',
                     'source/vehicle-physics/b10/replay/inputs/auxiliary_startup_identity.json',
                     'source/vehicle-physics/b10/replay/assets/GTSD_BJTU_B10_source_dynamics.blend',
                     'source/vehicle-physics/b10/replay/reports/b10_readback.json',
                     'local-replay-base/GTSD_BJTU_B8/source/GTSD_BJTU_B7_detailed.blend',
                     'local-replay-base/GTSD_BJTU_B8/reports/source_door_closed_rest.json'):
        include(REPO/relative)
    manifest=dict(schema='B11 source-cable equivalent incremental package v1',acceptance=acceptance['status'],
                  cases=6,contents={name:dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for name,p in sorted(files.items())},
                  scope='complete B11 source/data/Blender with frozen B8, B9 valve dependencies and B10 auxiliary code/probe; earlier full B9/B10 cases remain separate archives',
                  exclusions=['local environments/credentials/caches','diagnostic-only profile/prototype outputs','intermediate display saves','earlier full delivery archives'])
    destination=REPO/'artifacts/B11'; destination.mkdir(parents=True,exist_ok=True)
    archive=destination/'GTSD_BJTU_B11_source_cable_dynamics.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for name,path in sorted(files.items()):
            z.write(path,name)
        z.writestr('MANIFEST_B11.json',json.dumps(manifest,indent=2))
        z.writestr('START_HERE.txt','Read source/vehicle-physics/b11/README.md and RESULTS.md.\nOpen source/vehicle-physics/b11/replay/assets/GTSD_BJTU_B11_source_cable_dynamics.blend.\nInstall source/vehicle-physics/b9/requirements.txt to rerun.\nCable material/contact parameters are inferred and uncalibrated.\nB10 startup bench is separate and explicitly labelled inferred.\n')
    with zipfile.ZipFile(archive) as z:
        if z.testzip():
            raise RuntimeError('ZIP CRC failure')
        for name,record in manifest['contents'].items():
            data=z.read(name)
            if len(data)!=record['size'] or hashlib.sha256(data).hexdigest()!=record['sha256']:
                raise RuntimeError('package hash mismatch '+name)
    report=dict(passed=True,path=archive.relative_to(REPO).as_posix(),files=len(files),size=archive.stat().st_size,
                sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),zip_crc='passed',file_hashes='all passed',manifest=manifest)
    (destination/'PACKAGE_VERIFICATION.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='manifest'},indent=2))


if __name__=='__main__':
    main()
