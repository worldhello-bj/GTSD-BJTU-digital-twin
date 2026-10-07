"""Independently reproducible B12 maintenance bench with byte/CRC verification."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]


def main():
    acceptance=json.loads((ROOT/'results/verification.json').read_text(encoding='utf-8'))
    readback=json.loads((ROOT/'replay/reports/b12_readback.json').read_text(encoding='utf-8'))
    if not acceptance['passed'] or not readback['passed']:raise RuntimeError('physics and saved replay acceptance required')
    files={}
    for path in ROOT.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc','.blend1','.blend2','.log'):
            files[path.relative_to(REPO).as_posix()]=path
    for name in ('local-replay-base/GTSD_BJTU_B8/source/GTSD_BJTU_B7_detailed.blend','source/vehicle-physics/b9/requirements.txt'):
        files[name]=REPO/name
    manifest=dict(schema='B12 source-shaped removable-cover maintenance bench v1',acceptance=acceptance['status'],cases=5,
                  contents={name:dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for name,p in sorted(files.items())},
                  scope='complete independent cover bench; no moving-vehicle integration, true fastener or resolved lid mounting pose claim',
                  exclusions=['credentials/runtime environments/caches','previous full delivery archives','Blender backup saves'])
    destination=REPO/'artifacts/B12'; destination.mkdir(parents=True,exist_ok=True)
    archive=destination/'GTSD_BJTU_B12_cover_maintenance_bench.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for name,p in sorted(files.items()):z.write(p,name)
        z.writestr('MANIFEST_B12.json',json.dumps(manifest,indent=2))
        z.writestr('START_HERE.txt','Read source/vehicle-physics/b12/README.md and RESULTS.md.\nOpen source/vehicle-physics/b12/replay/assets/GTSD_BJTU_B12_cover_maintenance_bench.blend.\nInstall source/vehicle-physics/b9/requirements.txt for Python simulations.\nFour unmeasured .25 kg proxies on an inferred fixed bench; no vehicle mounting claim.\n')
    with zipfile.ZipFile(archive) as z:
        if z.testzip():raise RuntimeError('ZIP CRC failure')
        for name,record in manifest['contents'].items():
            data=z.read(name)
            if len(data)!=record['size'] or hashlib.sha256(data).hexdigest()!=record['sha256']:raise RuntimeError('package identity mismatch '+name)
    report=dict(passed=True,path=archive.relative_to(REPO).as_posix(),files=len(files),size=archive.stat().st_size,
                sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),zip_crc='passed',file_hashes='all passed',manifest=manifest)
    (destination/'PACKAGE_VERIFICATION.json').write_text(json.dumps(report,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps({k:v for k,v in report.items() if k!='manifest'},indent=2))


if __name__=='__main__':main()
