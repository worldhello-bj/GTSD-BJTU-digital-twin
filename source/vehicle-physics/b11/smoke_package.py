"""Run the accepted B11 package in a new directory without the original checkout."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]


def main():
    folder=REPO/'artifacts/B11'; archive=folder/'GTSD_BJTU_B11_source_cable_dynamics.zip'
    identity=json.loads((folder/'PACKAGE_VERIFICATION.json').read_text(encoding='utf-8'))
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest!=identity['sha256']:
        raise RuntimeError('archive identity changed')
    parent=REPO/'package-verification/b11'; parent.mkdir(parents=True,exist_ok=True)
    target=Path(tempfile.mkdtemp(prefix='unpacked-',dir=parent)).resolve()
    with zipfile.ZipFile(archive) as z:
        if any(not (target/n).resolve().is_relative_to(target) for n in z.namelist()):
            raise RuntimeError('archive path leaves unpack root')
        z.extractall(target)
    env=os.environ.copy(); env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    results=[]
    for args in [('verify.py',),('test_cables.py',),('test_integration.py',),('live.py','--headless','--settle-s','0','--duration','.02')]:
        process=subprocess.run([sys.executable,str(target/'source/vehicle-physics/b11'/args[0]),*args[1:]],cwd=target,env=env,
                               capture_output=True,text=True,encoding='utf-8',timeout=180)
        results.append(dict(script=args[0],arguments=list(args[1:]),exit_code=process.returncode,
                            stdout_tail=process.stdout[-2000:],stderr_tail=process.stderr[-3000:]))
        print(args[0],process.returncode,flush=True)
    report=dict(passed=all(r['exit_code']==0 for r in results),archive_sha256=digest,
                unpack_directory=target.relative_to(REPO).as_posix(),checks=results,
                scope='actual package output identity, components/regressions and headless startup; no six-case rerun')
    (folder/'UNPACKED_SMOKE.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'],checks=len(results),archive_sha256=digest),indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':
    main()
