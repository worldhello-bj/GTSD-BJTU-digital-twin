"""Unpack B12 and actually rerun a complete drop case independently."""
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
    folder=REPO/'artifacts/B12'; archive=folder/'GTSD_BJTU_B12_cover_maintenance_bench.zip'
    identity=json.loads((folder/'PACKAGE_VERIFICATION.json').read_text(encoding='utf-8'))
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest!=identity['sha256']:raise RuntimeError('archive changed')
    parent=REPO/'package-verification/b12'; parent.mkdir(parents=True,exist_ok=True)
    target=Path(tempfile.mkdtemp(prefix='unpacked-',dir=parent)).resolve()
    with zipfile.ZipFile(archive) as z:
        if any(not (target/n).resolve().is_relative_to(target) for n in z.namelist()):raise RuntimeError('unsafe archive path')
        z.extractall(target)
    env=os.environ.copy(); env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    results=[]
    for args in [('verify.py',),('test_covers.py',),('simulate.py','--case','drop'),('verify.py',)]:
        run=subprocess.run([sys.executable,str(target/'source/vehicle-physics/b12'/args[0]),*args[1:]],cwd=target,env=env,
                           capture_output=True,text=True,encoding='utf-8',timeout=180)
        results.append(dict(script=args[0],arguments=list(args[1:]),exit_code=run.returncode,stdout_tail=run.stdout[-2000:],stderr_tail=run.stderr[-2000:]))
        print(args[0],list(args[1:]),run.returncode,flush=True)
    report=dict(passed=all(r['exit_code']==0 for r in results),archive_sha256=digest,
                unpack_directory=target.relative_to(REPO).as_posix(),checks=results,
                scope='packaged source/output identity, component tests, actual full independent drop rerun and final acceptance; no full five-case rerun')
    (folder/'UNPACKED_SMOKE.json').write_text(json.dumps(report,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps(dict(passed=report['passed'],checks=len(results),archive_sha256=digest),indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
