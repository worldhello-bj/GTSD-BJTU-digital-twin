"""Unpack the delivered archive into a fresh contained directory and run it."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]


def main():
    folder = REPO/'artifacts/B10'
    archive = folder/'GTSD_BJTU_B10_auxiliary_dynamics.zip'
    identity = json.loads((folder/'PACKAGE_VERIFICATION.json').read_text(encoding='utf-8'))
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest != identity['sha256']:
        raise RuntimeError('archive changed after package verification')
    parent = REPO/'package-verification/b10'
    parent.mkdir(parents=True, exist_ok=True)
    target = Path(tempfile.mkdtemp(prefix='unpacked-', dir=parent)).resolve()
    with zipfile.ZipFile(archive) as z:
        for name in z.namelist():
            if not (target/name).resolve().is_relative_to(target):
                raise RuntimeError('archive path leaves unpack directory')
        z.extractall(target)
    env = os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    results = []
    for args in [('verify.py',), ('test_integration.py',), ('live.py','--headless','--settle-s','0','--duration','.02')]:
        command = [sys.executable, str(target/'source/vehicle-physics/b10'/args[0]), *args[1:]]
        result = subprocess.run(command, cwd=target, env=env, capture_output=True, text=True, encoding='utf-8', timeout=180)
        results.append(dict(script=args[0], arguments=list(args[1:]), exit_code=result.returncode,
                            stdout_tail=result.stdout[-3000:], stderr_tail=result.stderr[-3000:]))
        print(args[0], result.returncode, flush=True)
    report = dict(passed=all(r['exit_code']==0 for r in results), archive_sha256=digest,
                  unpack_directory=target.relative_to(REPO).as_posix(), checks=results,
                  scope='actual packaged source and numeric output verification, components/regressions and headless live startup; no full rerun')
    (folder/'UNPACKED_SMOKE.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], archive_sha256=digest, checks=len(results)), indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
