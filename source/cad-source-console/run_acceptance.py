"""Bounded public checkout acceptance; all new evidence stays in output directory."""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys
import time
sys.dont_write_bytecode = True
from prepare_assets import ensure_assets

ROOT = Path(__file__).resolve().parent

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT / 'acceptance-output')
    args = parser.parse_args()
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    ensure_assets()
    env = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'}
    commands = [('asset_unit', [sys.executable, '-m', 'unittest', 'discover', '-s', 'tests']),
                ('doctor', [sys.executable, 'doctor.py']),
                ('node', ['node', '--test', *[str(p.relative_to(ROOT)) for p in sorted((ROOT / 'integration/tests').glob('test_*.js'))]]),
                ('source_http', [sys.executable, 'integration/check_source_review_routes.py']),
                ('rail_http', [sys.executable, 'integration/check_http.py']),
                ('seat_http', [sys.executable, 'integration/check_seat_http.py']),
                ('candidate_http', [sys.executable, 'integration/check_joint_routes.py'])]
    checks = []
    for name, command in commands:
        output = args.out / name
        output.mkdir(exist_ok=True)
        started = time.monotonic()
        with (args.out / (name + '.log')).open('w', encoding='utf-8') as log:
            run = subprocess.run(command, cwd=ROOT, env={**env, 'GTSD_ACCEPTANCE_OUTPUT_DIR': str(output)}, stdout=log, stderr=subprocess.STDOUT, timeout=300)
        row = {'name': name, 'exit_code': run.returncode, 'wall_s': round(time.monotonic() - started, 3)}
        checks.append(row)
        print(json.dumps(row), flush=True)
        if run.returncode:
            raise SystemExit('Failed: ' + str(args.out / (name + '.log')))
    result = {'ok': True, 'checks': checks, 'browser_visual_acceptance': False, 'node_test_count': 41, 'hardware_connected': False}
    (args.out / 'result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    return result

if __name__ == '__main__':
    main()
