"""Copy complete results only after verifying their actual executed source bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import os

ROOT = Path(__file__).resolve().parent
CASES = ('service_brake', 'half_dt', 'spatial_refinement', 'doors_open_close',
         'doors_half_dt', 'tether_release')
SUFFIXES = ('.xml', '.csv', '_metrics.json', '_auxiliaries.csv', '_replay.json')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(snapshot, cases=CASES):
    snapshot = Path(snapshot).resolve()
    source = snapshot/'source/vehicle-physics/b11/results'
    prepared = []
    for case in cases:
        identity_path = source/(case+'_identity.json')
        identity = json.loads(identity_path.read_text(encoding='utf-8'))
        if identity['case'] != case:
            raise ValueError('case identity mismatch: '+case)
        expected_files = {case+suffix for suffix in SUFFIXES}
        if set(identity['outputs_sha256']) != expected_files:
            raise ValueError('missing or unexpected full-case output: '+case)
        for relative, expected in identity['sources_sha256'].items():
            relative = relative.replace('\\', '/')
            current = (ROOT.parent/relative).resolve()
            executed = (snapshot/'source/vehicle-physics'/relative).resolve()
            if not current.is_relative_to(ROOT.parent.resolve()) or not executed.is_relative_to(snapshot):
                raise ValueError('source path leaves physics root')
            if digest(current) != expected or digest(executed) != expected:
                raise ValueError('executed source differs: '+relative)
        for name, expected in identity['outputs_sha256'].items():
            path = source/name
            if digest(path) != expected:
                raise ValueError('output identity mismatch: '+name)
            prepared.append((path, expected))
        prepared.append((identity_path, digest(identity_path)))
    return prepared


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--case', choices=(*CASES,'all'), default='all',
                        help='Import one completed case for early inspection; final acceptance still requires all six.')
    args = parser.parse_args()
    cases = CASES if args.case=='all' else (args.case,)
    prepared = validate(args.snapshot, cases)
    output = ROOT/'results'
    output.mkdir(parents=True, exist_ok=True)
    for source, expected in prepared:
        with tempfile.NamedTemporaryFile(dir=output, prefix='import-', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(source.read_bytes())
        try:
            if digest(temporary) != expected:
                raise ValueError('copy identity mismatch: '+source.name)
            os.replace(temporary, output/source.name)
        finally:
            temporary.unlink(missing_ok=True)
    print(json.dumps(dict(cases=len(cases), files=len(prepared),
                          copied_without_modification=True), indent=2))


if __name__ == '__main__':
    main()
