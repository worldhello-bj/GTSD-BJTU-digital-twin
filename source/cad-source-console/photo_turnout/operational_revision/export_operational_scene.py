"""Reproduce the full operational scene using the repository's public inputs.

Run after prepare_assets.py has restored the public assets:
  python export_operational_scene.py --output /path/outside/the/console/export

The output directory must be new or empty and outside source/cad-source-console.
Both builders run in separate output folders; frozen runtime files are never
overwritten. No downloads, package installation, Blender, or solver are used.
See requirements-geometry.txt for the optional export environment.
"""
from pathlib import Path
import argparse
import ctypes
import datetime
import hashlib
import importlib.metadata
import json
import os
import platform
import shutil
import subprocess
import sys
import zlib

HERE = Path(__file__).resolve().parent
CONSOLE = HERE.parent.parent


def identity(path):
    h = hashlib.sha256()
    size = 0
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
            size += len(block)
    return {'bytes': size, 'sha256': h.hexdigest()}


def output_path(value, console=CONSOLE):
    raw = Path(value).expanduser()
    if raw.is_symlink():
        raise ValueError('Output must not be a symbolic link.')
    output = raw.resolve()
    if output == console.resolve() or output.is_relative_to(console.resolve()):
        raise ValueError('Choose an output directory outside source/cad-source-console.')
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError('Output must be a new or empty directory; existing files are preserved.')
    return output


def run_logged(command, log, environment):
    with log.open('x', encoding='utf-8') as stream:
        result = subprocess.run(command, cwd=CONSOLE, env=environment,
                                stdout=stream, stderr=subprocess.STDOUT, check=False)
    if result.returncode:
        raise RuntimeError(f'Builder exited with code {result.returncode}; see {log}')


def export(output):
    output = output_path(output)
    contract = json.loads((HERE / 'REPRODUCIBLE_EXPORT.json').read_text(encoding='utf-8'))
    if contract['schema'] != 'gtsd-operational-reproducible-export-v1':
        raise ValueError('Unknown reproducible-export contract.')
    node = shutil.which('node')
    if not node:
        raise RuntimeError('Node.js is required. See requirements-geometry.txt.')
    try:
        numpy_version = importlib.metadata.version('numpy')
        geos = ctypes.CDLL('libgeos_c.so.1')
        geos.GEOSversion.restype = ctypes.c_char_p
        geos_version = geos.GEOSversion().decode()
    except (importlib.metadata.PackageNotFoundError, OSError) as error:
        raise RuntimeError('The geometry export requires NumPy and the GEOS C library on Linux/WSL. See requirements-geometry.txt.') from error
    required = [
        CONSOLE / 'baseline_console/GTSD_Assembly_Console/assets/whole.glb.gz',
        CONSOLE / 'baseline_console/GTSD_Assembly_Console/app/data/catalog.json',
        HERE.parent / 'source_seed.json.gz',
        HERE.parent / 'photo_source_profiles.json',
        HERE.parent / 'hardware_revision/candidate_meshes.json.gz',
    ]
    missing = [str(path.relative_to(CONSOLE)) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError('Run python prepare_assets.py from the console directory first. Missing public assets: ' + ', '.join(missing))
    node_versions = json.loads(subprocess.check_output(
        [node, '-p', 'JSON.stringify({node:process.versions.node,zlib:process.versions.zlib})'], text=True))
    environment = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
                       PYTHONDONTWRITEBYTECODE='1')
    output.mkdir(parents=True, exist_ok=True)
    source_dir, operational_dir = output / 'source', output / 'operational'
    source_dir.mkdir()
    operational_dir.mkdir()
    print('Exporting the public source scene...', flush=True)
    run_logged([node, str(CONSOLE / 'mechanical_fit/export_scene.js'), str(source_dir)],
               output / 'source_export.log', environment)
    source = source_dir / 'source.json.gz'
    source_identity = identity(source)
    if source_identity['sha256'] != contract['source_scene_sha256']:
        raise ValueError('Source-scene SHA-256 does not match the pinned public seed. Check source-file identities and the tested dependency versions in requirements-geometry.txt.')
    print('Building the operational scene from the verified public source...', flush=True)
    run_logged([sys.executable, str(HERE / 'build_operational.py'), '--photo-assets', str(HERE.parent),
                '--source-scene', str(source), '--joint-gap-mm', str(contract['joint_gap_mm']),
                '--output', str(operational_dir)], output / 'operational_build.log', environment)
    target = operational_dir / contract['output']['filename']
    actual = identity(target)
    expected = {key: contract['output'][key] for key in ('bytes', 'sha256')}
    if actual != expected:
        raise ValueError('Operational scene differs from the frozen export identity. Outputs and logs are preserved for inspection.')
    report = {
        'schema': 'gtsd-operational-export-verification-v1',
        'completed_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'pass': True, 'source_identity': source_identity, 'output_identity': actual,
        'output_path': str(target), 'mesh_count': contract['mesh_count'],
        'public_inputs_only': True, 'frozen_runtime_files_overwritten': False,
        'environment': {'python': platform.python_version(), 'numpy': numpy_version,
                        'geos': geos_version, 'python_zlib': zlib.ZLIB_RUNTIME_VERSION,
                        'node': node_versions['node'], 'node_zlib': node_versions['zlib']},
    }
    with (output / 'verification.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path,
                        help='New or empty output directory outside source/cad-source-console')
    args = parser.parse_args()
    try:
        export(args.output)
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f'Export failed: {error}\n')


if __name__ == '__main__':
    main()
