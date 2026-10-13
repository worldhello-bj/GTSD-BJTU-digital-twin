"""Portable public entrypoint: restore local payloads, then start loopback console."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
from prepare_assets import ensure_assets
ensure_assets()
if '--research-rig' in sys.argv:
    sys.argv.remove('--research-rig')
    sys.path.insert(0, str(ROOT / 'research_rig'))
    def main():
        import runpy
        runpy.run_path(str(ROOT / "research_rig/server.py"), run_name="__main__")
else:
    sys.path.insert(0, str(ROOT / 'integration'))
    from source_console import main
if __name__ == '__main__':
    main()
