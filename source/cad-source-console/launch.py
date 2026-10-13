"""Portable public entrypoint: restore local payloads, then start loopback console."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
from prepare_assets import ensure_assets
ensure_assets()
sys.path.insert(0, str(ROOT / 'integration'))
from source_console import main
if __name__ == '__main__':
    main()
