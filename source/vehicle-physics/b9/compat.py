"""Private B8 imports keep the frozen source and ordinary B8 imports unchanged."""
import importlib.util
from pathlib import Path
import sys

B8 = Path(__file__).resolve().parents[1] / "b8"


def load_b8(script, name):
    if str(B8) not in sys.path:
        sys.path.append(str(B8))
    spec = importlib.util.spec_from_file_location(name, B8/(script+".py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module
