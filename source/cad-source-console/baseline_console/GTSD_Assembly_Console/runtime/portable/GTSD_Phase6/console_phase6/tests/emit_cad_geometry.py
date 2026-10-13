"""Stream reviewed composite geometry to a renderer; no model or duplicate file."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import phase6_server as S
config=S.load_config(ROOT/'phase6_config.json')
config['variant']=S.CAD_VARIANT
S.verify_cad_freeze(config['cad_body_root'])
S.source_modules(config)
from cad_body.geometry import visual_resource_bytes
sys.stdout.buffer.write(visual_resource_bytes())
