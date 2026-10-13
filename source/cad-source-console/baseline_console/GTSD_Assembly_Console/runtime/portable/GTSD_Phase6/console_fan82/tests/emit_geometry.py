from pathlib import Path
import os,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import fan82_server as F
c=F.load_config(os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'console82_config.json');F.fan_factory(c)
from fan82.geometry import resource
sys.stdout.buffer.write(resource()[0])
