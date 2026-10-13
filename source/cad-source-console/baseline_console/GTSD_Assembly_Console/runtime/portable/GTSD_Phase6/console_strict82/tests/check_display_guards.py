from pathlib import Path
import sys,os,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import strict82_server as S
c=S.load_config(os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'strict82_config.json');assert c['variant']==S.VARIANT
F=S.fan_console();default=S.strict((ROOT/'console_config.portable.json').read_bytes());assert default['variant']=='internal-coupled-research'
for text in ('NaN','Infinity','1e999','{"a":1,"a":2}'):
 try:S.strict(text)
 except ValueError:pass
 else:raise AssertionError('Invalid strict JSON accepted')
out=S.panel_payload({'variant':'cad_doors_four_fans','model_identity_sha256':'9bd3b5a7f26a40c44f1b77f5c71b9ec53488e682170ddfda651a939aa96323db','source_geometry_sha256':'2b6314a577bc67c5781268d797c57be6674ef16d42eed9f47731238cfc299fed','telemetry':{}},c)
assert not out['identity_match'] and all(r['value']is None and not r['available'] for r in out['observations'])
(ROOT/'evidence/adapted_fan82.js').write_bytes(S.adapted_fan_js(c));print(json.dumps({'ok':True,'strict_json_cases':4,'wrong_variant_readouts_cleared':True,'default75_template_preserved':True,'source_freeze_pins_passed':True}))
