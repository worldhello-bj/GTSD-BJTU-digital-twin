"""Validate published outputs against a supplied console root without modifying it.
Use --rebuild to regenerate both variants locally; no physics or network calls.
"""
from pathlib import Path
import argparse,json,gzip,hashlib,subprocess,sys
H=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--source',required=True,type=Path);ap.add_argument('--rebuild',action='store_true');a=ap.parse_args()
g=a.source/'mechanical_fit/components/track_support/rail_body_review/native_edge_graph.json'
geometry=lambda p:[(r['handle'],r['points'])for r in json.loads(p.read_text())['source_curves']]
assert geometry(g)==geometry(H/'native_edge_graph.json'),'Source finite geometry mismatch'
if a.rebuild:
 for script in ['build_candidate.py','hardware_revision/build_hardware.py']:
  subprocess.run([sys.executable,str(H/script)],check=True,cwd=H)
subprocess.run(['node',str(H/'build_source_profiles.js'),'--source',str(a.source.resolve())],check=True,cwd=H)
import cap_geometry as C
for folder in ['build','hardware_revision']:
 m=json.loads((H/folder/'change_manifest.json').read_text());p=json.load(gzip.open(H/folder/'candidate_meshes.json.gz','rt'));assert len(p['meshes'])==len(m['add'])
 assert all(C.topology(x)['closed']and C.topology(x)['positive_volume']for x in p['meshes'])
 assert len({x['name']for x in p['meshes']})==len(p['meshes'])
 assert not any('sleeper'in r['name']for r in m['remove'])
print('PORTABLE_SOURCE_IDENTITY_AND_OUTPUT_TOPOLOGY_PASS')
