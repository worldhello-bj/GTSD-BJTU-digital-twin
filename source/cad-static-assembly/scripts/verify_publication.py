from pathlib import Path
import gzip, hashlib, json, re
R=Path(__file__).resolve().parents[1]
p=R/'inputs/reference_assembly.json.gz'
meta=json.loads((R/'evidence/publication_provenance.json').read_text())
assert hashlib.sha256(p.read_bytes()).hexdigest()==meta['published_assembly_sha256']
raw=gzip.decompress(p.read_bytes());d=json.loads(raw)
assert not any((b'/' + x + b'/') in raw for x in [b'workspace', b'home', b'tmp'])
assert not re.search(rb'sediment:' + b'//|file:' + b'//|https?://',raw)
subset={k:d[k] for k in ['meshes','alternative_meshes','registry']}
assert hashlib.sha256(json.dumps(subset,sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()).hexdigest()==meta['geometry_registry_sha256']
assert 'shell_contact_boundary' not in d['cad_body_overlay']
assert len(d['meshes'])==1936 and len(d['registry'])==109
assert sum(len(m['indices'])//3 for m in d['meshes'])==1256416
assert all(m['native_pose_bound_in_this_scene'] is False for m in d['meshes'])
print('PASS: published bytes, unchanged geometry/registry, full triangle counts, static physics scope, and privacy scan')
