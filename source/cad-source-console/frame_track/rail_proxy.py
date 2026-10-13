"""Apply source-audited rail geometry to a NEW XML variant, never to a baseline file.

No MuJoCo import. Caller owns compilation, acceptance and the new model identity.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent

def digest(value):
    return hashlib.sha256(value.encode() if isinstance(value, str) else value).hexdigest()

def vector(text):
    values = list(map(float, text.split()))
    if len(values) != 3 or any(not __import__('math').isfinite(v) for v in values):
        raise ValueError('Finite three-vector required')
    return values

def close(a, b):
    return len(a) == len(b) and all(abs(x-y) < 2e-13 for x, y in zip(a, b))

def augment_xml(xml, contract=None):
    if type(xml) is not str:
        raise TypeError('Input must be a complete MJCF XML string')
    contract = json.loads((ROOT/'rail_proxy_contract.json').read_text()) if contract is None else contract
    if contract.get('schema') != 'gtsd-source-aligned-rail-proxy-contract-v1':
        raise ValueError('Unexpected rail source contract')
    root = ET.fromstring(xml); world = root.find('worldbody')
    if world is None or len(contract['rows']) != 2 or {r['geom_name'] for r in contract['rows']} != {'rail_-1', 'rail_1'}:
        raise ValueError('Exactly two named source rails and a worldbody are required')
    changes = []
    for row in contract['rows']:
        vectors = [row[k] for k in ('old_pos_m','old_half_size_m','new_pos_m','new_half_size_m')]
        if any(len(v) != 3 or any(not __import__('math').isfinite(x) for x in v) for v in vectors):
            raise ValueError('Finite source geometry vectors required')
        if any(x <= 0 for x in row['new_half_size_m']):
            raise ValueError('Positive source half-sizes required')
        pos,size = row['new_pos_m'],row['new_half_size_m']
        if (not close([pos[0]-size[0],pos[0]+size[0]],row['new_x_endpoints_m'])
            or not close([pos[1]-size[1],pos[1]+size[1]],row['new_y_top_bounds_m'])
            or abs(pos[2]+size[2]-row['new_top_z_m']) > 2e-13
            or pos[2] != row['old_pos_m'][2] or size[2] != row['old_half_size_m'][2]
            or row['added_mass_kg'] != 0 or row['added_dof'] != 0):
            raise ValueError('Rail contract exceeds verified position/size boundary')
        name = row['geom_name']; found = root.findall(f'.//geom[@name="{name}"]')
        node = world.find(f'geom[@name="{name}"]')
        if len(found) != 1 or node is None or found[0] is not node:
            raise ValueError('Rail must occur exactly once directly under worldbody: '+name)
        if (node.get('type') != 'box' or node.get('contype') != '1' or node.get('conaffinity') != '2'
                or any(k in node.attrib for k in ('quat','euler','axisangle','xyaxes','zaxis','fromto'))):
            raise ValueError('Unexpected rail collision type, mask or orientation: '+name)
        if not close(vector(node.get('pos', '')), row['old_pos_m']) or not close(vector(node.get('size', '')), row['old_half_size_m']):
            raise ValueError('Rail baseline mismatch or augmentation already applied: '+name)
        old = dict(node.attrib)
        node.set('pos', ' '.join(format(x, '.17g') for x in row['new_pos_m']))
        node.set('size', ' '.join(format(x, '.17g') for x in row['new_half_size_m']))
        changes.append({'name': name, 'old': old, 'new': dict(node.attrib)})
    result = ET.tostring(root, encoding='unicode')
    spec = {'schema': 'gtsd-source-aligned-rail-proxy-application-v1',
        'contract_sha256': digest(json.dumps(contract, sort_keys=True, separators=(',', ':'), allow_nan=False)),
        'module_sha256': digest(Path(__file__).read_bytes()), 'source_xml_sha256': digest(xml),
        'new_xml_sha256': digest(result), 'new_physics_identity_required': True,
        'old_checkpoint_compatible': False, 'added_mass_kg': 0., 'added_dof': 0,
        'native_acceptance_run': False, 'changes': changes}
    return result, spec
