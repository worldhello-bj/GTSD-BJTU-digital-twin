"""Read-only Phase 5 internal-system registration. No solver/network writes.

Public API: build_registry() returns independent JSON data; observe(snapshot)
accepts either a plant snapshot or Controller's {plant: snapshot} envelope.
All reported values are read at their native paths in that SAME snapshot.
"""
from __future__ import annotations
import copy
import json
import math
from functools import lru_cache
from pathlib import Path

HERE = Path(__file__).resolve().parent


@lru_cache(maxsize=2)
def _load_registry(mtime_ns):
    return json.loads((HERE / 'registry.json').read_text(encoding='utf-8'))


def _registry():
    return _load_registry((HERE / 'registry.json').stat().st_mtime_ns)


def build_registry():
    return copy.deepcopy(_registry())


def _get(value, path):
    for key in path:
        if isinstance(value, list):
            if type(key) is not int or key < 0 or key >= len(value):
                return False, None
            value = value[key]
        elif isinstance(value, dict) and key in value:
            value = value[key]
        else:
            return False, None
    return True, value


def _finite_tree(value):
    if isinstance(value, dict):
        return all(_finite_tree(v) for v in value.values())
    if isinstance(value, list):
        return all(_finite_tree(v) for v in value)
    return type(value) not in (int, float) or math.isfinite(value)


def observe(plant_snapshot, registry=None):
    """Project values without inventing unavailable data, advancing or mutating.

    Availability is per snapshot, never inferred from a saved sample. Zero and
    False are valid. Course channels remain UNAVAILABLE_HARDWARE_UNBOUND even
    if an analogous equivalent solver quantity exists elsewhere in this output.
    """
    if not isinstance(plant_snapshot, dict):
        raise TypeError('plant_snapshot must be a dictionary')
    plant = plant_snapshot.get('plant', plant_snapshot)
    if not isinstance(plant, dict):
        raise TypeError('plant must be a dictionary')
    reg = _registry() if registry is None else registry
    expected = reg['source_geometry_sha256']
    actual = plant.get('source_geometry_sha256')
    identity_ok = isinstance(actual, str) and actual == expected
    observations = plant.get('ideal_observations', {}).get('observations', [])
    observation_map = {o['signal_id']: o for o in observations
                       if isinstance(o, dict) and isinstance(o.get('signal_id'), str)}
    out = []
    for definition in reg['signals']:
        # Full immutable provenance and exact mesh selectors live in the static
        # catalog. Do not resend thousands of mesh IDs on every live sample.
        row = {k: copy.deepcopy(definition[k]) for k in (
            'id', 'unit')}
        row.update(value=None, available=False, availability='MISSING_IN_CURRENT_SNAPSHOT',
                   sample_time_s=plant.get('time_s'), source_path_used=None)
        if not identity_ok:
            row.update(availability='SOURCE_GEOMETRY_MISMATCH')
        elif definition.get('hardware_only'):
            row['availability'] = 'UNAVAILABLE_HARDWARE_UNBOUND'
        elif definition.get('requires_accepted_interval') and not plant.get('steps', 0):
            row['availability'] = 'NO_ACCEPTED_INTERVAL'
        elif definition.get('contact_active_path') and not _get(plant, definition['contact_active_path'])[1]:
            row['availability'] = 'INACTIVE_CONTACT_NO_MEASUREMENT'
        elif definition.get('required_coordinate_lengths') and any(
                len(plant.get(k, [])) != count for k, count in definition['required_coordinate_lengths'].items()):
            row['availability'] = 'COORDINATE_LAYOUT_MISMATCH'
        else:
            found, value = False, None
            if definition.get('observation_id'):
                ob = observation_map.get(definition['observation_id'])
                if ob is not None and 'value' in ob:
                    found, value = True, ob['value']
                    row['source_path_used'] = definition['telemetry_paths'][0]
                    row['sample_time_s'] = ob.get('sample_time_s', plant.get('time_s'))
            else:
                for path in definition.get('path_tokens', []):
                    found, value = _get(plant, path)
                    if found:
                        row['source_path_used'] = path
                        break
            if found:
                if value is None:
                    row['availability'] = 'NULL_IN_CURRENT_SNAPSHOT'
                elif isinstance(value, (float, int)) and not isinstance(value, bool) and not math.isfinite(value):
                    row['availability'] = 'INVALID_NONFINITE'
                elif definition['unit'] == 'boolean' and type(value) is not bool:
                    row['availability'] = 'INVALID_TYPE'
                elif definition['unit'] == 'enum' and type(value) is not str:
                    row['availability'] = 'INVALID_TYPE'
                elif definition['unit'] not in ('boolean', 'enum', None) and type(value) not in (int, float):
                    row['availability'] = 'INVALID_TYPE'
                else:
                    row.update(value=value, available=True,
                               availability='SOLVER_EQUIVALENT_UNCALIBRATED' if definition['unit'] is not None else 'PRESENT_UNIT_UNSPECIFIED')
        out.append(row)
    by_id = {row['id']: row for row in out}
    definitions = {row['id']: row for row in reg['signals']}
    for item in reg.get('items', []):
        selected = [by_id[key] for key in item.get('signal_ids', []) if key in by_id
                    and definitions[key].get('display_priority') != 'detail']
        values = [{'id': r['id'], 'label': definitions[r['id']]['label'], 'path': r['source_path_used'], 'value': r['value'],
                   'unit': r['unit'], 'availability': r['availability'],
                   'available': r['available'], 'sample_time_s': r['sample_time_s']}
                  for r in selected]
        out.append({'id': item['id'], 'label': item['label'], 'values': values,
                    'availability': 'SOURCE_GEOMETRY_MISMATCH' if not identity_ok else
                                    'HAS_SOLVER_VALUES' if any(v['available'] for v in values) else
                                    'UNAVAILABLE_HARDWARE_UNBOUND' if values else 'NO_TELEMETRY_MAPPING',
                    'available': any(v['available'] for v in values),
                    'sample_time_s': plant.get('time_s')})
    edges = []
    for edge in reg['edges']:
        e = {'id': edge['id'], 'from': edge['from'], 'to': edge['to'],
             'available': False, 'availability': 'COURSE_RELATION_NOT_LIVE_TOPOLOGY',
             'flow_kg_s': None, 'opening': None, 'cumulative_mass_kg': None,
             'sample_time_s': plant.get('time_s')}
        if not identity_ok:
            e['availability'] = 'SOURCE_GEOMETRY_MISMATCH'
        elif edge['kind'] == 'pneumatic_solver_edge':
            ok, flow = _get(plant, edge['flow_path'])
            if ok and type(flow) in (float, int) and math.isfinite(flow):
                e.update(available=True, availability='CURRENT_SOLVER_EDGE', flow_kg_s=flow)
                for field, path in [('opening', edge['opening_path']), ('cumulative_mass_kg', edge['mass_integral_path'])]:
                    present, value = _get(plant, path)
                    if present and type(value) in (float, int) and math.isfinite(value):
                        e[field] = value
            else:
                e['availability'] = 'NOT_IN_CURRENT_SOLVER_TOPOLOGY' if not ok else 'INVALID_FLOW_VALUE'
        edges.append(e)
    collections = []
    for definition in reg.get('collections', []):
        present, records = _get(plant, definition['path'])
        row = {'id': definition['id'], 'available': False, 'records': [],
               'availability': 'MISSING_IN_CURRENT_SNAPSHOT', 'sample_time_s': plant.get('time_s')}
        if not identity_ok:
            row['availability'] = 'SOURCE_GEOMETRY_MISMATCH'
        elif not plant.get('steps', 0):
            row['availability'] = 'NO_ACCEPTED_INTERVAL'
        elif present and isinstance(records, list):
            row.update(available=True, availability='ACTUAL_ACCEPTED_NATIVE_CONTACT_DECOMPOSITION')
            for record in records:
                if not isinstance(record, dict) or not _finite_tree(record):
                    row.update(available=False, availability='INVALID_COLLECTION_RECORD', records=[])
                    break
                r = copy.deepcopy(record)
                r['geometry_mesh_ids'] = sorted({definition['source_mesh_by_proxy_geom'][r[g]]
                    for g in ('geom1', 'geom2') if r.get(g) in definition['source_mesh_by_proxy_geom']})
                row['records'].append(r)
        collections.append(row)
    return {'schema': 'gtsd-internal-system-observations-v1',
            'sample_time_s': plant.get('time_s'), 'source_geometry_sha256': actual,
            'registry_geometry_sha256': expected, 'geometry_identity_match': identity_ok,
            'hardware_connected': False, 'calibrated': False,
            'available_count': sum(r['available'] for r in by_id.values()),
            'edges': edges,
            'collections': collections,
            'observations': out}
