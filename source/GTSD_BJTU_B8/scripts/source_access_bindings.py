"""Audited source-door binding contract; no physics or animation is synthesized.

Only enables when all six force-solved door bodies are in the replay.  The source
inventory is evaluated at frame 1 with cabinet_open_deg=0, never at the saved
100-degree inspection pose.  The cabinet body and all door masses belong to the
solver model, not to this display-only helper.
"""
from pathlib import Path
import json
import numpy as np

SOURCE_SHA256 = 'c96811c45220dcdfad0853b73e4743297be7e090d4fb3230a7531eb89b3a4c04'
OWNERS = {
    'BayDoor_L': 'access_door_bay_L',
    'BayDoor_R': 'access_door_bay_R',
    'BayDoor_L_Secondary': 'access_door_bay_L_secondary',
    'BayDoor_R_Secondary': 'access_door_bay_R_secondary',
    'B5_PWR 01_DoorPivot': 'access_door_cabinet_PWR',
    'B5_DAQ 01_DoorPivot': 'access_door_cabinet_DAQ',
}


def bindings(inventory_path, reference_world_matrices, source_sha256, x_shift=3.0):
    """Return closed-source part bindings only after identity and rest-pose QA.

    Matrices are row-major 4x4 world matrices. Return an inactive result for a
    historical B8 replay with no door extension; reject partial extensions.
    """
    required = set(OWNERS.values())
    found = required.intersection(reference_world_matrices)
    if not found:
        return {'enabled': False, 'source_frames': {}, 'object_targets': {},
                'source_owner_targets': {}, 'source_part_records': {}, 'checks': {}}
    if found != required:
        raise ValueError('Partial six-door extension: ' + str(sorted(required-found)))
    inventory = json.loads(Path(inventory_path).read_text())
    if source_sha256 != SOURCE_SHA256 or inventory['source_sha256'] != source_sha256:
        raise ValueError('Source door inventory does not match source blend identity')
    if inventory['source_frame'] != 1 or inventory['cabinet_open_deg'] != 0:
        raise ValueError('Source door inventory must use the closed rest pose')
    if {g['name'] for g in inventory['groups']} != set(OWNERS):
        raise ValueError('Source inventory does not contain exactly the six door owners')
    shift = np.eye(4); shift[0, 3] = x_shift
    source_frames, targets, records = {}, {}, {}
    max_error = 0.0
    for group in inventory['groups']:
        body = OWNERS[group['name']]
        src = np.asarray(group['source_hinge_world_matrix'], float)
        ref = np.asarray(reference_world_matrices[body], float)
        if src.shape != (4, 4) or ref.shape != (4, 4):
            raise ValueError('Door transform must be a 4x4 matrix')
        if not np.all(np.isfinite(src)) or not np.all(np.isfinite(ref)):
            raise ValueError('Door transform contains non-finite values')
        error = float(np.max(np.abs(shift @ src - ref)))
        max_error = max(max_error, error)
        if error > 2e-6:
            raise ValueError('Door zero-pose registration mismatch: ' + body + ' ' + str(error))
        source_frames[body] = src.tolist()
        if body.startswith('access_door_cabinet_'):
            support = body + '_fixed_support'
            if support not in reference_world_matrices:
                raise ValueError('Door cabinet has no recorded fixed support: ' + support)
            support_ref = np.asarray(reference_world_matrices[support], float)
            if support_ref.shape != (4,4) or not np.all(np.isfinite(support_ref)) or float(np.max(np.abs(support_ref-ref))) > 2e-6:
                raise ValueError('Cabinet support zero-pose registration mismatch: ' + support)
            source_frames[support] = src.tolist()
        for part in group['objects']:
            name = part['name']
            if name in targets:
                raise ValueError('Source door part has two owners: ' + name)
            targets[name] = body; records[name] = part
    if len(targets) != 84:
        raise ValueError('Expected 84 source door-owned parts, got ' + str(len(targets)))
    return {'enabled': True, 'source_frames': source_frames, 'object_targets': targets,
            'source_owner_targets': dict(OWNERS), 'source_part_records': records,
            'checks': {'source_sha256': source_sha256, 'leaves': 6, 'parts': len(targets),
                       'rest_matrix_max_error': max_error,
                       'physical_validation': 'Not tested here; replay matrices only.'}}
