"""Source-triangle inertial properties; collision pieces never carry density.

All coordinates are metres. Chosen mass/density is a research parameter, not a
material identification. Integration uses oriented tetrahedra around a nearby
origin to avoid subtracting large world-coordinate moments.
"""
import hashlib
import json
import numpy as np


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def integrate(mesh, *, mass_kg=None, density_kg_m3=None):
    if (mass_kg is None) == (density_kg_m3 is None):
        raise ValueError('Specify exactly one declared mass or density')
    value = mass_kg if mass_kg is not None else density_kg_m3
    if not np.isfinite(value) or value <= 0:
        raise ValueError('Mass/density must be finite and positive')
    v = np.asarray(mesh['vertices'], dtype=float).reshape(-1, 3)
    f = np.asarray(mesh['indices'], dtype=int).reshape(-1, 3)
    if not len(f) or not np.isfinite(v).all() or f.min() < 0 or f.max() >= len(v):
        raise ValueError('Invalid source triangles')
    uv, inverse = np.unique(v, axis=0, return_inverse=True)
    wf = inverse[f]
    edges = np.concatenate((wf[:, [0, 1]], wf[:, [1, 2]], wf[:, [2, 0]]))
    edge_keys = np.sort(edges, axis=1)
    _, idx, counts = np.unique(edge_keys, axis=0, return_inverse=True, return_counts=True)
    direction = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    if not np.all(counts == 2) or np.any(np.bincount(idx, direction)):
        raise ValueError('Source must be a closed consistently oriented manifold')
    origin = (v.min(axis=0) + v.max(axis=0)) / 2
    t = v[f] - origin
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    vol = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
    volume = float(vol.sum())
    if not np.isfinite(volume) or volume <= 0:
        raise ValueError('Source must have positive oriented volume')
    sums = a + b + c
    first = np.einsum('i,ij->j', vol, sums) / 4
    second = np.einsum('i,ij,ik->jk', vol, sums, sums)
    for q in (a, b, c):
        second += np.einsum('i,ij,ik->jk', vol, q, q)
    second /= 20
    center_local = first / volume
    covariance_integral = second - volume * np.outer(center_local, center_local)
    density = float(density_kg_m3) if density_kg_m3 is not None else float(mass_kg) / volume
    mass = volume * density
    inertia = density * (np.trace(covariance_integral) * np.eye(3) - covariance_integral)
    inertia = (inertia + inertia.T) / 2
    principal = np.linalg.eigvalsh(inertia)
    if principal[0] <= 0 or principal[2] > principal[:2].sum() * (1 + 1e-10):
        raise ValueError('Inertial tensor is not physically admissible')
    return {
        'mesh_name': mesh.get('name'),
        'geometry_sha256': canonical_sha({'vertices': mesh['vertices'], 'indices': mesh['indices']}),
        'volume_m3': volume, 'mass_kg': mass,
        'center_of_mass_world_m': (origin + center_local).tolist(),
        'inertia_com_world_kg_m2': inertia.tolist(),
        'principal_inertia_kg_m2': principal.tolist(),
        'declared_parameter': {'mass_kg': mass_kg, 'density_kg_m3': density_kg_m3},
        'parameter_status': 'UNCALIBRATED_RESEARCH_INPUT',
        'method': 'closed oriented source-triangle tetrahedral volume moments',
        'collision_density_used': False,
        'source_triangles': len(f), 'welded_vertices': len(uv),
    }


def combine(parts, *, hypothesis_id):
    """Rigid-group parallel-axis sum; repeated geometry ownership is rejected."""
    if not parts or not hypothesis_id:
        raise ValueError('An explicit rigid-group hypothesis is required')
    names = [p['mesh_name'] for p in parts]
    if None in names or len(set(names)) != len(names):
        raise ValueError('Each source mesh may contribute mass only once')
    mass = sum(p['mass_kg'] for p in parts)
    centers = np.array([p['center_of_mass_world_m'] for p in parts])
    center = np.sum(centers * np.array([p['mass_kg'] for p in parts])[:, None], axis=0) / mass
    inertia = np.zeros((3, 3))
    for p, c in zip(parts, centers):
        d = c - center
        inertia += np.array(p['inertia_com_world_kg_m2']) + p['mass_kg'] * (d @ d * np.eye(3) - np.outer(d, d))
    return {'mesh_names': names, 'hypothesis_id': hypothesis_id, 'mass_kg': mass,
            'center_of_mass_world_m': center.tolist(),
            'inertia_com_world_kg_m2': inertia.tolist(),
            'parameter_status': 'UNCALIBRATED_RESEARCH_INPUT', 'mass_contributions': parts}
