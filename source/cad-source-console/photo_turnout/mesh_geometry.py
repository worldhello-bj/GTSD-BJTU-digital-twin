"""Pure small-mesh geometry routines; no file, source-path, solver or network access."""
import itertools
import numpy as np
from scipy.spatial import ConvexHull

def unique(v):
    return np.unique(np.asarray(v, dtype=float), axis=0)

def hull_mesh(v):
    v = unique(v)
    h = ConvexHull(v)
    f = h.simplices.copy()
    for i, t in enumerate(f):
        if np.dot(np.cross(v[t[1]] - v[t[0]], v[t[2]] - v[t[0]]), h.equations[i, :3]) < 0:
            f[i] = t[[0, 2, 1]]
    return (v, f)

def blank_mesh(vertices):
    v = np.concatenate(vertices)
    xy = unique(v[:, :2])
    h = ConvexHull(xy)
    p = xy[h.vertices]
    n = len(p)
    z0 = v[:, 2].min()
    z1 = v[:, 2].max()
    w = np.vstack([np.c_[p, np.full(n, z0)], np.c_[p, np.full(n, z1)]])
    f = []
    for i in range(1, n - 1):
        f.extend([[0, i + 1, i], [n, n + i, n + i + 1]])
    for i in range(n):
        k = (i + 1) % n
        f.extend([[i, k, n + k], [i, n + k, n + i]])
    return (w, np.asarray(f))

def inside(v, f, p):
    tri = v[f]
    a = tri[:, 0, :2]
    b = tri[:, 1, :2] - a
    c = tri[:, 2, :2] - a
    d = p[:2] - a
    det = b[:, 0] * c[:, 1] - b[:, 1] * c[:, 0]
    ok = abs(det) > 1e-16
    u = np.zeros(len(f))
    w = u.copy()
    u[ok] = (d[ok, 0] * c[ok, 1] - d[ok, 1] * c[ok, 0]) / det[ok]
    w[ok] = (b[ok, 0] * d[ok, 1] - b[ok, 1] * d[ok, 0]) / det[ok]
    ok &= (u >= -1e-10) & (w >= -1e-10) & (u + w <= 1 + 1e-10)
    z = tri[:, 0, 2] + u * (tri[:, 1, 2] - tri[:, 0, 2]) + w * (tri[:, 2, 2] - tri[:, 0, 2])
    z = np.unique(np.round(z[ok & (z > p[2] + 1e-10)], 10))
    return len(z) % 2 == 1

def convex_separation(a, b, tol=1e-09):
    """A valid positive separating plane is a proof for all triangles, even nonconvex B."""
    va, fa = a
    vb, fb = b
    axes = []
    for v, f in [a, b]:
        tri = v[f]
        ns = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        norm = np.linalg.norm(ns, axis=1)
        ns = ns[norm > 1e-14] / norm[norm > 1e-14, None]
        axes.extend(ns)
    for ax in axes:
        pa = va @ ax
        pb = vb @ ax
        gap = max(pb.min() - pa.max(), pa.min() - pb.max())
        if gap >= -tol:
            return {'separated_or_coplanar': True, 'separating_axis': ax.tolist(), 'minimum_projected_gap_mm': float(gap * 1000)}
    return None

def edge_hit(p, q, t):
    d = q - p
    e1 = t[1] - t[0]
    e2 = t[2] - t[0]
    h = np.cross(d, e2)
    det = np.dot(e1, h)
    if abs(det) < 1e-18:
        return None
    s = p - t[0]
    u = np.dot(s, h) / det
    if u < -1e-09 or u > 1 + 1e-09:
        return None
    qq = np.cross(s, e1)
    v = np.dot(d, qq) / det
    tt = np.dot(e2, qq) / det
    if v < -1e-09 or u + v > 1 + 1e-09 or tt < -1e-09 or (tt > 1 + 1e-09):
        return None
    return p + tt * d

def coplanar_touch(a, b):
    n = np.cross(a[1] - a[0], a[2] - a[0])
    nn = np.linalg.norm(n)
    if nn < 1e-20:
        return False
    if max(abs((b - a[0]) @ n)) / nn > 1e-09:
        return False
    drop = np.argmax(abs(n))
    a = np.delete(a, drop, axis=1)
    b = np.delete(b, drop, axis=1)
    for p in [a, b]:
        for d in np.roll(p, -1, axis=0) - p:
            ax = np.array([-d[1], d[0]])
            aa = a @ ax
            bb = b @ ax
            if aa.max() < bb.min() - 1e-14 or bb.max() < aa.min() - 1e-14:
                return False
    return True

def exact_triangle_separation(a, b):
    va, fa = a
    vb, fb = b
    aa = va[fa]
    bb = vb[fb]
    blo = bb.min(1)
    bhi = bb.max(1)
    tested = 0
    for t in aa:
        ids = np.where(np.all(bhi >= t.min(0) - 1e-09, axis=1) & np.all(blo <= t.max(0) + 1e-09, axis=1))[0]
        for j in ids:
            u = bb[j]
            tested += 1
            if any((edge_hit(p, q, u) is not None for p, q in zip(t, np.roll(t, -1, axis=0)))):
                return {'status': 'SURFACE_INTERSECTION_OR_TOUCH', 'tested_triangle_pairs': tested}
            if any((edge_hit(p, q, t) is not None for p, q in zip(u, np.roll(u, -1, axis=0)))):
                return {'status': 'SURFACE_INTERSECTION_OR_TOUCH', 'tested_triangle_pairs': tested}
            if coplanar_touch(t, u):
                return {'status': 'SURFACE_INTERSECTION_OR_TOUCH', 'tested_triangle_pairs': tested}
    for v, f, w, g in [(va, fa, vb, fb), (vb, fb, va, fa)]:
        points = unique(v)
        points = points[np.all(points > w.min(0) + 1e-09, axis=1) & np.all(points < w.max(0) - 1e-09, axis=1)]
        for p in points:
            if inside(w, g, p):
                return {'status': 'CONTAINMENT_WITNESS', 'witness_world_m': p.tolist(), 'tested_triangle_pairs': tested}
    return {'status': 'EXACT_TRIANGULATED_MESHES_SEPARATED', 'tested_triangle_pairs': tested, 'method': 'Triangle AABB, six segment-triangle tests, coplanar 2D SAT, both unique-vertex containment directions', 'tolerance_m': 1e-09}

def pair_screen(a, b):
    va, fa = a
    vb, fb = b
    lo = np.maximum(va.min(0), vb.min(0))
    hi = np.minimum(va.max(0), vb.max(0))
    d = hi - lo
    if np.any(d <= 1e-08):
        return {'status': 'AABB_SEPARATED_OR_BOUNDARY_CONTACT', 'axis_gap_mm': (-d * 1000).tolist()}
    proof = convex_separation(a, b)
    if proof:
        return {'status': 'SEPARATING_PLANE_PROOF', **proof}
    for u in itertools.product([0.01, 0.1, 0.5, 0.9, 0.99], repeat=3):
        p = lo + np.array(u) * d
        if inside(*a, p) and inside(*b, p):
            return {'status': 'POSITIVE_MATERIAL_WITNESS', 'witness_world_m': p.tolist()}
    return exact_triangle_separation(a, b)

def volume(v, f):
    q = v - v.mean(0)
    t = q[f]
    return abs(np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6)

def closed(v, f):
    _, inv = np.unique(v, axis=0, return_inverse=True)
    f = inv[f]
    edges = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    _, n = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
    return bool(np.all(n == 2))
