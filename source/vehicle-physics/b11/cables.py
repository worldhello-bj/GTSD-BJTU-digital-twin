"""B11 source-anchored articulated cable equivalents; no coordinate drivers.

Finite-mass rigid capsules, passive ball-joint bending/torsion and floor/guide
contact. Source centerlines/anchors are evidence; constitutive inputs are not
measured. Initial centerlines are assumed stress-free. Rigid links have no axial
stretch, while soft endpoint connect constraints require measured error checks.
"""
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import sys
from xml.etree.ElementTree import fromstring, SubElement, tostring, indent
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent
B8 = ROOT.parent/"b8"
if str(B8) not in sys.path:
    sys.path.append(str(B8))
from full_model import model as b8_model


@dataclass(frozen=True)
class CableParameters:
    material_density_kg_m3: float = 1200.0
    reference_bending_EI_Nm2: float = .015
    reference_radius_m: float = .012
    angular_damping_Nm_s: float = .001
    damping_reference_length_m: float = .15
    floor_friction: float = .20
    chord_error_m: float = .003
    maximum_link_length_m: float = .6
    endpoint_time_constant_s: float = .0005
    floor_time_constant_s: float = .0005

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in asdict(self).values()):
            raise ValueError("positive finite cable parameters required")


def vec(v):
    return " ".join(format(float(x), ".12g") for x in v)


def add(parent, tag, **attributes):
    return SubElement(parent, tag, {k:str(v) for k,v in attributes.items()})


def simplify(points, tolerance, maximum_length):
    """RDP with exact source corner endpoints, then split long chords."""
    p = np.asarray(points, dtype=float)
    if len(p) < 2 or not np.isfinite(p).all():
        raise ValueError("finite polyline with at least two points required")
    def rdp(a):
        if len(a) <= 2:
            return a
        v = a[-1]-a[0]
        l2 = v@v
        if l2 == 0:
            distance = np.linalg.norm(a-a[0], axis=1)
        else:
            t = np.clip((a-a[0])@v/l2, 0, 1)
            distance = np.linalg.norm(a-(a[0]+t[:,None]*v), axis=1)
        k = int(np.argmax(distance))
        if distance[k] <= tolerance:
            return a[[0,-1]]
        return np.concatenate((rdp(a[:k+1])[:-1], rdp(a[k:])))
    corners = rdp(p)
    out = [corners[0]]
    for a,b in zip(corners, corners[1:]):
        length = np.linalg.norm(b-a)
        if length < 1e-10:
            continue
        count = max(1, math.ceil(length/maximum_length))
        out.extend(a+(b-a)*k/count for k in range(1, count+1))
    return np.asarray(out)


def source_routes():
    source = json.loads((ROOT/"source_cables.json").read_text(encoding="utf-8"))
    records = {r["name"]:r for r in source["records"]}
    routes = []
    for i in range(3):
        loop, lead = records["B4_MovingLoop_"+str(i)], records["B4_CarServiceLead_"+str(i)]
        if np.linalg.norm(np.array(loop["points_world_m"][-1])-lead["points_world_m"][0]) > 1e-6:
            raise ValueError("source floor/service centerline endpoints are disconnected")
        points = loop["points_world_m"]+lead["points_world_m"][1:]
        routes.append(dict(key="supply_"+str(i), source_objects=[loop["name"], lead["name"]], points=points,
                           radius_m=loop["radius_m"], moving_body="A_frame"))
    for code in ("PWR", "DAQ"):
        r = records["B5_"+code+" 01_BondingLoop"]
        routes.append(dict(key="bond_"+code, source_objects=[r["name"]], points=r["points_world_m"],
                           radius_m=r["radius_m"], moving_body="access_door_cabinet_"+code))
    return source, routes


LAST_SPEC = None


def model(params=None, return_spec=False, return_all_specs=False, *, cable_parameters=None):
    global LAST_SPEC
    cp = cable_parameters or replace(CableParameters(), maximum_link_length_m=(params or {}).get('cable_maximum_link_length_m', .6))
    xml, panto, doors = b8_model(params, return_all_specs=True)
    tree = fromstring(xml)
    tree.find("compiler").set("inertiafromgeom", "auto")  # existing explicit inertials take precedence
    # Bind anchors in the actual B8 reference pose rather than approximate
    # frame origins. Source and solver differ by the documented +3 m X shift.
    reference = mujoco.MjModel.from_xml_string(xml); state = mujoco.MjData(reference)
    mujoco.mj_forward(reference, state)
    world, equalities, contacts = tree.find("worldbody"), tree.find("equality"), tree.find("contact")
    source, routes = source_routes()
    records = []
    for route in routes:
        key, radius = route["key"], route["radius_m"]
        source_points = np.asarray(route["points"], dtype=float)+[3,0,0]
        points = simplify(source_points, cp.chord_error_m, cp.maximum_link_length_m)
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        original_length = float(np.linalg.norm(np.diff(source_points, axis=0), axis=1).sum())
        if abs(lengths.sum()/original_length-1) > .01:
            raise ValueError("articulated approximation loses more than 1% source centerline length")
        body_id = reference.body(route["moving_body"]).id
        local = state.xmat[body_id].reshape(3,3).T@(points[-1]-state.xpos[body_id])
        moving = next(el for el in world.iter("body") if el.get("name") == route["moving_body"])
        add(moving, "site", name="cable_"+key+"_anchor", pos=vec(local), size=".004", rgba="1 .6 .1 1")
        parent = world
        names = []
        for i, (a,b,length) in enumerate(zip(points, points[1:], lengths)):
            name = "cable_"+key+"_"+str(i)
            body = add(parent, "body", name=name, pos=vec(a if i == 0 else a-points[i-1]))
            ei = cp.reference_bending_EI_Nm2*(radius/cp.reference_radius_m)**4
            add(body, "joint", name=name+"_bend", type="ball", pos="0 0 0", limited="false",
                stiffness=ei/length, damping=cp.angular_damping_Nm_s*cp.damping_reference_length_m/length, armature="0")
            mass = cp.material_density_kg_m3*math.pi*radius**2*length
            color = ".06 .08 .10 1" if key == "supply_0" else ".04 .3 .62 1" if key == "supply_1" else ".1 .5 .2 1"
            add(body, "geom", name=name+"_capsule", type="capsule", fromto=vec([0,0,0])+" "+vec(b-a),
                size=radius, mass=mass, contype="4", conaffinity="4", friction=f"{cp.floor_friction} .001 .0001",
                solref=f"{cp.floor_time_constant_s} 1", solimp=".95 .99 .001", rgba=color)
            parent = body; names.append(name)
        add(parent, "site", name="cable_"+key+"_end", pos=vec(points[-1]-points[-2]), size=".004")
        add(equalities, "connect", name="cable_"+key+"_attachment", site1="cable_"+key+"_end",
            site2="cable_"+key+"_anchor", solref=f"{cp.endpoint_time_constant_s} 1", solimp=".995 .995 .001")
        # Capsule end caps overlap at tight centerline corners. These local
        # neighbors represent one continuous material, not two colliding rods.
        # Nonlocal self contacts and contacts between distinct routes remain.
        for i, name in enumerate(names):
            for neighbor in names[i+1:i+3]:
                add(contacts, "exclude", body1=name, body2=neighbor)
        records.append(dict(key=key, source_objects=route["source_objects"], moving_body=route["moving_body"],
                            source_length_m=original_length, link_length_sum_m=float(lengths.sum()),
                            source_length_relative_change=float(lengths.sum()/original_length-1),
                            radius_m=radius, link_count=len(lengths), body_names=names,
                            mass_kg=float(cp.material_density_kg_m3*math.pi*radius**2*lengths.sum()),
                            fixed_endpoint_world_m=points[0].tolist(), moving_anchor_local_m=local.tolist(),
                            reference_centerline_world_m=points.tolist()))
    # Source floor/trough supports cables only. The original wheel/rail force
    # query and explicitly paired brake/pantograph contacts stay unchanged.
    add(world, "geom", name="cable_floor", type="plane", pos="3 0 -.11", size="20 5 .1",
        contype="4", conaffinity="4", friction=f"{cp.floor_friction} .001 .0001", rgba=".25 .28 .28 1",
        solref=f"{cp.floor_time_constant_s} 1", solimp=".95 .99 .001")
    add(world, "geom", name="cable_trough_base", type="box", pos="4.62 1.8 -.094", size="1.875 .235 .013",
        contype="4", conaffinity="4", rgba=".12 .14 .15 1", friction=f"{cp.floor_friction} .001 .0001",
        solref=f"{cp.floor_time_constant_s} 1", solimp=".95 .99 .001")
    for side in (-1,1):
        add(world, "geom", name="cable_trough_side_"+str(side), type="box", pos=f"4.62 {1.8+side*.245} -.063",
            size="1.875 .012 .045", contype="4", conaffinity="4", rgba=".2 .24 .25 1",
            friction=f"{cp.floor_friction} .001 .0001", solref=f"{cp.floor_time_constant_s} 1",
            solimp=".95 .99 .001")
    LAST_SPEC = dict(source_sha256=source["source_sha256"], parameters=asdict(cp), routes=records,
                     total_cable_mass_kg=sum(r["mass_kg"] for r in records),
                     total_links=sum(r["link_count"] for r in records),
                     provenance="source centerlines/anchors; inferred material, stress-free bends and passive contact parameters",
                     limits=["articulated rigid links, no axial stretch", "isotropic ball-joint bending/torsion proxy",
                             "uncalibrated density/EI/damping/friction", "soft endpoint constraints require error audit",
                             "self contact excludes two local neighboring links to avoid overlapping end-cap forces",
                             "no electrical continuity or hose pressure-radius coupling"])
    indent(tree); result = tostring(tree, encoding="unicode")
    if return_all_specs:
        return result, panto, doors
    return (result, panto) if return_spec else result


if __name__ == "__main__":
    xml = model()
    m = mujoco.MjModel.from_xml_string(xml)
    (ROOT/"prototype.xml").write_text(xml, encoding="utf-8")
    (ROOT/"prototype_configuration.json").write_text(json.dumps(LAST_SPEC, indent=2), encoding="utf-8")
    print(json.dumps(dict(nv=m.nv, bodies=m.nbody, links=LAST_SPEC["total_links"], mass_kg=LAST_SPEC["total_cable_mass_kg"])))
