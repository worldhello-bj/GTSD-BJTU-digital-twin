"""Read-only geometry presentation for the Phase5 internal-system solvers.

The engineering disc/pad solids are rendered from actual MuJoCo geom_xpos and
geom_xmat, not animated commands or inferred source-mesh ownership. This adapter
does not alter the numerical factory, integration, commands, or checkpoint.
"""
import copy
import hashlib
from pathlib import Path

import mujoco


def native_brake_proxies(plant):
    result = []
    model, data = plant.model, plant.data
    for name in ("brake_disc_1_surface", "brake_disc_2_surface",
                 "brake_axle_1_minus_visual", "brake_axle_1_plus_visual",
                 "brake_axle_2_minus_visual", "brake_axle_2_plus_visual"):
        gid = model.geom(name).id
        is_disc = int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER)
        if not is_disc and int(model.geom_type[gid]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            raise ValueError("unexpected reviewed native-brake visual geom type")
        result.append({
            "kind": "cylinder" if is_disc else "box", "geom_name": name,
            "body_name": model.body(int(model.geom_bodyid[gid])).name,
            "position_world_m": data.geom_xpos[gid].tolist(),
            "rotation_world_matrix": data.geom_xmat[gid].reshape(3, 3).tolist(),
            "size_m": model.geom_size[gid, :2 if is_disc else 3].tolist(),
            "group": "bogie", "label": name + "（工程假设，实际求解姿态）",
            "source_mapping_confirmed": False,
        })
    return result


class _DisplayMixin:
    def snapshot(self):
        value = copy.deepcopy(super().snapshot())
        proxies = native_brake_proxies(self)
        value["solver_proxies"] = value.get("solver_proxies", []) + proxies
        for row in proxies:
            bid = self.model.body(row["body_name"]).id
            value["body_poses"][row["body_name"]] = {
                "position": self.data.xpos[bid].tolist(),
                "quaternion": self.data.xquat[bid].tolist(),
            }
        value["display_interface"] = {
            "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "geometry_source": "accepted MuJoCo geom world poses",
            "native_brake_physical_mount_confirmed": False,
        }
        value["title"] = "内部机电耦合研究控制台"
        value["subtitle"] = "真实盘片接触与变量容积缸 · 橙色内部机构为可替换工程假设"
        return value

    def checkpoint(self):
        return copy.deepcopy(super().checkpoint())


def display_plant(kind="brake", **kwargs):
    if kind == "brake":
        from brake_integrated.plant import IntegratedBrakePlant
        base = IntegratedBrakePlant
    elif kind == "coupled":
        from support_integrated.combined import CoupledInternalPlant
        base = CoupledInternalPlant
    else:
        raise ValueError("unknown internal research model")

    class DisplayPlant(_DisplayMixin, base):
        pass

    return DisplayPlant(**kwargs)
