"""Read-only presentation of named Phase6 bodies; no new solver or source mesh.

Auxiliary cylinders are declared equivalent-inertia research shapes positioned
from actual body xpos/xmat. They are not measured hardware or collision geoms.
"""
import copy
import hashlib
from pathlib import Path


def axis_frame(axis):
    """Right-handed display frame whose local Z follows the declared joint axis."""
    import numpy as np
    z = np.asarray(axis, dtype=float)
    if z.shape != (3,) or not np.isfinite(z).all() or np.linalg.norm(z) < 1e-12:
        raise ValueError("Invalid declared auxiliary display axis")
    z = z / np.linalg.norm(z)
    seed = np.array([1., 0., 0.]) if abs(z[0]) < .9 else np.array([0., 1., 0.])
    x = np.cross(seed, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


def auxiliary_body_proxies(plant):
    spec = plant.auxiliary_spec
    parameters = spec["parameters"]
    bodies = {row["body"]: row for row in spec["mass_ledger"]["added_bodies"]}
    result = []
    for name in ("compressor_rotor", "compressor_fan_rotor", "bay_fan_rotor", "compressor_piston"):
        bid = plant.model.body(name).id
        matrix = plant.data.xmat[bid].reshape(3, 3).copy()
        if name == "compressor_piston":
            radius = parameters["piston_radius_m"]
            length = parameters["piston_length_m"]
        else:
            radius = bodies[name]["equivalent_cylinder_radius_m"]
            length = parameters["compressor_rotor_length_m" if name == "compressor_rotor" else "fan_rotor_length_m"]
        # Existing UI cylinders use local Z. Follow the source's named axis,
        # including nondefault reviewed axes; never assume all rotors use X.
        matrix = matrix @ axis_frame(spec["coordinates"][name]["axis_local"])
        result.append({"kind": "cylinder", "geom_name": "phase6-display-only:" + name,
                       "body_name": name, "position_world_m": plant.data.xpos[bid].tolist(),
                       "rotation_world_matrix": matrix.tolist(), "size_m": [radius, length / 2],
                       "group": "body", "label": name + "（等效惯性显示代理，实算姿态，非实物/碰撞网格）",
                       "source_mapping_confirmed": False, "collision_geometry": False,
                       "proxy_dimension_source": "auxiliary_spec parameters / declared equivalent-cylinder inertia"})
    return result


def display_type(base_class):
    from phase5_display import _DisplayMixin

    class MountedDisplayPlant(_DisplayMixin, base_class):
        def snapshot(self):
            value = copy.deepcopy(super().snapshot())
            if not hasattr(self, "mount_runtime"):
                return value
            proxies = auxiliary_body_proxies(self)
            value["solver_proxies"] = value.get("solver_proxies", []) + proxies
            for row in proxies:
                bid = self.model.body(row["body_name"]).id
                value["body_poses"][row["body_name"]] = {
                    "position": self.data.xpos[bid].tolist(),
                    "quaternion": self.data.xquat[bid].tolist(),
                }
            value["title"] = "辅机安装反力耦合研究控制台"
            value["subtitle"] = "79DOF · 原生辅机转子/活塞 · 未标定安装与参数 · 真值证据独立显示"
            if value.get("variant") == "cad_body_mounted":
                value["title"] = "CAD截面车体与辅机耦合研究控制台"
                value["subtitle"] = "独立79DOF · 新合成几何/完整惯量/底板支承 · 安装迁移仍不完整"
                value["cad_migration_display_limits"] = [
                    "本冻结分支门铰轴到新截面侧边约197 mm；门尺寸/铰轴与安装尚未随新宽度迁移。",
                    "门、辅机安装、受电弓安装和轮轨滚动角色仍是候选；不得以新车体几何推出全装配一致。",
                    "原109字段矩阵的几何身份保持原hash；本分支不改expected_hash启用旧数组信号。",
                ]
            try:
                from aux_mount import bootstrap
                from dependency_admission import observer_file
                observer_file(Path(bootstrap.__file__).resolve().parent.parent)
                from aux_mount.observations import measure_auxiliary_world_state
                world = measure_auxiliary_world_state(self)
            except Exception as exc:
                # A read-only view failure must be visible without pretending
                # that the already accepted numerical plant itself failed.
                world = {"schema": "gtsd-phase6-readonly-world-observations-v1",
                         "endpoint_time_s": value.get("time_s"), "rows": {},
                         "available": False, "error": type(exc).__name__ + ": " + str(exc)}
            value["telemetry"]["auxiliary_world_observations"] = world
            value.setdefault("display_interface", {}).update({
                "phase6_display_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "auxiliary_proxy_count": len(proxies), "auxiliary_meshes_are_hardware": False,
                "original_compressor_casing_rotated_as_rotor": False,
            })
            return value

    return MountedDisplayPlant


def make_mounted_display(dt=.0002):
    from aux_mount.plant import MountedAuxiliaryPlant
    return display_type(MountedAuxiliaryPlant)(dt)


def make_cad_body_display(dt=.0002):
    from cad_body.plant import CadBodyPlant
    base = display_type(CadBodyPlant)

    class CadBodyDisplayPlant(base):
        @property
        def visual_resource_bytes(self):
            # The frozen numerical class exposes a read-only callable. The
            # existing controller expects immutable bytes, so adapt only here.
            value = CadBodyPlant.visual_resource_bytes(self)
            if type(value) is not bytes:
                raise ValueError("CAD visual resource must resolve to bytes")
            return value

    return CadBodyDisplayPlant(dt)
