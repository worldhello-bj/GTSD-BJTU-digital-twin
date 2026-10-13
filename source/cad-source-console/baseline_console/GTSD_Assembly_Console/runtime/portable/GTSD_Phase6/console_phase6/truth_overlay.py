"""Read-only, independently refreshable evidence projection. No solver imports.

Evidence availability does not certify an as-built machine. Geometry mapping,
identity, connections and seven truth fields are deliberately separate.
"""
from __future__ import annotations
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path

from console_launcher import ConfigError, canonical, read_bounded, strict_json

FIELDS = ("dimensions", "position", "connection", "quantity", "material", "mass", "motion_relation")
FIELD_LABELS = dict(zip(FIELDS, ("尺寸", "位置", "连接", "数量", "材料", "质量", "运动关系")))
FILES = {
    "matrix": ("truth_matrix_109.json", "gtsd-109-node-field-truth-matrix-v1"),
    "proposals": ("readonly_difference_proposals.json", "gtsd-readonly-difference-proposals-v1"),
    "constraints": ("geometry_constraints.json", "gtsd-evidence-constraints-v1"),
    "findings": ("independent_findings.json", "gtsd-independent-source-findings-v1"),
}
BOUNDARY = "109登记节点和源网格数量都不是实物总数；不据此计算真实1:1覆盖率。外形映射、身份、连接及参数真值分别判断。"


def check_matrix(matrix):
    rows = matrix.get("rows")
    if type(rows) is not list or len(rows) != 109:
        raise ConfigError("truth_rows", "Evidence matrix must contain exactly 109 registered rows")
    seen = set()
    for row in rows:
        if type(row) is not dict or type(row.get("id")) is not str or row["id"] in seen:
            raise ConfigError("truth_ids", "Evidence rows require unique string IDs")
        seen.add(row["id"])
        if type(row.get("fields")) is not dict or set(row["fields"]) != set(FIELDS):
            raise ConfigError("truth_fields", f"{row['id']} must retain the seven separate truth fields")
        for key, field in row["fields"].items():
            if type(field) is not dict or type(field.get("status")) is not str:
                raise ConfigError("truth_status", f"{row['id']}.{key} needs an explicit evidence status")
        if type(row.get("identity_status")) is not str or type(row.get("model_mapping")) is not dict:
            raise ConfigError("truth_identity", "Identity and geometry mapping must be independently supplied")
        if type(row.get("as_built_1to1_verified")) is not bool:
            raise ConfigError("truth_verified", "Per-row as-built flag must be an explicit boolean")
    if not {"DTB-C0020", "DTB-C0073", "DTB-C0074", "DTB-C0013", "DTB-C0067"} <= seen:
        raise ConfigError("truth_guard_targets", "Required semantic guard targets are missing")
    return rows


def semantic_guards(component_id):
    guards = []
    if component_id == "DTB-C0020":
        guards.append({"id": "C0020_IDENTITY", "display_identity": "CANDIDATE_IDENTITY",
                       "text": "黑筒外形可以显示；气缸身份与升弓力路仍为候选。HAS_DIRECT_GEOMETRY不证明实装气缸。"})
    if component_id in ("DTB-C0073", "DTB-C0074"):
        guards.append({"id": "ENCODER_SPEED_CANDIDATE", "observed_label": "3000RPM",
                       "signal_semantics": "ANALOG_SPEED_VARIANT_CANDIDATE",
                       "angle_conversion_available": False, "hardware_decoder_available": False,
                       "calibrated_range_rpm": None, "physical_axle_binding": None,
                       "text": "照片铭牌可见3000RPM；模拟量速度款是候选。4–20mA用途、量程、轴号/方向尚未确认，禁止默认角度换算或生成硬件报文；不能复制为两只已确认编码器。"})
    if component_id in ("DTB-C0013", "DTB-C0067"):
        guards.append({"id": "RAIL_AND_RADIUS_SEMANTICS",
                       "text": "517.833 mm为轨轮廓中心距候选；478.333 mm仅为39.5 mm轨头假设下的内侧面间距算例。162.25/152.35 mm是半径，不是直径；均不是实测验收值。"})
    return guards


class EvidenceStore:
    """Fresh bounded data reads only. Failed refreshes never publish partial data."""
    def __init__(self, root, candidate_catalog=None):
        self.root = Path(root).resolve()
        self.candidate_catalog = None if candidate_catalog is None else Path(candidate_catalog).resolve()

    def read(self, solver_geometry_sha256=None):
        documents, hashes, signatures = {}, {}, {}
        for key, (filename, schema) in FILES.items():
            path = self.root / filename
            try:
                stat = path.stat()
                before = (stat.st_mtime_ns, stat.st_size)
            except OSError as exc:
                raise ConfigError("truth_source", f"Evidence source unavailable: {path}: {exc}") from exc
            body = read_bounded(path, 4 * 1024 * 1024, "evidence " + filename)
            document = strict_json(body)
            if type(document) is not dict or document.get("schema") != schema:
                raise ConfigError("truth_schema", f"Unsupported evidence schema in {filename}")
            documents[key] = document
            hashes[filename] = hashlib.sha256(body).hexdigest()
            signatures[path] = before
        for path, before in signatures.items():
            try:
                stat = path.stat()
            except OSError as exc:
                raise ConfigError("truth_refresh_race", f"Evidence changed during refresh: {path}") from exc
            if before != (stat.st_mtime_ns, stat.st_size):
                raise ConfigError("truth_refresh_race", "Evidence changed while reading; retry a new read after the producer has finished")
        matrix = documents["matrix"]
        candidates = []
        if self.candidate_catalog is not None:
            body = read_bounded(self.candidate_catalog, 128 * 1024, "independent design candidate catalog")
            catalog = strict_json(body)
            if catalog.get("schema") != "gtsd-independent-design-candidates-v1" or type(catalog.get("candidates")) is not list:
                raise ConfigError("candidate_catalog", "Invalid independent design candidate catalog")
            candidates = catalog["candidates"]
            for row in candidates:
                if (type(row) is not dict or row.get("current_solver_binding") is not False
                        or row.get("enabled_by_default") is not False
                        or row.get("changes_mass_inertia_collision") is not False):
                    raise ConfigError("candidate_auto_binding", "Evidence catalog cannot enable or bind candidate geometry to the running solver")
            hashes["independent_design_candidates"] = hashlib.sha256(body).hexdigest()
        rows = check_matrix(matrix)
        proposals = documents["proposals"].get("proposals")
        constraints = documents["constraints"].get("constraints")
        findings = documents["findings"].get("findings")
        if any(type(value) is not list for value in (proposals, constraints, findings)):
            raise ConfigError("truth_collections", "Evidence collections must be lists")
        counts = {key: dict(Counter(row["fields"][key]["status"] for row in rows)) for key in FIELDS}
        projected = []
        for original in rows:
            row = copy.deepcopy(original)
            row["semantic_guards"] = semantic_guards(row["id"])
            row["display_identity_status"] = "CANDIDATE_IDENTITY" if row["id"] == "DTB-C0020" else row["identity_status"]
            row["difference_proposals"] = [copy.deepcopy(p) for p in proposals if row["id"] in p.get("targets", [])]
            row["truth_layer_applies_to_physics"] = False
            projected.append(row)
        geometry_hash = matrix.get("source_geometry_sha256")
        return {
            "schema": "gtsd-readonly-truth-overlay-v1",
            "evidence_revision_sha256": hashlib.sha256(canonical(hashes)).hexdigest(),
            "source_file_sha256": hashes,
            "source_geometry_sha256": geometry_hash,
            "solver_source_geometry_sha256": solver_geometry_sha256,
            "geometry_identity_match": None if solver_geometry_sha256 is None else geometry_hash == solver_geometry_sha256,
            "counts": {"registry_nodes": len(rows), "truth_fields_per_node": len(FIELDS),
                       "field_entries": len(rows) * len(FIELDS), "field_status_counts": counts,
                       "as_built_coverage_percentage": None},
            "field_labels": FIELD_LABELS, "coverage_warning": BOUNDARY,
            "current_running_geometry_identity": solver_geometry_sha256,
            "independent_design_candidates": copy.deepcopy(candidates),
            "parameter_truth_warning": "几何、CAD设计约束、照片邻接及数值参数均不自动成为实装尺寸、材料、质量、连接或性能标定。",
            "rows": projected, "constraints": copy.deepcopy(constraints),
            "independent_findings": copy.deepcopy(findings),
            "difference_proposals": copy.deepcopy(proposals),
            "status_definitions": copy.deepcopy(matrix.get("status_definitions", {})),
            "sources": {"photos": copy.deepcopy(matrix.get("photo_sources", {})),
                        "procurement": copy.deepcopy(matrix.get("procurement_sources", {}))},
            "physical_model_changed": False, "checkpoint_identity_changed": False,
            "hardware_connected": False, "raw_frame_decoder_available": False,
            "refresh_mode": "independent read-only GET; no solver mutation, no heartbeat renewal",
        }


def auxiliary_observations(plant):
    """Named snapshot paths only: never reuse Phase5 numeric qpos/qvel offsets."""
    if type(plant) is not dict:
        plant = {}
    mounted = plant.get("variant") in ("mounted_auxiliary", "cad_body_mounted")
    telemetry = plant.get("telemetry") or {}
    aux = telemetry.get("auxiliary_mount") or {}
    native = aux.get("native") or {}
    world = telemetry.get("auxiliary_world_observations")
    if not mounted or type(world) is not dict or world.get("schema") != "gtsd-phase6-readonly-world-observations-v1":
        world = {"available": False, "rows": {}, "reason": "NAMED_WORLD_OBSERVATIONS_NOT_PROVIDED"}
    else:
        world = copy.deepcopy(world)
    observations = []

    def add(name, path, unit, *, accepted_interval=False):
        value, found = plant, mounted
        if found:
            for key in path:
                if type(value) is not dict or key not in value:
                    found = False
                    break
                value = value[key]
        valid = found and type(value) in (int, float) and math.isfinite(value)
        if accepted_interval and not plant.get("steps", 0):
            valid, status = False, "NO_ACCEPTED_INTERVAL"
        elif not mounted:
            status = "NOT_MOUNTED_AUXILIARY_VARIANT"
        else:
            status = "NATIVE_SOLVER_RESEARCH_NOT_HARDWARE" if valid else "MISSING_OR_INVALID_NAMED_PATH"
        observations.append({"id": name, "value": value if valid else None,
                             "unit": unit, "available": valid, "availability": status,
                             "source_path": path, "sample_time_s": plant.get("time_s"),
                             "coordinate_index_used": None})

    for name in ("compressor_rotor", "compressor_fan_rotor", "bay_fan_rotor"):
        for field, unit in (("angle_rad", "rad"), ("omega_rad_s", "rad/s")):
            add(name + "." + field, ["telemetry", "auxiliary_mount", "native", field, name], unit)
    for field, unit in (("piston_displacement_m", "m"), ("piston_velocity_m_s", "m/s")):
        add("compressor." + field, ["telemetry", "auxiliary_mount", "native", field], unit)
    # Wrenches are already expressed in named frames by the source solver. Keep
    # the full vectors intact instead of guessing an axis or a force sign.
    sensor_records = []
    for name, record in (aux.get("last_sensor") or {}).items():
        if not mounted or not plant.get("steps", 0) or type(record) is not dict:
            continue
        safe = copy.deepcopy(record)
        try:
            canonical(safe)
        except (ValueError, TypeError):
            continue
        sensor_records.append({"mount": name, "record": safe,
                               "quality": "NATIVE_SOLVER_VIRTUAL_SENSOR_NOT_HARDWARE",
                               "meaning": "internal mounting transfer diagnostics; not an added global energy input"})
    return {"schema": "gtsd-phase6-named-auxiliary-observations-v1", "sample_time_s": plant.get("time_s"),
            "mounted_variant": mounted, "integrated_dof": (plant.get("capabilities") or {}).get("integrated_dof"),
            "observations": observations, "mount_sensor_records": sensor_records,
            "world_endpoint_observations": world,
            "cad_migration_display_limits": copy.deepcopy(plant.get("cad_migration_display_limits", [])),
            "legacy_coordinate_policy": "Phase5 coordinate-length guards retained; unavailable indexed signals are not relabeled for 79DOF",
            "state_owner": aux.get("state_owner"), "hardware_connected": False,
            "signal_conversion_policy": "These native rotor coordinates are not C0073/C0074 hardware encoder conversions",
            "renews_run_lease": False}
