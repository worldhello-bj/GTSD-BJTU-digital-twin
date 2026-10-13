"""Default-off independent candidate review, never a live solver replacement."""
import hashlib
from pathlib import Path
import re
from console_launcher import ConfigError, read_bounded, strict_json
import dependency_admission as admission

ALLOWED_ASSETS = {
    "outputs/render_comparison.png": "image/png",
    "outputs/render_candidate_front.png": "image/png",
    "outputs/render_candidate_rear.png": "image/png",
    "outputs/render_candidate_core.png": "image/png",
    "outputs/render_blind_bore_section.png": "image/png",
    "outputs/encoder_candidate_assembly.json": "application/json; charset=utf-8",
    "candidate_selection_contract.json": "application/json; charset=utf-8",
    "evidence/dimension_validation.json": "application/json; charset=utf-8",
    "evidence/exact_geometry_diff.json": "application/json; charset=utf-8",
}
BODY_ASSETS = {
    "outputs/body_CAD_cutaway.png": "image/png",
    "outputs/body_CAD_front.png": "image/png",
    "outputs/body_cad_candidate_meshes.json": "application/json; charset=utf-8",
    "evidence/body_geometry_validation.json": "application/json; charset=utf-8",
    "evidence/body_geometric_mass_properties.json": "application/json; charset=utf-8",
    "evidence/validation.json": "application/json; charset=utf-8",
}
PANTOGRAPH_ASSETS = {
    "outputs/cad_plan_candidate_overlay.png": "image/png",
    "outputs/render_plan_relief_oblique.png": "image/png",
    "outputs/pantograph_plan_candidate.json": "application/json; charset=utf-8",
    "outputs/side_top_alignment.json": "application/json; charset=utf-8",
    "candidate_selection_contract.json": "application/json; charset=utf-8",
}


class EncoderReview:
    allowed_assets = ALLOWED_ASSETS
    manifest_key = "encoder"
    def __init__(self, root):
        self.root = None if root is None else Path(root).resolve()

    def read_verified(self, relative):
        if self.root is None:
            raise ConfigError("candidate_disabled", "Independent encoder review source is not configured")
        if relative not in self.allowed_assets and relative != "DELIVERY.json":
            raise ConfigError("candidate_asset", "Candidate asset is not in the explicit read-only allowlist")
        manifest_bytes = read_bounded(self.root / "MANIFEST.sha256", 128 * 1024, "candidate manifest")
        admission.manifest(self.manifest_key, manifest_bytes)
        manifest = manifest_bytes.decode("utf-8")
        entries = {}
        for line in manifest.splitlines():
            match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
            if not match or match[2] in entries:
                raise ConfigError("candidate_manifest", "Invalid or duplicate candidate manifest entry")
            entries[match[2]] = match[1]
        body = read_bounded(self.root / relative, 4 * 1024 * 1024, "candidate " + relative)
        digest = hashlib.sha256(body).hexdigest()
        if entries.get(relative) != digest:
            raise ConfigError("candidate_changed", "Candidate bytes differ from their delivered manifest: " + relative)
        return body, digest

    def metadata(self):
        if self.root is None:
            return {"schema": "gtsd-independent-encoder-review-v1", "available": False,
                    "enabled_by_default": False, "current_solver_binding": False}
        delivery, _ = self.read_verified("DELIVERY.json")
        delivery = strict_json(delivery)
        body, contract_hash = self.read_verified("candidate_selection_contract.json")
        contract = strict_json(body)
        if (delivery.get("status") != "FROZEN_CANDIDATE_DELIVERABLE"
                or contract.get("schema") != "gtsd-candidate-selection-contract-v1"
                or contract.get("candidate_only") is not True
                or contract.get("enabled_by_default") is not False
                or contract.get("analog_angle_mapping_allowed") is not False
                or contract.get("hardware_transport_allowed") is not False):
            raise ConfigError("candidate_boundaries", "Candidate-only / default-off / no-hardware semantic boundaries changed")
        body, geometry_hash = self.read_verified("outputs/encoder_candidate_assembly.json")
        asset = strict_json(body)
        if asset.get("schema") != "gtsd-encoder-candidate-v1" or asset.get("candidate_only") is not True or asset.get("units") != "m":
            raise ConfigError("candidate_geometry", "Unsupported candidate geometry or units")
        return {"schema": "gtsd-independent-encoder-review-v1", "available": True,
                "enabled_by_default": False, "current_solver_binding": False,
                "physical_geometry_replaced": False, "contract_sha256": contract_hash,
                "candidate_geometry_sha256": geometry_hash, "mesh_count_for_asset_inventory_only": len(asset["meshes"]),
                "contract": contract, "review_url": "/candidate-review/encoder",
                "review_scope": "independent delivered renders/metadata, not current solver geometry or browser/GPU validation"}


def review_html():
    # Fixed HTML and allowlisted local URLs only; no data interpolated as markup.
    return '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>编码器独立候选审阅</title><link rel="stylesheet" href="/style.css"><link rel="stylesheet" href="/truth-evidence.css"></head><body><main class="candidate-review"><section class="panel"><h1>BRT38M厂家系列候选，独立审阅</h1><p class="truth-warning">本页默认不打开，不替换运行中75/79DOF的任何几何、绑定、质量、惯量或碰撞。实装型号、轴号与安装位未确认。3000RPM仅是模拟量速度款线索，禁止4–20mA默认角度换算。</p><p>以下是独立几何制作任务交付的离线渲染，不是当前控制台浏览器/GPU验收截图。旧16件的条件替换与21件杯罩保留仅在候选接入契约中描述，本页不执行替换。</p><figure><img src="/candidate-assets/encoder/outputs/render_comparison.png" alt="旧假定编码器与独立厂家系列候选同尺度渲染对照"><figcaption>左为冻结显示子集，右为未实装确证的厂家系列候选。</figcaption></figure><figure><img src="/candidate-assets/encoder/outputs/render_blind_bore_section.png" alt="候选盲孔剖面离线渲染"><figcaption>盲孔显示采用明确简化；不等于配合轴和实物间隙已验证。</figcaption></figure><p><a href="/candidate-assets/encoder/candidate_selection_contract.json">候选接入契约</a> · <a href="/candidate-assets/encoder/evidence/dimension_validation.json">独立尺寸校验</a> · <a href="/api/candidate-encoder">独立资产身份与边界</a></p><p><a href="/">返回控制台</a></p></section></main></body></html>'''.encode("utf-8")


class BodyReview(EncoderReview):
    def read_verified(self, relative):
        if self.root is None:
            raise ConfigError("candidate_disabled", "Independent CAD body review source is not configured")
        if relative not in BODY_ASSETS:
            raise ConfigError("candidate_asset", "CAD body asset is not allowlisted")
        manifest_bytes = read_bounded(self.root / "MANIFEST.json", 128 * 1024, "CAD body manifest")
        admission.manifest("body", manifest_bytes)
        manifest = strict_json(manifest_bytes)
        if manifest.get("schema") != "gtsd-cad-body-delivery-v1":
            raise ConfigError("candidate_manifest", "Unsupported CAD body manifest")
        row = manifest.get("files", {}).get(relative, {})
        body = read_bounded(self.root / relative, 4 * 1024 * 1024, "CAD body " + relative)
        digest = hashlib.sha256(body).hexdigest()
        if row.get("sha256") != digest or row.get("bytes") != len(body):
            raise ConfigError("candidate_changed", "CAD body bytes differ from delivered manifest: " + relative)
        return body, digest

    def metadata(self):
        if self.root is None:
            return {"schema": "gtsd-independent-body-review-v1", "available": False,
                    "enabled_by_default": False, "current_solver_binding": False}
        body, digest = self.read_verified("outputs/body_cad_candidate_meshes.json")
        asset = strict_json(body)
        if (asset.get("schema") != "gtsd-candidate-native-mesh-v1" or asset.get("units") != "m"
                or asset.get("assembly_candidate") is not True or asset.get("physics_binding") is not None):
            raise ConfigError("candidate_body_boundaries", "Independent CAD body asset must remain an unbound metre-unit design candidate")
        validation, validation_hash = self.read_verified("evidence/validation.json")
        validation = strict_json(validation)
        if validation.get("passed") is not True:
            raise ConfigError("candidate_body_validation", "CAD body geometry validation is not passed")
        return {"schema": "gtsd-independent-body-review-v1", "available": True,
                "enabled_by_default": False, "current_solver_binding": False, "physical_geometry_replaced": False,
                "candidate_geometry_sha256": digest, "validation_sha256": validation_hash,
                "mesh_count_for_asset_inventory_only": len(asset["meshes"]),
                "local_coordinates": "Z=0 at CAD body bottom; X from -0.65 to +0.65 m; Y from exact cross-section midline. Not the old world coordinates.",
                "source_identity": asset["source_identity"], "review_url": "/candidate-review/body",
                "review_scope": "frozen CAD design geometry and offline renders only; no automatic 75/79DOF mass, inertia, collision or binding substitution"}


def body_review_html():
    return '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>CAD车体独立设计审阅</title><link rel="stylesheet" href="/style.css"><link rel="stylesheet" href="/truth-evidence.css"></head><body><main class="candidate-review"><section class="panel"><h1>新CAD车体独立设计源审阅</h1><p class="truth-warning">本页独立查看26件CAD曲面/空心型材与底板，不替换当前75/79DOF画面、质量、惯量、碰撞或完整检查点。新CAD截面宽约1.120046933 m；设计矢量事实不等于实装验收。</p><p>资产局部Z=0在车体底面、X=±0.65 m、Y按精确截面中线。不能把这些局部坐标直接当原world坐标。CAD部分迁移可以通过另一个显式分支选择；请以主控制台当前variant和新geometry/绑定/模型身份判断。门铰轴/辅机安装仍未随新截面全部迁移。</p><figure><img src="/candidate-assets/body/outputs/body_CAD_front.png" alt="精确CAD截面重建的车体候选前视离线渲染"><figcaption>新CAD设计候选前视，非当前求解器截图。</figcaption></figure><figure><img src="/candidate-assets/body/outputs/body_CAD_cutaway.png" alt="CAD空心车体型材与底板离线渲染"><figcaption>空心型材与底板。材质密度及实物质量仍需独立真值。</figcaption></figure><p><a href="/api/candidate-body">候选几何身份与坐标约定</a> · <a href="/candidate-assets/body/evidence/validation.json">几何校验</a> · <a href="/">返回控制台</a></p></section></main></body></html>'''.encode("utf-8")


class PantographReview(EncoderReview):
    allowed_assets = PANTOGRAPH_ASSETS
    manifest_key = "pantograph"

    def metadata(self):
        if self.root is None:
            return {"schema": "gtsd-independent-pantograph-review-v1", "available": False,
                    "enabled_by_default": False, "current_solver_binding": False}
        body, contract_hash = self.read_verified("candidate_selection_contract.json")
        contract = strict_json(body)
        if (contract.get("schema") != "gtsd-local-candidate-selection-v1" or contract.get("mode") != "LOCAL_INSPECTION_ONLY"
                or contract.get("enabled_by_default") is not False or contract.get("allow_dynamics") is not False
                or contract.get("automatic_vehicle_placement") is not None or contract.get("hide_frozen_meshes") != []):
            raise ConfigError("pantograph_candidate_boundaries", "Local pantograph candidate must not be placed, hidden into the vehicle, or used for dynamics")
        body, geometry_hash = self.read_verified("outputs/pantograph_plan_candidate.json")
        asset = strict_json(body)
        if asset.get("schema") != "gtsd-pantograph-local-cad-relief-v1" or asset.get("units") != "m":
            raise ConfigError("pantograph_candidate_schema", "Unsupported local CAD relief")
        alignment, alignment_hash = self.read_verified("outputs/side_top_alignment.json")
        alignment = strict_json(alignment)
        return {"schema": "gtsd-independent-pantograph-review-v1", "available": True,
                "enabled_by_default": False, "current_solver_binding": False, "physical_geometry_replaced": False,
                "candidate_geometry_sha256": geometry_hash, "contract_sha256": contract_hash,
                "cross_view_registration_sha256": alignment_hash, "cross_view_registration_status": alignment.get("status"),
                "mesh_count_for_asset_inventory_only": len(asset["meshes"]), "local_coordinates": asset["coordinate_system"],
                "allow_dynamics": False, "automatic_vehicle_placement": None, "contract": contract,
                "review_url": "/candidate-review/pantograph",
                "review_scope": "local XY CAD outline relief only; display-only Z, no recovered spatial joint axes or closed chain"}


def pantograph_review_html():
    return '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>受电弓局部CAD候选审阅</title><link rel="stylesheet" href="/style.css"><link rel="stylesheet" href="/truth-evidence.css"></head><body><main class="candidate-review"><section class="panel"><h1>受电弓CAD俯视局部候选</h1><p class="truth-warning">LOCAL_INSPECTION_ONLY，默认关闭。XY轮廓可核，Z仅显示挤出；侧视/俯视实际配准未通过。不得把侧视图面高度直接当真实Z，不接入物理闭链，不隐藏原171网格，不自动按旧车宽缩放或安装到整车。</p><figure><img src="/candidate-assets/pantograph/outputs/cad_plan_candidate_overlay.png" alt="CAD俯视源轮廓与局部候选叠加对照"><figcaption>矢量源与局部候选核对，不是实物装配配准证明。</figcaption></figure><figure><img src="/candidate-assets/pantograph/outputs/render_plan_relief_oblique.png" alt="局部XY轮廓的显示用挤出离线渲染"><figcaption>Z厚度为明确显示假设，不能据此恢复真实空间运动机构。</figcaption></figure><p><a href="/candidate-assets/pantograph/outputs/side_top_alignment.json">侧视/俯视未通过配准记录</a> · <a href="/api/candidate-pantograph">局部候选身份与禁用边界</a> · <a href="/">返回控制台</a></p></section></main></body></html>'''.encode("utf-8")
