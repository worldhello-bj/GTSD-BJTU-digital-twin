#!/usr/bin/env python3
"""Standalone Phase6 console/evidence adapter; frozen Phase5 source stays read-only."""
from __future__ import annotations
import argparse
from functools import partial
import importlib
import hashlib
import os
from pathlib import Path
import sys
from urllib.parse import urlsplit

import console_launcher as L
import dependency_admission as admission
from truth_overlay import EvidenceStore, auxiliary_observations
from candidate_review import EncoderReview, BodyReview, PantographReview, ALLOWED_ASSETS, BODY_ASSETS, PANTOGRAPH_ASSETS, review_html, body_review_html, pantograph_review_html

ROOT = Path(__file__).resolve().parent
SCHEMA = "gtsd-phase6-console-config-v1"
VARIANT = "mounted-auxiliary-research"
CAD_VARIANT = "cad-body-mounted-research"
ALLOWED = ("internal-coupled-research", VARIANT, CAD_VARIANT)


def verify_auxiliary_freeze(root):
    root = Path(root)
    body = L.read_bounded(root / "PHASE6_SOURCE_FREEZE.json", L.MAX_CONFIG, "auxiliary source freeze")
    admission.manifest("auxiliary", body)
    value = L.strict_json(body)
    expected = {"aux_mount/" + name for name in ("__init__.py", "bootstrap.py", "model.py", "runtime.py", "pneumatics.py", "plant.py")}
    if value.get("schema") != "gtsd-phase6-source-freeze-v1" or set(value.get("physics_source_sha256", {})) != expected:
        raise L.ConfigError("auxiliary_freeze", "Unrecognized auxiliary frozen source manifest")
    for relative, expected_hash in value["physics_source_sha256"].items():
        actual = hashlib.sha256(L.read_bounded(root / relative, 1024 * 1024, relative)).hexdigest()
        if actual != expected_hash:
            raise L.ConfigError("auxiliary_source_changed", f"Frozen auxiliary source changed: {relative}; wait for reviewed refreeze")
    return value


def verify_cad_freeze(root):
    root = Path(root)
    body = L.read_bounded(root / "SOURCE_FREEZE.json", L.MAX_CONFIG, "CAD body source freeze")
    admission.manifest("cad_body", body)
    value = L.strict_json(body)
    expected = {"cad_body/" + name for name in ("__init__.py", "bootstrap.py", "geometry.py", "model.py", "plant.py")} | {"READONLY_DEPENDENCIES.json", "replacement_manifest.json"}
    if value.get("schema") != "gtsd-cad-body-source-freeze-v1" or set(value.get("physics_source_sha256", {})) != expected:
        raise L.ConfigError("cad_freeze", "Unsupported CAD body frozen source manifest")
    for relative, expected_hash in value["physics_source_sha256"].items():
        if hashlib.sha256(L.read_bounded(root / relative, 1024 * 1024, relative)).hexdigest() != expected_hash:
            raise L.ConfigError("cad_source_changed", "Frozen CAD body source changed: " + relative)
    return value


def read_evidence(store, encoder, source_hash=None, body_review=None, solver_context=None, pantograph_review=None):
    value = store.read(source_hash)
    try:
        metadata = encoder.metadata()
    except (L.ConfigError, OSError) as exc:
        metadata = {"available": False, "error": str(exc), "enabled_by_default": False, "current_solver_binding": False}
    value["encoder_independent_review"] = metadata
    if metadata.get("available"):
        value["source_file_sha256"].update(encoder_candidate_geometry=metadata["candidate_geometry_sha256"], encoder_candidate_contract=metadata["contract_sha256"])
        for candidate in value["independent_design_candidates"]:
            if candidate.get("id") == "ENCODER-BRT38M-DESIGN-CANDIDATE":
                candidate["geometry_sha256"] = metadata["candidate_geometry_sha256"]
                candidate["independent_review_url"] = metadata["review_url"]
    try:
        body_metadata = (body_review or BodyReview(None)).metadata()
    except (L.ConfigError, OSError) as exc:
        body_metadata = {"available": False, "error": str(exc), "enabled_by_default": False, "current_solver_binding": False}
    value["body_independent_review"] = body_metadata
    if body_metadata.get("available"):
        value["source_file_sha256"].update(body_candidate_geometry=body_metadata["candidate_geometry_sha256"], body_candidate_validation=body_metadata["validation_sha256"])
        for candidate in value["independent_design_candidates"]:
            if candidate.get("id") == "BODY-CAD-CROSS-SECTION-CANDIDATE":
                candidate.update(status="FROZEN_CAD_DESIGN_CANDIDATE_NOT_BOUND_CURRENT_SOLVER", geometry_sha256=body_metadata["candidate_geometry_sha256"],
                                 independent_review_url=body_metadata["review_url"], local_coordinates=body_metadata["local_coordinates"])
    try:
        panto_metadata = (pantograph_review or PantographReview(None)).metadata()
    except (L.ConfigError, OSError) as exc:
        panto_metadata = {"available": False, "error": str(exc), "enabled_by_default": False, "current_solver_binding": False}
    value["pantograph_independent_review"] = panto_metadata
    if panto_metadata.get("available"):
        value["source_file_sha256"].update(pantograph_candidate_geometry=panto_metadata["candidate_geometry_sha256"], pantograph_candidate_contract=panto_metadata["contract_sha256"], pantograph_registration=panto_metadata["cross_view_registration_sha256"])
        for candidate in value["independent_design_candidates"]:
            if candidate.get("id") == "PANTOGRAPH-LATERAL-CAD-CANDIDATE":
                candidate.update(status="FROZEN_LOCAL_XY_CAD_RELIEF_CROSS_VIEW_NOT_REGISTERED", geometry_sha256=panto_metadata["candidate_geometry_sha256"],
                                 independent_review_url=panto_metadata["review_url"], local_coordinates=panto_metadata["local_coordinates"],
                                 cross_view_registration_status=panto_metadata["cross_view_registration_status"], allow_dynamics=False)
    if type(solver_context) is dict:
        value["current_solver_context"] = {"variant": solver_context.get("variant"), "model_identity_sha256": solver_context.get("model_identity_sha256"),
                                           "cad_body": (solver_context.get("telemetry") or {}).get("cad_body"),
                                           "cad_migration_display_limits": solver_context.get("cad_migration_display_limits", [])}
        if solver_context.get("variant") == "cad_body_mounted":
            value["current_solver_context"]["cad_binding_status"] = "EXPLICIT_PARTIAL_CAD_MIGRATION_NOT_FULL_AS_BUILT"
            for candidate in value["independent_design_candidates"]:
                if candidate.get("id") == "BODY-CAD-CROSS-SECTION-CANDIDATE":
                    candidate.update(status="FROZEN_CAD_SOURCE_USED_BY_EXPLICIT_PARTIAL_MIGRATION", current_solver_derived_usage=True,
                                     current_composed_geometry_sha256=source_hash)
    value["evidence_revision_sha256"] = hashlib.sha256(L.canonical(value["source_file_sha256"])).hexdigest()
    return value


def load_config(path):
    path = Path(path).expanduser().resolve()
    raw = L.strict_json(L.read_bounded(path, L.MAX_CONFIG, "Phase6 configuration"))
    expected = {"schema", "source_root", "auxiliary_root", "evidence_root", "variant", "port", "resume"}
    if type(raw) is not dict or not expected <= set(raw) or set(raw) - expected - {"encoder_candidate_root", "body_candidate_root", "cad_body_root", "pantograph_candidate_root"} or raw.get("schema") != SCHEMA:
        raise L.ConfigError("phase6_config", "Expected exact Phase6 schema and fields: " + ", ".join(sorted(expected)))
    if raw["variant"] not in ALLOWED:
        raise L.ConfigError("phase6_variant", "Unknown explicit research variant; choose " + ", ".join(ALLOWED))
    if type(raw["port"]) is not int or not 0 <= raw["port"] <= 65535:
        raise L.ConfigError("invalid_port", "port must be an integer from 0 to 65535")
    config = dict(raw, config_path=str(path))
    config["encoder_candidate_root"] = None if raw.get("encoder_candidate_root") is None else str(L.resolve_path(raw["encoder_candidate_root"], path.parent, "encoder_candidate_root"))
    config["body_candidate_root"] = None if raw.get("body_candidate_root") is None else str(L.resolve_path(raw["body_candidate_root"], path.parent, "body_candidate_root"))
    config["cad_body_root"] = None if raw.get("cad_body_root") is None else str(L.resolve_path(raw["cad_body_root"], path.parent, "cad_body_root"))
    config["pantograph_candidate_root"] = None if raw.get("pantograph_candidate_root") is None else str(L.resolve_path(raw["pantograph_candidate_root"], path.parent, "pantograph_candidate_root"))
    if config["variant"] == CAD_VARIANT and (config["cad_body_root"] is None or not Path(config["cad_body_root"]).is_dir()):
        raise L.ConfigError("cad_body_path", "cad-body-mounted-research requires an explicit existing cad_body_root")
    for name in ("source_root", "auxiliary_root", "evidence_root"):
        config[name] = str(L.resolve_path(raw[name], path.parent, name))
        if not Path(config[name]).is_dir() and (name != "auxiliary_root" or config["variant"] in (VARIANT, CAD_VARIANT)):
            raise L.ConfigError("phase6_path", f"{name} directory does not exist: {config[name]}")
    if config["resume"] is not None:
        config["resume"] = str(L.resolve_path(config["resume"], path.parent, "resume"))
        L.checkpoint_header(Path(config["resume"]))
    return config


def doctor(config, check_bind=True):
    source_config = {key: config[key] for key in ("source_root", "port", "resume", "config_path")}
    source_config.update(schema=L.SCHEMA, variant="internal-coupled-research")
    result = L.doctor(source_config, check_bind=check_bind)
    for name in admission.phase5_hash_errors(result["source_file_sha256"]):
        result["errors"].append(L.issue("unreviewed_phase5_hash", "Required Phase5 source hash differs from reviewed release: " + name))
    result["schema"] = "gtsd-phase6-console-preflight-v1"
    result["config"] = config
    result["expected_dof"] = 79 if config["variant"] in (VARIANT, CAD_VARIANT) else 75
    try:
        evidence = read_evidence(EvidenceStore(config["evidence_root"], ROOT / "design_candidates.json"), EncoderReview(config.get("encoder_candidate_root")), body_review=BodyReview(config.get("body_candidate_root")), pantograph_review=PantographReview(config.get("pantograph_candidate_root")))
        result["evidence_revision_sha256"] = evidence["evidence_revision_sha256"]
        result["truth_fields"] = evidence["counts"]["field_entries"]
        result["encoder_independent_review_available"] = evidence["encoder_independent_review"].get("available", False)
        result["body_independent_review_available"] = evidence["body_independent_review"].get("available", False)
        result["pantograph_independent_review_available"] = evidence["pantograph_independent_review"].get("available", False)
        if evidence["encoder_independent_review"].get("error"):
            result["warnings"].append(L.issue("candidate_review_unavailable", evidence["encoder_independent_review"]["error"]))
    except L.ConfigError as exc:
        result["errors"].append(L.issue(exc.code, str(exc)))
    if config["variant"] in (VARIANT, CAD_VARIANT):
        for name in ("__init__.py", "bootstrap.py", "model.py", "runtime.py", "pneumatics.py", "plant.py"):
            path = Path(config["auxiliary_root"]) / "aux_mount" / name
            if not path.is_file():
                result["errors"].append(L.issue("auxiliary_source", f"Missing Phase6 source: {path}"))
        try:
            result["auxiliary_source_freeze"] = verify_auxiliary_freeze(config["auxiliary_root"])
        except L.ConfigError as exc:
            result["errors"].append(L.issue(exc.code, str(exc)))
        result["warnings"].append(L.issue("research_only", "79DOF is an explicit research variant; source freeze and numerical acceptance must be checked independently. No 1:1 or hardware certification."))
    if config["variant"] == CAD_VARIANT:
        try:
            result["cad_body_source_freeze"] = verify_cad_freeze(config["cad_body_root"])
        except L.ConfigError as exc:
            result["errors"].append(L.issue(exc.code, str(exc)))
    result["ok"] = not result["errors"]
    return result


def source_modules(config):
    sys.dont_write_bytecode = True
    root, auxiliary = Path(config["source_root"]), Path(config["auxiliary_root"])
    if config["variant"] in (VARIANT, CAD_VARIANT):
        selected = os.environ.get("GTSD_PHASE5_ROOT")
        if selected and Path(selected).expanduser().resolve() != root:
            raise L.ConfigError("baseline_environment_conflict", "GTSD_PHASE5_ROOT differs from configured source_root; choose one explicit baseline before starting")
        os.environ["GTSD_PHASE5_ROOT"] = str(root)
    if config["variant"] == CAD_VARIANT:
        dependencies = {"GTSD_CAD_PHASE5_ROOT": config["source_root"], "GTSD_CAD_PHASE6_ROOT": config["auxiliary_root"], "GTSD_CAD_CAD_ROOT": config.get("body_candidate_root")}
        for key, selected in dependencies.items():
            if selected is None:
                raise L.ConfigError("cad_dependency_path", key + " requires an explicit configured root")
            if os.environ.get(key) and Path(os.environ[key]).expanduser().resolve() != Path(selected):
                raise L.ConfigError("cad_environment_conflict", key + " differs from explicit configuration")
            os.environ[key] = selected
        if config["cad_body_root"] not in sys.path:
            sys.path.insert(0, config["cad_body_root"])
    for path in (root, root / "console", auxiliary):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    # The auxiliary source owns its supported portability contract. Do not
    # monkeypatch its module constants or silently change its identity source.
    module = importlib.import_module("server")
    if module.ROOT.parent.resolve() != root:
        raise L.ConfigError("baseline_module_conflict", "An already loaded console server belongs to a different source directory")
    return module


def build_handler(base, store, encoder=None, body_review=None, pantograph_review=None):
    encoder = EncoderReview(None) if encoder is None else encoder
    body_review = BodyReview(None) if body_review is None else body_review
    pantograph_review = PantographReview(None) if pantograph_review is None else pantograph_review
    class EvidenceHandler(base.Handler):
        def do_GET(self):
            path = urlsplit(self.path).path
            candidate_asset = path.removeprefix("/candidate-assets/encoder/") if path.startswith("/candidate-assets/encoder/") else None
            body_asset = path.removeprefix("/candidate-assets/body/") if path.startswith("/candidate-assets/body/") else None
            panto_asset = path.removeprefix("/candidate-assets/pantograph/") if path.startswith("/candidate-assets/pantograph/") else None
            if path not in ("/", "/api/truth-evidence", "/api/phase6-observations", "/api/candidate-encoder", "/candidate-review/encoder", "/api/candidate-body", "/candidate-review/body", "/api/candidate-pantograph", "/candidate-review/pantograph",
                            "/truth-evidence.js", "/truth-evidence-client.js", "/truth-evidence.css") and candidate_asset is None and body_asset is None and panto_asset is None:
                return super().do_GET()
            if not self._authorized():
                return
            try:
                if panto_asset is not None:
                    if panto_asset not in PANTOGRAPH_ASSETS:
                        self._send(404, {"error": "local pantograph asset not found"})
                        return
                    pantograph_review.metadata()
                    body, digest = pantograph_review.read_verified(panto_asset)
                    self._send(200, body, PANTOGRAPH_ASSETS[panto_asset])
                elif path == "/api/candidate-pantograph":
                    self._send(200, pantograph_review.metadata())
                elif path == "/candidate-review/pantograph":
                    if not pantograph_review.metadata()["available"]:
                        raise L.ConfigError("candidate_disabled", "Independent local pantograph source not configured")
                    self._send(200, pantograph_review_html(), "text/html; charset=utf-8")
                elif body_asset is not None:
                    if body_asset not in BODY_ASSETS:
                        self._send(404, {"error": "CAD body asset not found"})
                        return
                    body_review.metadata()
                    body, digest = body_review.read_verified(body_asset)
                    self._send(200, body, BODY_ASSETS[body_asset])
                elif path == "/api/candidate-body":
                    self._send(200, body_review.metadata())
                elif path == "/candidate-review/body":
                    if not body_review.metadata()["available"]:
                        raise L.ConfigError("candidate_disabled", "Independent CAD body source not configured")
                    self._send(200, body_review_html(), "text/html; charset=utf-8")
                elif candidate_asset is not None:
                    if candidate_asset not in ALLOWED_ASSETS:
                        self._send(404, {"error": "candidate asset not found"})
                        return
                    encoder.metadata()
                    body, digest = encoder.read_verified(candidate_asset)
                    self._send(200, body, ALLOWED_ASSETS[candidate_asset])
                elif path == "/api/candidate-encoder":
                    self._send(200, encoder.metadata())
                elif path == "/candidate-review/encoder":
                    metadata = encoder.metadata()
                    if not metadata["available"]:
                        raise L.ConfigError("candidate_disabled", "Independent encoder review source is not configured")
                    self._send(200, review_html(), "text/html; charset=utf-8")
                elif path == "/api/truth-evidence":
                    state = self.server.controller.snapshot(heartbeat=False)
                    source_hash = (state.get("plant") or {}).get("source_geometry_sha256")
                    self._send(200, read_evidence(store, encoder, source_hash, body_review, state.get("plant"), pantograph_review))
                elif path == "/api/phase6-observations":
                    state = self.server.controller.snapshot(heartbeat=False)
                    value = auxiliary_observations(state.get("plant"))
                    value.update(phase=state.get("phase"), session_id=state.get("session_id"),
                                 snapshot_age_s=state.get("snapshot_age_s"))
                    self._send(200, value)
                elif path == "/":
                    html = (base.WEB / "index.html").read_text(encoding="utf-8")
                    marker = '<section id="internal-systems-panel"'
                    if html.count(marker) != 1 or html.count("</head>") != 1 or html.count("</body>") != 1:
                        raise L.ConfigError("source_html_contract", "Frozen source HTML insertion anchors changed; review the overlay before serving")
                    html = html.replace("</head>", '<link rel="stylesheet" href="/truth-evidence.css"></head>')
                    html = html.replace(marker, '<section id="truth-evidence-panel" class="panel" aria-label="实物证据与模型差异"></section><section id="phase6-observations-panel" class="panel" hidden></section>' + marker)
                    html = html.replace("</body>", '<script src="/truth-evidence.js"></script><script src="/truth-evidence-client.js"></script></body>')
                    self._send(200, html.encode("utf-8"), "text/html; charset=utf-8")
                else:
                    asset = ROOT / "phase6_web" / path.lstrip("/")
                    mime = "text/css; charset=utf-8" if path.endswith(".css") else "text/javascript; charset=utf-8"
                    self._send(200, asset.read_bytes(), mime)
            except (L.ConfigError, OSError) as exc:
                self._send(409, {"error": str(exc), "code": getattr(exc, "code", "evidence_io"),
                                 "solver_changed": False, "evidence_is_not_current": True})
    return EvidenceHandler


def make_server(config):
    base = source_modules(config)
    if config["variant"] in (VARIANT, CAD_VARIANT):
        bootstrap = importlib.import_module("aux_mount.bootstrap")
        if bootstrap.BASELINE.resolve() != Path(config["source_root"]):
            raise L.ConfigError("baseline_identity", "Auxiliary numerical baseline differs from the configured display/controller source")
        from mounted_display import make_mounted_display, make_cad_body_display
        factory = make_cad_body_display if config["variant"] == CAD_VARIANT else make_mounted_display
    else:
        from phase5_display import display_plant
        factory = partial(display_plant, kind="coupled")
    resume = None if config["resume"] is None else L.strict_json(L.read_bounded(Path(config["resume"]), L.MAX_CHECKPOINT, "checkpoint"))
    controller = base.Controller(factory=factory, resume=resume)
    server = base.ConsoleServer(config["port"], controller)
    server.RequestHandlerClass = build_handler(base, EvidenceStore(config["evidence_root"], ROOT / "design_candidates.json"), EncoderReview(config.get("encoder_candidate_root")), BodyReview(config.get("body_candidate_root")), PantographReview(config.get("pantograph_candidate_root")))
    return server, controller


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("doctor", "serve", "evidence"))
    parser.add_argument("--config", type=Path, default=ROOT / "phase6_config.json")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.action == "evidence":
            L.emit(read_evidence(EvidenceStore(config["evidence_root"], ROOT / "design_candidates.json"), EncoderReview(config.get("encoder_candidate_root")), body_review=BodyReview(config.get("body_candidate_root")), pantograph_review=PantographReview(config.get("pantograph_candidate_root"))))
            return 0
        result = doctor(config, check_bind=args.action in ("doctor", "serve"))
        if args.action == "doctor" or not result["ok"]:
            L.emit(result)
            return 0 if result["ok"] else 2
        server, controller = make_server(config)
        controller.start()
        print(f"GTSD Phase6 {config['variant']} software-only console: http://127.0.0.1:{server.server_port}", flush=True)
        print("Evidence is independent, read-only and not a 1:1 certificate. Fresh plant paused; no hardware or external network.", flush=True)
        try:
            server.serve_forever(poll_interval=.1)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
            controller.close()
        return 0
    except (L.ConfigError, OSError, ImportError) as exc:
        L.emit({"ok": False, "error": {"code": getattr(exc, "code", "phase6_startup"), "message": str(exc)}})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
