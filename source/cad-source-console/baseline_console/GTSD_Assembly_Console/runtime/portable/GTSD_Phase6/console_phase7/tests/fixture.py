"""Tests resolve all dependencies through GTSD_CONSOLE_CONFIG or relative config."""
from pathlib import Path
import os,sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import phase7_server as P
CONFIG=P.load_config(os.environ.get('GTSD_CONSOLE_CONFIG'))
def saved_cad():
    return json.loads((Path(CONFIG['phase6_root'])/'evidence/cad_body_accepted_state.json').read_text())['plant']
def registry():
    cfg=CONFIG['base'].copy();cfg['variant']='cad-body-mounted-research';S=P.phase6_module();S.verify_cad_freeze(cfg['cad_body_root']);S.source_modules(cfg)
    from cad_body.geometry import visual_resource_bytes
    from lineage_registry import LineageRegistry,verify_stage
    prior=(Path(cfg['source_root'])/'web/geometry.json').read_bytes();current=visual_resource_bytes();source=json.loads(current)
    proof=json.loads(P.admitted(Path(cfg['cad_body_root'])/'evidence/retained_mesh_identity.json','cad_retained_map'))
    manifest=json.loads((Path(cfg['cad_body_root'])/'replacement_manifest.json').read_text());stage=verify_stage(prior,current,proof,manifest['replaced_source_mesh_names'],source['cad_body_overlay']['added_mesh_names'])
    snap=saved_cad();binding=json.loads((Path(CONFIG['phase6_root'])/'evidence/cad_body_accepted_binding.json').read_text());legacy=json.loads(P.admitted(Path(cfg['source_root'])/'internal_registry/registry.json','legacy_registry'))
    result=LineageRegistry(legacy,prior,current,binding,snap['model_identity_sha256'],snap['variant'],[stage]);result.augment_named(snap)
    return result,snap,(prior,current,proof,manifest['replaced_source_mesh_names'],source['cad_body_overlay']['added_mesh_names'])
