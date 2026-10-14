"""Cross-check the frozen numerical, HTTP, DOM and native-render identities."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(path):
    data = Path(path).read_bytes()
    return json.loads(data), hashlib.sha256(data).hexdigest()


def verify(protocol_path, http_path, ui_path, render_path, output):
    records = {}
    hashes = {}
    for key, path in [('protocol', protocol_path), ('http', http_path),
                      ('ui', ui_path), ('render', render_path)]:
        records[key], hashes[key] = read(path)
    protocol, http, ui, render = (records[k] for k in ('protocol', 'http', 'ui', 'render'))
    identity = protocol['coarse']['identity_sha256']
    if not protocol.get('passed') or not protocol['checks'] or not all(protocol['checks'].values()):
        raise ValueError('The complete numerical protocol has not passed')
    if not http.get('passed') or not all(http['checks'].values()):
        raise ValueError('The real HTTP checks have not passed')
    if not http.get('complete_demo') or not http['checks'].get('complete_http_demo_and_terminal_checkpoint_restore'):
        raise ValueError('The complete HTTP demonstration has not been exercised')
    if ui.get('result') != 'pass' or not ui['checks'] or not all(ui['checks'].values()):
        raise ValueError('The captured native data/DOM checks have not passed')
    if any(value != identity for value in (http['identity_sha256'], ui['identity_sha256'],
                                           render['runtime_identity_sha256'])):
        raise ValueError('Numerical, HTTP, UI and render identities differ')
    if render['assembly_identity_sha256'] != protocol['coarse']['assembly_identity_sha256']:
        raise ValueError('Rendered assembly identity differs')
    if not render['native_reopened'] or not render['all_indices_and_membership_verified']:
        raise ValueError('The native rendered model has not been reopened and checked')
    if render['displacement_scale'] != 1. or render['maximum_world_vertex_error_mm'] > render['tolerance_mm']:
        raise ValueError('Rendered geometry differs from the recorded native pose')
    if ui['http_evidence']['report_sha256'] != hashes['http']:
        raise ValueError('DOM evidence uses a different HTTP bundle')
    for relative, expected in ui['web_sha256'].items():
        path = ROOT / relative
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('UI changed after its DOM test: ' + relative)
    now = datetime.now(timezone.utc)
    report = {'schema': 'gtsd-operational-release-acceptance-v1', 'passed': True,
              'identity_sha256': identity, 'assembly_identity_sha256': render['assembly_identity_sha256'],
              'numerical_protocol_passed': True, 'complete_http_demonstration_passed': True,
              'captured_native_ui_dom_passed': True, 'native_blender_pose_verified': True,
              'webgl_browser_visual_tested': False, 'browser_download_completion_tested': False,
              'evidence_sha256': hashes, 'finished_utc': now.isoformat(),
              'finished_beijing': now.astimezone(timezone(timedelta(hours=8))).isoformat()}
    Path(output).write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('protocol', 'http', 'ui', 'render', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.protocol, args.http, args.ui, args.render, args.output), indent=2))
