"""Real loopback commands, native continuation and fixed UI resource acceptance."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import http.client
import json
from pathlib import Path
import threading
import time
import uuid

from server import make_server


def verify(output, complete_demo=False, expected_checkpoint=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    server, session = make_server()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    checks = {}
    requests = []
    started = time.perf_counter()
    full_demo = None

    def get(path, payload=None, expected=200):
        request_started = time.perf_counter()
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=60)
        body = json.dumps(payload) if payload is not None else None
        connection.request('POST' if payload is not None else 'GET', path, body=body,
                           headers={'Content-Type': 'application/json'})
        response = connection.getresponse()
        raw = response.read()
        headers = dict(response.getheaders())
        requests.append({'method': 'POST' if payload is not None else 'GET',
                         'path': path.split('?')[0], 'status': response.status,
                         'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(),
                         'elapsed_wall_s': time.perf_counter() - request_started})
        connection.close()
        assert response.status == expected, (path, response.status, raw[:500])
        return json.loads(raw) if 'application/json' in headers.get('Content-Type', '') else raw, headers

    def api(name, payload=None, expected=200):
        return get('/api/operational-rig/' + name, payload, expected)[0]

    def identity(state):
        return {k: state[k] for k in ('server_generation_id', 'session_id')} | {
            'expected_revision': state['revision'], 'request_id': uuid.uuid4().hex}

    def command(action, **fields):
        return api('command', identity(api('status')) | {'action': action} | fields)

    def paused():
        deadline = time.perf_counter() + 30
        while time.perf_counter() < deadline:
            result = api('status')
            if result['phase'] != 'running': return result
            time.sleep(.01)
        raise AssertionError('Pause did not reach an integration boundary')

    try:
        empty = api('status')
        assert not empty['initialized'] and empty['steps'] == 0
        for path in ('/operational-rig', '/operational-rig/app.js', '/operational-rig/core.js',
                     '/operational-rig/style.css', '/operational-rig/model.js',
                     '/operational-rig/parameters', '/operational-rig/parameters.js'):
            data, headers = get(path)
            assert data and headers['Cache-Control'] == 'no-store'
        meta = api('metadata'); geometry = api('geometry')
        assert len(meta['parameter_sources']) >= len(meta['defaults'])
        assert not api('status')['initialized']
        checks['read_only_startup_and_seven_ui_resources'] = True

        state = api('initialize', identity(empty) | {'mode': 'operational_assembly', 'parameters': {}})
        assert state['initialized'] and state['steps'] == 0
        meta = api('metadata'); geometry = api('geometry')
        assert geometry['identity_sha256'] == state['identity_sha256']
        names = {m['name']: m for m in geometry['meshes']}
        assert len(names) == len(geometry['meshes'])
        for binding in meta['contract']['bindings']:
            assert names[binding['mesh_name']]['geometry_sha256'] == binding['source_geometry_sha256']
            assert binding['body_name'] in state['body_poses']
        checks['native_initialization_and_every_source_binding'] = True

        step_payload = identity(state) | {'action': 'step', 'steps': 10, 'loads': {}}
        first = api('command', step_payload)
        assert first['steps'] == 10 and first['phase'] == 'paused'
        assert api('command', step_payload) == first
        assert api('status')['steps'] == 10
        checks['repeated_http_step_advances_once'] = True
        checkpoint = command('checkpoint')['created_checkpoint_id']
        exported, headers = get('/api/operational-rig/checkpoint-export?id=' + checkpoint + '&session_id=' + first['session_id'])
        assert 'attachment;' in headers['Content-Disposition']
        assert exported['checkpoint']['state'] == session._plant.checkpoint()
        (output / 'exported_checkpoint.json').write_text(json.dumps(exported, separators=(',', ':')))
        checks['real_checkpoint_download_response'] = True

        loads = {'balise_02': [0., 0., 1.]}
        command('step', steps=50, loads=loads)
        terminal = session._plant.checkpoint()
        command('restore', checkpoint_id=checkpoint)
        command('step', steps=50, loads=loads)
        assert session._plant.checkpoint() == terminal
        checks['http_restore_native_complete_state_and_continuation_exact'] = True

        stale = identity(first) | {'action': 'step', 'steps': 1, 'loads': {}}
        before = session._plant.checkpoint()
        api('command', stale, expected=409)
        assert session._plant.checkpoint() == before
        checks['stale_revision_changes_no_native_state'] = True

        command('demo')
        command('pause')
        stopped = paused()
        assert stopped['phase'] == 'paused' and not stopped['demo_active']
        paused_native = session._plant.checkpoint()
        time.sleep(.02)
        assert paused_native == session._plant.checkpoint()
        checks['demo_pause_at_native_boundary'] = True
        command('reset')
        assert api('status')['steps'] == 0
        command('run', steps=10, loads={})
        assert paused()['steps'] == 10
        checks['reset_and_bounded_run'] = True

        if complete_demo:
            command('reset')
            command('demo')
            deadline = time.perf_counter() + 900
            samples = []
            while time.perf_counter() < deadline:
                state = api('status')
                samples.append({'steps': state['steps'], 'time_s': state['time_s'],
                                'phase': state['phase'], 'revision': state['revision']})
                if state['phase'] != 'running':
                    break
                time.sleep(.25)
            assert state['phase'] == 'paused' and not state['demo_active']
            duration = session._plant.contract['demo']['duration_s']
            dt = session._plant.contract['parameters']['time_step_s']
            assert abs(state['time_s'] - duration) <= dt
            assert state['steps'] > 0 and not any(w.number for w in session._plant.data.warning)
            final_native = session._plant.checkpoint()
            exact = None
            if expected_checkpoint is not None:
                exact = final_native == json.loads(Path(expected_checkpoint).read_text())
                assert exact, 'HTTP complete demo differs from the qualified native checkpoint'
            checkpoint = command('checkpoint')['created_checkpoint_id']
            payload, _ = get('/api/operational-rig/checkpoint-export?id=' + checkpoint
                             + '&session_id=' + state['session_id'])
            assert payload['checkpoint']['state'] == final_native
            command('step', steps=1, loads={})
            command('restore', checkpoint_id=checkpoint)
            assert session._plant.checkpoint() == final_native
            full_demo = {'duration_s': state['time_s'], 'steps': state['steps'],
                         'complete_checkpoint_matches_producer': exact,
                         'native_checkpoint_sha256': hashlib.sha256(json.dumps(final_native,
                             sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                         'status_samples': samples}
            checks['complete_http_demo_and_terminal_checkpoint_restore'] = True

        state = api('status'); meta = api('metadata'); geometry = api('geometry')
        for name, data in [('status', state), ('metadata', meta), ('geometry', geometry)]:
            (output / (name + '.json')).write_text(json.dumps(data, separators=(',', ':')))
        checks['actual_http_bundle_saved'] = True
        now = datetime.now(timezone.utc)
        report = {'schema': 'gtsd-operational-native-http-acceptance-v1', 'checks': checks,
                  'passed': all(checks.values()), 'identity_sha256': state['identity_sha256'],
                  'mesh_bindings': len(meta['contract']['bindings']), 'requests': requests,
                  'wall_s': time.perf_counter() - started, 'finished_utc': now.isoformat(),
                  'finished_beijing': now.astimezone(timezone(timedelta(hours=8))).isoformat(),
                  'browser_visual_tested': False,
                  'checkpoint_response_bytes_verified': True,
                  'browser_download_completion_tested': False,
                  'complete_demo': full_demo}
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: report[k] for k in ('passed', 'checks', 'mesh_bindings', 'wall_s', 'finished_utc', 'finished_beijing')}))
        return report
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=10)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--complete-demo', action='store_true')
    parser.add_argument('--expected-checkpoint', type=Path)
    args = parser.parse_args()
    verify(args.output, args.complete_demo, args.expected_checkpoint)
