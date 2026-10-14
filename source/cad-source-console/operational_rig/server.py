"""Fixed-route, same-origin loopback UI. No filesystem paths are accepted."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
from urllib.parse import parse_qs, urlsplit
from operational_session import RigSession, RigError, RigConflict

ROOT = Path(__file__).resolve().parent
BASE_WEB = ROOT.parent / 'baseline_console/GTSD_Assembly_Console/app/web'
PREFIX = '/api/operational-rig/'
STATIC = {
    '/': (ROOT / 'web/index.html', 'text/html; charset=utf-8'),
    '/operational-rig': (ROOT / 'web/index.html', 'text/html; charset=utf-8'),
    '/operational-rig/': (ROOT / 'web/index.html', 'text/html; charset=utf-8'),
    '/operational-rig/parameters': (ROOT / 'web/parameters.html', 'text/html; charset=utf-8'),
    '/operational-rig/app.js': (ROOT / 'web/app.js', 'text/javascript; charset=utf-8'),
    '/operational-rig/core.js': (ROOT / 'web/core.js', 'text/javascript; charset=utf-8'),
    '/operational-rig/style.css': (ROOT / 'web/style.css', 'text/css; charset=utf-8'),
    '/operational-rig/parameters.html': (ROOT / 'web/parameters.html', 'text/html; charset=utf-8'),
    '/operational-rig/parameters.js': (ROOT / 'web/parameters.js', 'text/javascript; charset=utf-8'),
    '/operational-rig/model.js': (BASE_WEB / 'model.js', 'text/javascript; charset=utf-8'),
}


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise RigError('Duplicate JSON key.')
            result[key] = value
        return result
    def reject(value):
        raise RigError('Non-finite JSON numbers are forbidden.')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=reject)


class Handler(BaseHTTPRequestHandler):
    server_version = 'GTSDOperationalLoopback/1'

    def log_message(self, *_args):
        pass

    def _send(self, code, data, mime='application/json; charset=utf-8', attachment=None):
        body = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Cross-Origin-Resource-Policy', 'same-origin')
        self.send_header('Content-Security-Policy', "default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'; form-action 'none'")
        if attachment:
            self.send_header('Content-Disposition', 'attachment; filename="' + attachment + '"')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _authorized(self):
        hosts = self.headers.get_all('Host', [])
        allowed = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
        if self.client_address[0] != '127.0.0.1' or len(hosts) != 1 or hosts[0] not in allowed:
            self.close_connection = True
            self._send(403, {'error': 'Exact local Host and loopback peer required.'})
            return False
        origins = self.headers.get_all('Origin', [])
        if len(origins) > 1 or (origins and origins[0] != 'http://' + hosts[0]):
            self.close_connection = True
            self._send(403, {'error': 'Cross-origin requests are refused.'})
            return False
        if self.headers.get('Sec-Fetch-Site') not in (None, 'same-origin', 'none'):
            self.close_connection = True
            self._send(403, {'error': 'Cross-site requests are refused.'})
            return False
        return True

    def do_GET(self):
        if not self._authorized():
            return
        parts = urlsplit(self.path)
        try:
            if parts.scheme or parts.netloc or parts.fragment:
                raise RigError('Only fixed origin-form paths are accepted.')
            if parts.path == PREFIX + 'checkpoint-export':
                query = parse_qs(parts.query, keep_blank_values=True)
                if set(query) != {'id', 'session_id'} or any(len(v) != 1 for v in query.values()):
                    raise RigError('One saved checkpoint and its session ID are required.')
                if not re.fullmatch('[a-f0-9]{32}', query['id'][0]) or not re.fullmatch('[a-f0-9]{32}', query['session_id'][0]):
                    raise RigError('Malformed checkpoint identity.')
                result = self.server.rig_session.export_checkpoint(query['id'][0], query['session_id'][0])
                return self._send(200, result, attachment='GTSD_operational_checkpoint_' + query['id'][0][:8] + '.json')
            if parts.query:
                raise RigError('This fixed route takes no query arguments.')
            if parts.path in STATIC:
                path, mime = STATIC[parts.path]
                return self._send(200, path.read_bytes(), mime)
            readers = {'metadata': self.server.rig_session.metadata, 'status': self.server.rig_session.snapshot,
                       'geometry': self.server.rig_session.geometry}
            reader = readers.get(parts.path.removeprefix(PREFIX)) if parts.path.startswith(PREFIX) else None
            if reader:
                return self._send(200, reader())
            return self._send(404, {'error': 'Unknown fixed route.'})
        except RigConflict as exc:
            return self._send(409, {'error': str(exc)})
        except (RigError, ValueError, TypeError) as exc:
            return self._send(400, {'error': str(exc)})
        except OSError:
            return self._send(503, {'error': 'A fixed local source asset is unavailable.'})

    def do_POST(self):
        if not self._authorized():
            return
        parts = urlsplit(self.path)
        if parts.scheme or parts.netloc or parts.query or parts.fragment or parts.path not in (PREFIX + 'initialize', PREFIX + 'command'):
            self.close_connection = True
            return self._send(404, {'error': 'Only explicit initialization and bounded commands exist. No upload, import or hardware routes.'})
        try:
            lengths = self.headers.get_all('Content-Length', [])
            if len(lengths) != 1 or self.headers.get('Transfer-Encoding'):
                raise RigError('Exactly one Content-Length and no Transfer-Encoding required.')
            if not lengths[0].isdigit() or not 1 <= int(lengths[0]) <= 16384:
                raise RigError('JSON body must be 1–16384 bytes.')
            if len(self.headers.get_all('Content-Type', [])) != 1 or self.headers['Content-Type'].split(';')[0].strip() != 'application/json':
                raise RigError('application/json required.')
            size = int(lengths[0])
            self.connection.settimeout(3.)
            raw = self.rfile.read(size)
            if len(raw) != size:
                raise RigError('Incomplete body; no command executed.')
            body = strict_json(raw)
            method = self.server.rig_session.initialize if parts.path.endswith('/initialize') else self.server.rig_session.command
            return self._send(200, method(body))
        except RigConflict as exc:
            return self._send(409, {'error': str(exc)})
        except (RigError, ValueError, TypeError, OverflowError, RecursionError) as exc:
            self.close_connection = True
            return self._send(400, {'error': str(exc)})
        except (TimeoutError, OSError):
            self.close_connection = True
            return self._send(408, {'error': 'Body read failed or timed out; refresh status before any retry.'})
        except Exception:
            self.close_connection = True
            return self._send(500, {'error': 'Command failed. Refresh status before continuing.'})


class LoopbackServer(ThreadingHTTPServer):
    daemon_threads = True
    def server_close(self):
        self.rig_session.close()
        super().server_close()


def make_server(port=0, session=None):
    server = LoopbackServer(('127.0.0.1', port), Handler)
    server.rig_session = session or RigSession()
    return server, server.rig_session


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8176)
    args = parser.parse_args()
    server, session = make_server(args.port)
    print(f'http://127.0.0.1:{server.server_port}/operational-rig (initialize the default assembly to start)', flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
