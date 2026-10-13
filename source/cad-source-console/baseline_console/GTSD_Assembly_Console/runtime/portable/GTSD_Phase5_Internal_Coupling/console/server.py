"""Loopback-only live console for the v5 new-geometry software plant."""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from functools import partial
from pathlib import Path
from urllib.parse import urlsplit

from phase5_controller import RegisteredController as Controller

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
MAX_BODY = 2 * 1024 * 1024


def strict_json(body):
    def no_constant(value):
        raise ValueError("nonfinite JSON constant: " + value)

    def unique_object(pairs):
        out = {}
        for key, value in pairs:
            if key in out:
                raise ValueError("duplicate JSON key")
            out[key] = value
        return out

    return json.loads(body, parse_constant=no_constant, object_pairs_hook=unique_object)


class ConsoleServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, port, controller):
        self.controller = controller
        # There is intentionally no public-host, serial, GPIO or hardware option.
        super().__init__(("127.0.0.1", port), Handler)
        self.allowed_hosts = {f"127.0.0.1:{self.server_port}", f"localhost:{self.server_port}"}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def log_message(self, *args):
        pass

    def _send(self, code, data, content_type="application/json; charset=utf-8"):
        body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if type(data) is dict else data
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass  # A disconnected observer must not affect the solver owner.
        self.close_connection = True

    def _authorized(self):
        host, origin = self.headers.get("Host", ""), self.headers.get("Origin")
        if (host not in self.server.allowed_hosts or origin is not None and origin != "http://" + host
                or self.headers.get("Sec-Fetch-Site") == "cross-site"):
            self._send(403, {"error": "only same-origin loopback requests accepted"})
            return False
        return True

    def do_GET(self):
        if not self._authorized():
            return
        path = urlsplit(self.path).path
        if path == "/api/state":
            self._send(200, self.server.controller.snapshot())
        elif path in ("/api/export", "/api/observation"):
            self._send(200, self.server.controller.snapshot(heartbeat=False))
        elif path == "/api/checkpoint":
            try:
                self._send(200, self.server.controller.export_checkpoint())
            except (ValueError, TimeoutError) as exc:
                self._send(409, {"error": str(exc)})
        elif path == "/api/internal-registry":
            from internal_registry.registry import build_registry
            self._send(200, build_registry())
        elif path == "/api/binding":
            # Frozen exact geometry bindings are a read-only construction product.
            controller = self.server.controller
            with controller.lock:
                binding = getattr(controller, "binding", None)
            if binding is None:
                self._send(409, {"error": "model binding is not ready"})
            else:
                self._send(200, binding)
        else:
            if path == "/geometry.json" and self.server.controller.visual_resource_bytes is not None:
                self._send(200, self.server.controller.visual_resource_bytes, "application/json; charset=utf-8")
                return
            assets = {
                "/": (WEB / "index.html", "text/html; charset=utf-8"),
                "/app.js": (WEB / "app.js", "text/javascript; charset=utf-8"),
                "/ui-model.js": (WEB / "ui-model.js", "text/javascript; charset=utf-8"),
                "/contact-sections.js": (WEB / "contact-sections.js", "text/javascript; charset=utf-8"),
                "/internal-systems.js": (WEB / "internal-systems.js", "text/javascript; charset=utf-8"),
                "/style.css": (WEB / "style.css", "text/css; charset=utf-8"),
                "/physics-model.js": (ROOT.parent / "web/physics-model.js", "text/javascript; charset=utf-8"),
                "/render-model.js": (ROOT.parent / "web/render-model.js", "text/javascript; charset=utf-8"),
                "/geometry.json": (ROOT.parent / "web/geometry.json", "application/json; charset=utf-8"),
            }
            if path not in assets:
                self._send(404, {"error": "not found"})
                return
            file, mime = assets[path]
            try:
                self._send(200, file.read_bytes(), mime)
            except FileNotFoundError:
                self._send(404, {"error": "verified console asset is missing"})

    def do_POST(self):
        if not self._authorized():
            return
        if urlsplit(self.path).path != "/api/command":
            self._send(404, {"error": "not found"})
            return
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            self._send(415, {"error": "application/json required"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= MAX_BODY or self.headers.get("Transfer-Encoding"):
                raise ValueError("request needs a bounded Content-Length (maximum 2 MiB)")
            payload = strict_json(self.rfile.read(length))
            result = self.server.controller.submit(payload)
        except (ValueError, UnicodeError, TimeoutError, RecursionError) as exc:
            self._send(400, {"error": str(exc)})
            return
        self._send(202, result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--variant", choices=("base", "extended", "extended-cables", "cover-bench", "brake-contact-bench", "cable-support-bench", "legacy-drawbar", "legacy-closed-pantograph", "contact-sections-research", "internal-brake-research", "internal-coupled-research"), default="base",
                        help="explicit separate solver choice; independent benches have no vehicle coupling")
    parser.add_argument("--include-cables", action="store_true",
                        help="extended variant only: add actual segmented cable proxy dynamics")
    parser.add_argument("--resume", type=Path, help="explicit v5 complete JSON checkpoint; invalid files fail closed")
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("invalid port")
    if args.include_cables and args.variant not in ("extended", "extended-cables"):
        parser.error("--include-cables requires --variant extended")
    resume = None
    if args.resume:
        if args.resume.stat().st_size > MAX_BODY:
            parser.error("checkpoint exceeds 2 MiB")
        resume = strict_json(args.resume.read_bytes())
    from controller import make_plant
    factory = make_plant
    if args.variant in ("extended", "extended-cables"):
        from export_isolation import IsolatedExportsExtendedPlant
        factory = partial(IsolatedExportsExtendedPlant, include_cables=args.include_cables or args.variant == "extended-cables")
    elif args.variant == "cover-bench":
        from bench_adapter import BenchPlant
        factory = BenchPlant
    elif args.variant == "cable-support-bench":
        from research_bench_adapter import CableSupportPlant
        factory = CableSupportPlant
    elif args.variant == "brake-contact-bench":
        from research_bench_adapter import BrakeContactPlant
        factory = BrakeContactPlant
    elif args.variant in ("legacy-drawbar", "legacy-closed-pantograph"):
        from legacy_research import Bench
        factory = partial(Bench, kind="drawbar" if args.variant == "legacy-drawbar" else "closed-pantograph")
    elif args.variant in ("internal-brake-research", "internal-coupled-research"):
        from phase5_display import display_plant
        factory = partial(display_plant, kind="brake" if args.variant == "internal-brake-research" else "coupled")
    elif args.variant == "contact-sections-research":
        from sections_export_isolation import IsolatedExportsSectionVehiclePlant
        factory = partial(IsolatedExportsSectionVehiclePlant, include_cables=False)
    controller = Controller(resume=resume, factory=factory)
    server = ConsoleServer(args.port, controller)
    controller.start()
    print(f"GTSD v5 {args.variant} software-only console: http://127.0.0.1:{server.server_port}", flush=True)
    print("Fresh model starts paused. Run loses its lease after 5 s without state reads. No legacy cache is read or written.", flush=True)
    if args.variant == "contact-sections-research":
        print("OPTIONAL RESEARCH ONLY: two fixed tread sections per wheel; uncalibrated; strict stepwise normal passivity unproved; no cables; Only the listed 5-second protocol at two timesteps was audited; strict stepwise normal passivity did not pass; two negative-side flange candidates were inactive. Slow research example: about 0.01 simulated seconds per wall second; no wall-clock catch-up.", flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
