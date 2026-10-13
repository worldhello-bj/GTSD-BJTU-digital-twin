"""Focused new-adapter tests. No long physics runs or browser claims."""
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import console_launcher as L
UNSET = object()


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="gtsd-config-test-")
        self.root = Path(self.temp.name)
        self.source = self.root / "source with spaces"
        self.source.mkdir()
        self.path = self.root / "config.json"
        self.data = {"schema": L.SCHEMA, "source_root": "source with spaces",
                     "variant": "base", "port": 0, "resume": None}

    def tearDown(self):
        self.temp.cleanup()

    def write(self, value=UNSET):
        self.path.write_text(json.dumps(self.data if value is UNSET else value))
        return L.load_config(self.path)

    def fails(self, code, value):
        with self.assertRaises(L.ConfigError) as caught:
            self.write(value)
        self.assertEqual(caught.exception.code, code)

    def test_relative_paths_resolve_at_config_not_working_directory(self):
        self.assertEqual(self.write()["source_root"], str(self.source))

    def test_all_reviewed_variants(self):
        for name in L.VARIANTS:
            with self.subTest(name=name):
                self.assertEqual(self.write(dict(self.data, variant=name))["variant"], name)

    def test_unknown_variant_rejected(self):
        for value in ("contact-research", "hardware", "", [], None):
            with self.subTest(value=value):
                self.fails("invalid_variant", dict(self.data, variant=value))

    def test_unknown_fields_including_host_and_physics_override_rejected(self):
        for field in ("host", "dt", "hardware_port", "include_cables", "auto_run", "outbound_url"):
            with self.subTest(field=field):
                self.fails("config_fields", dict(self.data, **{field: "forbidden"}))

    def test_required_fields_and_top_level_type(self):
        for key in self.data:
            value = dict(self.data)
            del value[key]
            self.fails("config_fields", value)
        for value in ([], "config", None, 0):
            self.fails("config_fields", value)

    def test_bad_schema(self):
        self.fails("config_schema", dict(self.data, schema="gtsd-v999"))

    def test_port_strict_type_and_range(self):
        for value in (True, False, 8766.0, "8766", -1, 65536, None):
            with self.subTest(value=value):
                self.fails("invalid_port", dict(self.data, port=value))
        for value in (0, 1, 65535):
            self.assertEqual(self.write(dict(self.data, port=value))["port"], value)

    def test_invalid_and_missing_paths(self):
        for value in (None, 12, "", "  ", "bad\x00path"):
            self.fails("invalid_path", dict(self.data, source_root=value))
        self.fails("source_missing", dict(self.data, source_root="absent"))
        self.fails("missing_file", dict(self.data, resume="missing-checkpoint.json"))

    def test_duplicate_keys_and_all_nonfinite_spellings(self):
        for body, code in ((b'{"x":1,"x":2}', "duplicate_key"),
                           (b'{"x":NaN}', "nonfinite_json"),
                           (b'{"x":Infinity}', "nonfinite_json"),
                           (b'{"x":-Infinity}', "nonfinite_json"),
                           (b'{"x":1e999}', "nonfinite_json")):
            with self.subTest(body=body), self.assertRaises(L.ConfigError) as caught:
                L.strict_json(body)
            self.assertEqual(caught.exception.code, code)

    def test_invalid_utf8_and_bad_json(self):
        for body in (b'{', b'\xff', b'{"a":}'):
            with self.assertRaises(L.ConfigError) as caught:
                L.strict_json(body)
            self.assertEqual(caught.exception.code, "invalid_json")

    def test_configuration_size_is_bounded(self):
        self.path.write_bytes(b" " * (L.MAX_CONFIG + 1))
        with self.assertRaises(L.ConfigError) as caught:
            L.load_config(self.path)
        self.assertEqual(caught.exception.code, "file_too_large")

    def checkpoint(self, plant=None):
        value = {"schema": "gtsd-v5-console-checkpoint-v1", "plant": plant or {"state": [0, 1]}}
        value["sha256"] = hashlib.sha256(L.canonical(value)).hexdigest()
        path = self.root / "checkpoint.json"
        path.write_text(json.dumps(value))
        return path, value

    def test_checkpoint_relative_path_and_envelope_only_label(self):
        path, value = self.checkpoint()
        config = self.write(dict(self.data, resume="checkpoint.json"))
        self.assertEqual(config["resume"], str(path))
        self.assertIn("envelope_only", L.checkpoint_header(path)["validation"])

    def test_checkpoint_digest_and_observation_rejection(self):
        path, value = self.checkpoint()
        value["plant"]["state"][0] = 4
        path.write_text(json.dumps(value))
        with self.assertRaises(L.ConfigError) as caught:
            L.checkpoint_header(path)
        self.assertEqual(caught.exception.code, "checkpoint_digest")
        path.write_text('{"schema":"gtsd-v5-console-state-v1"}')
        with self.assertRaises(L.ConfigError) as caught:
            L.checkpoint_header(path)
        self.assertEqual(caught.exception.code, "checkpoint_shape")

    def test_checkpoint_too_large_rejected(self):
        path = self.root / "huge.json"
        path.write_bytes(b" " * (L.MAX_CHECKPOINT + 1))
        with self.assertRaises(L.ConfigError) as caught:
            L.checkpoint_header(path)
        self.assertEqual(caught.exception.code, "file_too_large")

    def test_missing_resources_doctor_lists_exact_paths(self):
        with patch.object(L.importlib.metadata, "version", side_effect=lambda name: L.DEPENDENCIES[name]):
            result = L.doctor(self.write(), check_bind=False)
        self.assertFalse(result["ok"])
        self.assertEqual(len(result["errors"]), len(L.REQUIRED_FILES))
        self.assertIn("console/server.py", result["errors"][1]["message"])

    def test_wrong_or_missing_dependencies_are_actionable(self):
        def version(name):
            if name == "mujoco":
                raise L.importlib.metadata.PackageNotFoundError(name)
            return "0.0.0"
        with patch.object(L.importlib.metadata, "version", side_effect=version):
            result = L.doctor(self.write(), check_bind=False)
        problems = [row for row in result["errors"] if row["code"] == "dependency_version"]
        self.assertEqual(len(problems), 2)
        self.assertIn(sys.executable, problems[0]["message"])

    def test_port_check_is_loopback_and_does_not_stop_existing_service(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            sock.listen(1)
            port = sock.getsockname()[1]
            with self.assertRaises(L.ConfigError) as caught:
                L.check_port(port)
            self.assertEqual(caught.exception.code, "port_in_use")
            self.assertEqual(sock.getsockname()[1], port)
        L.check_port(0)

    def test_argument_array_no_shell_interpolation(self):
        config = self.write()
        self.assertEqual(L.server_arguments(config), ["--variant", "base", "--port", "0"])
        config["resume"] = str(self.root / "spaces ; shell $(text).json")
        self.assertEqual(L.server_arguments(config)[-1], config["resume"])

    def test_serve_reuses_source_main_and_preserves_port_race_diagnostic(self):
        calls = []
        with patch.dict(sys.modules, {"server": SimpleNamespace(main=lambda args: calls.append(args) or 0)}):
            self.assertEqual(L.serve(self.write()), 0)
        self.assertEqual(calls, [["--variant", "base", "--port", "0"]])
        def occupied(args):
            raise OSError(L.errno.EADDRINUSE, "Address already in use")
        with patch.dict(sys.modules, {"server": SimpleNamespace(main=occupied)}):
            with self.assertRaises(L.ConfigError) as caught:
                L.serve(self.write())
        self.assertEqual(caught.exception.code, "port_in_use")


class ObservationTests(unittest.TestCase):
    def test_non_heartbeat_endpoint_and_dynamic_registry_retained(self):
        requests = []
        state = {"schema": "gtsd-v5-console-state-v1", "phase": "paused", "error": None,
                 "plant": {"steps": 0, "time_s": 0, "dt_s": .0004, "solver_proxies": []},
                 "command_registry": {"x": {"args": {"force_N": {"type": "number"}}}}}
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                requests.append((self.path, self.headers.get("Host")))
                body = json.dumps(state).encode()
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            def log_message(self, *args):
                pass
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            result = L.describe_state(L.inspect_port(server.server_port))
            self.assertEqual(requests, [("/api/observation", f"127.0.0.1:{server.server_port}")])
            self.assertEqual(result["command_registry"], state["command_registry"])
            self.assertIs(result["renews_run_lease"], False)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_inspect_rejects_zero_and_boolean_ports(self):
        for port in (0, True, -1, 65536):
            with self.assertRaises(L.ConfigError) as caught:
                L.inspect_port(port)
            self.assertEqual(caught.exception.code, "inspect_port")


if __name__ == "__main__":
    unittest.main(verbosity=2)
