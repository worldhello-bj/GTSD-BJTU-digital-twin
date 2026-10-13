"""Actual launcher serve lifecycle, fresh base model, zero integration steps.

Run explicitly, supplying the full frozen source directory as the only argument.
This complements smoke, which tests the generated original source CLI directly.
"""
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import console_launcher as L


def main():
    source = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix="serve-config-", dir=ROOT / "evidence") as directory:
        config = Path(directory) / "config with spaces.json"
        config.write_text(json.dumps({"schema": L.SCHEMA, "source_root": str(source),
                                      "variant": "base", "port": 0, "resume": None}))
        process = subprocess.Popen([sys.executable, str(ROOT / "console_launcher.py"),
                                    "serve", "--config", str(config)],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            line = process.stdout.readline()
            assert "http://127.0.0.1:" in line, line
            port = int(line.split("http://127.0.0.1:", 1)[1].strip())
            deadline = time.monotonic() + 20
            while True:
                state = L.inspect_port(port)
                assert state["phase"] != "error", state.get("error")
                if state["phase"] == "paused":
                    break
                assert time.monotonic() < deadline, "startup timeout"
                time.sleep(.03)
            assert state["plant"]["steps"] == 0
            assert state["plant"]["time_s"] == 0
            assert state["capabilities"]["integrated_dof"] == 15
            assert state["capabilities"]["hardware_control"] is False
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=20)
            assert process.returncode == 0, (process.returncode, stderr)
            print(json.dumps({"ok": True, "scope": "actual configuration adapter serve lifecycle, base model, zero steps, graceful SIGINT exit",
                              "port_type": "ephemeral loopback", "phase": state["phase"],
                              "steps": state["plant"]["steps"], "time_s": state["plant"]["time_s"],
                              "integrated_dof": state["capabilities"]["integrated_dof"],
                              "exit_code": process.returncode, "stderr": stderr}, indent=2))
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=5)


if __name__ == "__main__":
    main()
