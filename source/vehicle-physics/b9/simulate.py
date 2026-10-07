"""Run the B8 mechanical solver with B9 dynamic metering-port forces/areas."""
from dataclasses import replace
import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform

from compat import load_b8, B8
from valve_dynamics import SpoolParameters, DynamicValveNetwork, make_b8_network

ROOT = Path(__file__).resolve().parent
OUT = ROOT/"results"
CASES = dict(service_brake=dict(torque=True, brake=True),
             half_dt=dict(torque=True, brake=True, timestep_s=.000025),
             brake_coil_failure=dict(torque=True, brake=True, brake_coil_failure=True),
             slow_brake_valves=dict(torque=True, brake=True, slow_brake_valves=True),
             normal_exhaust=dict(torque=True, brake=True, normal_exhaust=True))


class RecordedNetwork(DynamicValveNetwork):
    def __init__(self, gas, cfg):
        overrides = {}
        if cfg.get("slow_brake_valves"):
            overrides = {n: replace(SpoolParameters(), damping_N_s_m=80)
                         for n in gas.orifices if "_pad_supply" in n or "_pad_exhaust" in n}
        super().__init__(gas, port_parameters=overrides)
        if cfg.get("brake_coil_failure"):
            for n in self.spools:
                if "_pad_supply" in n:
                    self.set_coil_failure(n, True)
        self.selected = [next(n for n in self.spools if "_pad_supply" in n),
                         next(n for n in self.spools if "_pad_exhaust" in n),
                         "pantograph_supply", "pantograph_exhaust", "pantograph_quick_exhaust"]
        self.records = []
        self.last_record_s = -1.0
        self.max_energy_residual_J = 0.0

    def update(self, *args, **kwargs):
        result = super().update(*args, **kwargs)
        if self.time_s-self.last_record_s >= .01-1e-10:
            self.last_record_s = self.time_s
            audit = self.spool_audit()
            self.max_energy_residual_J = max(self.max_energy_residual_J, abs(audit["balance_residual_J"]))
            row = dict(network_time_s=self.time_s, **{k: v for k,v in audit.items() if k not in ("port_count", "mechanical_state_count")})
            for n in self.selected:
                s = self.spools[n]
                row.update({n+"_position_m": s.position_m, n+"_velocity_m_s": s.velocity_m_s,
                            n+"_opening": s.opening, n+"_command": s.command})
            self.records.append(row)
        return result


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(name, cfg, duration=10, replay=True):
    OUT.mkdir(exist_ok=True)
    base = load_b8("simulate_full", "b9_private_batch")
    base.OUT = OUT
    networks = []
    def factory(*args, **kwargs):
        net = RecordedNetwork(make_b8_network(*args, **kwargs), cfg)
        networks.append(net)
        return net
    base.make_vehicle_network = factory
    metrics = base.run(name, cfg, duration, replay)
    net = networks[0]
    xml = OUT/(name+".xml")
    # B8 was written on Unix. Preserve the embedded LF XML identity on Windows.
    xml.write_bytes(xml.read_text(encoding="utf-8").encode("utf-8"))
    if sha(xml) != metrics["model_xml_sha256"]:
        raise RuntimeError("solver XML identity mismatch")
    with (OUT/(name+"_spools.csv")).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=net.records[0])
        writer.writeheader(); writer.writerows(net.records)
    metrics.update(version="B9 dynamic metering ports on frozen B8 mechanical solver",
                   spool_audit=net.spool_audit(), max_sampled_spool_balance_residual_J=net.max_energy_residual_J,
                   final_spool_states={n:s.snapshot() for n,s in net.spools.items()})
    (OUT/(name+"_metrics.json")).write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    files = [xml, OUT/(name+".csv"), OUT/(name+"_metrics.json"), OUT/(name+"_spools.csv")]
    if replay:
        path = OUT/(name+"_replay.json")
        data = json.loads(path.read_text(encoding="utf-8"))
        data.update(schema="B9 force driven full3D replay v1",
                    pneumatic_configuration=net.configuration(),
                    dynamic_metering_trace=name+"_spools.csv")
        path.write_text(json.dumps(data, separators=(",",":")), encoding="utf-8")
        files.append(path)
    sources = [B8/(n+".py") for n in ("simulate_full", "full_model", "compliant_contact", "pneumatics", "pantograph", "access_doors")]
    sources += [B8.parent/"build_model.py", ROOT/"valve_dynamics.py", ROOT/"compat.py", ROOT/"simulate.py"]
    identity = dict(case=name, python=platform.python_version(), mechanical_solver="MuJoCo "+base.mujoco.__version__,
                    numpy=base.np.__version__, b8_sources_sha256={p.name:sha(p) for p in sources if p.parent != ROOT},
                    b9_sources_sha256={p.name:sha(p) for p in sources if p.parent == ROOT},
                    outputs_sha256={p.name:sha(p) for p in files},
                    force_model="B8 mechanical/contact/gas laws + B9 pressure-balanced spool force/area dynamics",
                    parameters=net.configuration()["dynamic_metering_ports"])
    (OUT/(name+"_identity.json")).write_text(json.dumps(identity, indent=2), encoding="utf-8")
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", default="service_brake", choices=list(CASES)+["all"])
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--no-replay", action="store_true")
    args = parser.parse_args()
    if args.duration < 10:
        parser.error("integrated cases require the full 10 s observation window")
    for name in CASES if args.case == "all" else [args.case]:
        run(name, CASES[name], args.duration, not args.no_replay)
