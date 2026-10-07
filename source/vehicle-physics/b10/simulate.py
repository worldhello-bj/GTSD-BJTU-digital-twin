"""Settled full-vehicle B10 cases; auxiliary states share the finite gas network."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform

from auxiliaries import B9, ROOT, AuxiliaryNetwork, make_vehicle_network
from compat import load_b8, B8

OUT = ROOT / "results"
CASES = dict(service_brake=dict(torque=True, brake=True, reservoir_initial_pressure_Pa=401325),
             half_dt=dict(torque=True, brake=True, reservoir_initial_pressure_Pa=401325, timestep_s=.000025),
             compressor_power_failure=dict(torque=True, brake=True, reservoir_initial_pressure_Pa=401325,
                                           compressor_power_failure=True))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes():
    return {"b8": {p.name:sha(p) for p in [B8/(n+".py") for n in
            ("simulate_full", "full_model", "compliant_contact", "pneumatics", "pantograph", "access_doors")]},
            "b9": {n:sha(B9/n) for n in ("valve_dynamics.py", "compat.py")},
            "b10": {n:sha(ROOT/n) for n in ("auxiliaries.py", "simulate.py")},
            "parent": {"build_model.py":sha(B8.parent/"build_model.py")}}


def flatten_auxiliary(net):
    row = dict(network_time_s=net.time_s, reservoir_pressure_Pa=net.pressure_Pa("reservoir"),
               delivered_air_kg=net.edge_mass_kg["compressor_discharge"],
               ambient_intake_kg=-net.edge_mass_kg["compressor_intake"],
               spool_balance_residual_J=net.spool_audit()["balance_residual_J"])
    for name, state in {"pump": net.pump.snapshot(), **{n:f.snapshot() for n,f in net.fans.items()},
                        **{n:t.snapshot() for n,t in net.thermal.items()}}.items():
        row.update({name+"_"+k: v for k,v in state.items() if isinstance(v, (int, float, bool))})
    return row


def install_recorder(net, cfg):
    if cfg.get("compressor_power_failure"):
        net.pump.motor.power_failed = True
    net.records = [flatten_auxiliary(net)]
    original_update = net.update
    def recorded_update(*args, **kwargs):
        result = original_update(*args, **kwargs)
        if not net.records or net.time_s-net.records[-1]["network_time_s"] >= .01-1e-10:
            net.records.append(flatten_auxiliary(net))
        return result
    net.update = recorded_update
    return net


def run(name, cfg, duration=10, replay=True):
    OUT.mkdir(exist_ok=True)
    sources = source_hashes()
    base = load_b8("simulate_full", "b10_private_batch")
    base.OUT = OUT
    networks = []
    def factory(*args, **kwargs):
        net = install_recorder(make_vehicle_network(*args, **kwargs), cfg)
        networks.append(net)
        return net
    base.make_vehicle_network = factory
    metrics = base.run(name, cfg, duration, replay)
    net = networks[0]
    xml = OUT/(name+".xml")
    xml.write_bytes(xml.read_text(encoding="utf-8").encode("utf-8"))
    if sha(xml) != metrics["model_xml_sha256"]:
        raise RuntimeError("mechanical XML identity mismatch")
    if abs(net.records[-1]["network_time_s"]-net.time_s) > 1e-8:
        net.records.append(flatten_auxiliary(net))
    with (OUT/(name+"_auxiliaries.csv")).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=net.records[0]); writer.writeheader(); writer.writerows(net.records)
    # B8's coupling column compares vehicle actuator work to ALL gas work.
    # Retain that raw column; add a scoped vehicle-only comparison using the
    # actual pump geometry work, with linear alignment of the 100 Hz ledger.
    a = base.np.genfromtxt(OUT/(name+".csv"), names=True, delimiter=",")
    times = [r["network_time_s"] for r in net.records]
    works = [r["pump_gas_geometry_work_J"] for r in net.records]
    initial = base.np.interp(6, times, works)
    pump_work = base.np.interp(a["time_s"]+6, times, works)-initial
    rows = []
    for i in range(len(a)):
        row = {column: float(a[column][i]) for column in a.dtype.names}
        row.update(pump_gas_work_J=float(pump_work[i]),
                   vehicle_gas_useful_work_J=row["gas_useful_work_J"]-float(pump_work[i]),
                   vehicle_coupling_work_discrepancy_J=row["coupling_work_discrepancy_J"]+float(pump_work[i]))
        rows.append(row)
    with (OUT/(name+".csv")).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    metrics.update(version="B10 inferred force-driven compressor and fan bench + B9 valves + B8 vehicle",
                   auxiliary_final_state=dict(pump=net.pump.snapshot(), fans={n:f.snapshot() for n,f in net.fans.items()},
                                              thermal={n:t.snapshot() for n,t in net.thermal.items()}),
                   spool_audit=net.spool_audit(), final=rows[-1],
                   coupling_work_scope="raw B8 column includes pump; vehicle-only column removes recorded pump geometry work",
                   pneumatic_configuration=net.configuration())
    (OUT/(name+"_metrics.json")).write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    files = [xml, OUT/(name+".csv"), OUT/(name+"_metrics.json"), OUT/(name+"_auxiliaries.csv")]
    if replay:
        path = OUT/(name+"_replay.json")
        data = json.loads(path.read_text(encoding="utf-8"))
        data.update(schema="B10 inferred auxiliary bench and force-driven vehicle replay v1",
                    pneumatic_configuration=net.configuration(), auxiliary_trace=name+"_auxiliaries.csv",
                    auxiliary_trace_time_offset_s=6.0)
        path.write_text(json.dumps(data, separators=(",",":")), encoding="utf-8")
        files.append(path)
    if source_hashes() != sources:
        raise RuntimeError("executed source changed during simulation; rerun before accepting results")
    identity = dict(case=name, python=platform.python_version(), mujoco=base.mujoco.__version__,
                    numpy=base.np.__version__, sources_sha256=sources,
                    outputs_sha256={p.name:sha(p) for p in files},
                    configuration=net.configuration())
    (OUT/(name+"_identity.json")).write_text(json.dumps(identity, indent=2), encoding="utf-8")
    print(json.dumps(dict(case=name, pump=net.pump.audit(), final=rows[-1]["vehicle_coupling_work_discrepancy_J"])), flush=True)
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=list(CASES)+["all"], default="service_brake")
    parser.add_argument("--duration", type=float, default=10)
    args = parser.parse_args()
    if args.duration < 10:
        parser.error("full acceptance requires 6 s settling and at least 10 s observation")
    for name in CASES if args.case == "all" else [args.case]:
        run(name, CASES[name], args.duration)
