"""120 s compressor/thermal bench including startup, faults and half-step case."""
import argparse
import csv
import json
import platform

from auxiliaries import ROOT, make_vehicle_network, AMBIENT_PA
from simulate import flatten_auxiliary, sha, B8, B9

OUT = ROOT/"bench_results"
CASES = dict(charge=dict(), half_dt=dict(dt=.0001),
             power_loss=dict(power_loss=True), outlet_blocked=dict(outlet_blocked=True),
             fan_power_loss=dict(fan_power_loss=True),
             pressure_switch=dict(reservoir_volume_m3=.0002))


def run(name):
    sources = {"b8": {"pneumatics.py":sha(B8/"pneumatics.py")},
               "b9": {"valve_dynamics.py":sha(B9/"valve_dynamics.py")},
               "b10": {n:sha(ROOT/n) for n in ("auxiliaries.py", "simulate.py", "bench.py")}}
    cfg = CASES[name]
    dt = cfg.get("dt", .0002)
    net = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=AMBIENT_PA,
                               reservoir_volume_m3=cfg.get("reservoir_volume_m3", .002))
    net.pump.discharge_failed_closed = cfg.get("outlet_blocked", False)
    rows = [flatten_auxiliary(net)]
    switch_edges = []
    previous_switch = net.pump.pressure_switch_on
    for k in range(round(120/dt)):
        if k*dt >= 10:
            if cfg.get("power_loss"):
                net.pump.motor.power_failed = True
            if cfg.get("fan_power_loss"):
                for fan in net.fans.values():
                    fan.motor.power_failed = True
        net.update(dt, snapshot=False)
        if net.pump.pressure_switch_on != previous_switch:
            switch_edges.append(dict(time_s=net.time_s, pressure_Pa=net.pressure_Pa("reservoir"), on=net.pump.pressure_switch_on))
            previous_switch = net.pump.pressure_switch_on
        if (k+1) % round(.01/dt) == 0:
            rows.append(flatten_auxiliary(net))
    OUT.mkdir(exist_ok=True)
    path = OUT/(name+".csv")
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    gas = net.audit()
    report = dict(case=name, duration_s=120, dt_s=dt, config=cfg, final=rows[-1], gas_audit=gas,
                  pneumatic_configuration=net.configuration(), switch_edges=switch_edges,
                  thresholds=dict(mass_kg=1e-10, gas_energy_J=1e-5, auxiliary_mechanical_J=1e-6,
                                  motor_electrical_J=1e-6, thermal_J=1e-5))
    report_path = OUT/(name+"_metrics.json")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    for version, values in sources.items():
        for file, expected in values.items():
            if sha(ROOT.parent/version/file) != expected:
                raise RuntimeError("executed source changed during bench; rerun before accepting results")
    identity = dict(python=platform.python_version(), sources_sha256=sources,
                    outputs_sha256={p.name:sha(p) for p in (path, report_path)})
    (OUT/(name+"_identity.json")).write_text(json.dumps(identity, indent=2), encoding="utf-8")
    print(json.dumps(dict(case=name, reservoir_pressure_Pa=net.pressure_Pa("reservoir"),
                         delivered_air_kg=net.edge_mass_kg["compressor_discharge"],
                         pump_balance_J=net.pump.audit()["mechanical_balance_residual_J"],
                         cabinet_temperature_K=net.thermal["cabinet"].temperature_K,
                         switch_edges=switch_edges)), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=list(CASES)+["all"], default="charge")
    args = parser.parse_args()
    for name in CASES if args.case == "all" else [args.case]:
        run(name)
