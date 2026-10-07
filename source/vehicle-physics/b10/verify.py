"""B10 acceptance with unchanged B8 train thresholds and independent bench gates."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

from auxiliaries import ROOT, B9
from simulate import CASES, B8
from bench import CASES as BENCH_CASES


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        names = reader.fieldnames
        rows = [tuple(1.0 if row[n] == "True" else 0.0 if row[n] == "False" else float(row[n])
                      for n in names) for row in reader]
    return np.array(rows, dtype=[(n, "f8") for n in names])


def main():
    checks, train, benches, arrays = [], {}, {}, {}
    def check(name, ok, detail):
        checks.append(dict(name=name, passed=bool(ok), detail=detail))
    def peak(name, data, column, limit):
        value = float(max(abs(data[column])))
        check(name, value < limit, dict(maximum=value, limit=limit, column=column))
    def identity(folder, name):
        j = json.loads((folder/(name+"_identity.json")).read_text(encoding="utf-8"))
        for file, expected in j["outputs_sha256"].items():
            check("output_"+name+"_"+file, hashlib.sha256((folder/file).read_bytes()).hexdigest() == expected, "exact executed output")
        for version, sources in j["sources_sha256"].items():
            directory = ROOT.parent if version == "parent" else ROOT.parent/version
            for file, expected in sources.items():
                check("source_"+name+"_"+version+"_"+file, hashlib.sha256((directory/file).read_bytes()).hexdigest() == expected, "actual executed source")
    for name in ("components", "regression"):
        j = json.loads((ROOT/"component_results"/(name+".json")).read_text(encoding="utf-8"))
        check(name, j["passed"], dict(tests=j["tests_run"]))
    for name in CASES:
        folder = ROOT/"results"
        path = folder/(name+"_metrics.json")
        if not path.exists():
            check("complete_"+name, False, "missing settled full case"); continue
        m = json.loads(path.read_text(encoding="utf-8")); train[name] = m
        a = read_csv(folder/(name+".csv"))
        trace = read_csv(folder/(name+"_auxiliaries.csv"))
        check("finite_"+name, all(np.isfinite(a[c]).all() for c in a.dtype.names) and all(np.isfinite(trace[c]).all() for c in trace.dtype.names), "all physical samples")
        check("topology_"+name, m["dofs"] == 77 and len(m["pneumatic_configuration"]["volumes"]) == 27
              and m["spool_audit"]["port_count"] == 51 and not m["pneumatic_configuration"]["compressors"],
              "77 vehicle coordinates, 27 gas masses, 51 B9 ports, no algebraic compressor")
        check("warnings_"+name, not any(m["warning_counts"]), m["warning_counts"])
        check("full_window_"+name, m["duration_s"] >= 10 and abs(trace["network_time_s"][-1]-16) < 1e-8,
              "6 s natural settling + 10 s observation")
        for label, column, limit in (("mechanical", "mechanical_balance_residual_J", .15),
                                     ("gas", "gas_energy_residual_J", 1e-5),
                                     ("vehicle_coupling", "vehicle_coupling_work_discrepancy_J", .05),
                                     ("momentum", "momentum_balance_residual_Ns", .05)):
            peak(label+"_"+name, a, column, limit)
        peak("mass_"+name, a, "air_mass_residual_kg", 1e-10)
        peak("spool_"+name, trace, "spool_balance_residual_J", 1e-7)
        for obj in ("pump", "compressor_fan", "bay_fan"):
            peak("mechanical_"+obj+"_"+name, trace, obj+"_mechanical_balance_residual_J", 1e-6)
            peak("electrical_"+obj+"_"+name, trace, obj+"_electrical_balance_residual_J", 1e-6)
        peak("pump_geometry_"+name, trace, "pump_gas_geometry_work_discrepancy_J", 1e-6)
        for obj in ("compressor_motor", "cabinet"):
            peak("thermal_"+obj+"_"+name, trace, obj+"_balance_residual_J", 1e-5)
        check("contact_passivity_"+name, max(a["contact_friction_work_J"]) < 1e-8 and max(a["contact_dissipation_work_J"]) < 1e-8, "unchanged B8 criterion")
        replay = json.loads((folder/(name+"_replay.json")).read_text(encoding="utf-8"))
        steps = round(m["duration_s"]/m["dt_s"]); stride = max(1, round((1/30)/m["dt_s"]))
        count = steps//stride+1+int(steps % stride != 0)
        check("replay_"+name, len(replay["frames"]) == count and len(replay["reference_zero_qpos_body_transforms"]) == 44,
              dict(frames=len(replay["frames"]), expected=count, bodies=44))
        check("xml_"+name, hashlib.sha256((folder/(name+".xml")).read_bytes()).hexdigest() == m["model_xml_sha256"] == replay["model_xml_sha256"], "exact XML association")
        identity(folder, name)
    for name in BENCH_CASES:
        folder = ROOT/"bench_results"
        path = folder/(name+"_metrics.json")
        if not path.exists():
            check("bench_complete_"+name, False, "missing 120 s bench"); continue
        m = json.loads(path.read_text(encoding="utf-8")); benches[name] = m
        a = read_csv(folder/(name+".csv")); arrays[name] = a
        check("bench_window_"+name, m["duration_s"] == 120 and abs(a["network_time_s"][-1]-120) < 1e-6, "120 s")
        check("bench_finite_"+name, all(np.isfinite(a[c]).all() for c in a.dtype.names) and min(a["pump_cylinder_pressure_Pa"]) > 0, "finite samples, positive cylinder pressure")
        for key, limit in (("mass_balance_residual_kg", 1e-10), ("gas_energy_balance_residual_J", 1e-5), ("exergy_balance_residual_J", 1e-5)):
            check("bench_"+key+"_"+name, abs(m["gas_audit"][key]) < limit, dict(value=m["gas_audit"][key], limit=limit))
        for obj in ("pump", "compressor_fan", "bay_fan"):
            peak("bench_mechanical_"+obj+"_"+name, a, obj+"_mechanical_balance_residual_J", 1e-6)
            peak("bench_electrical_"+obj+"_"+name, a, obj+"_electrical_balance_residual_J", 1e-6)
        peak("bench_geometry_"+name, a, "pump_gas_geometry_work_discrepancy_J", 1e-6)
        for obj in ("compressor_motor", "cabinet"):
            peak("bench_thermal_"+obj+"_"+name, a, obj+"_balance_residual_J", 1e-5)
        identity(folder, name)
    if len(train) == len(CASES):
        normal, half, fault = (train[n] for n in ("service_brake", "half_dt", "compressor_power_failure"))
        check("sustained_stop", normal["stop_distance_m"] is not None and abs(normal["final_speed_mps"]) < .01, normal["stop_distance_m"])
        diff = abs(half["stop_distance_m"]/normal["stop_distance_m"]-1) if half["stop_distance_m"] and normal["stop_distance_m"] else float("inf")
        check("train_half_step", diff < .025, dict(relative_change=diff, unchanged_B8_limit=.025))
        check("collector_half_step", abs(half["peak_collector_single_contact_N"]/normal["peak_collector_single_contact_N"]-1) < .1,
              dict(main_N=normal["peak_collector_single_contact_N"], half_N=half["peak_collector_single_contact_N"], unchanged_B8_limit=.1))
        check("pump_failure_reduces_reservoir_pressure", normal["final"]["reservoir_pressure_Pa"] > fault["final"]["reservoir_pressure_Pa"]+1000,
              dict(normal_Pa=normal["final"]["reservoir_pressure_Pa"], fault_Pa=fault["final"]["reservoir_pressure_Pa"]))
        check("failed_pump_no_electrical_input", fault["auxiliary_final_state"]["pump"]["electrical_work_J"] == 0, "open circuit from startup")
    if len(benches) == len(BENCH_CASES):
        normal, half, fault, block, fanfault, switch = (benches[n] for n in BENCH_CASES)
        final = normal["final"]
        diff = abs(half["final"]["delivered_air_kg"]/final["delivered_air_kg"]-1)
        check("bench_half_step_mass", diff < .025, dict(relative_change=diff, limit=.025))
        check("bench_compression_from_ambient", final["reservoir_pressure_Pa"] > 300000 and final["delivered_air_kg"] > .001, "no prescribed gas pressure or flow")
        check("blocked_outlet_no_delivery", block["final"]["delivered_air_kg"] == 0 and block["final"]["reservoir_pressure_Pa"] == 101325, block["final"]["reservoir_pressure_Pa"])
        check("power_failure_limits_delivery", fault["final"]["delivered_air_kg"] < final["delivered_air_kg"]/4, fault["final"]["delivered_air_kg"])
        a = arrays["power_loss"]; boundary = np.argmin(abs(a["network_time_s"]-10))
        check("fault_cuts_electrical_work", abs(a["pump_electrical_work_J"][-1]-a["pump_electrical_work_J"][boundary]) < 1e-6,
              "state coasts after open circuit, no continuing power input")
        check("fan_failure_reduces_cooling", fanfault["final"]["cabinet_temperature_K"] > final["cabinet_temperature_K"]+1,
              dict(normal_K=final["cabinet_temperature_K"], fault_K=fanfault["final"]["cabinet_temperature_K"]))
        check("pressure_switch_opens", any(not e["on"] for e in switch["switch_edges"]), switch["switch_edges"])
    frozen = json.loads((B8/"results/physics_run_identity.json").read_text(encoding="utf-8"))
    for name, expected in frozen["source_files"].items():
        check("frozen_b8_"+name, hashlib.sha256((B8/name).read_bytes()).hexdigest() == expected, "B8 untouched")
    b9 = json.loads((B9/"results/service_brake_identity.json").read_text(encoding="utf-8"))
    for name, expected in b9["b9_sources_sha256"].items():
        check("frozen_b9_"+name, hashlib.sha256((B9/name).read_bytes()).hexdigest() == expected, "B9 increment preserved")
    writes = []
    for path in (ROOT/"simulate.py", ROOT/"live.py", ROOT/"auxiliaries.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                writes += [ast.unparse(t) for t in targets if "qpos" in ast.unparse(t) or "qvel" in ast.unparse(t)]
    check("no_vehicle_state_override", not writes, writes)
    passed = len(train) == len(CASES) and len(benches) == len(BENCH_CASES) and all(c["passed"] for c in checks)
    report = dict(status="PASS_WITH_MODEL_LIMITATIONS" if passed else "INCOMPLETE_OR_FAILED", passed=passed,
                  integrated_cases=len(train), bench_cases=len(benches), component_tests=16, regression_tests=36,
                  checks=checks, scope="B10 3 full vehicle + 6 independent 120 s auxiliary cases",
                  limitations=["Inferred internal topology and uncalibrated parameters", "Fixed auxiliary bench; no vehicle mount reaction",
                               "Massless ideal check seats", "Isothermal gas and quasi-static electrical armature",
                               "Fan flow and cooling proxies, no device curves or CFD", "No physical cable/cover upgrade in B10",
                               "B8 historical low-adhesion/overload failures remain; no hardware certification"])
    (ROOT/"results/verification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(dict(status=report["status"], checks=len(checks), failed=[c for c in checks if not c["passed"]]), indent=2))
    return passed


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
