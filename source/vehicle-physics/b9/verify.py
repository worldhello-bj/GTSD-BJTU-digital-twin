"""Scoped B9 acceptance; retain the B8 mechanical/gas thresholds verbatim."""
import ast
import hashlib
import json
from pathlib import Path
import numpy as np
from simulate import CASES

ROOT = Path(__file__).resolve().parent
OUT = ROOT/"results"


def main():
    checks, metrics, data, spools = [], {}, {}, {}
    def check(name, ok, detail):
        checks.append(dict(name=name, passed=bool(ok), detail=detail))
    for report in ("verification", "regression"):
        j = json.loads((ROOT/"component_results"/(report+".json")).read_text(encoding="utf-8"))
        check("component_"+report, j["passed"], dict(tests_run=j["tests_run"]))
    for name in CASES:
        path = OUT/(name+"_metrics.json")
        if not path.exists():
            check("complete_"+name, False, "missing full settled case"); continue
        m = json.loads(path.read_text(encoding="utf-8")); metrics[name] = m
        a = np.genfromtxt(OUT/(name+".csv"), names=True, delimiter=","); data[name] = a
        trace = np.genfromtxt(OUT/(name+"_spools.csv"), names=True, delimiter=","); spools[name] = trace
        check("topology_"+name, m["dofs"] == 77 and m["spool_audit"]["port_count"] == 51 and len(m["pneumatic_configuration"]["volumes"]) == 26,
              dict(mechanical_coordinates=m["dofs"], gas_mass_states=26, metering_ports=m["spool_audit"]["port_count"], extra_spool_states=102))
        check("finite_"+name, all(np.isfinite(a[k]).all() for k in a.dtype.names) and all(np.isfinite(trace[k]).all() for k in trace.dtype.names), "CSV physical samples finite")
        check("solver_"+name, not any(m["warning_counts"]), m["warning_counts"])
        for label, column, limit in (("mechanical_work", "mechanical_balance_residual_J", .15),
                                      ("gas_work", "gas_energy_residual_J", 1e-5),
                                      ("coupling_work", "coupling_work_discrepancy_J", .05),
                                      ("momentum", "momentum_balance_residual_Ns", .05)):
            peak = float(max(abs(a[column])))
            check(label+"_"+name, peak < limit, dict(maximum=peak, unchanged_B8_limit=limit))
        check("gas_mass_"+name, m["max_air_mass_residual_kg"] < 1e-10, m["max_air_mass_residual_kg"])
        check("contact_passivity_"+name, max(a["contact_friction_work_J"]) < 1e-8 and max(a["contact_dissipation_work_J"]) < 1e-8, "same B8 passive contact criterion")
        check("spool_energy_"+name, m["max_sampled_spool_balance_residual_J"] < 1e-7,
              dict(maximum_J=m["max_sampled_spool_balance_residual_J"], limit_J=1e-7))
        check("spool_flow_bounds_"+name, all(0 <= s["opening"] <= 1 for s in m["final_spool_states"].values()), "bounded actual metering areas")
        identity = json.loads((OUT/(name+"_identity.json")).read_text(encoding="utf-8"))
        for fname, expected in identity["outputs_sha256"].items():
            check("output_hash_"+name+"_"+fname, hashlib.sha256((OUT/fname).read_bytes()).hexdigest() == expected, "exact output association")
        for version in ("b8", "b9"):
            folder = ROOT.parent/version
            for fname, expected in identity[version+"_sources_sha256"].items():
                src = ROOT.parent/fname if fname == "build_model.py" else folder/fname
                check("source_hash_"+name+"_"+fname, hashlib.sha256(src.read_bytes()).hexdigest() == expected, "actual executed force/flow source")
        xb = (OUT/(name+".xml")).read_bytes()
        j = json.loads((OUT/(name+"_replay.json")).read_text(encoding="utf-8"))
        check("xml_replay_identity_"+name, hashlib.sha256(xb).hexdigest() == m["model_xml_sha256"] == j["model_xml_sha256"], m["model_xml_sha256"])
        step_count = round(m["duration_s"]/m["dt_s"])
        replay_stride = max(1, round((1/30)/m["dt_s"]))
        expected_frames = step_count//replay_stride+1+int(step_count % replay_stride != 0)
        check("full_replay_"+name, len(j["frames"]) == expected_frames and len(j["reference_zero_qpos_body_transforms"]) == 44 and abs(j["frames"][-1]["time_s"]-m["duration_s"]) < 1e-8,
              dict(frames=len(j["frames"]), expected_frames=expected_frames, rigid_bodies=len(j["reference_zero_qpos_body_transforms"]), rule="whole physics-step replay stride plus exact final endpoint"))
    if len(metrics) == len(CASES):
        m, h, fault, slow = [metrics[n] for n in ("service_brake", "half_dt", "brake_coil_failure", "slow_brake_valves")]
        check("normal_pressure_braking_stops", m["stop_distance_m"] is not None and abs(m["final_speed_mps"]) < .01, dict(stop_distance_m=m["stop_distance_m"], stopped_after_brake_s=m["stopped_after_brake_s"]))
        if m["stop_distance_m"] and h["stop_distance_m"]:
            diff = abs(h["stop_distance_m"]/m["stop_distance_m"]-1)
            check("halfstep_stop_distance", diff < .025, dict(relative_change=diff, unchanged_B8_limit=.025))
        else:
            check("halfstep_stop_distance", False, "missing sustained stop")
        check("halfstep_collector_peak", abs(h["peak_collector_single_contact_N"]/m["peak_collector_single_contact_N"]-1) < .1,
              dict(main_N=m["peak_collector_single_contact_N"], half_N=h["peak_collector_single_contact_N"], unchanged_B8_limit=.1))
        # Hardware-fault effects are outputs, never scheduled kinematic stops.
        check("supply_coil_failure_prevents_stop", fault["stop_distance_m"] is None and fault["final_speed_mps"] > .15,
              dict(final_speed_mps=fault["final_speed_mps"], brake_pressure_Pa=fault["final"]["brake_pressure_Pa"]))
        check("failed_supply_spools_stay_closed", all(s["coil_failed"] and s["opening"] == 0 for n,s in fault["final_spool_states"].items() if n.endswith("_pad_supply")), "all 16 brake supply proxy coils fail")
        check("slow_valves_increase_braking_distance", slow["stop_distance_m"] is not None and slow["stop_distance_m"] > m["stop_distance_m"], dict(normal_m=m["stop_distance_m"], slow_m=slow["stop_distance_m"]))
        q = np.argmin(abs(data["service_brake"]["time_s"]-8.3))
        normal = data["normal_exhaust"]
        nn = np.argmin(abs(normal["time_s"]-8.3))
        check("quick_exhaust_remains_faster", data["service_brake"]["pantograph_pressure_Pa"][q] < normal["pantograph_pressure_Pa"][nn],
              dict(quick_Pa=float(data["service_brake"]["pantograph_pressure_Pa"][q]), normal_Pa=float(normal["pantograph_pressure_Pa"][nn])))
    frozen = json.loads((ROOT.parent/"b8/results/physics_run_identity.json").read_text(encoding="utf-8"))
    for name, expected in frozen["source_files"].items():
        check("frozen_b8_"+name, hashlib.sha256((ROOT.parent/"b8"/name).read_bytes()).hexdigest() == expected, "frozen B8 source identity retained")
    tree = ast.parse((ROOT/"simulate.py").read_text(encoding="utf-8"))
    writes = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            writes += [ast.unparse(t) for t in targets if "qpos" in ast.unparse(t) or "qvel" in ast.unparse(t)]
    check("no_b9_vehicle_state_overrides", not writes, writes)
    passed = bool(len(metrics) == len(CASES) and all(c["passed"] for c in checks))
    report = dict(status="PASS_WITH_MODEL_LIMITATIONS" if passed else "INCOMPLETE_OR_FAILED", passed=passed,
                  integrated_cases=len(metrics), component_tests=19, regression_tests=35,
                  checks=checks, scope="5 settled B9 valve-coupling cases only; B8 frozen 22 cases are historical, not rerun B9 acceptance",
                  limitations=["All added valve masses, strokes, forces, damping and land widths are uncalibrated.",
                               "51 independent balanced metering-port proxies are a numerical topology, not 51 real hardware valves.",
                               "Valve mount reactions and pressure imbalance/swept spool volumes are outside this balanced external-bench model.",
                               "No new source valve-internal mesh motion is asserted; rigid-body replay uses the frozen B8 adapter.",
                               "Soft cables/hoses, compressor crank/piston and fan impellers remain unphysicalized.",
                               "No B9 low-adhesion, full 22-case, hardware calibration or safety acceptance claim."])
    (OUT/"verification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(dict(status=report["status"], checks=len(checks), failed=[c for c in checks if not c["passed"]]), indent=2))
    return passed


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
