"""Measure native stepping cost without changing geometry chord tolerance."""
from dataclasses import replace
import json
import time
import mujoco
import cables


def main():
    rows = []
    for length in (.6, 1.0):
        cp = replace(cables.CableParameters(), maximum_link_length_m=length)
        m = mujoco.MjModel.from_xml_string(cables.model(cable_parameters=cp))
        d = mujoco.MjData(m)
        start = time.perf_counter(); mujoco.mj_step(m,d,nstep=100)
        rows.append(dict(max_link_length_m=length, chord_error_m=cp.chord_error_m,
                         links=cables.LAST_SPEC["total_links"], nv=m.nv,
                         runtime_100_native_steps_s=time.perf_counter()-start,
                         total_cable_mass_kg=cables.LAST_SPEC["total_cable_mass_kg"],
                         maximum_source_length_relative_change=max(abs(r["source_length_relative_change"]) for r in cables.LAST_SPEC["routes"])))
    (cables.ROOT/"resolution_benchmark.json").write_text(json.dumps(rows,indent=2),encoding="utf-8")
    print(json.dumps(rows,indent=2))


if __name__ == "__main__":
    main()
