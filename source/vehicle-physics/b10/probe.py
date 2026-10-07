"""High-rate solved startup poses for a clearly labelled inferred bench scene."""
import csv
import json

from auxiliaries import ROOT, AMBIENT_PA, make_vehicle_network
from simulate import flatten_auxiliary, sha


def main():
    net = make_vehicle_network([], include_pantograph=False, reservoir_initial_pressure_Pa=AMBIENT_PA)
    rows = [flatten_auxiliary(net)]
    for k in range(2500):
        net.update(.0002, snapshot=False)
        if (k+1) % 5 == 0:
            rows.append(flatten_auxiliary(net))
    folder = ROOT/"replay/inputs"; folder.mkdir(parents=True, exist_ok=True)
    path = folder/"auxiliary_startup.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    identity = dict(source_sha256={n:sha(ROOT/n) for n in ("auxiliaries.py", "probe.py", "simulate.py")},
                    output_sha256=sha(path), frames=len(rows), physical_step_s=.0002, sample_s=.001,
                    physical_duration_s=.5, display_fps=30, time_dilation=1/(30*.001),
                    configuration=net.configuration())
    (folder/"auxiliary_startup_identity.json").write_text(json.dumps(identity, indent=2), encoding="utf-8")
    print(json.dumps({k:v for k,v in identity.items() if k != "configuration"}, indent=2))


if __name__ == "__main__":
    main()
