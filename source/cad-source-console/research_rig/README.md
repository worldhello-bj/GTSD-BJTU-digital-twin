# Three-component finite-mount and pin-contact research rig

This is an independent MuJoCo model of source-registered support seats, balise
assemblies and catenary clamp candidates on one compliant rig. It preserves the
PR5 source geometry and has its own identity. It does not accept strict82/H1
checkpoints, and it does not certify the photographed machine as built.

## Run

From `source/cad-source-console`:

```sh
python prepare_assets.py
python research_rig/server.py --port 8175
```

Use the pinned `requirements.txt` environment (MuJoCo 3.3.7 / NumPy 2.2.6), then
open `http://127.0.0.1:8175/research-rig` on that same computer. See [CONSOLE.md](CONSOLE.md)
for bounded run/pause, loads, checkpoints, reinitialization and provenance controls.
The existing source assembly console remains a separate launch.

## Two explicit modes

| Mode | Source meshes | Native velocity DOF | Contacts |
|---|---:|---:|---|
| `mount_only` | 33 | 50 | None; native finite mount springs and damping |
| `pin_core_contact` | 35 | 56 | Two named pins against their B535 bores / F643 slots only |

Each complete source solid contributes mass once. Collision partitions contribute
zero mass. Source triangle integration supplies center of mass and full inertia;
balise assembly mass is a declared 0.6 kg hypothesis, and other components use a
declared 7850 kg/m³ density. The common rig is a declared 100 kg research support,
not an inferred vehicle mass. Stiffness, damping, friction and contact regularization
are adjustable research inputs, not measurements.

948 convex cells preserve four original plate/slot solids, including holes and
three tiny nonplanar wedges. Two full pin meshes use the same vertices for display,
contact and inertia. `nominal_d5` retains exact nominal slot tangency; its transient
response is not qualified. `research_d4p98` is a separately named **complete 4.98 mm
pin geometry hypothesis**, with visible clearance and a new source/body identity.
No collision-only shrinkage is used. It is not evidence of actual pin tolerance.

Gravity, body loads, spring/damping reactions and pin contact forces are computed
by MuJoCo. Applied world forces act at the actual body center of mass. The frame
transmits reactions across all three branches. Display follows the current native
body pose directly; tiny motion is not exaggerated.

## Validation and protocol versions

```sh
python -m unittest discover -s research_rig/tests -v
node --test research_rig/tests/test_ui_*.js
python research_rig/validate_mount_stage.py --output /tmp/gtsd-mount-check
python research_rig/validate_contact_stage.py --output /tmp/gtsd-contact-check
python research_rig/validate_gravity_stage.py --output /tmp/gtsd-gravity-check
```

The contact V2 default compares 100 and 50 µs over 150 ms, with a 1 N smooth
50 ms axial pulse at each station. It reports all-body displacement/rotation,
per-DOF velocity, signed reaction impulse, raw and 1 ms averaged force peaks,
contact onset, penetration, energy balance, native/generalized contact-work
closure and exact checkpoint continuation. Velocity tolerance is explicitly the
larger of 1e-4 (m/s or rad/s per DOF) and 2% of that DOF's peak magnitude.
The earlier V1 strict absolute-only failure and coarser 200/100 µs failures are
retained as historical evidence. V2 does not relabel them as passed.

The gravity check uses a separate declared 400 kN/m rig translation stiffness,
50 µs steps and a two-second maximum preparation. It requires every DOF to remain
below the stated speed and force-imbalance thresholds continuously for 0.1 s.
Its energy residual is reported separately; the zero-gravity pulse tolerance is
not copied onto this loaded settling case.

Each run writes full per-step output and an identity-bound checkpoint. Public
release evidence contains compact per-step ledgers, sampled poses and reports;
full diagnostic records are retained separately. Re-running the scripts produces
those complete records again. UTC and Beijing completion times are recorded.

Optional partition rebuild requires NumPy, SciPy and GEOS (with constrained
Delaunay support). The successful rebuild used NumPy 2.3.5, SciPy 1.17.0 and
GEOS 3.13.1; these optional builder dependencies are separate from the pinned
MuJoCo runtime. It is not needed to run the included exact partition:

```sh
python research_rig/contact_builder/build_collision.py --source . --output /tmp/gtsd-partition-rebuild
```

## Acceptance boundary

- Same-process real HTTP/native initialization, short stepping and source binding
  checks are valid. DOM tests cover interrupted/repeated controls.
- The cloud browser cannot reach executor loopback in this environment. Actual
  WebGL/Canvas browser interaction and browser-download completion remain unverified.
- The new photograph-led frog/wing/plate model is a separately selectable static
  source candidate. It is not silently substituted into this rig's tested masses
  or collision identity.
- Rail/seat, balise/environment, 4079/wire, tube/ring and lower-jaw/wire contact are
  outside this release's contact admission. Finite mounts are explicit equivalent
  constraints, not a claim to reproduce their unmeasured fastenings.
- No hardware connection or public server deployment is created.

See [DATA_BOUNDARY.md](DATA_BOUNDARY.md) for the remaining specific dimensions,
physical measurements and separately executable software work.
