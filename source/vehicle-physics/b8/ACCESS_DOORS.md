# B8 six source-correct, force-driven access doors

## Scope and provenance

This module adds exactly six independent access-door hinge DOFs: four inspection
leaves on source car A and two doors on world-fixed PWR/DAQ cabinets. It moves all
84 audited source parts through their owning dynamic body: 44 car-door parts and
40 cabinet-door parts. It does not turn component covers, pressure sensors, source
handles, or the legacy `ADD_Valve` bone into extra moving mechanisms.

Geometry is taken from the read-only source Blender file with SHA-256
`c96811c45220dcdfad0853b73e4743297be7e090d4fb3230a7531eb89b3a4c04`.
The snapshot `door_results/source_door_closed_rest.json` was evaluated at frame 1
with `cabinet_open_deg=0`, rather than the source file's saved 100-degree inspection
pose. It records every leaf part, world bounds, and hinge-local transform.

Relevant source code:

- `refine_b1.py`, `door_bones()` and `shell()`: four source-Z hinges and leaf ownership
- `detail_internals_b5.py`, `cabinet_shell()`: two real cabinet pivots and their 20 child parts each
- `source_motion_inventory.json`: primary source travel 0 to -1.31 rad; opposite leaves originally mirrored by a driver
- `source_mechanism_coverage.json`: mechanism inventory used to avoid inventing moving covers or an ADD valve stem

The primary source door animation and secondary mirror drivers are replaced by
independent physical hinges. Their old timing is not replayed and does not command
angles. The closing catch described below is an explicit engineering assumption,
not a claim that the source contains working latch internals.

## Composition API

```python
from access_doors import add_access_doors

spec = add_access_doors(
    cars['A'], worldbody, actuators,
    source_x_shift=3.0,
    equality=equality_section,
)
# Explicit allocation, only if car A still contains the original full aggregate.
spec.apply_car_mass_allocation(cars['A'])
# Then compile the complete model.
bound = spec.bind(model)

# Keep default latches closed during ordinary train manoeuvres.
# Explicit unlock followed by a hand-torque input opens a selected leaf.
bound.release_latch(data, ['bay_L'])
bound.set_opening_torques(data, {'bay_L': 1.5})
mujoco.mj_step(model, data)

# Release torque: the real spring/damper/friction dynamics close the door.
bound.release(data)
# Attempt catch engagement only after actual physical return.
result = bound.try_latch(data, ['bay_L'])

mujoco.mj_forward(model, data)  # refresh derived force values after stepping
angles = bound.angles(data)    # signed source-Z radians
forces = bound.forces(data)    # actuator / spring / viscous / constraint Nm
telemetry = bound.telemetry(data)
```

`set_torques(data, mapping)` takes signed source-Z torque in Nm.
`set_opening_torques` instead takes positive opening torque, applying each leaf's
source direction. A negative value assists closing. Omitted keys release to zero.
Neither method unlocks a catch implicitly. Inputs are finite-checked and clamped;
MuJoCo independently limits motor control and motor force. The six motors have no
position/velocity bias, activation trajectory, or servo feedback.

Both bound control methods and catch methods never write `qpos` or `qvel`.
Catch methods change only `eq_active`. No time-based opening/closing law exists in
the production module. Timing in the standalone test is an external torque-input
experiment, not a motion prescription.

If the optional `equality` section is omitted, no catch is created and the spec
explicitly reports spring/friction-only closure. Whole-vehicle integration should
supply the section so the default six catches remain engaged during manoeuvres.

## Exact source-to-physics frames

All six zero-pose body axes match source world axes. Their positive hinge axis is
source/world +Z. Car-root coordinates are `(1.3, 0, 1.095)` before registration.
The entire laboratory, including stationary cabinets, receives source X + 3 m.
Audit floats are embedded without rounding in `SOURCE_LEAVES`; the following
readable coordinates are rounded.

| Key | Source owner | Source hinge m | MJCF opening range rad |
|---|---|---|---|
| bay_L | BayDoor_L | (2.52, .479, 1) | [-1.31, 0] |
| bay_R | BayDoor_R | (.08, -.479, 1) | [-1.31, 0] |
| bay_L_secondary | BayDoor_L_Secondary | (.08, .479, 1) | [0, 1.31] |
| bay_R_secondary | BayDoor_R_Secondary | (2.52, -.479, 1) | [0, 1.31] |
| cabinet_PWR | B5_PWR 01_DoorPivot | (2.264, 2.719, .05) | [-1.919862177, 0] |
| cabinet_DAQ | B5_DAQ 01_DoorPivot | (3.284, 2.719, .05) | [-1.919862177, 0] |

Body names are `access_door_` plus key. Joint names append `_hinge`; torque
actuators append `_torque`. Optional catch constraints append `_ideal_closed_latch`.

Car leaf bodies are direct children of car A; their local position subtracts
`SOURCE_CAR_CENTER` from the source hinge. Their full moving mass reacts on car A.
Cabinet leaves are at local `(0,0,0)` inside respective
`access_door_cabinet_PWR_fixed_support` and
`access_door_cabinet_DAQ_fixed_support`. The fixed support world origins are the
source hinges translated +3 X. Cabinet supports are infinite world attachments;
there is no cabinet chassis vibration or overturning model.

`spec.source_body_mapping()` is the exact object-name-to-body-name map. Every
source object's rest geometry can be transformed into hinge coordinates using the
source hinge matrix inverse, then driven by the solver's absolute body matrix.
This includes the fan/frame/grille/bolts owned by `bay_R_secondary`.

## Mechanical assumptions and declared limits

The source contains geometry and kinematic intent, not measured masses, material
densities, inertia, spring constants, friction, latch compliance, or hand torque.
The following are explicit, replaceable assumptions:

| Property | Ordinary bay leaf | Bay fan leaf | Each cabinet door |
|---|---:|---:|---:|
| Effective mass kg | 1.2 | 1.4 | 6.0 |
| Torsion stiffness Nm/rad | .35 | .35 | 1.0 |
| Viscous damping Nms/rad | .32 | .32 | 1.1 |
| Dry hinge friction Nm | .004 | .004 | .015 |
| Maximum hand torque magnitude Nm | 1.5 | 1.5 | 5.0 |

Each leaf's COM is the midpoint of its complete audited closed-rest AABB. Its
positive finite COM inertia is that of the assigned mass uniformly distributed
inside that AABB: `Ixx=m(dy²+dz²)/12`, and cyclic permutations. This is an effective
rigid-leaf inertia model, not a density-integrated mesh mass estimate. The fan leaf
uses its larger true envelope, including its inward projection. Cabinet door
bounds include all fittings, not the closed shell alone.

The spring reference lies 0.04 rad beyond the closed stop, providing slight
closing preload. Dry friction and viscous damping dissipate motion. The spring
alone is not represented as a positive latch.

When enabled, the ideal catch is a scalar joint equality `q=0`, initially active.
It supplies a constraint reaction and does not command a trajectory. Unlocking is
explicit. Relatching is accepted only when the actual absolute hinge angle is at
most 0.006 rad and absolute angular speed at most 0.03 rad/s. Attempts outside
that capture envelope fail without changing the state. Catch compliance is finite
numerically; there is no independently modeled handle, spindle, pawl, impact, or
strength/failure limit. This assumption prevents normally closed access doors
from swinging during ordinary train motion and must not be read as source-proven
hardware or a safety-certified interlock.

Both travel ends use actual unilateral MuJoCo joint-limit constraints. Finite
constraint compliance allows small measured penetration; no angle clamping or
velocity reset hides it. `solreflimit=(.002,1)` and
`solimplimit=(.99,.999,.001)` are solver/compliance assumptions. The catch uses the
same 0.002 s constraint time constant.

The MJCF box envelopes are noncolliding visual proxies. Real door collision with
cabinet contents, other leaves, people, or lab fixtures is not covered. The
physical obstruction in this module is the unilateral hinge travel stop. Source
meshes remain the display geometry, bound by the exact source part map.

## Car A mass allocation: no double counting

The original car A aggregate is assumed to include its four closed door leaves:
60 kg, COM at the root, COM tensor diagonal `(10.025,36.7625,35.7625)` kgm².
The four moving leaves total exactly 5 kg. Merely adding these bodies without
reallocating the fixed aggregate would incorrectly increase car A to 65 kg.

`spec.car_mass_allocation()` applies the parallel-axis theorem and preserves the
closed configuration's original total mass, first moment and full inertia tensor.
It returns every moving leaf's exact mass, COM and 3×3 tensors about its own COM
and the car origin, plus the complete residual-body allocation. Defaults yield:

- Residual fixed car-body mass: 55 kg
- Residual COM, car-local m: `(-0.00223817527294, 0.00135018169880, 0.00809091069482)`
- MJCF full inertia order xx yy zz xy xz yz, kgm²:
  `(8.71840782230284,34.0536608156353,32.1374928867809,-0.0458732273157,-0.0119518583268,0.00720997170086)`
- Residual principal inertias kgm²:
  `(8.71831867606980,32.1374716825123,34.0537711661369)`

The residual is positive definite and satisfies the rigid-body principal-moment
triangle inequalities. `apply_car_mass_allocation` is an explicit convenience
mutation of the supplied car `<inertial>` only; the builder itself does not hide
that mutation. It rejects an already-modified aggregate mass. For additional
mass-allocated subsystems, combine all subtractions against the same original
aggregate and retain all cross terms. For a loaded aggregate, supply its actual
original mass, COM and inertia to the allocation call. Fixed cabinet leaves are
never subtracted from car A.

The machine-readable complete values are in
`door_results/access_doors_spec.json` and `access_doors_results.json`.

## Reproducible verification

Run `python vehicle-physics/b8/test_access_doors.py` from the workspace, or
`python b8/test_access_doors.py` from `vehicle-physics`. The test loads the existing
vendored MuJoCo and performs no dependency installation.

Checks cover:

1. Exactly six hinge joints, six bounded torque actuators, six optional ideal catches, and the correct 84 source bindings
2. Exact rest hinge frames, positive finite moving inertias, source directions and angular ranges
3. Exact reconstruction of original car mass/COM/full inertia after allocation, and rejection of accidental double allocation
4. Unlocked, zero-input rest: no spontaneous opening
5. Bounded opening torque reaches the real stops, then zero torque permits spring-return closing
6. Mechanical stop reaction and measured finite penetration at both ends
7. Doubling mass and inertia changes the response under identical torque
8. Held-closed catches resist opening torque; open/fast relatch attempts fail; actual closed/slow relatch succeeds without a pose reset
9. Finite/bounded and atomic input validation, plus an AST check rejecting any generalized-state writes in the module
10. Free-car action/reaction and near-conservation of total car-subtree angular momentum
11. Full-cycle trajectory comparison at 0.1 ms and 0.05 ms timestep

Results are standalone mechanism validation, not evidence that the complete train
and its contacts have passed their separate regression suite. The half-timestep
comparison is made at the train's nominal 0.1 ms. A deliberately coarser 0.5/0.25 ms
pair produced up to 0.0241 rad trace disagreement around impacts and passive return;
that pair is not accepted as a converged door simulation.

### Recorded result (MuJoCo 3.3.7)

All checks pass on the recorded test run. Maximum open-stop penetration was
0.001775889 rad; maximum closed-stop penetration was 0.000399352 rad. The passive
return was within 1.4e-9 rad of the closed stop at the recorded end of release.
The initially engaged and re-engaged catches held maximum opening torque with
maximum excursion 1.72e-7 rad. No-input unlocked maximum opening was zero.
The largest 0.1/0.05 ms trace difference was 0.004704 rad, below the declared
0.005-rad acceptance threshold. Doubling every leaf's mass/inertia reduced each
0.4-second same-torque opening response to about 52–53% of baseline.

`door_results/access_doors_replay.json` contains 341 sampled, solver-generated
absolute body poses, with a SHA-256 binding to `access_doors_standalone.xml`.
Its body-transform format matches the full-vehicle replay, but its schema and
scope explicitly identify this fixed-car standalone fixture. It contains no
fabricated train movement. At about 4.0001 s all six leaves are physically at their
opening stops; at about 16.4001 s the springs have returned them closed. The
source-mesh visual adapter can use these poses to inspect actual door geometry,
without changing any physics state or replaying the source animation drivers.
