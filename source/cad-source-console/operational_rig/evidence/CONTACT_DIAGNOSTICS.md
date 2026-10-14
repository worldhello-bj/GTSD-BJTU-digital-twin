# V3 landing diagnosis and bounded trials

The V3 50/25 µs full protocol passes 12 of 14 gates; only wheel position and vertical-velocity convergence remain open. Every study below uses a declared common physical state at 1.10 s from the fine V3 trace and runs only to 1.20 s. Each case creates its own model identity, initializes qpos/qvel/drive time, rebuilds forward quantities and resets the energy reference. No foreign checkpoint is relabeled or restored.

## Excluded causes

- During the first upper-wing collision only one rail geom and one contact are active. There are no coincident contacts from different rail triangles.
- The reconstructed contact lies on the original upper-wing external top edge. Its source height is zero to 0.024 nm compiled precision, 1 mm above the artificial prism bottom. The two internal triangle edges are roughly 29 and 172 mm away.
- Cold-start Newton solves require at most 10 iterations. Raising the cap from 80 to 800 leaves acceleration exactly unchanged, and original contact forces reproduce within 2.43e-8 N.
- First-impact outgoing vertical velocities differ by about 0.0034 m/s. Subsequent free flight amplifies the position difference, shifting the next landing and its force/velocity peak.

These observations reject internal triangle multiplicity, prism depth and solver iteration exhaustion as the first-impact cause. An exact convex-union prototype preserves all seven source boundaries, but was not adopted as a remedy because the event does not touch its removed internal edges.

## Common-state numerical comparisons

| Method | Maximum wheel position difference | Maximum vertical velocity difference | Outcome |
|---|---:|---:|---|
| Original V3, implicitfast, 50/25 µs | 92.53 µm | 0.11408 m/s | Position and velocity fail |
| Double normal reference damping, same stiffness | 24.54 µm | 0.06582 m/s | Velocity fails |
| Same 384-sided mesh, square rim | 96.66 µm | 0.12398 m/s | Position and velocity fail |
| Same mesh resolution, 0.5 mm rim fillet | 52.03 µm | 0.07133 m/s | Position and velocity fail |
| Native RK4, original V3 geometry/law, 50/25 µs | 50.188 µm | 0.07170 m/s | Position and velocity fail |

No declared acceptance threshold was changed. The RK4 trial is a state-accuracy screen only: the existing last-stage contact ledger is not a four-stage quadrature and is not accepted as a complete RK4 energy/impulse validation.

## Damping trial definition

MuJoCo's positive solref format uses b = 2/(d_width t) and k = d(r)/(d_width² t² ζ²). Here k has reference-acceleration units s⁻² and b has units s⁻¹; they are not a separately measured material stiffness and dashpot. The model's contact inertia converts reference acceleration into the native constraint force.

With d_width = 0.99, changing (t, ζ) from (1.2 ms, 1) to (0.6 ms, 2) retains k/d(r) = 708544.4796 s⁻² and doubles b from 1683.5017 to 3367.0034 s⁻¹. Explicit wheel/rail pairs keep the tangential reference at (1.2 ms, 1); pin and wire laws are unchanged. Peak normal force rises from 1.40 to 4.58 kN and peak upward speed drops from 0.244 to 0.193 m/s. This trial was not adopted.

Reference: [MuJoCo 3.3.7 solver parameters](https://mujoco.readthedocs.io/en/3.3.7/modeling.html#solver-parameters).

## Fillet trial definition

The engineering wheel keeps a 45 mm outer radius, 12 mm total width and 1 kg mass. A 0.5 mm rim fillet changes the original square-rim boundary by at most 0.2071 mm. The complete closed mesh is used for display, collision and mass integration: 384 radial sectors and 16 arc segments per quadrant give 1.506 µm radial and 0.602 µm fillet discretization bounds. Independent source/compiled-vertex registration is within 2.613 nm and native inertia matches the integrated source tensor. Source track geometry and the V3 contact laws remain unchanged. This trial was not adopted.

## Final bounded refinement

The additional native RK4 12.5 µs case completed at 2026-10-14T03:59:56.562533Z (Beijing 11:59:56), using 105.54 wall seconds for the same 0.10 s window. Compared with the existing 25 µs case, all body positions and all 21 velocity DOF meet the original state tolerances. Maximum body-position difference is 15.271472 µm at 1.1714 s. Wheel vertical-velocity difference is 0.022148802 m/s, below its 0.024429740 m/s limit. There are no position or velocity exceedance intervals. After 1.18 s, maximum body-position difference is 1.681951 µm and wheel vertical-velocity difference is 0.000631233 m/s.

This is a successful state-accuracy screen, not a full-passage RK4 qualification. Every RK4 stage's native contact forces, work and impulses still require quadrature and independent verification against the official integrator. Production remains the unchanged V3 implicitfast model. None of the bounded prototype variants has been adopted.
