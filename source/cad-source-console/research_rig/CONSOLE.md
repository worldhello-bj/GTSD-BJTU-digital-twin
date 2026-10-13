# Isolated finite-mount / pin-core console

From `source/cad-source-console`, with the repository's pinned MuJoCo 3.3.7 and
NumPy 2.2.6 environment active:

```sh
python research_rig/server.py --port 8175
```

Open `http://127.0.0.1:8175/research-rig` on the same computer. This server binds
only `127.0.0.1`; it does not create a remote tunnel, hardware connection, file
upload, or general filesystem route. It reuses the existing source ModelView
WebGL/Canvas renderer and fixed public source JSONs. Private photos are not served.

## Experiment workflow

1. Opening or refreshing the page only reads static source geometry and state.
   It does not import MuJoCo, initialize a session, step, or renew a run lease.
2. Choose a mode and explicitly acknowledge the uncalibrated assumptions.
   `mount_only` stays the initial mode. Selecting `pin_core_contact` visibly
   fills a zero-gravity, 50 µs, complete D4.98 geometric-clearance preset; inspect
   and edit its parameter JSON before initialization. The D5 option remains
   available and clearly labeled as an unqualified zero-clearance transient.
3. Initialize to obtain a new session ID and model identity. Geometry/parameter
   changes require a new session, which clears the prior saved checkpoints.
4. Select a rigid body and bounded world force at its center of mass, then step
   1/10/50 times or run 1–500 steps. A run freezes its input and ends automatically.
   Pause takes effect at a native-step boundary; closing a browser tab is not a
   guarantee of an immediate stop. The native experiment horizon is the smaller
   of 5 simulated seconds and 100,000 steps, matching the plant.
5. Save up to eight in-memory checkpoints. Restore/export only a known saved
   checkpoint in the current session. Export contains complete native state,
   energy/force ledgers and the research contract. No import route exists.

Every mutating command carries `server_generation_id`, `session_id`, `expected_revision`, and a bounded
`request_id`. Replaying the same request returns its original result; reusing an
ID with changed inputs or using a stale revision is rejected. IDs are never
silently evicted: after 1,024 accepted command IDs, restart the local server.
Numerical failures roll back the entire short-step/run transaction, including
native state and ledgers, and require reset or a saved checkpoint restore.

## Display and interpretation

- The three component branches and individual source meshes can be selected.
  Source vertices are transformed from the contract's reference pose to the
  solver's current wxyz rigid-body pose. No animation clock or motion multiplier
  is used. Missing/mismatched source bindings fail closed.
- `mount_only`: 33 source meshes are bound to the finite-mount plant; the two
  F643 carriers are identified static unbound background.
- `pin_core_contact`: 35 source meshes are bound. Only the two named pin/B535
  bore/F643 slot interfaces have native contact ownership. Complete D4.98 display,
  collision and inertia geometry share their new identity.
- The selected-mesh panel shows geometry identity, source metadata, body, mass,
  mount generalized reactions, native-contact coverage and unresolved interfaces.
  Historical metadata is source provenance; the separate active solver contract
  determines this session's physics coverage.
- Neither mode asserts as-built assembly or whole-machine contact acceptance.
  The historical combined geometry is separate from the new photo-layered branch.

## Verification

```sh
python -m unittest discover -s research_rig/tests -p 'test_session.py' -v
python -m unittest discover -s research_rig/tests -p 'test_server.py' -v
node --test research_rig/tests/test_ui_*.js
```

These cover real native initialization/short steps for both contact variants and
mount-only, source identities, checkpoint replay, idempotency, stale controls,
full-transaction rollback, bounded run/pause, no native work on GET, strict HTTP
Host/Origin/body handling, and UI repeated/interrupted/navigation flows.

DOM-contract and transformation tests are not browser visual acceptance. In the
current cloud environment the official cloud browser could not reach the executor
loopback (`ERR_CONNECTION_REFUSED`). Same-process HTTP checks are supported and
passed; real browser WebGL/Canvas interaction remains unverified. No sandbox or
network restriction was bypassed.

After a server restart, Refresh adopts the new server generation atomically; late
responses from the retired generation cannot overwrite the new session or geometry.
