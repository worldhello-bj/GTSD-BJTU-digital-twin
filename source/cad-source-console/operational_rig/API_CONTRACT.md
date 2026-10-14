# Operational solver API

`build_rig.DEFAULTS` is read from the flat JSON object in `parameters.json`.
`parameter_sources.json` contains a row for every default key, with a Chinese label, actual units, classification and the specific parameter choice.

`build_rig.load_geometry(parameters=None, root=SOURCE_ROOT, contract=None)` returns:
- `schema`, `units: "m"`, `source_profile`, `source_sha256`
- `meshes[]` with `name`, `id`, `branch`, complete world-coordinate `vertices` and `indices`, `geometry_sha256`, `body_name`, `material_id`, `geometry_role`, `mass_model`
- Visual-only rail caps additionally have `source_alias_of` and `alias_hidden_by_default: true`. These still have source/body bindings, but contribute neither mass nor collision.

`Plant(parameters=None, root=None)` provides the existing session protocol:
- `contract`, `identity`, `body_ids`, `steps`
- `step(loads=None) -> trace`, `snapshot() -> state`
- `checkpoint()`, `restore(checkpoint)`
- `demo_loads(time_s) -> {body_name: [world_Fx, world_Fy, world_Fz]}`

Every native step advances the same declared wheel-drive program using simulation time. `demo_loads` supplies the optional three-branch pulse program. A manual step still advances the wheel drive; `wheel_drive_target_m_s` reports its current command. After the declared duration its command is zero.

## Identity and body mapping

`contract.identity_sha256` identifies the complete assembly construction, source geometry, parameters and builder code. `Plant.identity` and `state.identity_sha256` additionally bind that assembly to the runner code, MuJoCo version, native state specification and dimensions. They are intentionally separate hashes.

For a saved protocol case, `summary.identity_sha256` matches every saved state/checkpoint runtime identity, and `summary.assembly_identity_sha256` matches the assembly contract identity.

Every `contract.bindings[]` row has:
- `mesh_name`, `body_name`, `branch`, `source_geometry_sha256`
- `body_reference_position_m`, `body_reference_quaternion_wxyz`
- `rigid_group_hypothesis`, `geometry_role`, `mass_model`, `mass_contribution_kg`

Apply the actual body transform to the source geometry relative to the stored reference pose. No displacement multiplier is applied.

## Native snapshot

The existing fields remain: `identity_sha256`, `steps`, `time_s`, `qpos`, `qvel`, `body_poses`, `mount_generalized_forces`, `energy`, `ledger`, `energy_balance_residual_J`, `native_contact_count`, `minimum_contact_gap_m`.

Additional telemetry:
- `branch_response` is a dictionary keyed by `rig`, `track`, `foundation`, `balise`, `catenary`, `vehicle`; each row has `max_displacement_m` and `mean_displacement_m`.
- `contact_interfaces` is a dictionary keyed by `wheel_rail`, `pin_50DF4`, `pin_51409`, `messenger_jaw`; each row has `normal_force_N`, `tangential_force_N`, `normal_impulse_Ns`, `count`. Force is from the just-used native interval; impulse is cumulative.
- `wheel_position_world_m`, `wheel_track_relative_position_m`, `wheel_travel_m`, `wheel_vertical_drop_m`, `wheel_drive_target_m_s`.

`energy` contains `potential_J`, `kinetic_J`, `total_J`. `ledger` separately accumulates `input_work_J`, `actuator_work_J`, `known_damping_J`, `native_constraint_work_J`, and each interface's normal/tangential impulse and contact work. Gravitational potential and native joint spring energy are included in the MuJoCo potential-energy value.

`step` also returns `used_contacts`, exact applied body loads, drive target, work increments and `contact_generalized_work_closure_error_J`. A contact includes both geom names, interface ID, world point, signed gap, normal, force/torque on geom2, normal/tangential forces and interval work/impulses.

Checkpoints include all native integration state, integer step clock, ledger, command and last interval contact diagnostics. Foreign identities are rejected before replacing live state. Failed native steps restore the pre-step checkpoint.

## Collision admission and wheel geometry

`contract.contact_admission` describes the selected physical interfaces; it is not the numerical protocol result. `contract.wheel_representation` records the analytic cylinder and the inscribed 192-face display approximation, including the radial bound. The wheel's mass and inertia use the analytic cylinder.

The model identity binds `collision_guard.py`. Its model-specific dispatcher rejects only pin/cell pairs with a certified positive compiled-geometry axial gap above 1 pm plus roundoff. Native overlap contacts remain unchanged. The dispatcher has no integration-dependent state.
