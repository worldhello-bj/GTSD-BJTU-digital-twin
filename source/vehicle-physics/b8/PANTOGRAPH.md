# B8 source-consistent single-arm Z pantograph

## Status and source fidelity

The final `pantograph.py` follows the existing B2/B7 source's single-arm Z topology with parallel lower and upper balancing rods. The earlier diamond is retained separately as **DIAGNOSTIC ONLY** in `diamond_pantograph.py`, `test_diamond_pantograph.py`, `DIAGNOSTIC_ONLY_DIAMOND_PANTOGRAPH.md`, and `diamond_pantograph_results/`. It is not imported by the final module and is not the source vehicle.

Read source files:

- `blender-transfer/verified-source/blendermcp/config/pantograph_b2.json`
- `blender-transfer/verified-source/blendermcp/src/refine_pantograph_b2.py`
- `blender-transfer/verified-source/blendermcp/output/b2/raised_side.png`, visually inspected

The config explicitly labels its dimensions `UNVERIFIED_DIMENSION` and its motion illustrative kinematics. Its geometry is retained here; animation keyframes, expression drivers, and extension trajectories are not used by the physics solver.

Source SHA-256 values:

- Config: `848b5e97c3edb11372a4a74b5e4dcfa983942a3949fd7aaceafe0133f31f7a9b`
- Refinement script: `e3a320f0e8575e5104a812083bcef01fabc3e313244f2893a2154710b8235403`

**This is an uncalibrated force-driven prototype, not a validated railway model.** The source supplies geometry and a kinematic relationship, not masses, gas parameters, contact stiffness, or the actual concealed synchronization drive. The light moving masses below are explicitly chosen assumptions, not mass/density extraction from the source's visual meshes. They materially affect required pressure and contact load.

## Geometry retained from the source

The lower pivot is A. In its local x–z plane:

- Main lower and upper arm lengths L = 0.160 m each
- Balancing-rod root offset = +0.034 m in x
- Source balancing bars are at y = +0.091 m
- Lower main bars at y = ±0.052 m; upper main bars at y = ±0.026 m
- Folded / raised top pivot lift h = 0.055 / 0.220 m
- θ = asin(h / (2L)); elbow E = (L cos θ, 0, L sin θ)
- Top pivot H = (0, 0, 2L sin θ)
- Source lower pivot is 0.034 m above the base top
- Fixed cylinder pin G = (−0.085, −0.105, 0) m
- Moving cylinder pin P = (0.080 cos θ, −0.105, 0.080 sin θ) m
- Two carbon contact strips are centered at H + (0.017 ± 0.031, 0, 0.045) m
- Each strip is 0.020 × 0.360 × 0.009 m; the highest level surface is H.z + 0.0495 m

The strips therefore sit above the source's `minimum_lift_m` and `maximum_lift_m`, which refer to top pivot H, not the contact surface. This distinction matters when setting overhead height.

Capsules and boxes approximate the rigid visual members for standalone testing. The source assembly adapter can use the original meshes with these body frames. The mechanism's physical topology and pivots are preserved; no claim is made that these simple primitive visuals reproduce every source detail.

## Genuine linked dynamics, and the remaining idealization

Six real hinge joints connect the lower arm, elbow carrier, upper arm, head, lower balance rod, and upper balance rod. Two site-to-site loop-closure constraints close the lower and upper parallelograms. These loops keep the elbow carrier parallel to the base and the head parallel to the elbow carrier through physical constraint reactions. The head has no imposed world-level trajectory.

One **assumed passive ideal fold synchronizer** couples the upper-stage hinge to the lower-stage hinge with a −1 ratio. This preserves the equal-fold relationship in the source's analytical animation. The source does not show enough information to identify the actual inter-stage rod/gear/cable design, so this static mechanical constraint is explicitly an idealization. It is not a commanded angle, joint-position actuator, or time trajectory. Its precise real implementation still requires drawings or measurements.

Gravity, mass/inertia, return springs, bearing damping, pressure force, joint limits, and actual collector–overhead contact determine the motion. The dynamics loop never assigns qpos or collector height. Only isolated geometry verification and read-only rendering replay set recorded/consistent poses.

## Retraction chamber: correct actuator sign

The source's cylinder gets **shorter** when the pantograph rises. Using extension force here would reverse the physical action.

- Cylinder length ℓ = ||P − G|| = sqrt(0.080² + 0.085² + 2 × 0.080 × 0.085 cos θ)
- Folded reference ℓ₀ = 0.16438556878943447 m
- Positive pneumatic travel x = ℓ₀ − ℓ, the physical retraction
- Maximum source retraction = 0.011084859290655863 m
- dℓ/dθ = −0.080 × 0.085 sin θ / ℓ
- dx/dθ = −dℓ/dθ, positive while raising
- Pressure force F = (p_annular − p_ambient) × A_annular
- Assumed annular effective area A = 0.0002 m²
- MJCF spatial-tendon actuator `gear="-1"` maps positive F to shortening
- Cylinder mechanical power F dx/dt equals generalized actuator power

The chamber volume is V = Vdead + A x. The opposite chamber is treated as atmospheric. Area is an assumed annular area, not a full piston area. A small negative gauge force remains possible during transient expansion; the actuator accepts −200 to +200 N and does not clip it to zero. The cylinder is an ideal massless two-force member with geometry-dependent moment arm; barrel/piston inertia and seals are not modeled.

The 80 N/m spatial-tendon spring has rest length ℓ₀. Retraction compresses this bidirectional spring, causing a passive restoring force toward the folded position. A separate 0.03 N·m/rad lower-root spring also biases folding. A real return-spring arrangement and its coefficients remain unverified.

## Public API and mounting frames

The parent compiler must use radians. SI units throughout.

```python
from pantograph import add_pantograph, add_overhead_rail

spec = add_pantograph(parent_body, worldbody, equality, tendon, actuators,
                      prefix='panto', base_z=.609)
# base_z specifies LOWER PIVOT A in parent-local coordinates.
# Mount at a different x by passing an appropriately positioned parent body.
add_overhead_rail(worldbody, underside_z=1.93, center_x=5., half_length=5.)

mujoco.mj_forward(model, data)
panto = spec.bind(model)
x = panto.travel(data)      # l_folded - current physical cylinder length
xdot = panto.velocity(data) # negative physical tendon length rate
# Update the pneumatic gas model using measured x, then apply its force:
panto.set_force(data, chamber_gauge_Pa * .0002)
mujoco.mj_step(model, data)
```

The builder only appends the mechanism. The optional rail is separately created in world coordinates, with no prescribed motion. All collector collision geoms and the rail explicitly use collision bit 8 by default, independent of inherited B7 zero-collision defaults. `params={'collision_bit': ...}` can override it. Prefix namespaces every element.

Default names and helper fields:

- `panto_cylinder_force`: force actuator, positive N means retraction
- `panto_cylinder`: physical cylinder-length spatial tendon
- `panto_lower_hinge`: primary folding hinge
- `panto_collector`: body origin H, the source head pivot
- `panto_head_center`: center between the two carbon strips, at H + (0.017, 0, 0.045)
- `panto_collector_contact`, `panto_collector_contact_2`: actual contact strips
- `panto_lower_carrier_pin` / `panto_lower_balance_pin`: lower-loop endpoints
- `panto_upper_head_pin` / `panto_upper_balance_pin`: upper-loop endpoints
- Legacy `panto_left_top_pin` / `panto_right_top_pin` aliases now identify the upper-loop endpoints, not a diamond

`panto.contact_force(model,data)` sums both strips' actual normal contact forces. `panto.moment_arm(data)` reads the actual sparse force-transmission Jacobian for positive pneumatic travel. `qpos_address` and `qvel_address` remain distinct for fullcar free-joint compatibility. The unchanged `maximum_cylinder_travel_m` field is now maximum retraction. `folded_head_center_above_mount_m = 0.100 m` refers to the strip-center site; folded H is only 0.055 m above A.

Body frames for source mesh adaptation:

| Body | Origin | Local +x |
|---|---|---|
| panto_mount | A | Parent forward |
| panto_lower_arm | A | A → E |
| panto_elbow_carrier | E | Parallel to base |
| panto_upper_arm | E | E → H |
| panto_collector | H | Parallel to elbow/base |
| panto_lower_balance | A + (0.034, 0.091, 0) | Parallel to lower arm |
| panto_upper_balance | E + (0.034, 0.091, 0) | Parallel to upper arm |

All local y axes are transverse. The balancing-body origins include the source bars' +0.091 m y offset, whereas the Blender source bone roots put this offset in the mesh itself. The adapter must account for that difference.

## Assumed dynamics and boundaries

| Parameter | Assumption |
|---|---:|
| Each main rigid arm | 0.025 kg |
| Each balancing rod | 0.007 kg |
| Elbow carrier | 0.010 kg |
| Head assembly | 0.080 kg |
| Total moving mass | 0.154 kg |
| Fixed base/mount mass | 0.150 kg |
| Each hinge damping | 0.003 N·m·s/rad |
| Lower-root return spring | 0.03 N·m/rad |
| Cylinder-length return spring | 80 N/m |
| Annular chamber area | 0.0002 m² |
| Collector/rail sliding friction | 0.12 |
| Nominal / fine timestep | 0.0002 / 0.0001 s |
| Integrator / solver | implicitfast / Newton |

All masses, inertias, spring rates, damping, pressure/area, friction, and solver softness are **CALIBRATION_ASSUMPTION**. No density-derived source mass model, aerodynamic lift, electrical arcing, wear, structural elasticity, backlash, seal friction, cylinder inertial model, or distributed flexible catenary is claimed. The source's nearly collinear folded actuator has weak initial leverage, so mass assumptions strongly affect the pressure needed to start lifting. More substantial measured masses may require different operating pressure or the real mechanism's currently unidentified spring/transmission arrangement.

## Verification

Run from `vehicle-physics`:

```sh
PYTHONPATH=vendor python b8/test_pantograph.py
```

The current final-source test checks E/H positions, source horizontal leveling, both parallelogram closures, cylinder formula, finite-difference Jacobian, actuator sign, pressure-equivalent raising, vented dropping, finite contact, imposed external load, force-disturbed overhead, 140 N stress, and timestep halving. It records zero MuJoCo warnings in all five dynamic cases. The standalone input is explicitly a pressure-equivalent force diagnostic, not the integrated pneumatic chamber model.

At 100 N retraction force, with lower pivot A.z = 1 m and fixed rail underside 1.260 m:

- Strip center rises from settled 1.09981 m to 1.255514 m, approximately 155.7 mm lift
- Steady normal force is 10.7047 N
- Vented force returns the mechanism to the folded state with zero rail contact
- A −12 N collector load for 0.05 s causes ~20.95 mm head drop and physical contact recovery
- A separate −20 N load on a 0.5 kg, 6000 N/m, 35 N·s/m overhead support moves it ~4.69 mm; the collector responds without a prescribed trajectory
- Maximum closed-loop mismatch is below 36 μm
- Cylinder formula error is below 1e−15 m and actuator power mapping error below 1.5e−14 W

The fast 0.3 s diagnostic force ramp causes 214.35 N impact and 2.75 mm soft contact penetration. At 140 N stress the peak is 248.97 N with 3.19 mm penetration. These are material limitations of this uncalibrated soft-contact diagnostic, not acceptable-force certifications. Halving timestep changes peak force by approximately 0.617% / 0.737%, and nominal disturbance drop by 2 μm. The integrated finite-volume pneumatic ramp can produce a different impact. Exact results are in `pantograph_results/verification.json`.

Artifacts include the exact MJCF, CSV time histories, comparison JSON, `pantograph_force_contact.png`, and the visually inspected `pantograph_contact_sequence.png`. Final-source results are separate from the retired diamond diagnostics.

## Official simulator references

- [MuJoCo 3.3.7 equality connect](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#equality-connect): material-site point loop closures.
- [MuJoCo 3.3.7 equality joint](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#equality-joint): the declared passive inter-stage scalar-joint ratio.
- [MuJoCo 3.3.7 spatial tendon](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#tendon-spatial): geometric cylinder span and passive spring.
- [MuJoCo 3.3.7 actuation model](https://mujoco.readthedocs.io/en/3.3.7/computation/index.html#actuation-model): scalar force and signed transmission mapping.

These references establish simulator semantics, not source dimensions, mechanical calibration, or real operating safety.
