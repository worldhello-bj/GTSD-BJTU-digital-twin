# B8 finite-volume pneumatic network

## 实现状态与边界

这是未标定的机械物理原型组件。气路模型不生成或覆盖机械位移；储气罐、制动缸、受电弓气缸和空气弹簧的压力，由实际气体质量与实际机械位移对应的气室体积计算。阀门只改变通流面积。有限供气、压力不足、泄漏、排气和机械负载可以改变动作结果。

本模块采用统一、恒定温度的理想气体和准稳态可压缩孔口流量。它不是完整热力学气动模型，也不代表实机阀性能、安全制动能力、真实空气弹簧或实体受电弓认证。所有默认设备参数均为 `CALIBRATION_ASSUMPTION`，没有使用实测标定数据。

Files, all new under `b8/`:

- `pneumatics.py`: standard-library-only API
- `test_pneumatics.py`: executable validation suite
- `pneumatic_results/test_report.json`: complete numerical test results
- `pneumatic_results/convergence.csv`: timestep refinement data
- `PNEUMATICS.md`: this integration and physical-model specification

No B7 files are changed by this module or its tests.

## State, force and flow

Each rigid reservoir or piston chamber stores a finite air mass `m` in kg. Temperature is `T = 293.15 K` and dry-air engineering constants are `R = 287.05 J/(kg K)` and `gamma = 1.4`.

- Absolute pressure: `p = m R T / V`
- Cylinder volume: `V = V_dead + A x`
- Gauge pressure force: `F = (p - p_ambient) A`, positive along **increasing cylinder extension**
- Ambient pressure: `101325 Pa`

`x` must come from the mechanical solver. A slider joint displacement or spatial-tendon length minus a fixed reference length can be used. The same coordinate and sign must be used to apply the pressure force. The mechanical solver supplies piston/link inertia, gravity, joint losses, friction, return springs, constraints, stroke stops, and contact. The pneumatic module supplies none of those implicitly.

Small negative extension at a compliant mechanical stop is accepted without clamping, provided `V > 0`. For example, the pantograph's few-micrometer stop penetration does not invalidate its 40 cc dead volume. There is no artificial pressure clamp, piston-position clamp, commanded trajectory, or force-controller following a geometric animation. Nonpositive volume is an error. Nominal stroke limits belong to the actual mechanical joints, not to this gas solver.

For upstream pressure `p_u`, downstream pressure `p_d`, ratio `r = p_d/p_u`, effective area `A_o * opening`, and coefficient `C_d`:

- Critical ratio: `r_crit = (2/(gamma+1))^(gamma/(gamma-1)) = 0.5282818…`
- Choked flow: `m_dot = C_d A_o p_u / sqrt(R T) * sqrt(gamma) * (2/(gamma+1))^((gamma+1)/(2(gamma-1)))`
- Subcritical flow: `m_dot = C_d A_o p_u / sqrt(R T) * sqrt(2 gamma/(gamma-1) * (r^(2/gamma) - r^((gamma+1)/gamma)))`

The upstream side is selected from actual pressures. Bidirectional ports admit atmospheric air if expansion drops chamber pressure below atmospheric. A one-way valve prevents reverse flow instead. Reservoir-to-actuator supply ports use one-way valves by default. Exhaust/leak ports are bidirectional; a permanent-open vacuum intake check valve is not separately modeled.

The orifice model represents an idealized pressure-driven throttle with sonic choking, not the manufacturer-specific ISO flow characteristic of an identified valve. Pipe transport delay, acoustic waves, spool inertia, humidity, temperature gradients and distributed pipe volumes are absent.

## API and coupling

```python
from pneumatics import GasVolume, make_vehicle_network

brakes = [f'{bogie}_{axle}_{side}_{face}_pad'
          for bogie in ('A', 'B')
          for axle in ('front', 'rear')
          for side in ('L', 'R')
          for face in ('minus', 'plus')]

bellows = [GasVolume(f'air_{bogie}_{side}',
                     dead_volume_m3=0.00025,
                     initial_pressure_Pa=200000,
                     piston_area_m2=0.003)
           for bogie in ('A', 'B') for side in ('L', 'R')]

net = make_vehicle_network(
    brake_names=brakes,
    include_pantograph=True,  # False for vehicle-only tests
    extra_chambers=bellows,
    initial_displacements_m={name: actual_initial_x[name]
                             for name in actual_initial_x},
    with_compressor=False,
)

# At each mechanical step:
net.update(0, valve_commands=commands)  # Set commands for the coming interval.
forces = net.forces_N()
# Apply forces[name] via each matching positive-extension actuator/Jacobian.
# Integrate the mechanical model once using its own forces, constraints/contact.
# Read the actual endpoint piston/tendon extensions from the new solver state.
net.update(dt, displacements_m=actual_endpoint_x, snapshot=False)

# Only when needed for output:
pressure_abs = net.pressures_Pa()
audit = net.audit()
state = net.snapshot()
```

The example is explicit staggered coupling. Forces are held over the mechanical step, so the coupled vehicle must pass a separate timestep study. This module's work is the integral along the two observed displacement endpoints, with linear interpolation **only within the fluid substep**. This interpolation never writes mechanical coordinates. Compare `useful_mechanical_work_J` against the mechanical solver's pressure-actuator work to quantify partitioned-coupling work error. Do not conceal the difference by redefining the mechanical work.

Optional initial displacements should be supplied from the same physical initialization used for the mechanical model. Initial pressure is a precharge assumption at that initialized volume. If air springs carry gravity during settling, integrate their volumes throughout the settling phase, rather than resetting their masses after settling.

Commands are real-valued openings from 0 to 1. They persist until changed, and unknown command names fail validation. Explicitly close the opposite valve when switching supply/exhaust. Opening both is allowed and intentionally consumes reservoir air through a flow path to atmosphere. `close_all_valves()` closes every valve and optional compressor.

Factory command names:

- `{brake_name}_supply`, `{brake_name}_exhaust`, `{brake_name}_leak`
- `pantograph_supply`, `pantograph_exhaust`, `pantograph_quick_exhaust`, `pantograph_leak` when included
- `{extra_chamber_name}_supply`, `{extra_chamber_name}_exhaust`
- `reservoir_leak`
- `compressor` when `with_compressor=True`

All ports start closed. Reservoir pressure is finite. A vented reservoir cannot recharge an actuator. However, **loss of reservoir pressure does not automatically vent an already-charged actuator behind a closed/check valve**. To model a pipe rupture, ADD or release fault, open the corresponding chamber's actual leakage/exhaust path. This distinction prevents a physically incorrect “supply lost, therefore set every pressure to zero” shortcut.

For a different topology use:

```python
from pneumatics import GasVolume, Orifice, Compressor, PneumaticNetwork

net = PneumaticNetwork(
    volumes=[GasVolume('tank', 0.002, 601325),
             GasVolume('cylinder', 4e-5, 101325, 0.0002, 0.0)],
    orifices=[Orifice('fill', 'tank', 'cylinder', 6e-8,
                     discharge_coefficient=0.7, one_way=True),
              Orifice('vent', 'cylinder', None, 6e-7)],
    compressors=[],
    max_substep_s=0.0005,
)
```

`None` is the ambient boundary only; it does not create an infinite compressed supply. Constructor specifications are copied. State dictionaries exposed through `nodes`/`volumes` are for inspection and must not be mutated by callers. Direct state manipulation would bypass the mass/work audit.

## Assumed default parameters

| Component | Parameter | Default | Unit / interpretation |
|---|---|---:|---|
| All gas volumes | Temperature | 293.15 | K, ideal heat bath |
| All orifices | Discharge coefficient | 0.7 | dimensionless |
| Reservoir | Volume | 0.002 | m³, 2 L |
| Reservoir | Initial pressure | 601325 | Pa absolute, 500 kPa gauge |
| Brake chamber | Area | 0.000060 | m², 30 N at 500 kPa gauge |
| Brake chamber | Dead volume | 0.000002 | m³, 2 cc per chamber |
| Brake supply | Orifice area | 8e-9 | m² |
| Brake exhaust | Orifice area | 1.6e-8 | m² |
| Brake fault leak | Orifice area | 3e-9 | m², closed until commanded |
| Pantograph | Area | 0.000200 | m², 60 N at 300 kPa gauge |
| Pantograph | Dead volume | 0.000040 | m³, 40 cc |
| Pantograph supply / normal exhaust | Orifice area | 6e-8 | m² each |
| Pantograph quick exhaust | Orifice area | 6e-7 | m², 10× normal; no prescribed fall time |
| Pantograph fault leak | Orifice area | 1e-7 | m², closed until commanded |
| Reservoir fault leak | Orifice area | 2e-6 | m², closed until commanded |
| Extra chamber fill / exhaust | Orifice area | 6e-8 | m² each, normally isolated |
| Optional compressor | Mass rate ceiling | 0.0001 | kg/s |
| Optional compressor | Cutoff pressure | 601325 | Pa absolute |
| Optional compressor | Isothermal efficiency | 0.65 | synthetic efficiency |
| Internal integration | Maximum substep | 0.0005 | s |

Default pantograph area was selected to match the B8 articulated linkage's assumed force scale, not from an identified real cylinder. The four example air springs have 0.003 m² effective area, 0.00025 m³ nominal dead volume and 200000 Pa initial absolute pressure. Their dimensions are supplied explicitly, rather than hidden in the generic factory.

Gas-spring local stiffness follows the assumed geometry: for a closed isothermal chamber, `-dF/dx = p A²/V`. There is no hidden linear-spring force in the pneumatic module. A fixed effective area, rigid wall, lumped volume, uniform temperature, and optional finite throttle are still strong idealizations: no rubber-membrane elasticity, rolling-lobe area curve, heat transfer dynamics, height-control valve logic or experimentally fitted damping are claimed.

## Discrete conservation and accuracy

Each internal substep performs:

1. Advance chamber volumes halfway to the observed endpoint, retaining each mass.
2. Evaluate finite-orifice flow and transfer the same mass out of one node and into its neighbor.
3. Advance volumes through the remaining half step.

Flow edges are swept sequentially; edge order alternates to reduce directional bias. Each transfer is capped at the exact mass needed to equalize that pair at the current volumes. This cap prevents flow-direction overshoot and negative mass for very large timesteps. It is a positivity/stability guard, **not** proof that a large timestep is accurate. `equalization_caps` and `substeps` are reported. This flow integrator is effectively first-order globally; the split geometry treatment does not justify claiming a second-order network integrator.

Closed-chamber isothermal volume work is integrated exactly on each fixed-mass geometry substep: `W_pdv = m R T ln(V_new/V_old)`. With simultaneous flow and volume change the operator split introduces timestep error, tested by refinement. The input `max_substep_s` should be reduced for small dead volumes, larger valve areas or fast cylinder motion.

## Mass, heat, work and exergy accounting

Positive work is energy **leaving the gas into mechanical boundary motion**. Useful actuator work excludes atmospheric displacement work:

- `W_gas = integral p dV`
- `W_useful = integral (p - p_ambient) dV`
- `W_ambient = p_ambient DeltaV`

These have different meanings; a closed cylinder can exchange heat and do mechanical work while its ideal-gas internal energy remains constant.

All volumes are connected to an explicitly modeled ideal heat bath at fixed temperature. With `cv=R/(gamma-1)`, `cp=cv+R`:

- Stored internal energy: `U = sum(m cv T)`
- External mass enthalpy: `H_in/out = cp T * m_in/out`
- Gas heat from bath: `Q_gas = DeltaU + W_gas - H_in + H_out`
- Gas first-law residual: `DeltaU - (Q_gas + H_in - H_out - W_gas)`

Fixed-mass expansion requires heat from the bath; fixed-mass compression rejects heat. Isothermal venting also needs heat inflow to offset the enthalpy carried out by the lost mass. Closed equalization transfers enthalpy internally and has zero **net** gas heat exchange at this common temperature, though it irreversibly destroys exergy. No total thermal-energy-conservation claim is made for an insulated vehicle; the heat bath is external and explicit. The first-law ledger is an algebraic consistency check of the selected isothermal model, not a separately solved temperature equation or measured heat transfer.

The non-flow exergy of each volume, relative to the same-temperature atmosphere, is:

`B = m R T ln(p/p_ambient) - m R T + p_ambient V`.

At fixed mass its decrease equals useful mechanical work. At fixed volume, each equalizing internal transfer or atmospheric vent/intake decreases `B`; this decrease is logged as `flow_exergy_destroyed_J`. Exhaust destruction includes final equilibration/mixing with ambient beyond the orifice. These quantities expose nonphysical energy creation that a mass-only check would miss.

The optional compressor draws air from ambient at the bath temperature, injects finite mass, and records external energy explicitly. Its minimum isothermal compression work equals the destination exergy increase above ambient; actual work divides that minimum by the assumed efficiency. It rejects the corresponding compressor/aftercooler heat to the bath. Below-ambient intake requires no compression work, and any crossing of ambient pressure charges compression work only for the above-ambient portion. No pressure or mass is silently reset.

Reported audit identities:

- `Delta m - m_external_in + m_external_out = 0`
- `Delta U - (Q_gas + H_in - H_out - W_gas) = 0`
- Combined gas/compressor energy uses `Q_total = Q_gas - compressor_heat_to_bath` and adds `compressor_work_in`.
- `Delta B + W_useful + B_destroyed - W_compressor = 0`
- `entropy_generated_J_per_K = B_destroyed/T` under the same-temperature environmental assumptions.

All audit values are cumulative since initialization. Ambient flow, compressor additions, heat and work use separate signed ledgers. The module does not estimate brake-disc temperature or unmodeled gas temperature.

## Reproducible validation

Run from the existing `vehicle-physics` directory:

```sh
python b8/test_pneumatics.py
```

The suite verifies:

1. Continuous subcritical/choked transition and pressure/area limits
2. Closed finite-volume equalization against analytic mass-weighted final pressure
3. Exact closed-chamber expansion/compression pressure, gas work, useful work and heat
4. Distinct finite normal/quick/leak exhaust and atmospheric intake
5. Lost reservoir supply and preservation of an isolated charged actuator
6. Falling pressure in a finite reservoir feeding 16 brake chambers
7. Compressor mass, minimum work, inefficiency and aftercooler heat, including below-ambient start
8. Very-large-step positivity guard and invalid input rejection before mutation
9. Four gas-spring chambers with restoring pressure/force response to compression and extension
10. Timestep convergence with simultaneous flow and varying observed volume
11. A free inertial mass pushed by gas force against an external spring/damper, with no position prescription

The analytic position function in test 10 is only a fluid numerical-validation boundary input. Test 11 actually integrates mass acceleration and supplies the resulting endpoint displacement; integrated vehicle code must likewise use actual solver output.

Current run: all 11 tests pass. The equalization fixture reaches `433333.333333… Pa`. Doubling a closed 100 cc chamber initially at 500000 Pa gives `250000 Pa`, `34.657359 J` total gas work and `24.524859 J` useful gauge-pressure work. With simultaneous flow and motion, pressure error against the 0.0625 ms reference falls from approximately `11.65, 5.55, 2.62, 1.20, 0.51, 0.17 Pa` as timestep halves from `4, 2, 1, 0.5, 0.25, 0.125 ms`. These are test-fixture results, not observed vehicle performance.

A 20-chamber workload (16 brakes plus 4 air springs), 10000 updates plus force queries at 0.1 ms, took approximately 1.36 s in this cloud environment before later minor numerical-stability changes. Timing is informative, not a realtime performance guarantee.

## Engineering basis

The local requirements used were `GTSD–BJTU 机械物理化数字孪生补充要求`, 2026-10-07, especially sections 4–6 and acceptance criteria E/G/H/I. They supplied the ideal-gas state, actual chamber-volume feedback, pressure-force convention, finite-reservoir requirement and thermal-model disclosure requirement.

Their primary references for further implementation review are [MathWorks Pneumatic Piston Chamber](https://www.mathworks.com/help/simscape/ref/pneumaticpistonchamber.html) and [MathWorks Modeling Gas Systems](https://www.mathworks.com/help/simscape/ug/modeling-gas-systems.html). The legacy chamber page is explanatory only; this module does not depend on the deprecated Simscape component or claim API compatibility with it.

## Provenance and proposed sensitivity ranges

These ranges are **proposed numerical sensitivity brackets**, not measured component tolerances, procurement specifications, safe operating limits or validated physical predictions. Their purpose is to expose which unknowns change response before obtaining actual device data. Change one group at a time; keep all chamber volumes positive over the mechanical stroke and use a new timestep refinement when valve/volume ratios change substantially.

| Parameter group | Assumed sensitivity bracket | Source / reason for default |
|---|---|---|
| Reservoir volume | 0.001–0.004 m³ | Engineering assumption: finite 2 L baseline; no device reservoir identification |
| Initial reservoir gauge pressure | 0–600 kPa | Engineering assumption: 500 kPa gives 30 N with baseline brake area; zero-supply fault included |
| Brake area | 3e-5–1.2e-4 m² | 6e-5 m² selected to reproduce B7's assumed 30 N pad-force scale at 500 kPa; not a cylinder measurement |
| Brake dead volume | 1e-6–8e-6 m³ | Engineering assumption; strongly controls pressure-rise time and volume feedback |
| Pantograph area | 1e-4–4e-4 m² | 2e-4 m² matched to sibling B8 physical linkage's 60 N / 300 kPa assumed operating scale |
| Pantograph dead volume | 2e-5–8e-5 m³ | Engineering assumption; no actual chamber or hose volume available |
| All valve effective areas | 0.25–4× listed nominal | Engineering assumption; numerical conductance sweep, not identified valve flow coefficient |
| Quick/normal exhaust area ratio | 2–20 | Default 10× chosen to create distinct causal flow resistance; no prescribed or claimed ADD fall time |
| Discharge coefficient | 0.5–0.9 | Engineering approximation around 0.7; depends on actual valve geometry |
| Airspring effective area | 0.0015–0.006 m² | Engineering assumption; baseline gives about 296 N per spring at 200000 Pa absolute |
| Airspring dead volume | 0.000125–0.0005 m³ | Engineering assumption; choose with area/stroke so minimum volume remains positive |
| Airspring precharge pressure | 150000–300000 Pa absolute | Engineering assumption around 200000 Pa; gravity equilibrium is a required independent check |
| Common isothermal temperature | 273.15–323.15 K | Environmental scenario, not dynamic gas heating/cooling |
| Ambient pressure | 90000–105000 Pa | Environmental scenario for generic constructor |
| Compressor mass-rate ceiling | 2.5e-5–4e-4 kg/s | Optional synthetic source, not a compressor performance map |
| Compressor isothermal efficiency | 0.4–0.8 | Optional synthetic source; actual input-work/thermal data absent |

Initial state, actual configured dimensions, valve areas, optional compressor settings and current valve openings are available as `net.configuration()`. Include this dictionary in coupled-case metrics alongside mechanical parameters and the command schedule. It reflects deliberate fault-area overrides and retains construction-time initial positions/pressures after the state has evolved.
