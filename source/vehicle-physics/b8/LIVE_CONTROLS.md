# B8 实时交互物理控制 / live forward-dynamics controls

This launcher advances the current B8 plant with MuJoCo at its fixed **0.00005 s**
physics timestep. Controls change motor torque, pneumatic valve openings, or a
timed collector force. The mechanical plant's positions and velocities are
never prescribed by the controls or by an animation.

**Simulation only. It does not connect to or control physical hardware.** This is
the same uncalibrated source-topology prototype described in
[SOURCE_CONSISTENCY.md](SOURCE_CONSISTENCY.md), [PANTOGRAPH.md](PANTOGRAPH.md), and
[PNEUMATICS.md](PNEUMATICS.md). A working control path is not a safety certification
or a claim of accurate real-equipment dynamics.

## Files and launch

- `live_viewer_full.py`: headless `LiveVehicle` class and optional MuJoCo passive viewer
- `test_live_controls.py`: short headless control/coupling tests
- `LIVE_CONTROLS.md`: this guide

From `vehicle-physics/`, on a machine with an interactive graphical desktop:

```sh
python b8/live_viewer_full.py
```

The launcher uses the existing `vendor/` dependencies when available. Otherwise,
the selected Python environment must already provide NumPy and MuJoCo (tested
here with MuJoCo 3.3.7). It does not install software. On macOS, MuJoCo's passive
viewer requires the supplied `mjpython` interpreter:

```sh
mjpython b8/live_viewer_full.py
```

The default launch first integrates **6 simulated seconds of passive settling**,
with brakes and pantograph vented and all eight air springs sealed. This can take
longer than six wall-clock seconds. The original gas mass is retained throughout
settling; no pose, pressure, or mass reset produces the starting condition.

This cloud environment has neither `DISPLAY` nor `WAYLAND_DISPLAY` set. **The
interactive GUI and its keyboard callback have not been exercised in a window
here.** The headless engine, key-to-action mapping, and display-state isolation
were tested. On Linux without a detected graphical display, the launcher gives a
clear error before settling; use `--headless` for non-GUI execution.

## Keyboard commands

Focus the MuJoCo window before pressing keys:

| Key | Command | Actual input to the plant |
|---|---|---|
| W | Drive | Ramp A motor torque over 0.15 s, subject to 4 N·m and 250 W limits |
| S | Coast | Zero A motor torque on the next physics step |
| B | Brake apply | Open all 16 brake supply valves; close normal exhaust valves |
| N | Brake release | Close brake supply; open normal exhaust |
| R | Raise pantograph | Open reservoir-to-pantograph supply; close both exhaust paths |
| H | Hold pantograph | Close supply and both exhaust paths; existing leaks remain effective |
| E | Normal pantograph exhaust | Close supply; open normal exhaust |
| Q | Quick pantograph exhaust | Close supply; open the larger quick-exhaust path |
| 1 | Toggle brake-cylinder downstream branch leaks | Open/close the existing leak valve on each brake chamber |
| 2 | Toggle reservoir leak | Open/close reservoir-to-atmosphere path |
| 3 | Toggle one air-spring leak | Open/close A-front-left air chamber exhaust |
| 4 | Toggle pantograph leak | Open/close pantograph leakage path |
| T | Collector disturbance | Apply −50 N in world z at collector COM for 0.2 simulated seconds |
| O | Unlock/open all 6 doors | Release ideal latch; apply bounded hinge torques |
| K | Release/close doors | Zero opening torque; springs close; guarded latch capture |
| P | Pause/resume physics | Suspend/restart integration; do not reset the physical state |
| Esc | Close window | Exit the launcher |

Fault toggles are independent of ordinary valve commands: releasing a brake does
not repair a leak. A second press of a leak key closes that same leak path. A
second T replaces the current pulse and restarts its duration. Pausing suspends
the pulse timer because it uses simulation time. No brake or pantograph action
implicitly turns drive off: press S when you intend to coast. H traps finite gas
mass and is not a position servo.

Default leak areas are the existing network values: each brake leak 3e−9 m²,
reservoir leak 2e−6 m², pantograph leak 1e−7 m²; the A-FL fault uses its 6e−8 m²
exhaust port. The official suite's stronger brake-cylinder downstream rupture variant separately
uses 2e−7 m² per brake leak. This launcher leaves the nominal network parameters
unchanged. Restoring a closed leak valve does not refill a chamber. There is no
compressor in the nominal network, so the finite reservoir can be depleted.

MuJoCo's built-in camera/UI controls affect a separate presentation copy. Built-in
Reset, state sliders, pose drags, and physics settings do not reset or tune the
actual B8 plant. Use only the listed operator keys for plant commands. Restart
the process for a fresh plant; there is no hidden live state-reset action.

## Headless use

Short smoke run without a graphical display:

```sh
python b8/live_viewer_full.py --headless --settle-s 0 --duration-s 0.05 \
  --action drive --action brake_apply --action panto_raise
python b8/test_live_controls.py
```

`--settle-s 0` is useful for quick control-wiring tests. It does **not** represent a
settled starting state and must not be used to claim a stopping distance. Omit it
to use the nominal six-second settling phase. Headless output is JSON on stdout;
the short startup message goes to stderr. Repeated `--action` options are applied
in order before the controlled interval begins; duration is rounded to whole
physics steps.

For dynamic control sequences in a Python process started from `b8/`:

```python
from live_viewer_full import LiveVehicle

vehicle = LiveVehicle()  # Six simulated seconds of actual passive settling.
vehicle.set_drive(True)
vehicle.set_pantograph("raise")
vehicle.step(vehicle.steps_for_duration(0.5))
vehicle.set_drive(False)
vehicle.set_brake("apply")
vehicle.step(vehicle.steps_for_duration(0.5))
vehicle.set_leak("reservoir", True)
vehicle.pulse_collector_force(force_z_N=-12.0, duration_s=0.05)
vehicle.step(vehicle.steps_for_duration(0.1))
print(vehicle.telemetry())
```

`set_brake` accepts `apply`, `release`, or `hold`. `set_pantograph` accepts `raise`,
`hold`, `normal_exhaust`, or `quick_exhaust`. `set_leak` accepts `brake_cylinder_branch`,
`reservoir`, `air_A_FL`, or `pantograph`. This API is single-threaded. The GUI
callback only queues action names; the simulation thread applies them between
frames. It never mutates the plant from a GUI callback.

## Force, geometry, and time conventions

- Two independent car bodies, each with one two-axle bogie, connected by the
  current compliant coupler. Only A has a motor; B has no duplicated motor.
- The existing source Z single-arm pantograph, its parallel balancing rods, and
  its declared passive ideal synchronizer are retained. There is no diamond
  replacement.
- Positive pantograph pressure force is **retraction**. Actual chamber travel is
  folded-reference cylinder length minus current solver tendon length, and the
  existing actuator has `gear = -1`. No sign reversal is invented in the viewer.
- All eight air-spring gas volumes use actual solver endpoint length minus their
  initial reference length. Brake volumes use actual piston-slide displacements.
- Every integration step applies `F = (p_absolute − p_ambient) × A`, advances the
  mechanical model once, refreshes actual geometry, then updates gas masses and
  volumes using those new endpoints and the commanded valves. This is the same
  first-order partitioned coupling used by `simulate_full.py`; it is not an
  implicit fully coupled thermodynamic solver.
- Force telemetry distinguishes the force applied over the **last step** from
  pressure calculated at the **current endpoint**. Those differ by one explicit
  integration interval, as intended by the coupling scheme.
- The 250 W cap uses A's rotor speed at the start of each physics step. The
  reported signed `motor_A_power_at_step_start_W` matches this cap calculation.
  It is mechanical shaft input, not a model of 24 V battery current, efficiency,
  or regenerative electricity.

`--fps 30 --speed 1` requests 30 displayed frames per wall second and roughly one
simulated second per wall second. Only the number of fixed-dt steps per frame
changes. Rounded batching gives 667 steps/frame at the defaults (0.03335 simulated
seconds/frame). If the computer is slow, the simulation runs slower; no physics
step is enlarged and no mechanical state is skipped. Console telemetry includes
the achieved simulated-seconds/wall-second ratio. It includes pause time.

The view receives one-way snapshots of freshly solved mechanical states via
`mj_getState`/`mj_setState` into its **separate display data**. It does not advance
the display plant or send display poses back to `LiveVehicle.data`. This isolates
the actual forward solver from the passive viewer's built-in state-edit tools.

## Telemetry and limitations

The viewer prints a JSON telemetry record about once per wall second. All numeric
fields use SI: seconds, metres, metres/second, radians/second, newtons,
newton-metres, watts, pascals, cubic metres, and kilograms. Pressures are explicitly
**absolute**, with ambient 101325 Pa. `simulation_time_s` includes initial
settling; `operator_time_s` starts after settling. The collector height is its
head pivot height, not the carbon contact surface height.

Telemetry includes both car positions and speeds, coupler length, motor torque
and cap-reference power, representative brake pressure, pantograph pressure and
retraction, summed collector normal contact force, all 26 gas-node pressures and
volumes, last external pulse force, operator commands, solver warnings, and gas
mass-balance residual. Nonfinite state or any MuJoCo warning stops integration
with an error rather than silently continuing.

The existing rail spans x = 0 to 40 m. The operator can drive beyond the modeled
track; there is no invented track-end stop, automatic safety brake, or infinitely
repeated rail. Vehicle/coupler limits, primitive capsule wheel treads, flange
discs, rectangular rails, rigid overhead boundary, ideal gears, assumed masses,
and isothermal air remain the documented B8 approximations. They do not establish
real stopping performance, wheel-climb safety, flexible catenary behavior, thermal
gas behavior, or structural stress.

## Verification record

Current final HC77/dt50us source passes the headless tests in results/final_unit_tests.log and results/live_controls_verification.json. The current contact-law tests are separate and include continuity, passivity, spin torque, virtual work, geometry identity and query-setting restoration.

The launcher uses the same explicit continuous wheel/rail force law as the batch solver. XML alone is insufficient; retain the Python source and parameter identity. The legacy `brake_line` API alias means downstream cylinder branch only, never automatic train-brake pipe logic.

The integrated 6-door full-cycle and dt25us checks are in the 22-case report. Short live control tests establish command wiring, not door hardware calibration. The interactive GUI remains untested in this display-less cloud environment. Historical numerical smoke values from the superseded native contact model are not presented as this model's output.
