"""B10 inferred compressor/fan bench, coupled to the frozen B8/B9 gas laws.

SI. No target-device internal topology or measured parameters were available.
The single-acting Scotch-yoke is an explicit equivalent mechanism, not a claim
about the source shell. States evolve from torque and pressure work only.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
B9 = ROOT.parent / "b9"
if str(B9) not in sys.path:
    sys.path.append(str(B9))
from valve_dynamics import DynamicValveNetwork, make_b8_network
from pneumatics import GasVolume, Orifice, PneumaticNetwork, AMBIENT_PA

PROVENANCE = "INFERRED_EQUIVALENT: unmeasured single-acting Scotch-yoke and DC fans on a fixed external bench"


def positive_parameters(obj):
    if not all(math.isfinite(v) and v > 0 for v in asdict(obj).values()):
        raise ValueError("all parameters must be positive and finite")


@dataclass(frozen=True)
class MotorParameters:
    voltage_V: float = 24.0
    resistance_ohm: float = 12.0
    torque_back_emf_constant: float = 0.055

    def __post_init__(self):
        positive_parameters(self)


class Motor:
    """Quasi-static PM DC armature: V=R I+K omega, tau=K I, Kt=Ke in SI.

    Inductance is neglected explicitly. Power loss opens the circuit, whereas a
    zero-voltage connected drive brakes through its resistance. No speed clamp.
    """
    def __init__(self, parameters=None):
        self.parameters = parameters or MotorParameters()
        self.electrical_work_J = 0.0
        self.copper_heat_J = 0.0
        self.shaft_work_J = 0.0
        self.current_A = 0.0
        self.power_failed = False

    def coefficients(self, voltage_fraction):
        p = self.parameters
        if self.power_failed:
            return 0.0, 0.0
        return (p.torque_back_emf_constant*p.voltage_V*voltage_fraction/p.resistance_ohm,
                p.torque_back_emf_constant**2/p.resistance_ohm)

    def record(self, h, voltage_fraction, omega_mid):
        p = self.parameters
        voltage = p.voltage_V*voltage_fraction
        current = 0.0 if self.power_failed else (voltage-p.torque_back_emf_constant*omega_mid)/p.resistance_ohm
        self.current_A = current
        self.electrical_work_J += voltage*current*h
        self.copper_heat_J += p.resistance_ohm*current**2*h
        self.shaft_work_J += p.torque_back_emf_constant*current*omega_mid*h

    def audit(self):
        return dict(electrical_work_J=self.electrical_work_J, copper_heat_J=self.copper_heat_J,
                    shaft_work_J=self.shaft_work_J, current_A=self.current_A,
                    electrical_balance_residual_J=self.electrical_work_J-self.copper_heat_J-self.shaft_work_J)


@dataclass(frozen=True)
class PumpParameters:
    bore_m: float = 0.012
    crank_radius_m: float = 0.006
    clearance_fraction: float = 0.15
    piston_mass_kg: float = 0.04
    flywheel_inertia_kg_m2: float = 2e-5
    viscous_friction_Nm_s: float = 1e-5
    intake_area_m2: float = 2e-6
    discharge_area_m2: float = 1e-6
    cut_in_pressure_Pa: float = 531325.0
    cut_out_pressure_Pa: float = 601325.0
    maximum_step_s: float = 0.0002

    def __post_init__(self):
        positive_parameters(self)
        if self.cut_out_pressure_Pa <= self.cut_in_pressure_Pa:
            raise ValueError("pressure switch requires ordered hysteresis thresholds")

    @property
    def area_m2(self):
        return math.pi*self.bore_m**2/4

    @property
    def dead_volume_m3(self):
        return self.clearance_fraction*2*self.crank_radius_m*self.area_m2


class ReciprocatingPump:
    """One constrained coordinate theta, canonical momentum p, finite piston mass.

    M(theta)=J+m r² sin²(theta); x=r(1-cos(theta)). A symmetric discrete
    gradient closes kinetic + pressure work at fixed gas mass. Ideal massless
    suction/discharge check seats are pressure operated; reed dynamics remain
    outside this equivalent. Mount forces are outside the fixed bench boundary.
    """
    def __init__(self, gas, parameters=None, motor_parameters=None):
        self.gas = gas
        self.parameters = parameters or PumpParameters()
        self.motor = Motor(motor_parameters)
        self.angle_rad = 0.0
        self.phase_rad = 0.0  # bounded physical phase; unwrapped angle is telemetry
        self.momentum_kg_m2_s = 0.0
        self.gas_work_J = 0.0
        self.gas_geometry_work_J = 0.0
        self.friction_heat_J = 0.0
        self.pressure_switch_on = False
        self.intake_failed_closed = False
        self.discharge_failed_closed = False
        self.max_step_balance_residual_J = 0.0

    def mass_matrix(self, angle):
        p = self.parameters
        return p.flywheel_inertia_kg_m2+p.piston_mass_kg*p.crank_radius_m**2*math.sin(angle)**2

    @property
    def omega_rad_s(self):
        return self.momentum_kg_m2_s/self.mass_matrix(self.phase_rad)

    @property
    def displacement_m(self):
        return self.parameters.crank_radius_m*(1-math.cos(self.phase_rad))

    @property
    def piston_velocity_m_s(self):
        return self.parameters.crank_radius_m*math.sin(self.phase_rad)*self.omega_rad_s

    def kinetic_energy_J(self):
        return self.momentum_kg_m2_s**2/(2*self.mass_matrix(self.phase_rad))

    def switch(self):
        p = self.gas.pressure_Pa("reservoir")
        if p <= self.parameters.cut_in_pressure_Pa:
            self.pressure_switch_on = True
        elif p >= self.parameters.cut_out_pressure_Pa:
            self.pressure_switch_on = False
        return float(self.pressure_switch_on)

    def check_commands(self):
        # Ambient is the B8 external boundary, not a finite supply tank. Intake
        # direction is selected here because B8's one_way flag permits a->b.
        return dict(compressor_intake=float(not self.intake_failed_closed and
                                            self.gas.pressure_Pa("compressor_cylinder") < self.gas.ambient_pressure_Pa),
                    compressor_discharge=float(not self.discharge_failed_closed))

    def advance_geometry(self, h, command):
        p = self.parameters
        a, p0 = self.phase_rad, self.momentum_kg_m2_s
        m0 = self.mass_matrix(a)
        volume0 = self.gas.volumes["compressor_cylinder"].volume_m3
        mRT = self.gas.volumes["compressor_cylinder"].mass_kg*self.gas.RT
        drive, electromagnetic_damping = self.motor.coefficients(command)
        damping = p.viscous_friction_Nm_s+electromagnetic_damping
        def evaluate(delta):
            b = a+delta
            m1 = self.mass_matrix(b)
            velocity = delta/h
            p1 = 4*velocity/(1/m0+1/m1)-p0
            dv = p.area_m2*p.crank_radius_m*2*math.sin(a+delta/2)*math.sin(delta/2)
            gas_work = mRT*math.log1p(dv/volume0)-self.gas.ambient_pressure_Pa*dv
            if abs(delta) < 1e-12:
                inv_mass_gradient = -p.piston_mass_kg*p.crank_radius_m**2*math.sin(2*a)/(m0*m1)
                gas_gradient = (-mRT/volume0+self.gas.ambient_pressure_Pa)*p.area_m2*p.crank_radius_m*math.sin(a)
            else:
                inv_mass_gradient = (-p.piston_mass_kg*p.crank_radius_m**2*
                                     math.sin(2*a+delta)*math.sin(delta)/(m0*m1*delta))
                gas_gradient = -gas_work/delta
            gradient = (p0*p0+p1*p1)*inv_mass_gradient/4+gas_gradient
            residual = (p1-p0)/h-drive+damping*velocity+gradient
            return residual, p1, gas_work
        delta = self.omega_rad_s*h+0.5*drive*h*h/m0
        for _ in range(16):
            residual, p1, gas_work = evaluate(delta)
            if abs(residual) < 2e-11:
                break
            epsilon = 1e-7
            derivative = (evaluate(delta+epsilon)[0]-evaluate(delta-epsilon)[0])/(2*epsilon)
            if not math.isfinite(derivative) or derivative <= 0:
                raise ArithmeticError("pump discrete-gradient solve left its step domain; reduce dt")
            correction = residual/derivative
            delta -= correction
        else:
            raise ArithmeticError("pump discrete-gradient solve did not converge")
        if abs(delta) > math.pi/2:
            raise ArithmeticError("pump step exceeds quarter turn; reduce dt")
        before = self.kinetic_energy_J()
        self.angle_rad += delta
        self.phase_rad = math.remainder(a+delta, math.tau)
        self.momentum_kg_m2_s = p1
        self.motor.record(h, command, delta/h)
        friction = p.viscous_friction_Nm_s*(delta/h)**2*h
        self.gas_work_J += gas_work
        self.friction_heat_J += friction
        # Only geometry changes here. This existing gas helper computes exact
        # isothermal p dV work and heat; it neither changes time nor air mass.
        gas_before = self.gas.ledger["useful_mechanical_work_J"]
        self.gas._move_volumes({"compressor_cylinder": self.displacement_m})
        self.gas_geometry_work_J += self.gas.ledger["useful_mechanical_work_J"]-gas_before
        step_residual = self.kinetic_energy_J()-before-gas_work+friction-(drive-electromagnetic_damping*delta/h)*delta
        self.max_step_balance_residual_J = max(self.max_step_balance_residual_J, abs(step_residual))

    def audit(self):
        motor = self.motor.audit()
        return dict(**motor, kinetic_energy_J=self.kinetic_energy_J(), gas_work_J=self.gas_work_J,
                    gas_geometry_work_J=self.gas_geometry_work_J,
                    gas_geometry_work_discrepancy_J=self.gas_geometry_work_J-self.gas_work_J,
                    friction_heat_J=self.friction_heat_J,
                    compressor_work_to_gas_J=-self.gas_work_J,
                    mechanical_balance_residual_J=self.kinetic_energy_J()-self.gas_work_J+self.friction_heat_J-motor["shaft_work_J"],
                    max_step_balance_residual_J=self.max_step_balance_residual_J)

    def snapshot(self):
        return dict(angle_rad=self.angle_rad, phase_rad=self.phase_rad, omega_rad_s=self.omega_rad_s,
                    momentum_kg_m2_s=self.momentum_kg_m2_s, piston_displacement_m=self.displacement_m,
                    piston_velocity_m_s=self.piston_velocity_m_s,
                    cylinder_pressure_Pa=self.gas.pressure_Pa("compressor_cylinder"),
                    pressure_switch_on=self.pressure_switch_on, power_failed=self.motor.power_failed,
                    intake_failed_closed=self.intake_failed_closed, discharge_failed_closed=self.discharge_failed_closed,
                    **self.audit())


@dataclass(frozen=True)
class FanParameters:
    inertia_kg_m2: float = 2e-6
    bearing_drag_Nm_s: float = 1e-6
    aerodynamic_drag_Nm_s2: float = 1e-7
    flow_m3_per_rad: float = 1e-5

    def __post_init__(self):
        positive_parameters(self)


class Fan:
    def __init__(self, parameters=None):
        self.parameters = parameters or FanParameters()
        self.motor = Motor(MotorParameters(resistance_ohm=24, torque_back_emf_constant=0.03))
        self.angle_rad = 0.0
        self.omega_rad_s = 0.0
        self.air_work_J = 0.0
        self.bearing_heat_J = 0.0

    def advance(self, h, command):
        p = self.parameters
        drive, damping = self.motor.coefficients(command)
        damping += p.bearing_drag_Nm_s
        # Midpoint J*(w1-w0)/h=tau; aerodynamic drag is monotone odd.
        b = 2*p.inertia_kg_m2/h+damping
        rhs = 2*p.inertia_kg_m2*self.omega_rad_s/h+drive
        magnitude = 2*abs(rhs)/(b+math.sqrt(b*b+4*p.aerodynamic_drag_Nm_s2*abs(rhs)))
        mid = math.copysign(magnitude, rhs)
        self.angle_rad += mid*h
        self.omega_rad_s = 2*mid-self.omega_rad_s
        self.motor.record(h, command, mid)
        self.air_work_J += p.aerodynamic_drag_Nm_s2*abs(mid)**3*h
        self.bearing_heat_J += p.bearing_drag_Nm_s*mid*mid*h

    def audit(self):
        motor = self.motor.audit()
        kinetic = 0.5*self.parameters.inertia_kg_m2*self.omega_rad_s**2
        return dict(**motor, kinetic_energy_J=kinetic, air_work_J=self.air_work_J,
                    bearing_heat_J=self.bearing_heat_J,
                    mechanical_balance_residual_J=kinetic+self.air_work_J+self.bearing_heat_J-motor["shaft_work_J"])

    def snapshot(self):
        return dict(angle_rad=self.angle_rad, omega_rad_s=self.omega_rad_s,
                    flow_proxy_m3_s=self.parameters.flow_m3_per_rad*abs(self.omega_rad_s),
                    power_failed=self.motor.power_failed, **self.audit())


class ThermalLump:
    """First-order calorimeter, diagnostic only; no gas-temperature feedback."""
    def __init__(self, capacity_J_K, natural_UA_W_K, forced_UA_per_rad_s, ambient_K=293.15):
        if not all(math.isfinite(x) and x > 0 for x in (capacity_J_K, natural_UA_W_K, forced_UA_per_rad_s, ambient_K)):
            raise ValueError("positive finite thermal parameters required")
        self.capacity_J_K = capacity_J_K
        self.natural_UA_W_K = natural_UA_W_K
        self.forced_UA_per_rad_s = forced_UA_per_rad_s
        self.ambient_K = ambient_K
        self.temperature_K = ambient_K
        self.heat_input_J = 0.0
        self.heat_to_ambient_J = 0.0

    def advance(self, h, heat_J, fan_omega):
        ua = self.natural_UA_W_K+self.forced_UA_per_rad_s*abs(fan_omega)
        old = self.temperature_K-self.ambient_K
        new = ((self.capacity_J_K-h*ua/2)*old+heat_J)/(self.capacity_J_K+h*ua/2)
        self.temperature_K = self.ambient_K+new
        self.heat_input_J += heat_J
        self.heat_to_ambient_J += h*ua*(old+new)/2

    def snapshot(self):
        storage = self.capacity_J_K*(self.temperature_K-self.ambient_K)
        return dict(temperature_K=self.temperature_K, heat_input_J=self.heat_input_J,
                    heat_to_ambient_J=self.heat_to_ambient_J, stored_heat_J=storage,
                    balance_residual_J=storage-self.heat_input_J+self.heat_to_ambient_J)


class AuxiliaryNetwork(DynamicValveNetwork):
    def __init__(self, gas, *, pump_parameters=None, **kwargs):
        super().__init__(gas, **kwargs)
        self.pump = ReciprocatingPump(gas, pump_parameters)
        self.fans = {n: Fan() for n in ("compressor_fan", "bay_fan")}
        self.thermal = dict(compressor_motor=ThermalLump(120, 1.0, 0.01),
                            cabinet=ThermalLump(500, 2.0, 0.018))
        self.commands.update(compressor=0.0, compressor_fan=1.0, bay_fan=1.0)
        self.automatic_compressor = True
        self.cabinet_external_heat_W = 80.0

    def close_all_valves(self):
        auxiliary = {n: self.commands[n] for n in ("compressor", *self.fans)}
        super().close_all_valves()
        self.commands.update(auxiliary)

    def update(self, dt, displacements_m=None, valve_commands=None, *, snapshot=True):
        positions, commands = dict(displacements_m or {}), dict(valve_commands or {})
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("finite nonnegative dt required")
        if "compressor_cylinder" in positions:
            raise ValueError("pump displacement is owned by force dynamics")
        for n, x in positions.items():
            if n not in self.gas.volumes or self.gas.volumes[n].piston_area_m2 <= 0:
                raise ValueError("unknown displacement: "+n)
            if not math.isfinite(x) or self.gas.volumes[n].dead_volume_m3+self.gas.volumes[n].piston_area_m2*x <= 0:
                raise ValueError("invalid displacement: "+n)
            if dt == 0 and x != self.gas.volumes[n].displacement_m:
                raise ValueError("position change requires dt > 0")
        for n, u in commands.items():
            if n not in self.commands or not math.isfinite(u) or not 0 <= u <= 1:
                raise ValueError("invalid command: "+n)
            if n in ("compressor_intake", "compressor_discharge"):
                raise ValueError("check seats are pressure operated, not operator commands")
        self.commands.update(commands)
        if dt == 0:
            return self.snapshot() if snapshot else None
        hmax = min(self.pump.parameters.maximum_step_s, self.gas.max_substep_s)
        count = max(1, math.ceil(dt/hmax))
        h = dt/count
        start = {n: self.gas.volumes[n].displacement_m for n in positions}
        edge_before = dict(self.gas.edge_mass_kg)
        auxiliary_keys = {"compressor", *self.fans}
        for k in range(count):
            actual = {n: u for n, u in self.commands.items() if n not in auxiliary_keys}
            actual.update(self.pump.check_commands())
            for n, s in self.spools.items():
                actual[n] = s.advance(h, self.commands[n])
            mid = {n: start[n]+(x-start[n])*(k+0.5)/count for n, x in positions.items()}
            end = {n: start[n]+(x-start[n])*(k+1)/count for n, x in positions.items()}
            self.gas.update(h/2, mid, actual, snapshot=False)
            pump_before = self.pump.motor.copper_heat_J+self.pump.friction_heat_J
            command = self.pump.switch() if self.automatic_compressor else self.commands["compressor"]
            self.pump.advance_geometry(h, command)
            actual.update(self.pump.check_commands())
            self.gas.update(h/2, end, actual, snapshot=False)
            for n, s in self.spools.items():
                self.gas.orifices[n].opening = s.opening
            fan_heat = {}
            for n, fan in self.fans.items():
                before = fan.motor.copper_heat_J+fan.bearing_heat_J
                old_omega = fan.omega_rad_s
                fan.advance(h, self.commands[n])
                fan_heat[n] = (fan.motor.copper_heat_J+fan.bearing_heat_J-before,
                               (old_omega+fan.omega_rad_s)/2)
            pump_heat = self.pump.motor.copper_heat_J+self.pump.friction_heat_J-pump_before
            self.thermal["compressor_motor"].advance(h, pump_heat+fan_heat["compressor_fan"][0], fan_heat["compressor_fan"][1])
            self.thermal["cabinet"].advance(h, self.cabinet_external_heat_W*h+fan_heat["bay_fan"][0], fan_heat["bay_fan"][1])
        self.gas.last_flows_kg_s = {n: (dm-edge_before[n])/dt for n, dm in self.gas.edge_mass_kg.items()}
        return self.snapshot() if snapshot else None

    def configuration(self):
        return super().configuration() | dict(auxiliary_model=dict(
            provenance=PROVENANCE, gas_state_count=len(self.gas.volumes),
            pump=asdict(self.pump.parameters), pump_motor=asdict(self.pump.motor.parameters),
            fan_parameters={n: asdict(f.parameters) for n, f in self.fans.items()},
            fan_motors={n: asdict(f.motor.parameters) for n, f in self.fans.items()},
            auxiliary_mechanical_coordinates=3, auxiliary_canonical_states=6,
            thermal_states=2, automatic_compressor=self.automatic_compressor,
            cabinet_external_heat_W=self.cabinet_external_heat_W,
            thermal_parameters={n:dict(capacity_J_K=t.capacity_J_K, natural_UA_W_K=t.natural_UA_W_K,
                                       forced_UA_per_rad_s=t.forced_UA_per_rad_s, ambient_K=t.ambient_K)
                                for n,t in self.thermal.items()},
            limits=["external bench: no vehicle mount reaction", "inferred Scotch-yoke: not source hardware",
                    "ideal massless check seats: no reed inertia", "isothermal gas: fixed ambient heat bath",
                    "quasi-static armature: no inductance", "fan flow/UA proxies: no measured curve or CFD",
                    "thermal diagnostics do not alter pneumatic temperature"]))

    def snapshot(self):
        return super().snapshot() | dict(pump=self.pump.snapshot(), fans={n:f.snapshot() for n,f in self.fans.items()},
                                         thermal={n:t.snapshot() for n,t in self.thermal.items()})


def make_vehicle_network(*args, pump_parameters=None, **kwargs):
    if kwargs.pop("with_compressor", False):
        raise ValueError("B10 replaces the algebraic compressor; do not request B8 source")
    p = pump_parameters or PumpParameters()
    base = make_b8_network(*args, **kwargs)
    vv = list(base.volumes.values())+[GasVolume("compressor_cylinder", p.dead_volume_m3,
                                               base.ambient_pressure_Pa, p.area_m2)]
    oo = list(base.orifices.values())+[
        Orifice("compressor_intake", "compressor_cylinder", None, p.intake_area_m2),
        Orifice("compressor_discharge", "compressor_cylinder", "reservoir", p.discharge_area_m2, one_way=True)]
    gas = PneumaticNetwork(vv, oo, temperature_K=base.temperature_K,
                           ambient_pressure_Pa=base.ambient_pressure_Pa,
                           gas_constant=base.gas_constant, gamma=base.gamma,
                           max_substep_s=base.max_substep_s)
    return AuxiliaryNetwork(gas, pump_parameters=p)
