"""B9 pressure-balanced metering-port proxies coupled to the frozen B8 gas law.

Each moving spool has mass, spring, viscous damping and compliant end stops.
Commands are coil-force fractions, never prescribed displacement/opening.
The discrete-gradient solve closes the mechanical energy ledger at every step.
SI; all component parameters are uncalibrated assumptions, not source geometry.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import sys
from typing import Mapping

B8 = Path(__file__).resolve().parents[1] / "b8"
if str(B8) not in sys.path:
    sys.path.append(str(B8))
from pneumatics import make_vehicle_network as make_b8_network

PROVENANCE = "CALIBRATION_ASSUMPTION: balanced 2/2 metering-port proxy, not measured valve hardware"


@dataclass(frozen=True)
class SpoolParameters:
    mass_kg: float = 0.01
    spring_N_m: float = 1800.0
    damping_N_s_m: float = 12.0
    travel_m: float = 0.002
    land_m: float = 0.0002
    coil_force_N: float = 4.5
    stop_stiffness_N_m: float = 100000.0

    def __post_init__(self):
        values = asdict(self)
        if not all(math.isfinite(v) and v > 0 for v in values.values()):
            raise ValueError("all spool parameters must be positive and finite")
        if self.land_m >= self.travel_m:
            raise ValueError("metering land must be shorter than travel")


class Spool:
    def __init__(self, parameters: SpoolParameters | None = None, *,
                 position_m: float = 0.0, velocity_m_s: float = 0.0):
        self.parameters = parameters or SpoolParameters()
        if not all(math.isfinite(v) for v in (position_m, velocity_m_s)):
            raise ValueError("spool initial state must be finite")
        self.position_m = float(position_m)
        self.velocity_m_s = float(velocity_m_s)
        self.command = 0.0
        self.coil_failed = False
        self.coil_work_J = 0.0
        self.damping_loss_J = 0.0
        self.initial_energy_J = self.energy_J()
        self.max_stop_penetration_m = 0.0

    def opening_at(self, position_m: float) -> float:
        p = self.parameters
        return min(1.0, max(0.0, (position_m - p.land_m) / (p.travel_m - p.land_m)))

    @property
    def opening(self):
        return self.opening_at(self.position_m)

    def potential_J(self, x):
        p = self.parameters
        return (0.5 * p.spring_N_m * x*x + 0.5 * p.stop_stiffness_N_m *
                (min(x, 0.0)**2 + max(x-p.travel_m, 0.0)**2))

    def energy_J(self):
        return 0.5*self.parameters.mass_kg*self.velocity_m_s**2 + self.potential_J(self.position_m)

    @staticmethod
    def _square_positive_gradient(a, b):
        """Divided difference of 0.5*max(x,0)^2 without cancellation."""
        if a >= 0 and b >= 0:
            return 0.5*(a+b)
        if a <= 0 and b <= 0:
            return 0.0
        return 0.5*(max(b, 0.0)**2-max(a, 0.0)**2)/(b-a)

    def _potential_gradient(self, a, b):
        p = self.parameters
        return (0.5*p.spring_N_m*(a+b) + p.stop_stiffness_N_m *
                (-self._square_positive_gradient(-a, -b) +
                 self._square_positive_gradient(a-p.travel_m, b-p.travel_m)))

    def advance(self, dt_s: float, command: float | None = None):
        if not math.isfinite(dt_s) or dt_s < 0:
            raise ValueError("dt must be finite and nonnegative")
        if command is not None and (not math.isfinite(command) or not 0 <= command <= 1):
            raise ValueError("coil command must lie in [0,1]")
        if command is not None:
            self.command = float(command)
        if dt_s == 0:
            return self.opening
        p = self.parameters
        a, v0, h = self.position_m, self.velocity_m_s, dt_s
        force = 0.0 if self.coil_failed else p.coil_force_N*self.command
        if a == 0 and v0 == 0 and force == 0:
            return 0.0
        stiffness = p.spring_N_m
        offset = 0.0
        if a < 0:
            stiffness += p.stop_stiffness_N_m
        elif a > p.travel_m:
            stiffness += p.stop_stiffness_N_m
            offset = -p.stop_stiffness_N_m*p.travel_m
        inertia = 2*p.mass_kg/(h*h) + p.damping_N_s_m/h
        rhs = 2*p.mass_kg*v0/h + force
        b = a + (rhs-stiffness*a-offset)/(inertia+0.5*stiffness)
        same_region = ((a < 0 and b < 0) or (a > p.travel_m and b > p.travel_m) or
                       (0 <= a <= p.travel_m and 0 <= b <= p.travel_m))
        if not same_region:
            # Strongly monotone scalar equation; no state clipping or reset.
            def residual(x):
                return inertia*(x-a) + self._potential_gradient(a, x) - rhs
            radius = max(p.travel_m, abs(b-a), abs(v0*h))
            lo, hi = a-radius, a+radius
            while residual(lo) > 0:
                radius *= 2; lo = a-radius
            while residual(hi) < 0:
                radius *= 2; hi = a+radius
            for _ in range(48):
                b = 0.5*(lo+hi)
                if residual(b) > 0:
                    hi = b
                else:
                    lo = b
            b = 0.5*(lo+hi)
        dx = b-a
        self.position_m = b
        self.velocity_m_s = 2*dx/h-v0
        self.coil_work_J += force*dx
        self.damping_loss_J += p.damping_N_s_m*(dx/h)**2*h
        self.max_stop_penetration_m = max(self.max_stop_penetration_m, -b, b-p.travel_m)
        return self.opening_at(0.5*(a+b))

    def audit(self):
        return dict(energy_J=self.energy_J(), coil_work_J=self.coil_work_J,
                    damping_loss_J=self.damping_loss_J,
                    balance_residual_J=self.energy_J()-self.initial_energy_J-self.coil_work_J+self.damping_loss_J,
                    max_stop_penetration_m=self.max_stop_penetration_m)

    def snapshot(self):
        return dict(position_m=self.position_m, velocity_m_s=self.velocity_m_s,
                    opening=self.opening, coil_force_fraction=self.command,
                    coil_failed=self.coil_failed, **self.audit())


class DynamicValveNetwork:
    """Compatible B8 network facade; all metering commands act on spool forces.

    Leak ports retain their fault-aperture inputs; compressor retains its B8
    lumped source. Spools are pressure balanced (zero differential-pressure
    force/swept gas volume), supported by an external test bench. Their mass is
    not added to vehicle mass; mount reaction/inertia is outside this model.
    """
    def __init__(self, gas, *, parameters: SpoolParameters | None = None,
                 port_parameters: Mapping[str, SpoolParameters] | None = None):
        self.gas = gas
        names = [n for n in gas.orifices if n.endswith(("_supply", "_exhaust"))]
        overrides = dict(port_parameters or {})
        if set(overrides)-set(names):
            raise ValueError("unknown dynamic port parameter override")
        if any(gas.orifices[n].opening != 0 for n in names):
            raise ValueError("dynamic ports must initially be closed")
        self.spools = {n: Spool(overrides.get(n, parameters)) for n in names}
        self.commands = ({n: o.opening for n, o in gas.orifices.items()} |
                         {n: c.opening for n, c in gas.compressors.items()})
        self.initial_spool_states = {n: s.snapshot() for n, s in self.spools.items()}

    def __getattr__(self, name):
        return getattr(self.gas, name)

    def set_coil_failure(self, port: str, failed: bool):
        if port not in self.spools or not isinstance(failed, bool):
            raise ValueError("known dynamic port and boolean fault required")
        self.spools[port].coil_failed = failed

    def close_all_valves(self):
        # Commands go to zero; mechanical inertia and spring closure persist.
        self.commands = {n: 0.0 for n in self.commands}
        for s in self.spools.values():
            s.command = 0.0
        for n, o in self.gas.orifices.items():
            if n not in self.spools:
                o.opening = 0.0
        for c in self.gas.compressors.values():
            c.opening = 0.0

    def update(self, dt, displacements_m=None, valve_commands=None, *, snapshot=True):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        positions = dict(displacements_m or {})
        commands = dict(valve_commands or {})
        # Complete validation before any command, gas or spool state mutation.
        for n, x in positions.items():
            if n not in self.gas.volumes or self.gas.volumes[n].piston_area_m2 <= 0:
                raise ValueError("unknown/non-actuated displacement node: "+n)
            v = self.gas.volumes[n]
            if not math.isfinite(x) or v.dead_volume_m3+v.piston_area_m2*x <= 0:
                raise ValueError("invalid chamber displacement: "+n)
            if dt == 0 and x != v.displacement_m:
                raise ValueError("a displacement change requires dt > 0")
        for n, u in commands.items():
            if n not in self.commands or not math.isfinite(u) or not 0 <= u <= 1:
                raise ValueError("invalid valve command: "+n)
        self.commands.update(commands)
        for n, s in self.spools.items():
            s.command = self.commands[n]
        if dt == 0:
            return self.snapshot() if snapshot else None
        count = max(1, math.ceil(dt/self.gas.max_substep_s))
        h = dt/count
        start = {n: self.gas.volumes[n].displacement_m for n in positions}
        edge_before = dict(self.gas.edge_mass_kg)
        for k in range(count):
            actual = dict(self.commands)
            for n, s in self.spools.items():
                actual[n] = s.advance(h)
            endpoint = {n: start[n]+(x-start[n])*(k+1)/count for n, x in positions.items()}
            self.gas.update(h, endpoint, actual, snapshot=False)
            for n, s in self.spools.items():
                self.gas.orifices[n].opening = s.opening
        self.gas.last_flows_kg_s = {n: (mass-edge_before[n])/dt for n, mass in self.gas.edge_mass_kg.items()}
        return self.snapshot() if snapshot else None

    def spool_audit(self):
        audits = [s.audit() for s in self.spools.values()]
        return dict(port_count=len(audits), mechanical_state_count=2*len(audits),
                    energy_J=math.fsum(a["energy_J"] for a in audits),
                    coil_work_J=math.fsum(a["coil_work_J"] for a in audits),
                    damping_loss_J=math.fsum(a["damping_loss_J"] for a in audits),
                    balance_residual_J=math.fsum(a["balance_residual_J"] for a in audits),
                    max_individual_balance_residual_J=max((abs(a["balance_residual_J"]) for a in audits), default=0.0),
                    max_stop_penetration_m=max((a["max_stop_penetration_m"] for a in audits), default=0.0))

    def configuration(self):
        return self.gas.configuration() | dict(dynamic_metering_ports=dict(
            provenance=PROVENANCE, pressure_balance_area_m2=0.0,
            mount="external fixed test bench; no vehicle mount reaction coupling",
            port_count=len(self.spools), state_count=2*len(self.spools),
            topology="one independent normally-closed proxy per B8 metering edge; not a hardware valve count",
            parameters={n: asdict(s.parameters) for n, s in self.spools.items()},
            initial_states=self.initial_spool_states,
            coil_failures={n: s.coil_failed for n, s in self.spools.items()},
            requested_coil_force_fractions=dict(self.commands), audit=self.spool_audit()))

    def snapshot(self):
        return self.gas.snapshot() | dict(spools={n: s.snapshot() for n, s in self.spools.items()},
                                         spool_audit=self.spool_audit())


def make_vehicle_network(*args, spool_parameters=None, port_parameters=None, **kwargs):
    return DynamicValveNetwork(make_b8_network(*args, **kwargs),
                               parameters=spool_parameters, port_parameters=port_parameters)
