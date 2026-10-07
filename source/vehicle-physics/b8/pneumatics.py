"""B8 finite-volume, isothermal pneumatic network (SI units).

This module supplies forces from pressures, never cylinder trajectories. Chamber
volumes must be updated from the mechanical solver's actual piston extension.
All default component dimensions/pressures are CALIBRATION_ASSUMPTION values.
Only the Python standard library is required. See PNEUMATICS.md for conventions,
operator-split integration accuracy, heat-bath and exergy accounting boundaries.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
from typing import Mapping, Iterable

AIR_R = 287.05  # J/(kg K), dry-air ideal-gas engineering approximation
AIR_GAMMA = 1.4
AMBIENT_PA = 101325.0
TEMPERATURE_K = 293.15
PROVENANCE = "CALIBRATION_ASSUMPTION: synthetic engineering prototype, not measured"


def orifice_mass_flow_kg_s(p_up: float, p_down: float, temperature_K: float,
                           area_m2: float, discharge_coefficient: float = 0.7,
                           gas_constant: float = AIR_R,
                           gamma: float = AIR_GAMMA) -> float:
    """Nonnegative upstream-to-downstream quasi-steady compressible flow.

    Reservoir pressures are absolute; upstream stagnation temperature is used.
    Input p_up < p_down is an error: network edges handle reversal explicitly.
    Zero area/equal pressure returns zero. Transition into choked flow is
    continuous at (2/(gamma+1))**(gamma/(gamma-1)).
    """
    vals = (p_up, p_down, temperature_K, area_m2, discharge_coefficient,
            gas_constant, gamma)
    if not all(math.isfinite(x) for x in vals):
        raise ValueError("orifice inputs must be finite")
    if p_up <= 0 or p_down < 0 or p_down > p_up or temperature_K <= 0:
        raise ValueError("require p_up > 0, 0 <= p_down <= p_up, T > 0")
    if area_m2 < 0 or not 0 <= discharge_coefficient <= 1 or gas_constant <= 0 or gamma <= 1:
        raise ValueError("invalid area, discharge coefficient or gas constants")
    if area_m2 == 0 or discharge_coefficient == 0 or p_up == p_down:
        return 0.0
    ratio = p_down / p_up
    critical = (2.0 / (gamma + 1.0)) ** (gamma / (gamma - 1.0))
    if ratio <= critical:
        factor = math.sqrt(gamma) * (2.0 / (gamma + 1.0)) ** ((gamma + 1.0) / (2.0 * (gamma - 1.0)))
    else:
        # expm1 avoids cancellation as pressure ratio tends to one.
        log_r = math.log(ratio)
        term = math.exp(2.0 * log_r / gamma) * (-math.expm1((gamma - 1.0) * log_r / gamma))
        factor = math.sqrt(max(0.0, 2.0 * gamma / (gamma - 1.0) * term))
    return discharge_coefficient * area_m2 * p_up / math.sqrt(gas_constant * temperature_K) * factor


@dataclass
class GasVolume:
    name: str
    dead_volume_m3: float
    initial_pressure_Pa: float = AMBIENT_PA
    piston_area_m2: float = 0.0
    displacement_m: float = 0.0
    mass_kg: float = 0.0  # Initialized by PneumaticNetwork, not a supplied state.

    @property
    def volume_m3(self) -> float:
        return self.dead_volume_m3 + self.piston_area_m2 * self.displacement_m


@dataclass
class Orifice:
    name: str
    node_a: str
    node_b: str | None  # None is the unbounded ambient boundary, not a reservoir.
    area_m2: float
    discharge_coefficient: float = 0.7
    opening: float = 0.0
    one_way: bool = False  # True permits only a -> b; False permits actual pressure reversal.


@dataclass
class Compressor:
    name: str
    destination: str
    max_mass_flow_kg_s: float
    cutoff_pressure_Pa: float
    isothermal_efficiency: float = 0.65
    opening: float = 0.0


class PneumaticNetwork:
    """Mass-conservative network of ideal-gas chambers at one fixed temperature.

    update(dt, displacements_m, valve_commands) moves chamber volume along the
    straight segment between two observed mechanical states; it does NOT move
    mechanical states. Query forces_N() after update and apply via the mechanical
    model's matching positive-extension coordinates/Jacobians.

    Gas/valve specifications are copied. Unknown command/node keys are errors.
    Commands persist until changed; call close_all_valves() when appropriate.
    Atmospheric ports are bidirectional unless their Orifice.one_way is true.
    """
    def __init__(self, volumes: Iterable[GasVolume], orifices: Iterable[Orifice] = (),
                 compressors: Iterable[Compressor] = (), *,
                 temperature_K: float = TEMPERATURE_K, ambient_pressure_Pa: float = AMBIENT_PA,
                 gas_constant: float = AIR_R, gamma: float = AIR_GAMMA,
                 max_substep_s: float = 0.0005):
        self.temperature_K = float(temperature_K)
        self.ambient_pressure_Pa = float(ambient_pressure_Pa)
        self.gas_constant = float(gas_constant)
        self.gamma = float(gamma)
        self.max_substep_s = float(max_substep_s)
        if not all(math.isfinite(v) and v > 0 for v in (temperature_K, ambient_pressure_Pa, gas_constant, max_substep_s)) or not math.isfinite(gamma) or gamma <= 1:
            raise ValueError("positive finite temperature, ambient pressure, R, dt and gamma > 1 required")
        self.RT = self.gas_constant * self.temperature_K
        self.cv = self.gas_constant / (self.gamma - 1.0)
        self.cp = self.cv + self.gas_constant
        vv = [GasVolume(**asdict(v)) for v in volumes]
        oo = [Orifice(**asdict(v)) for v in orifices]
        cc = [Compressor(**asdict(v)) for v in compressors]
        self.volumes = {v.name: v for v in vv}
        self.orifices = {v.name: v for v in oo}
        self.compressors = {v.name: v for v in cc}
        if not vv or len(self.volumes) != len(vv) or len(self.orifices) != len(oo) or len(self.compressors) != len(cc):
            raise ValueError("at least one volume and unique names within each component class required")
        if self.orifices.keys() & self.compressors.keys():
            raise ValueError("valve and compressor command names must be distinct")
        for v in self.volumes.values():
            if not all(math.isfinite(x) for x in (v.dead_volume_m3, v.initial_pressure_Pa, v.piston_area_m2, v.displacement_m)):
                raise ValueError("non-finite volume definition")
            if v.dead_volume_m3 <= 0 or v.volume_m3 <= 0 or v.initial_pressure_Pa <= 0 or v.piston_area_m2 < 0:
                raise ValueError("positive pressure and volume, nonnegative piston area required")
            v.mass_kg = v.initial_pressure_Pa * v.volume_m3 / self.RT
        for o in self.orifices.values():
            if o.node_a not in self.volumes or (o.node_b is not None and o.node_b not in self.volumes) or o.node_a == o.node_b:
                raise ValueError(f"invalid endpoint of {o.name}")
            if not math.isfinite(o.area_m2) or o.area_m2 < 0 or not 0 <= o.discharge_coefficient <= 1 or not 0 <= o.opening <= 1:
                raise ValueError(f"invalid orifice {o.name}")
        for c in self.compressors.values():
            if c.destination not in self.volumes or not math.isfinite(c.max_mass_flow_kg_s) or c.max_mass_flow_kg_s < 0 or not math.isfinite(c.cutoff_pressure_Pa) or c.cutoff_pressure_Pa <= 0 or not 0 < c.isothermal_efficiency <= 1 or not 0 <= c.opening <= 1:
                raise ValueError(f"invalid compressor {c.name}")
        self.initial_conditions = {name: dict(pressure_Pa=v.initial_pressure_Pa,
                                              displacement_m=v.displacement_m,
                                              mass_kg=v.mass_kg, volume_m3=v.volume_m3)
                                   for name, v in self.volumes.items()}
        self.time_s = 0.0
        self.initial_mass_kg = self.total_mass_kg()
        self.initial_internal_energy_J = self.internal_energy_J()
        self.initial_exergy_J = self.exergy_J()
        self.ledger = dict(external_mass_in_kg=0.0, external_mass_out_kg=0.0,
                           gas_boundary_work_J=0.0, useful_mechanical_work_J=0.0,
                           ambient_displacement_work_J=0.0, gas_heat_from_bath_J=0.0,
                           external_enthalpy_in_J=0.0, external_enthalpy_out_J=0.0,
                           compressor_work_in_J=0.0, compressor_heat_to_bath_J=0.0,
                           flow_exergy_destroyed_J=0.0, compressor_exergy_destroyed_J=0.0)
        self.edge_mass_kg = {name: 0.0 for name in self.orifices}
        self.compressor_mass_kg = {name: 0.0 for name in self.compressors}
        self.last_flows_kg_s = {name: 0.0 for name in self.orifices}
        self.equalization_caps = 0
        self.substeps = 0

    @property
    def nodes(self):
        """Alias for read-only inspection by the caller; do not mutate states."""
        return self.volumes

    def pressure_Pa(self, name: str) -> float:
        v = self.volumes[name]
        return v.mass_kg * self.RT / v.volume_m3

    def pressures_Pa(self) -> dict[str, float]:
        return {name: self.pressure_Pa(name) for name in self.volumes}

    def forces_N(self) -> dict[str, float]:
        """Gauge pressure forces, positive along increasing cylinder extension."""
        return {n: (self.pressure_Pa(n) - self.ambient_pressure_Pa) * v.piston_area_m2
                for n, v in self.volumes.items() if v.piston_area_m2 > 0}

    def total_mass_kg(self) -> float:
        return math.fsum(v.mass_kg for v in self.volumes.values())

    def internal_energy_J(self) -> float:
        return self.total_mass_kg() * self.cv * self.temperature_K

    def _node_exergy_J(self, name: str) -> float:
        v = self.volumes[name]
        p = self.pressure_Pa(name)
        # Non-flow availability relative to the same-T atmospheric environment.
        delta = (p - self.ambient_pressure_Pa) / self.ambient_pressure_Pa
        if abs(delta) < 1e-5:
            phi = delta * delta * (0.5 + delta * (-1.0 / 6.0 + delta * (1.0 / 12.0 - delta / 20.0)))
        else:
            phi = (1.0 + delta) * math.log1p(delta) - delta
        return self.ambient_pressure_Pa * v.volume_m3 * phi

    def exergy_J(self) -> float:
        return math.fsum(self._node_exergy_J(n) for n in self.volumes)

    def close_all_valves(self) -> None:
        for o in self.orifices.values():
            o.opening = 0.0
        for c in self.compressors.values():
            c.opening = 0.0

    def _move_volumes(self, positions: Mapping[str, float]) -> None:
        for name, x in positions.items():
            v = self.volumes[name]
            old_V = v.volume_m3
            new_V = v.dead_volume_m3 + v.piston_area_m2 * x
            delta_V = new_V - old_V
            work = v.mass_kg * self.RT * math.log1p(delta_V / old_V)
            ambient_work = self.ambient_pressure_Pa * delta_V
            v.displacement_m = x
            self.ledger['gas_boundary_work_J'] += work
            self.ledger['useful_mechanical_work_J'] += work - ambient_work
            self.ledger['ambient_displacement_work_J'] += ambient_work
            # Fixed-mass isothermal geometry substep: ΔU=0, Q=W_pdv.
            self.ledger['gas_heat_from_bath_J'] += work

    def _flow_edge(self, o: Orifice, h: float) -> float:
        if o.opening == 0 or o.area_m2 == 0:
            return 0.0
        pa = self.pressure_Pa(o.node_a)
        pb = self.ambient_pressure_Pa if o.node_b is None else self.pressure_Pa(o.node_b)
        if pa == pb or (o.one_way and pa < pb):
            return 0.0
        sign = 1.0 if pa > pb else -1.0
        rate = orifice_mass_flow_kg_s(max(pa, pb), min(pa, pb), self.temperature_K,
                                     o.area_m2 * o.opening, o.discharge_coefficient,
                                     self.gas_constant, self.gamma)
        inv_volume = 1.0 / self.volumes[o.node_a].volume_m3
        if o.node_b is not None:
            inv_volume += 1.0 / self.volumes[o.node_b].volume_m3
        # Exact transfer to equalize this pair at fixed volume. This cap never
        # creates mass, negative density, or overshoot, even for oversized dt.
        dm_equalize = abs(pa - pb) / (self.RT * inv_volume)
        requested = rate * h
        if requested > dm_equalize:
            self.equalization_caps += 1
        dm = sign * min(requested, dm_equalize)
        involved = [o.node_a] + ([o.node_b] if o.node_b is not None else [])
        before_B = math.fsum(self._node_exergy_J(n) for n in involved)
        self.volumes[o.node_a].mass_kg -= dm
        if o.node_b is not None:
            self.volumes[o.node_b].mass_kg += dm
        else:
            # Signed positive dm leaves the modeled pneumatic volumes.
            key = 'external_mass_out_kg' if dm >= 0 else 'external_mass_in_kg'
            self.ledger[key] += abs(dm)
            key = 'external_enthalpy_out_J' if dm >= 0 else 'external_enthalpy_in_J'
            self.ledger[key] += abs(dm) * self.cp * self.temperature_K
            self.ledger['gas_heat_from_bath_J'] += self.RT * dm
        after_B = math.fsum(self._node_exergy_J(n) for n in involved)
        destruction = before_B - after_B
        if destruction < -1e-9 * max(1.0, abs(before_B)):
            raise ArithmeticError("orifice step increased exergy")
        self.ledger['flow_exergy_destroyed_J'] += max(0.0, destruction)
        self.edge_mass_kg[o.name] += dm
        return dm

    def _run_compressor(self, c: Compressor, h: float) -> None:
        v = self.volumes[c.destination]
        p = self.pressure_Pa(c.destination)
        if not c.opening or p >= c.cutoff_pressure_Pa:
            return
        dm = min(c.opening * c.max_mass_flow_kg_s * h,
                 (c.cutoff_pressure_Pa - p) * v.volume_m3 / self.RT)
        before_B = self._node_exergy_J(c.destination)
        v.mass_kg += dm
        delta_B = self._node_exergy_J(c.destination) - before_B
        after_B = self._node_exergy_J(c.destination)
        # Ambient intake below atmospheric pressure requires no compressor work;
        # if a step crosses atmospheric pressure, charge work only above it.
        if self.pressure_Pa(c.destination) <= self.ambient_pressure_Pa:
            minimum_work = 0.0
        elif p < self.ambient_pressure_Pa:
            minimum_work = after_B
        else:
            minimum_work = max(0.0, delta_B)
        work = minimum_work / c.isothermal_efficiency
        self.ledger['external_mass_in_kg'] += dm
        self.ledger['external_enthalpy_in_J'] += dm * self.cp * self.temperature_K
        self.ledger['gas_heat_from_bath_J'] -= self.RT * dm
        self.ledger['compressor_work_in_J'] += work
        # Aftercooling keeps both inlet/outlet air at the heat-bath temperature.
        self.ledger['compressor_heat_to_bath_J'] += work
        self.ledger['compressor_exergy_destroyed_J'] += work - delta_B
        self.compressor_mass_kg[c.name] += dm

    def update(self, dt: float, displacements_m: Mapping[str, float] | None = None,
               valve_commands: Mapping[str, float] | None = None,
               *, snapshot: bool = True) -> dict | None:
        """Advance dt seconds to observed endpoint cylinder displacements.

        dt=0 permits setting commands/reading state but cannot change positions.
        Unspecified displacements hold their last observed value; unspecified
        commands retain the last command. Set snapshot=False for fast stepping.
        The coupling caller is responsible for its own dt-convergence test.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        positions = {} if displacements_m is None else dict(displacements_m)
        commands = {} if valve_commands is None else dict(valve_commands)
        for n, x in positions.items():
            if n not in self.volumes or self.volumes[n].piston_area_m2 <= 0:
                raise ValueError(f"unknown/non-actuated displacement node {n}")
            v = self.volumes[n]
            if not math.isfinite(x) or v.dead_volume_m3 + v.piston_area_m2 * x <= 0:
                raise ValueError(f"nonpositive or nonfinite chamber volume: {n} at x={x}")
            if dt == 0 and x != v.displacement_m:
                raise ValueError("a displacement change requires dt > 0")
        for n, opening in commands.items():
            if n not in self.orifices and n not in self.compressors:
                raise ValueError(f"unknown valve/compressor command {n}")
            if not math.isfinite(opening) or not 0 <= opening <= 1:
                raise ValueError(f"opening for {n} must lie in [0, 1]")
        # Validation above precedes mutation, including commands.
        for n, opening in commands.items():
            (self.orifices[n] if n in self.orifices else self.compressors[n]).opening = opening
        if dt == 0:
            return self.snapshot() if snapshot else None
        count = max(1, math.ceil(dt / self.max_substep_s))
        h = dt / count
        start = {n: self.volumes[n].displacement_m for n in positions}
        edges = list(self.orifices.values())
        dm_this_step = {n: 0.0 for n in self.orifices}
        for k in range(count):
            # Geometry/flow/geometry split. Only observed endpoints enter here.
            mid = {n: start[n] + (x - start[n]) * ((k + 0.5) / count) for n, x in positions.items()}
            end = {n: start[n] + (x - start[n]) * ((k + 1.0) / count) for n, x in positions.items()}
            self._move_volumes(mid)
            # Alternate edge order every step to reduce directional sweep bias.
            ordered = edges if self.substeps % 2 == 0 else reversed(edges)
            for o in ordered:
                dm_this_step[o.name] += self._flow_edge(o, h)
            for c in self.compressors.values():
                self._run_compressor(c, h)
            self._move_volumes(end)
            self.substeps += 1
        self.time_s += dt
        self.last_flows_kg_s = {n: dm / dt for n, dm in dm_this_step.items()}
        return self.snapshot() if snapshot else None

    def audit(self) -> dict[str, float]:
        l = self.ledger
        mass_delta = self.total_mass_kg() - self.initial_mass_kg
        energy_delta = self.internal_energy_J() - self.initial_internal_energy_J
        enthalpy = l['external_enthalpy_in_J'] - l['external_enthalpy_out_J']
        destruction = l['flow_exergy_destroyed_J'] + l['compressor_exergy_destroyed_J']
        exergy_delta = self.exergy_J() - self.initial_exergy_J
        gas_resid = energy_delta - (l['gas_heat_from_bath_J'] + enthalpy - l['gas_boundary_work_J'])
        total_heat = l['gas_heat_from_bath_J'] - l['compressor_heat_to_bath_J']
        combined_resid = energy_delta - (total_heat + l['compressor_work_in_J'] + enthalpy - l['gas_boundary_work_J'])
        return dict(**l, total_mass_kg=self.total_mass_kg(),
                    mass_balance_residual_kg=mass_delta - l['external_mass_in_kg'] + l['external_mass_out_kg'],
                    internal_energy_J=self.internal_energy_J(), internal_energy_delta_J=energy_delta,
                    gas_energy_balance_residual_J=gas_resid,
                    combined_energy_balance_residual_J=combined_resid,
                    combined_heat_from_bath_J=total_heat,
                    exergy_J=self.exergy_J(), exergy_delta_J=exergy_delta,
                    exergy_destroyed_J=destruction,
                    entropy_generated_J_per_K=destruction / self.temperature_K,
                    exergy_balance_residual_J=exergy_delta + l['useful_mechanical_work_J'] + destruction - l['compressor_work_in_J'])

    def configuration(self) -> dict:
        """JSON-ready actual topology/parameters, including modified fault areas.

        Report this alongside mechanical parameters for full reproducibility.
        Initial conditions remain the true construction-time state, even after
        motion. Openings describe the commands at the time of this query.
        """
        return dict(model='isothermal ideal gas, finite-volume compressible orifice network',
                    provenance=PROVENANCE, temperature_K=self.temperature_K,
                    ambient_pressure_Pa=self.ambient_pressure_Pa,
                    gas_constant_J_kg_K=self.gas_constant, gamma=self.gamma,
                    max_substep_s=self.max_substep_s,
                    volumes={n: dict(dead_volume_m3=v.dead_volume_m3,
                                     piston_area_m2=v.piston_area_m2,
                                     initial_condition=dict(self.initial_conditions[n]))
                             for n, v in self.volumes.items()},
                    orifices={n: asdict(o) for n, o in self.orifices.items()},
                    compressors={n: asdict(c) for n, c in self.compressors.items()})

    def snapshot(self) -> dict:
        return dict(time_s=self.time_s, pressure_Pa=self.pressures_Pa(), force_N=self.forces_N(),
                    mass_kg={n: v.mass_kg for n, v in self.volumes.items()},
                    volume_m3={n: v.volume_m3 for n, v in self.volumes.items()},
                    displacement_m={n: v.displacement_m for n, v in self.volumes.items() if v.piston_area_m2},
                    valve_opening={n: o.opening for n, o in self.orifices.items()},
                    flow_kg_s=dict(self.last_flows_kg_s), cumulative_edge_mass_kg=dict(self.edge_mass_kg),
                    audit=self.audit(), equalization_caps=self.equalization_caps, substeps=self.substeps)


def make_vehicle_network(brake_names: Iterable[str] = ('brake_front_left', 'brake_front_right',
                                                      'brake_rear_left', 'brake_rear_right'), *,
                         reservoir_volume_m3: float = 0.002,
                         reservoir_initial_pressure_Pa: float = AMBIENT_PA + 500000.0,
                         brake_area_m2: float = 6e-5, brake_dead_volume_m3: float = 2e-6,
                         brake_supply_orifice_m2: float = 8e-9,
                         brake_exhaust_orifice_m2: float = 1.6e-8,
                         pantograph_area_m2: float = 0.0002,
                         pantograph_dead_volume_m3: float = 4e-5,
                         pantograph_supply_orifice_m2: float = 6e-8,
                         pantograph_exhaust_orifice_m2: float = 6e-8,
                         pantograph_quick_exhaust_orifice_m2: float = 6e-7,
                         initial_displacements_m: Mapping[str, float] | None = None,
                         include_pantograph: bool = True,
                         extra_chambers: Iterable[GasVolume] = (),
                         extra_supply_orifice_m2: float = 6e-8,
                         extra_exhaust_orifice_m2: float = 6e-8,
                         with_compressor: bool = False,
                         max_substep_s: float = 0.0005) -> PneumaticNetwork:
    """Assumed-scale B8 network; every configurable numeric default is uncalibrated.

    Supply valves include check-flow behavior; exhaust and leak ports are
    bidirectional to model atmospheric intake under sub-ambient pressure.
    Leak ports default shut; command them explicitly to inject a leak failure.
    Supply-loss test: command reservoir_leak=1 and close compressor, rather than
    setting any pressure or mechanical state directly.
    """
    names = list(brake_names)
    if len(names) != len(set(names)) or any(n in ('reservoir', 'pantograph') for n in names):
        raise ValueError('brake chamber names must be unique and distinct from reservoir/pantograph')
    extras = [GasVolume(**asdict(v)) for v in extra_chambers]
    extra_names = [v.name for v in extras]
    if len(extra_names) != len(set(extra_names)) or set(extra_names) & (set(names) | {'reservoir', 'pantograph'}):
        raise ValueError('extra chamber names must be distinct from all existing nodes')
    if any(v.piston_area_m2 <= 0 for v in extras):
        raise ValueError('extra_chambers must have positive piston areas')
    x = {} if initial_displacements_m is None else dict(initial_displacements_m)
    if set(x) - set(names) - set(extra_names) - ({'pantograph'} if include_pantograph else set()):
        raise ValueError('unknown initial displacement')
    vv = [GasVolume('reservoir', reservoir_volume_m3, reservoir_initial_pressure_Pa),
          GasVolume('pantograph', pantograph_dead_volume_m3, AMBIENT_PA,
                    pantograph_area_m2, x.get('pantograph', 0.0))]
    oo = [Orifice('pantograph_supply', 'reservoir', 'pantograph', pantograph_supply_orifice_m2, one_way=True),
          Orifice('pantograph_exhaust', 'pantograph', None, pantograph_exhaust_orifice_m2),
          Orifice('pantograph_quick_exhaust', 'pantograph', None, pantograph_quick_exhaust_orifice_m2),
          Orifice('pantograph_leak', 'pantograph', None, 1e-7),
          Orifice('reservoir_leak', 'reservoir', None, 2e-6)]
    if not include_pantograph:
        vv = [v for v in vv if v.name != 'pantograph']
        oo = [o for o in oo if not o.name.startswith('pantograph_')]
    for v in extras:
        if v.name in x:
            v.displacement_m = x[v.name]
        vv.append(v)
        oo += [Orifice(v.name + '_supply', 'reservoir', v.name, extra_supply_orifice_m2, one_way=True),
               Orifice(v.name + '_exhaust', v.name, None, extra_exhaust_orifice_m2)]
    for n in names:
        vv.append(GasVolume(n, brake_dead_volume_m3, AMBIENT_PA, brake_area_m2, x.get(n, 0.0)))
        oo += [Orifice(n + '_supply', 'reservoir', n, brake_supply_orifice_m2, one_way=True),
               Orifice(n + '_exhaust', n, None, brake_exhaust_orifice_m2),
               Orifice(n + '_leak', n, None, 3e-9)]
    cc = [Compressor('compressor', 'reservoir', 1e-4, AMBIENT_PA + 500000.0)] if with_compressor else []
    return PneumaticNetwork(vv, oo, cc, max_substep_s=max_substep_s)
