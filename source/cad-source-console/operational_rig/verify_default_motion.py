"""Fresh-checkout default motion check without duplicating the full trace archive."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
from pathlib import Path
import time

from plant import Plant


def verify(output, expected_checkpoint=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    plant = Plant()
    dt = plant.contract['parameters']['time_step_s']
    count = math.ceil(plant.contract['demo']['duration_s'] / dt - 1e-10)
    interfaces = set()
    contact_geoms = set()
    maximum_penetration = maximum_energy_residual = closure = 0.
    energy_scale = 1e-9
    maximum_wheel_drop = 0.
    branch_displacement = {name: 0. for name in plant.body_ids}
    trajectory = hashlib.sha256()
    route = set()
    for _ in range(count):
        trace = plant.step(plant.demo_loads(plant.steps * dt))
        state = trace['state']
        maximum_energy_residual = max(maximum_energy_residual, abs(state['energy_balance_residual_J']))
        energy_scale = max(energy_scale,
            abs(state['energy']['total_J'] - plant.energy_reference_J),
            abs(state['ledger']['input_work_J']) + abs(state['ledger']['actuator_work_J'])
            + state['ledger']['known_damping_J'] + abs(state['ledger']['native_constraint_work_J']))
        maximum_wheel_drop = max(maximum_wheel_drop, state['wheel_vertical_drop_m'])
        for name, pose in state['body_poses'].items():
            distance = math.dist(pose['position'], plant.reference_positions[name])
            branch_displacement[name] = max(branch_displacement[name], distance)
        closure = max(closure, abs(trace['contact_generalized_work_closure_error_J']))
        for contact in trace['used_contacts']:
            maximum_penetration = max(maximum_penetration, -contact['signed_gap_m'])
            if contact['normal_force_N'] > 1e-8:
                interfaces.add(contact['interface_id'])
                contact_geoms.update((contact['geom1'], contact['geom2']))
        for name, x in plant.contract['demo']['route_events_x_m'].items():
            if state['wheel_position_world_m'][0] >= x: route.add(name)
        trajectory.update(json.dumps([state['steps'], state['qpos'], state['qvel']],
                                     separators=(',', ':'), allow_nan=False).encode())
        if plant.steps % 10000 == 0:
            print(json.dumps({'phase': 'native_motion', 'steps': plant.steps, 'target_steps': count}), flush=True)
    terminal = plant.checkpoint()
    exact = None
    if expected_checkpoint is not None:
        expected = json.loads(Path(expected_checkpoint).read_text())
        exact = terminal == expected
        assert exact, 'Fresh default checkpoint differs from the qualified producer checkpoint'
    assert len(interfaces) == 4
    assert len(route) == 4
    route_sources = ('rail_OP_approach_', 'rail_PHOTO_FROG_', 'rail_PHOTO_WING_upper_',
                     'rail_PHOTO_RETAIN_right_ENV_main_rail_1_')
    assert all(any(g.startswith(prefix) for g in contact_geoms) for prefix in route_sources)
    assert maximum_wheel_drop < .002
    assert all(branch_displacement[name] > 1e-9 for name in (
        'track_assembly', 'balise_02', 'balise_03', 'catenary_50DF4', 'catenary_51409'))
    assert maximum_penetration < 100e-6
    assert maximum_energy_residual / energy_scale < .02
    assert closure < 1e-8
    assert not any(x.number for x in plant.data.warning)
    now = datetime.now(timezone.utc)
    report = {'schema': 'gtsd-fresh-default-motion-v1', 'passed': True,
              'identity_sha256': plant.identity, 'steps': plant.steps,
              'duration_s': plant.steps * dt, 'time_step_s': dt,
              'interfaces_with_force': sorted(interfaces), 'route_events_crossed': sorted(route),
              'maximum_penetration_m': maximum_penetration,
              'maximum_energy_residual_J': maximum_energy_residual,
              'maximum_energy_residual_fraction': maximum_energy_residual / energy_scale,
              'maximum_wheel_vertical_drop_m': maximum_wheel_drop,
              'maximum_branch_displacement_m': branch_displacement,
              'wheel_contact_geoms_seen': sorted(g for g in contact_geoms if g.startswith('rail_')),
              'maximum_contact_work_closure_error_J': closure,
              'complete_checkpoint_matches_producer': exact,
              'native_qpos_qvel_sequence_sha256': trajectory.hexdigest(),
              'wall_s': time.perf_counter() - started,
              'finished_utc': now.isoformat(),
              'finished_beijing': now.astimezone(timezone(timedelta(hours=8))).isoformat()}
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'checkpoint.json').write_text(json.dumps(terminal, separators=(',', ':')) + '\n')
    print(json.dumps(report), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-checkpoint', type=Path)
    args = parser.parse_args()
    verify(args.output, args.expected_checkpoint)
