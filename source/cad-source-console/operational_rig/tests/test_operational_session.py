import copy
import math
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import operational_session as mod
from operational_session import RigSession, RigError, RigConflict

PARAMETERS = {'mode': 'operational_assembly', 'time_step_s': .001,
              'rig_mass_kg': 100., 'gravity_m_s2': [0., 0., -9.81], 'rig_fixed': False}


class FakePlant:
    def __init__(self, parameters):
        if parameters['rig_mass_kg'] == 999: raise ValueError('Constructor failure')
        self.contract = {'parameters': copy.deepcopy(parameters), 'bindings': [], 'bodies': [],
                         'demo': {'duration_s': .02}}
        self.identity = 'fixture-' + str(parameters['time_step_s'])
        self.body_ids = {'rig': 0}
        self.steps = 0
        self.ledger = {'input_work_J': 0.}
        self.command_force = {}
        self.fail_at = None

    def checkpoint(self):
        return copy.deepcopy({'identity': self.identity, 'steps': self.steps,
                              'ledger': self.ledger, 'command_force': self.command_force})

    def restore(self, cp):
        if cp['identity'] != self.identity: raise ValueError('Identity mismatch')
        self.steps = cp['steps']
        self.ledger = copy.deepcopy(cp['ledger'])
        self.command_force = copy.deepcopy(cp['command_force'])

    def snapshot(self):
        return {'identity_sha256': self.identity, 'time_s': self.steps * self.contract['parameters']['time_step_s'],
                'steps': self.steps, 'body_poses': {}, 'ledger': copy.deepcopy(self.ledger)}

    def step(self, loads):
        self.steps += 1
        self.command_force = copy.deepcopy(loads)
        self.ledger['input_work_J'] += sum(sum(v) for v in loads.values())
        if self.steps == self.fail_at: raise RuntimeError('Injected post-mutation failure')
        return {'step': self.steps, 'input_world_force_N': copy.deepcopy(loads), 'state': self.snapshot()}

    def demo_loads(self, time_s):
        return {'rig': [time_s, 0., 0.]}


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.p = patch.object(mod, 'parameter_document', return_value={'defaults': PARAMETERS})
        self.p.start()
        self.s = RigSession(FakePlant, step_delay_s=.001)

    def tearDown(self):
        self.s.close()
        self.p.stop()

    def payload(self, **more):
        return dict(server_generation_id=self.s.server_generation_id, session_id=self.s.session_id,
                    expected_revision=self.s.revision, request_id='request-' + str(self.s.revision), **more)

    def initialize(self, parameters=None):
        return self.s.initialize(self.payload(mode='operational_assembly', parameters=parameters or {}))

    def command(self, action, **more):
        return self.s.command(self.payload(action=action, **more))

    def finish(self):
        limit = time.monotonic() + 3
        while self.s.snapshot()['phase'] == 'running' and time.monotonic() < limit: time.sleep(.002)
        self.assertNotEqual(self.s.snapshot()['phase'], 'running')

    def test_read_metadata_does_not_initialize(self):
        meta = self.s.metadata()
        self.assertEqual(meta['schema'], 'gtsd-operational-rig-metadata-v1')
        self.assertNotIn('assumption_ack_required', meta)
        self.assertIsNone(meta['contract'])
        self.assertFalse(self.s.snapshot()['initialized'])

    def test_single_initialization_without_ack(self):
        state = self.initialize()
        self.assertTrue(state['initialized'])
        self.assertEqual(state['mode'], 'operational_assembly')
        self.assertEqual(state['schema'], mod.SCHEMA)

    def test_invalid_parameter_before_native_allocation(self):
        for cfg in ({'unknown': 1}, {'mode': 'old'}, {'rig_mass_kg': True},
                    {'gravity_m_s2': [0, 0]}, {'time_step_s': math.nan}):
            with self.assertRaises(RigError): self.initialize(cfg)
        self.assertFalse(self.s.snapshot()['initialized'])

    def test_constructor_failure_keeps_previous_state(self):
        self.initialize(); before = self.s.snapshot()
        with self.assertRaises(ValueError): self.initialize({'rig_mass_kg': 999})
        self.assertEqual(before, self.s.snapshot())

    def test_repeated_initialization_and_step_integrate_once(self):
        p = self.payload(mode='operational_assembly', parameters={})
        a = self.s.initialize(p)
        self.assertEqual(a, self.s.initialize(p))
        p = self.payload(action='step', steps=10, loads={})
        results = []
        jobs = [threading.Thread(target=lambda: results.append(self.s.command(p))) for _ in range(5)]
        for t in jobs: t.start()
        for t in jobs: t.join()
        self.assertEqual(len(results), 5)
        self.assertTrue(all(x == results[0] for x in results))
        self.assertEqual(self.s._plant.steps, 10)

    def test_generation_and_revision_block_stale_command(self):
        self.initialize()
        p = self.payload(action='step', steps=1, loads={})
        p['server_generation_id'] = 'old'
        with self.assertRaises(RigConflict): self.s.command(p)
        p = self.payload(action='step', steps=1, loads={}); p['expected_revision'] -= 1
        with self.assertRaises(RigConflict): self.s.command(p)

    def test_complete_demo_uses_time_dependent_loads(self):
        self.initialize(); self.command('demo'); self.finish()
        state = self.s.snapshot()
        self.assertEqual(state['steps'], 20)
        self.assertAlmostEqual(self.s._plant.ledger['input_work_J'], sum(i * .001 for i in range(20)))
        self.assertEqual(state['demo']['progress'], 1.)
        self.assertFalse(state['demo_active'])
        with self.assertRaises(RigConflict): self.command('demo')

    def test_demo_replay_request_does_not_integrate_twice(self):
        self.initialize(); p = self.payload(action='demo')
        first = self.s.command(p); self.finish()
        self.assertEqual(first, self.s.command(p))
        self.assertEqual(self.s._plant.steps, 20)

    def test_complete_demo_uses_total_horizon_not_manual_batch_cap(self):
        self.s.max_run_steps = 5
        self.initialize(); self.command('demo'); self.finish()
        self.assertEqual(self.s._plant.steps, 20)
        self.assertEqual(self.s.snapshot()['demo']['progress'], 1.)

    def test_pause_demo_resume_and_restore_match_uninterrupted(self):
        self.initialize(); self.command('demo'); self.command('pause'); self.finish()
        state = self.s.snapshot(); self.assertLess(state['steps'], 20)
        self.command('checkpoint'); cid = self.s.snapshot()['checkpoints'][0]['id']
        self.command('demo'); self.finish(); final = self.s._plant.checkpoint()
        self.command('restore', checkpoint_id=cid)
        self.command('demo'); self.finish()
        self.assertEqual(final, self.s._plant.checkpoint())

    def test_whole_demo_failure_rolls_back_native_and_ledger(self):
        self.initialize(); before = self.s._plant.checkpoint(); self.s._plant.fail_at = 7
        self.command('demo'); self.finish()
        self.assertEqual(before, self.s._plant.checkpoint())
        self.assertEqual(self.s.snapshot()['phase'], 'error')
        self.assertFalse(self.s.snapshot()['demo_active'])
        with self.assertRaises(RigConflict): self.command('demo')
        self.command('reset'); self.assertEqual(self.s.snapshot()['steps'], 0)

    def test_short_step_failure_restores_prior_trace(self):
        self.initialize(); self.command('step', steps=1, loads={})
        before = self.s._plant.checkpoint(); trace = self.s.snapshot()['last_step_trace']
        self.s._plant.fail_at = 5
        self.command('step', steps=10, loads={})
        self.assertEqual(before, self.s._plant.checkpoint())
        self.assertEqual(trace, self.s.snapshot()['last_step_trace'])

    def test_saved_checkpoint_export_detached_and_reset(self):
        self.initialize(); self.command('step', steps=10, loads={'rig': [1., 0., 0.]})
        cid = self.command('checkpoint')['created_checkpoint_id']
        a = self.s.export_checkpoint(cid, self.s.session_id)
        self.assertEqual(a['schema'], 'gtsd-operational-rig-checkpoint-export-v1')
        a['checkpoint']['state']['ledger']['input_work_J'] = -1
        self.command('reset'); self.command('restore', checkpoint_id=cid)
        self.assertEqual(self.s._plant.ledger['input_work_J'], 10.)
        with self.assertRaises(RigConflict): self.s.export_checkpoint(cid, 'old')

    def test_limits_are_reported_and_enforced(self):
        self.initialize()
        self.assertEqual(self.s.metadata()['limits']['run_steps_max'], 40000)
        for value in (False, 0, 2, 51):
            with self.assertRaises(RigError): self.command('step', steps=value)
        with self.assertRaises(RigError): self.command('run', steps=40001)
        with self.assertRaises(RigError): self.command('step', steps=1, loads={'rig': [21, 0, 0]})


if __name__ == '__main__': unittest.main()
