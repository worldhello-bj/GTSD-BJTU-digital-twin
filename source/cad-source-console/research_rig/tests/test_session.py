import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from session import RigSession, RigError, RigConflict, ACK


class FakePlant:
    def __init__(self, cfg):
        if cfg.get('rig_mass_kg') == 999:
            raise ValueError('deliberate constructor failure')
        self.contract = {'parameters': cfg, 'bindings': [], 'bodies': [], 'unknown_interfaces': []}
        self.body_ids = {'rig': 0}
        self.identity = 'test-model-' + str(cfg['time_step_s'])
        self.steps = 0
        self.ledger = {'work': 0.}
        self.fail_at = None
        self.command_force = {}
    def checkpoint(self):
        return copy.deepcopy({'steps': self.steps, 'ledger': self.ledger, 'identity': self.identity,
                              'command_force': self.command_force, 'native_state': [self.steps * self.contract['parameters']['time_step_s'], self.steps]})
    def restore(self, cp):
        if cp['identity'] != self.identity:
            raise ValueError('identity mismatch')
        self.steps = cp['steps']; self.ledger = copy.deepcopy(cp['ledger']); self.command_force = copy.deepcopy(cp['command_force'])
    def snapshot(self):
        return {'identity_sha256': self.identity, 'steps': self.steps,
                'time_s': self.steps * self.contract['parameters']['time_step_s'], 'body_poses': {}, 'ledger': copy.deepcopy(self.ledger)}
    def step(self, loads):
        self.steps += 1; self.ledger['work'] += 0.125; self.command_force = copy.deepcopy(loads)
        if self.steps == self.fail_at:
            raise RuntimeError('deliberate numerical failure after state mutation')
        return {'step': self.steps, 'used_contacts': [], 'state': self.snapshot()}


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.s = RigSession(FakePlant, step_delay_s=.003)
    def tearDown(self):
        self.s.close()
    def init(self, **kwargs):
        payload = {'mode': 'mount_only', 'parameters': {}, 'assumption_ack': ACK, 'server_generation_id': self.s.server_generation_id, 'session_id': self.s.session_id,
                   'expected_revision': self.s.revision, 'request_id': 'init-' + str(self.s.revision)}
        payload.update(kwargs)
        return self.s.initialize(payload)
    def command(self, action, **kwargs):
        p = {'action': action, 'server_generation_id': self.s.server_generation_id, 'session_id': self.s.session_id, 'expected_revision': self.s.revision,
             'request_id': 'command-' + str(self.s.revision)}
        p.update(kwargs)
        return self.s.command(p)
    def await_stop(self):
        deadline = time.monotonic() + 5
        while self.s.snapshot()['phase'] == 'running' and time.monotonic() < deadline:
            time.sleep(.002)
        self.assertNotEqual(self.s.snapshot()['phase'], 'running')
    def test_server_generation_changes_on_restart_and_binds_commands(self):
        a = self.s.snapshot()
        restarted = RigSession(FakePlant)
        try:
            b = restarted.snapshot()
            self.assertNotEqual(a['server_generation_id'], b['server_generation_id'])
            for reader in (self.s.metadata, self.s.geometry):
                self.assertEqual(reader()['server_generation_id'], a['server_generation_id'])
            payload = {'mode': 'mount_only', 'parameters': {}, 'assumption_ack': ACK,
                       'server_generation_id': a['server_generation_id'], 'session_id': None,
                       'expected_revision': 0, 'request_id': 'old-generation-initialization'}
            with self.assertRaises(RigConflict): restarted.initialize(payload)
            self.assertFalse(restarted.snapshot()['initialized'])
            self.init()
            generation = self.s.server_generation_id
            self.init()
            self.assertEqual(self.s.server_generation_id, generation)
            command = {'action': 'pause', 'server_generation_id': b['server_generation_id'],
                       'session_id': self.s.session_id, 'expected_revision': self.s.revision,
                       'request_id': 'wrong-generation-command'}
            with self.assertRaises(RigConflict): self.s.command(command)
        finally:
            restarted.close()

    def test_reads_are_disabled_and_native_import_free(self):
        code = "import sys;sys.path.insert(0," + repr(str(ROOT)) + ");from session import RigSession;s=RigSession();s.metadata();s.snapshot();s.geometry();assert 'mujoco' not in sys.modules;assert 'plant' not in sys.modules;assert s.session_id is None"
        subprocess.run([sys.executable, '-c', code], check=True)
        self.assertEqual(self.s.snapshot()['phase'], 'disabled')
    def test_explicit_ack_mode_and_parameters(self):
        with self.assertRaises(RigError): self.init(assumption_ack=False)
        with self.assertRaises(RigError): self.init(mode='strict82')
        with self.assertRaises(RigError): self.init(parameters={'mode': 'mount_only'})
        with self.assertRaises(RigError): self.init(parameters={'rig_mass_kg': True})
        with self.assertRaises(RigError): self.init(parameters={'gravity_m_s2': [0, 0, float('nan')]})
        self.assertFalse(self.s.snapshot()['initialized'])
    def test_initialization_is_idempotent(self):
        payload={'mode':'mount_only','parameters':{},'assumption_ack':ACK,'server_generation_id':self.s.server_generation_id,'session_id':None,'expected_revision':0,'request_id':'a'}
        first=self.s.initialize(payload)
        self.assertEqual(first,self.s.initialize(payload))
        payload['mode']='pin_core_contact'
        with self.assertRaises(RigConflict):self.s.initialize(payload)
    def test_revision_and_request_identity(self):
        a=self.init();p={'action':'step','server_generation_id':a['server_generation_id'],'session_id':a['session_id'],'expected_revision':a['revision'],'request_id':'once','steps':10}
        first=self.s.command(p);self.assertEqual(first,self.s.command(p));self.assertEqual(self.s._plant.steps,10)
        p['steps']=50
        with self.assertRaises(RigConflict):self.s.command(p)
        p['request_id']='new'
        with self.assertRaises(RigConflict):self.s.command(p)
    def test_step_choices_and_force_bounds(self):
        self.init()
        for n in [True,2,0,51,1.0]:
            with self.assertRaises(RigError):self.command('step',steps=n)
        for loads in [{'missing':[0,0,0]},{'rig':[20,20,0]},{'rig':[True,0,0]},{'rig':[0,0]},{'rig':[0,0,float('inf')]}]:
            with self.assertRaises(RigError):self.command('step',steps=1,loads=loads)
        self.assertEqual(self.command('step',steps=1,loads={'rig':[0,0,20]})['steps'],1)
    def test_failed_short_step_rolls_back_everything(self):
        self.init();self.command('step',steps=1);before=self.s._plant.checkpoint();trace=self.s.snapshot()['last_step_trace']
        self.s._plant.fail_at=5
        result=self.command('step',steps=10)
        self.assertEqual(self.s._plant.checkpoint(),before);self.assertEqual(result['last_step_trace'],trace)
        self.assertEqual(result['phase'],'error');self.assertIn('rolled back',result['failure'])
        with self.assertRaises(RigConflict):self.command('step',steps=1)
        self.assertEqual(self.command('reset')['steps'],0)
    def test_run_failure_rolls_back_whole_run(self):
        self.init();self.command('step',steps=1);before=self.s._plant.checkpoint();self.s._plant.fail_at=5
        self.command('run',steps=50,loads={'rig':[1,0,0]});self.await_stop()
        self.assertEqual(before,self.s._plant.checkpoint());self.assertEqual(self.s.snapshot()['run']['outcome'],'rolled_back')
    def test_pause_at_boundary_and_busy_rejection(self):
        self.init();self.command('run',steps=500)
        with self.assertRaises(RigConflict):self.command('reset')
        with self.assertRaises(RigConflict):self.init()
        self.command('pause');self.await_stop();snap=self.s.snapshot()
        self.assertLess(snap['steps'],500);self.assertEqual(snap['run']['outcome'],'paused')
        steps=snap['steps'];time.sleep(.02);self.assertEqual(self.s.snapshot()['steps'],steps)
    def test_bounded_run_finishes_without_reads_driving_it(self):
        self.init();self.command('run',steps=10);self.await_stop()
        self.assertEqual(self.s.snapshot()['steps'],10);self.assertEqual(self.s.snapshot()['run']['outcome'],'completed')
        before=self.s._plant.checkpoint();self.s.metadata();self.s.snapshot();self.s.geometry();self.assertEqual(before,self.s._plant.checkpoint())
    def test_checkpoint_restore_export_and_detachment(self):
        self.init();self.command('step',steps=10,loads={'rig':[1,0,0]});cp=self.command('checkpoint')['created_checkpoint_id'];before=self.s._plant.checkpoint()
        exported=self.s.export_checkpoint(cp,self.s.session_id);exported['checkpoint']['state']['ledger']['work']=999
        self.command('step',steps=50);self.command('restore',checkpoint_id=cp);self.assertEqual(before,self.s._plant.checkpoint())
        with self.assertRaises(RigError):self.command('restore',checkpoint_id='../model.json')
        with self.assertRaises(RigConflict):self.s.export_checkpoint(cp,'old-session')
        for _ in range(7):self.command('checkpoint')
        with self.assertRaises(RigError):self.command('checkpoint')
    def test_configuration_change_new_session_only_and_atomic_failure(self):
        self.init();cp=self.command('checkpoint')['created_checkpoint_id'];old_id=self.s.session_id;before=self.s.snapshot()
        with self.assertRaises(ValueError):self.init(parameters={'rig_mass_kg':999})
        self.assertEqual(before,self.s.snapshot())
        self.init(parameters={'time_step_s':.0001});self.assertNotEqual(old_id,self.s.session_id);self.assertEqual(self.s.snapshot()['checkpoints'],[])
        with self.assertRaises(RigError):self.s.export_checkpoint(cp,self.s.session_id)
    def test_concurrent_repeated_click_integrates_once(self):
        a=self.init();payload={'action':'step','steps':50,'server_generation_id':a['server_generation_id'],'session_id':a['session_id'],'expected_revision':a['revision'],'request_id':'shared'}
        results=[];threads=[threading.Thread(target=lambda:results.append(self.s.command(payload))) for _ in range(6)]
        for t in threads:t.start()
        for t in threads:t.join()
        self.assertEqual(self.s._plant.steps,50);self.assertEqual(len(results),6);self.assertTrue(all(r==results[0] for r in results))
    def test_no_eviction_replays_and_no_extra_fields(self):
        self.init();self.s.max_requests=2;self.command('pause')
        with self.assertRaises(RigConflict):self.command('pause')
        self.assertEqual(len(self.s._requests),2)
        with self.assertRaises(RigError):self.command('pause',parameters={})


class NativeSessionTests(unittest.TestCase):
    def test_actual_native_checkpoint_and_pose(self):
        if importlib.util.find_spec('mujoco') is None:self.skipTest('MuJoCo runtime not installed')
        s=RigSession()
        try:
            state=s.initialize({'server_generation_id':s.server_generation_id,'mode':'mount_only','parameters':{},'assumption_ack':ACK,'session_id':None,'expected_revision':0,'request_id':'init'})
            self.assertEqual(len(s.metadata()['contract']['bindings']),33)
            p=lambda action,**kw:dict(action=action,server_generation_id=s.server_generation_id,session_id=s.session_id,expected_revision=s.revision,request_id=str(s.revision),**kw)
            s.command(p('step',steps=10,loads={'rig':[0,0,1]}));cp=s.command(p('checkpoint'))['created_checkpoint_id'];before=s._plant.checkpoint()
            s.command(p('step',steps=10));s.command(p('restore',checkpoint_id=cp));self.assertEqual(before,s._plant.checkpoint())
            geometry=s.geometry();binding=s.metadata()['contract']['bindings'];meshes={m['name']:m for m in geometry['meshes']}
            for row in binding:self.assertEqual(row['source_geometry_sha256'],meshes[row['mesh_name']]['geometry_sha256'])
        finally:s.close()


if __name__=='__main__':unittest.main()
