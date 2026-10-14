"""Independent no-native regressions for atomic terminal/demo publication."""
import hashlib,json,sys,threading,time,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tests'))
import operational_session as mod
from test_operational_session import FakePlant,PARAMETERS

class AuditTerminal(unittest.TestCase):
 def setUp(self):
  self.patch=patch.object(mod,'parameter_document',return_value={'defaults':PARAMETERS});self.patch.start()
  self.s=mod.RigSession(FakePlant,step_delay_s=0);self.observations=[]
  event=self.s._event
  def observe(action):
   if action.startswith('run_') and action!='run_start':self.observations.append((action,self.s.phase,self.s._demo_active))
   return event(action)
  self.s._event=observe
  self.s.initialize(self.payload(mode='operational_assembly',parameters={}))
 def tearDown(self):self.s.close();self.patch.stop()
 def payload(self,**kw):
  return dict(server_generation_id=self.s.server_generation_id,session_id=self.s.session_id,request_id=str(time.monotonic_ns()),expected_revision=self.s.revision,**kw)
 def test_complete_clears_program_before_terminal_event(self):
  self.s.command(self.payload(action='demo'));self.s._worker.join(3)
  self.assertEqual(self.observations,[('run_completed','paused',False)])
 def test_failure_clears_program_and_rolls_back_atomically(self):
  before=self.s._plant.checkpoint();self.s._plant.fail_at=2
  self.s.command(self.payload(action='demo'));self.s._worker.join(3)
  self.assertEqual(self.observations,[('run_rolled_back','error',False)])
  self.assertEqual(self.s._plant.checkpoint(),before)
 def test_following_manual_step_has_only_requested_loads(self):
  self.s.command(self.payload(action='demo'));self.s._worker.join(3)
  self.s.command(self.payload(action='step',steps=1,loads={'rig':[2.,0.,0.]}))
  self.assertEqual(self.s._plant.command_force,{'rig':[2.,0.,0.]})
 def test_terminal_and_new_demo_no_stale_clear(self):
  old_finished=threading.Event();release_old=threading.Event();new_entered=threading.Event();release_new=threading.Event();index=[0]
  original=self.s._run_steps
  def wrapped(*args):
   index[0]+=1;n=index[0]
   if n==1:
    original(*args);old_finished.set();release_old.wait(3)
   else:
    new_entered.set();release_new.wait(3);original(*args)
  self.s._run_steps=wrapped
  try:
   self.s.command(self.payload(action='run',steps=1,loads={}));self.assertTrue(old_finished.wait(3));old=self.s._worker
   self.s.command(self.payload(action='demo'));self.assertTrue(new_entered.wait(3));self.assertTrue(self.s.snapshot()['demo_active'])
   release_old.set();old.join(3)
   self.assertEqual((self.s.snapshot()['phase'],self.s.snapshot()['demo_active']),('running',True))
  finally:release_old.set();release_new.set()

if __name__ == '__main__': unittest.main()
