import copy
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from plant import Plant
from build_rig import build


class PlantTest(unittest.TestCase):
    def test_real_compile_mapping(self):
        p=Plant({'gravity_m_s2':[0.,0.,0.]})
        self.assertEqual(p.admission['mesh_bindings'],33)
        self.assertEqual(p.model.nv,50)
        self.assertFalse(p.snapshot()['contact_admitted'])
        self.assertEqual(p.admission['initial_pose_max_error_m'],0.)

    def test_identity_parameters_and_checkpoint(self):
        p=Plant({'gravity_m_s2':[0.,0.,0.]});load={'balise_02':[0.,0.,1.]}
        for _ in range(20):p.step(load)
        cp=p.checkpoint()
        for _ in range(30):p.step(load)
        expected=p.snapshot();p.restore(cp)
        for _ in range(30):p.step(load)
        self.assertEqual(p.snapshot(),expected)
        q=Plant({'gravity_m_s2':[0.,0.,0.],'balise_total_mass_kg':0.7})
        self.assertNotEqual(q.identity,p.identity)
        before=q.snapshot()
        with self.assertRaises(ValueError):q.restore(cp)
        self.assertEqual(q.snapshot(),before)

    def test_world_load_cross_branch_reaction(self):
        p=Plant({'gravity_m_s2':[0.,0.,0.]});fixed=Plant({'gravity_m_s2':[0.,0.,0.],'rig_fixed':True})
        load={'balise_02':[0.,0.,1.]}
        for _ in range(500):p.step(load);fixed.step(load)
        a=p.snapshot();b=fixed.snapshot()
        for name in ['support_03','FIT_50DF4_F643_NOMINAL_PIN5_8A2C_CANDIDATE']:
            reference=p.reference_positions[name]
            moved=np.linalg.norm(np.array(a['body_poses'][name]['position'])-reference)
            locked=np.linalg.norm(np.array(b['body_poses'][name]['position'])-reference)
            self.assertGreater(moved,1e-10)
            self.assertLess(locked,1e-13)

    def test_reject_nonfinite_and_unimplemented_contact(self):
        p=Plant();before=p.snapshot()
        for load in ({'balise_02':[0,0,99.]},{'balise_02':[float('nan'),0,0]},{'bad':[0,0,1]}):
            with self.assertRaises(ValueError):p.step(load)
            self.assertEqual(before,p.snapshot())
        with self.assertRaises(ValueError):build({'mode':'contact_coupled'})
        cp=p.checkpoint();cp['native_state'][1]=float('nan')
        with self.assertRaises(ValueError):p.restore(cp)
        self.assertEqual(before,p.snapshot())

    def test_nondividing_step_never_exceeds_five_second_horizon(self):
        p=Plant({'gravity_m_s2':[0.,0.,0.],'time_step_s':.00019})
        limit=min(100000,int(5./p.model.opt.timestep))
        self.assertLessEqual(limit*p.model.opt.timestep,5.)
        self.assertGreater((limit+1)*p.model.opt.timestep,5.)
        p.steps=limit
        with self.assertRaisesRegex(ValueError,'horizon exhausted'):p.step()

    def test_no_input_zero_gravity_preserves_rest(self):
        p=Plant({'gravity_m_s2':[0.,0.,0.]})
        for _ in range(50):p.step()
        self.assertEqual(p.snapshot()['energy_balance_residual_J'],0.)
        self.assertTrue(np.all(np.array(p.snapshot()['qvel'])==0))

if __name__=='__main__':unittest.main()
