import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from validate_contact_stage import compare


def case(dt=.0001):
    sample={'state':{'qvel':[.02,0.], 'body_poses':{'b':{'position':[0,0,0],'quaternion_wxyz':[1,0,0,0]}},
                     'energy_balance_residual_J':1e-8},'signed_axial_impulse_Ns':.01}
    return {'samples':[copy.deepcopy(sample) for _ in range(2)], 'reactions_N':[1.]*40,
            'time_step_s':dt,'onset_time_s':.001,'minimum_used_gap_m':-1e-6,
            'checkpoint_exact_replay':True,'contact_work_generalized_closure_max_J':1e-24,'wall_s':.2}


class MetricsTest(unittest.TestCase):
    def test_exact_match(self):
        self.assertTrue(compare(case(),case(.00005))['passed_declared_gates'])

    def test_per_dof_velocity_gate(self):
        b=case(.00005)
        for sample in b['samples']:sample['state']['qvel'][0]+=.0003
        self.assertTrue(compare(case(),b)['gates']['velocity_per_dof_max_1e_4_or_2percent_peak'])
        b['samples'][0]['state']['qvel'][1]=.0002
        self.assertEqual(compare(case(),b)['failed_velocity_dofs'],[1])

    def test_raw_spike_is_not_hidden_by_smoothing(self):
        b=case(.00005);b['reactions_N'][10]=2.
        self.assertFalse(compare(case(),b)['gates']['raw_peak_5percent'])

    def test_shifted_onset_and_failed_replay_fail(self):
        b=case(.00005);b['onset_time_s']=.0015;b['checkpoint_exact_replay']=False
        result=compare(case(),b)
        self.assertFalse(result['gates']['onset_two_coarse_steps'])
        self.assertFalse(result['gates']['checkpoint_exact'])
