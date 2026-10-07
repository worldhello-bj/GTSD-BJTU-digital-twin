"""Headless candidate tests; synthetic initial states are confined to tests."""
import unittest
from xml.etree import ElementTree as ET
import numpy as np
from compliant_contact import ContactParameters,CompliantWheelRail,candidate_xml,point_law,mujoco
from full_model import model

class ContactLawTests(unittest.TestCase):
    def test_onset_is_continuous_and_nonadhesive(self):
        p=ContactParameters()
        for vn in [-1.,-.1,0,.01,.1,1.]:
            previous=None
            for depth in [1e-3,1e-6,1e-9,1e-12,0,-1e-12]:
                n,f,u,d,w=point_law(-depth,vn,[1.,2.,3.],p)
                self.assertGreaterEqual(n,0);self.assertGreaterEqual(u,0)
                self.assertLessEqual(d,1e-14);self.assertLessEqual(w,1e-14)
                if depth<=0:self.assertEqual(n,0);self.assertEqual(u,0)
                if previous is not None:self.assertLessEqual(n,previous)
                previous=n
    def test_friction_bound_and_dissipation_including_cutoff(self):
        p=ContactParameters();rng=np.random.default_rng(14)
        for _ in range(3000):
            gap=rng.uniform(-.01,.01);vn=rng.uniform(-2,2);vt=rng.normal(size=3)
            n,f,u,d,w=point_law(gap,vn,vt,p)
            self.assertLessEqual(np.linalg.norm(f),p.friction_coefficient*n+1e-12)
            self.assertLessEqual(d,1e-12);self.assertLessEqual(w,1e-12)
            elastic=p.stiffness_Npm*max(0,-gap)
            # dU/dt=-k*delta*vn, hence P_normal+dU/dt=D.
            self.assertAlmostEqual(n*vn-elastic*vn,d,places=8)
    def test_invalid_parameters(self):
        for kwargs in [dict(stiffness_Npm=0),dict(dissipation_spm=-1),dict(friction_regularization_mps=0),dict(friction_coefficient=-1)]:
            with self.assertRaises(ValueError):ContactParameters(**kwargs)

class GeometryAndWrenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=model({"compliant_wheel_rail": False, "include_access_doors": False});cls.xml,cls.removed=candidate_xml(cls.original)
    def setup_contact(self):
        m=mujoco.MjModel.from_xml_string(self.xml);d=mujoco.MjData(m)
        d.qpos[2]-=.0002 # legitimate prescribed initial penetration, test only
        mujoco.mj_forward(m,d);return m,d,CompliantWheelRail(m)
    def test_geometry_and_all_other_elements_unchanged(self):
        a=ET.fromstring(self.original);b=ET.fromstring(self.xml)
        self.assertEqual(len(self.removed),16)
        for section in ['worldbody','actuator','equality','option','tendon','default']:
            self.assertEqual(ET.tostring(a.find(section)),ET.tostring(b.find(section)))
        orig=list(a.find('contact'));left=list(b.find('contact'))
        self.assertEqual(len(orig)-len(left),16)
        for e in left:self.assertTrue(any(e.attrib==x.attrib for x in orig))
    def test_query_settings_are_restored(self):
        m,d,c=self.setup_contact();tol=float(m.opt.ccd_tolerance);iters=int(m.opt.ccd_iterations)
        c.evaluate(d)
        self.assertEqual(m.opt.ccd_tolerance,tol);self.assertEqual(m.opt.ccd_iterations,iters)
    def test_tread_normal_and_no_native_double_force(self):
        m,d,c=self.setup_contact();r=c.evaluate(d,True)
        self.assertGreater(sum(r['normals_N']),0)
        for record in r['details']:
            self.assertEqual(record['kind'],'wheel')
            self.assertGreater(record['normal_on_wheel'][2],.999)
        ids={pair[1] for pair in c.pairs}
        self.assertFalse(any(ids.intersection(con.geom) for con in d.contact))
    def test_flange_pushes_inward(self):
        m,d,c=self.setup_contact();d.qpos[1]+=.0042;mujoco.mj_forward(m,d)
        r=c.evaluate(d,True);fl=[p for p in r['details'] if p['name']=='A_front_flange_R']
        self.assertEqual(len(fl),1);self.assertGreater(fl[0]['normal_force_N'],0)
        self.assertLess(fl[0]['normal_on_wheel'][1],-.999)
    def test_actual_spin_makes_friction_torque_and_virtual_work(self):
        m,d,c=self.setup_contact();idx=int(m.joint('A_front_axle_y').dofadr[0]);d.qvel[idx]=10
        mujoco.mj_forward(m,d);r=c.evaluate(d,True)
        self.assertLess(r['qforce'][idx],0)
        touched=[x for x in r['details'] if x['name'].startswith('A_front_')]
        self.assertGreater(min(x['tangent_speed_mps'] for x in touched),1.3)
        self.assertAlmostEqual(float(r['qforce']@d.qvel),r['normal_power_W']+r['friction_power_W'],places=9)
        self.assertAlmostEqual(r['wheel_friction_power_W']+r['flange_friction_power_W'],r['friction_power_W'],places=12)
    def test_gap_gradient_matches_normal_velocity(self):
        m,d,c=self.setup_contact();d.qvel[2]=-.07;mujoco.mj_forward(m,d)
        before=c.evaluate(d,True);eps=1e-7
        mujoco.mj_integratePos(m,d.qpos,d.qvel,eps);mujoco.mj_forward(m,d)
        after=c.evaluate(d,True)
        byname={x['name']:x for x in after['details']}
        for x in before['details']:
            measured=(byname[x['name']]['gap_m']-x['gap_m'])/eps
            self.assertAlmostEqual(measured,x['separation_speed_mps'],places=7)
    def test_energy_identity_is_exact_at_a_state(self):
        m,d,c=self.setup_contact();d.qvel[:]=np.random.default_rng(5).normal(size=m.nv)*.01
        mujoco.mj_forward(m,d);r=c.evaluate(d,True)
        udot=sum(c.parameters.stiffness_Npm*max(0,-x['gap_m'])*(-x['separation_speed_mps']) for x in r['details'])
        self.assertAlmostEqual(float(r['qforce']@d.qvel)+udot,r['normal_dissipation_power_W']+r['friction_power_W'],places=8)

if __name__=='__main__':unittest.main(verbosity=2)
