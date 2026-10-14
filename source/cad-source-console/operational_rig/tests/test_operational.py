import copy
import json
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from operational_rig.build_rig import validate_parameters,source_meshes,load_geometry,parameter_sources
from operational_rig.plant import Plant

class Operational(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.plant=Plant()
    def setUp(self):self.plant.restore(self.initial if hasattr(self,'initial') else self.plant.checkpoint())
    def test_all_source_binding_and_mass_unique(self):
        p=self.plant;rows,_=source_meshes();self.assertEqual(len(rows),len(p.contract['bindings']))
        names=[x['mesh_name'] for prop in p.contract['inertial_properties'].values() for x in prop['mass_contributions']]
        self.assertEqual(len(names),len(set(names)));self.assertTrue(all(c['mass_kg']==0 for c in p.contract['colliders']))
        self.assertEqual(p.contract['inertial_properties']['fixed_foundation']['mass_kg'],0)
    def test_geometry_registration(self):
        self.assertLess(self.plant.admission['initial_pose_max_error_m'],1e-12)
        self.assertEqual(load_geometry()['units'],'m')
    def test_recover_exact(self):
        p=self.plant;cp=p.checkpoint()
        for _ in range(30):p.step(p.demo_loads(p.data.time))
        end=p.checkpoint();p.restore(cp)
        for _ in range(30):p.step(p.demo_loads(p.data.time))
        self.assertEqual(p.checkpoint(),end);p.restore(cp)
    def test_foreign_checkpoint_rejected(self):
        cp=self.plant.checkpoint();cp['identity_sha256']='historical-rig'
        with self.assertRaises(ValueError):self.plant.restore(cp)
    def test_bad_step_atomic(self):
        p=self.plant;cp=p.checkpoint()
        with self.assertRaises(ValueError):p.step({'wheel':[21,0,0]})
        self.assertEqual(cp,p.checkpoint())
    def test_parameter_sources_complete(self):
        self.assertEqual(set(validate_parameters()),set(x['key'] for x in parameter_sources()))
    def test_full_pin_variant_geometry(self):
        rows,_=source_meshes(parameters={'pin_diameter_mm':4.9})
        m=next(x for x in rows if x['name']=='OP_PIN_50DF4');v=np.array(m['vertices']).reshape(-1,3)
        self.assertAlmostEqual(np.ptp(v[:,0]),.0049,12)
    def test_shared_rig_transmits_single_branch_load(self):
        p=self.plant;cp=p.checkpoint()
        for _ in range(100):p.step({})
        baseline=p.snapshot();p.restore(cp)
        for _ in range(100):p.step({'balise_02':[0,1,0]})
        loaded=p.snapshot();p.restore(cp)
        for name in ('rig','catenary_50DF4','balise_03'):
            a=np.array(baseline['body_poses'][name]['position']);b=np.array(loaded['body_poses'][name]['position'])
            self.assertGreater(float(np.linalg.norm(a-b)),1e-12,name)

    def test_contact_owners_complete(self):
        self.assertEqual(set(self.plant.contract['geometry_to_interface'].values()),{'wheel_rail','pin_50DF4','pin_51409','messenger_jaw'})



class CollisionRepresentation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.p=Plant()
    def test_analytic_wheel_inertia_and_display_bound(self):
        p=self.p;r=p.contract['parameters']['wheel_radius_m'];w=p.contract['parameters']['wheel_width_m'];m=p.contract['parameters']['wheel_mass_kg']
        self.assertLess(p.contract['wheel_representation']['maximum_radial_display_error_m'],6.1e-6)
        inertia=np.array(p.contract['inertial_properties']['wheel']['inertia_com_world_kg_m2'])
        np.testing.assert_allclose(np.diag(inertia),[m*(3*r*r+w*w)/12,m*r*r/2,m*(3*r*r+w*w)/12],rtol=1e-12)
    def test_separation_certificate_and_true_overlap(self):
        import mujoco
        p=self.p;r=json.loads((ROOT/'operational_rig/evidence/v2_pin_event_row.json').read_text());cp=p.checkpoint()
        p.data.qpos[:]=np.array(r['qpos'])-.000025*np.array(r['qvel']);mujoco.mj_kinematics(p.model,p.data)
        pin=mujoco.mj_name2id(p.model,mujoco.mjtObj.mjOBJ_GEOM,'pin_50DF4');cell=mujoco.mj_name2id(p.model,mujoco.mjtObj.mjOBJ_GEOM,'OP_CAD_50DF4_F643_profile_0_cell_0133')
        certificate=p.collision_guard.certificate(p.data,pin,cell);self.assertGreater(certificate['gap_m']-certificate['roundoff_bound_m'],1e-12);self.assertEqual(p.collision_guard.reject(p.data,pin,cell),1)
        cb=p.collision_guard.geom_bodies[cell];original=p.data.xpos[cb].copy();axis=np.array(certificate['axis_world']);accepted=False
        for sign in [-1,1]:
            p.data.xpos[cb]=original+sign*axis*1e-9
            if p.collision_guard.certificate(p.data,pin,cell)['gap_m']<0:
                self.assertEqual(p.collision_guard.reject(p.data,pin,cell),0);accepted=True
        self.assertTrue(accepted);p.restore(cp)
    def test_mixed_module_dispatch_and_fresh_restore_data(self):
        import importlib,mujoco
        from operational_rig import collision_guard as a
        sys.path.insert(0,str(ROOT/'operational_rig'));b=importlib.import_module('collision_guard')
        p=self.p;q=Plant({'friction_coefficient':.25});a.install(p.model);b.install(q.model);a.install(p.model)
        # Keep guards alive through their owning Plants, as production does.
        p.collision_guard=a.install(p.model);q.collision_guard=b.install(q.model)
        cp=p.checkpoint()
        for _ in range(10):p.step({});q.step({})
        expected=p.checkpoint();p.restore(cp)
        for _ in range(10):p.step({})
        self.assertEqual(p.checkpoint(),expected);p.restore(cp)
        unknown=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="1 1 .1" contype="0" conaffinity="0"/><body pos="0 0 .01"><freejoint/><geom type="sphere" size=".1" contype="0" conaffinity="0"/></body></worldbody></mujoco>')
        d=mujoco.MjData(unknown);mujoco.mj_forward(unknown,d);self.assertEqual(d.ncon,0)


    def test_native_contact_sweep_preserves_true_overlap(self):
        import mujoco
        p=self.p;cp=p.checkpoint();r=json.loads((ROOT/'operational_rig/evidence/v2_pin_event_row.json').read_text())
        q0=np.array(r['qpos'])-.000025*np.array(r['qvel'])
        j=mujoco.mj_name2id(p.model,mujoco.mjtObj.mjOBJ_JOINT,'catenary_50DF4_translation_0');adr=int(p.model.jnt_qposadr[j])
        pin=mujoco.mj_name2id(p.model,mujoco.mjtObj.mjOBJ_GEOM,'pin_50DF4')
        cells=[mujoco.mj_name2id(p.model,mujoco.mjtObj.mjOBJ_GEOM,'OP_CAD_50DF4_F643_profile_0_'+c) for c in ('cell_0133','cell_0175')]
        dispatcher=mujoco.get_mjcb_contactfilter();removed=0;preserved=0
        def hits(data,cell):
            return [(float(c.dist),c.pos.tolist(),c.frame.tolist()) for c in data.contact if {int(c.geom1),int(c.geom2)}=={pin,cell}]
        try:
            for offset in (-1000,-300,-100,-85.09134,-50,-25.93419,-10,-1,0,1,10,25,25.93419,50,85.09134,100,300,1000):
                d=mujoco.MjData(p.model);d.qpos[:]=q0;d.qpos[adr]+=offset*1e-12
                mujoco.set_mjcb_contactfilter(None);mujoco.mj_fwdPosition(p.model,d);before={c:hits(d,c) for c in cells}
                mujoco.set_mjcb_contactfilter(dispatcher);mujoco.mj_fwdPosition(p.model,d)
                for cell in cells:
                    cert=p.collision_guard.certificate(d,pin,cell);after=hits(d,cell)
                    if cert['strictly_separated']:
                        self.assertEqual(after,[]);removed+=len(before[cell])
                    else:
                        self.assertEqual(after,before[cell]);preserved+=len(after)
                        if cert['gap_m']<0:
                            self.assertTrue(all(v[0]>=cert['gap_m']-1e-9 for v in after))
            self.assertGreaterEqual(removed,4);self.assertGreaterEqual(preserved,8)
        finally:
            mujoco.set_mjcb_contactfilter(dispatcher);p.restore(cp)

    def test_unknown_parent_weld_filter_semantics(self):
        import mujoco
        dispatcher=mujoco.get_mjcb_contactfilter()
        def contacts(m):
            d=mujoco.MjData(m);mujoco.mj_fwdPosition(m,d)
            return [(int(c.geom1),int(c.geom2),float(c.dist),c.pos.tolist(),c.frame.tolist()) for c in d.contact]
        try:
            for welded in (False,True):
                for disabled in (False,True):
                    option='<option><flag filterparent="disable"/></option>' if disabled else ''
                    joint='' if welded else '<joint type="slide"/>'
                    xml=f'<mujoco>{option}<worldbody><body><joint type="slide"/><geom type="box" size=".1 .1 .1"/><body>{joint}<geom type="box" size=".1 .1 .1"/></body></body><body pos=".05 0 0"><joint type="slide"/><geom type="box" size=".1 .1 .1"/></body></worldbody></mujoco>'
                    model=mujoco.MjModel.from_xml_string(xml)
                    mujoco.set_mjcb_contactfilter(None);expected=contacts(model)
                    mujoco.set_mjcb_contactfilter(dispatcher);self.assertEqual(contacts(model),expected)
        finally:mujoco.set_mjcb_contactfilter(dispatcher)

if __name__=='__main__':unittest.main()
