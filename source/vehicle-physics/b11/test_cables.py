"""Geometry preservation, source anchor binding and bidirectional force tests."""
import json
import hashlib
import unittest
import numpy as np
import mujoco

import cables
import live
from cable_contact_audit import CableContactAudit


class CableTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = cables.model()
        cls.model = mujoco.MjModel.from_xml_string(cls.xml)
        cls.spec = cables.LAST_SPEC
        cls.base = mujoco.MjModel.from_xml_string(cables.b8_model())

    def test_all_eight_source_centerlines_are_five_connected_routes(self):
        self.assertEqual(len(self.spec["routes"]), 5)
        self.assertEqual(sum(len(r["source_objects"]) for r in self.spec["routes"]), 8)
        self.assertEqual(self.model.nv, 77+3*self.spec["total_links"])
        for r in self.spec["routes"]:
            self.assertLess(abs(r["source_length_relative_change"]), .001)
        self.assertGreater(self.spec["total_cable_mass_kg"], 1)

    def test_existing_vehicle_inertials_and_joint_properties_are_unchanged(self):
        for i in range(self.base.nbody):
            name = self.base.body(i).name
            j = self.model.body(name).id
            for field in ("body_mass", "body_inertia", "body_ipos", "body_iquat"):
                np.testing.assert_array_equal(getattr(self.model, field)[j], getattr(self.base, field)[i])
        for i in range(self.base.njnt):
            j = self.model.joint(self.base.joint(i).name).id
            self.assertEqual(self.model.jnt_stiffness[j], self.base.jnt_stiffness[i])

    def test_reference_attachments_overlap_real_body_anchors(self):
        d = mujoco.MjData(self.model); mujoco.mj_forward(self.model, d)
        for r in self.spec["routes"]:
            end = d.site("cable_"+r["key"]+"_end").xpos
            anchor = d.site("cable_"+r["key"]+"_anchor").xpos
            self.assertLess(np.linalg.norm(end-anchor), 1e-9)
        self.assertEqual({r["moving_body"] for r in self.spec["routes"]},
                         {"A_frame", "access_door_cabinet_PWR", "access_door_cabinet_DAQ"})

    def test_compiled_supports_do_not_inherit_unintended_material_defaults(self):
        parameters=self.spec['parameters']
        for name in ('cable_floor','cable_trough_base','cable_trough_side_-1','cable_trough_side_1'):
            index=self.model.geom(name).id
            self.assertAlmostEqual(self.model.geom_friction[index,0],parameters['floor_friction'])
            self.assertAlmostEqual(self.model.geom_solref[index,0],parameters['floor_time_constant_s'])

    def test_contact_audit_reads_known_penetration_without_advancing_plant(self):
        xml='<mujoco><worldbody><geom type="plane" size="1 1 .1"/><body><freejoint/><geom name="cable_probe_capsule" type="capsule" fromto="0 0 .008 0 0 .028" size=".01" mass="1"/></body></worldbody></mujoco>'
        model=mujoco.MjModel.from_xml_string(xml)
        data=mujoco.MjData(model); mujoco.mj_forward(model,data)
        before=(data.qpos.copy(),data.qvel.copy(),data.time)
        audit=CableContactAudit(model); audit.record(data)
        report=audit.snapshot()
        self.assertAlmostEqual(report['maximum_penetration_m'],.002,places=12)
        self.assertAlmostEqual(report['maximum_penetration_radius_ratio'],.2,places=12)
        self.assertEqual(report['integrated_position_states'],1)
        np.testing.assert_array_equal(data.qpos,before[0]); np.testing.assert_array_equal(data.qvel,before[1])
        self.assertEqual(data.time,before[2])

    def test_rigid_link_lengths_persist_under_applied_force(self):
        m = mujoco.MjModel.from_xml_string(self.xml); m.opt.gravity[:] = 0
        d = mujoco.MjData(m)
        key = self.spec["routes"][0]["body_names"][-1]
        d.xfrc_applied[m.body(key).id,0] = 5
        mujoco.mj_step(m, d, nstep=40); mujoco.mj_forward(m,d)
        for r in self.spec["routes"]:
            points = np.array(r["reference_centerline_world_m"])
            for name, a,b in zip(r["body_names"], points, points[1:]):
                rotation = d.xmat[m.body(name).id].reshape(3,3)
                self.assertLess(abs(np.linalg.norm(rotation@(b-a))-np.linalg.norm(b-a)), 1e-10)
        self.assertEqual(sum(d.warning.number), 0)

    def test_end_force_reacts_on_vehicle_without_state_assignment(self):
        m = mujoco.MjModel.from_xml_string(self.xml); m.opt.gravity[:] = 0
        a, b = mujoco.MjData(m), mujoco.MjData(m)
        key = self.spec["routes"][0]["body_names"][-1]
        b.xfrc_applied[m.body(key).id,0] = 5
        mujoco.mj_step(m,a,nstep=20); mujoco.mj_step(m,b,nstep=20)
        root = m.joint("A_car_free").dofadr[0]
        self.assertGreater(abs(a.qvel[root]-b.qvel[root]), 1e-9)
        self.assertGreater(abs(b.qfrc_constraint[root]), .001)

    def test_release_only_changes_constraint_and_prevents_endpoint_reaction(self):
        m = mujoco.MjModel.from_xml_string(self.xml); m.opt.gravity[:] = 0
        a, b = mujoco.MjData(m), mujoco.MjData(m)
        original = b.qpos.copy()
        r = self.spec["routes"][0]
        b.eq_active[m.equality("cable_"+r["key"]+"_attachment").id] = False
        np.testing.assert_array_equal(original, b.qpos)
        for d in (a,b):
            d.xfrc_applied[m.body(r["body_names"][-1]).id,0] = 5
            mujoco.mj_step(m,d,nstep=20)
        root = m.joint("A_car_free").dofadr[0]
        self.assertGreater(abs(a.qvel[root]-b.qvel[root]), 1e-9)

    def test_live_wiring_and_no_constraint_release_teleport(self):
        v = live.LiveVehicle(settle_s=0)
        self.assertEqual(len(v.network.volumes), 27)
        state = v.data.qpos.copy()
        v.release_cable_attachment("supply_0")
        np.testing.assert_array_equal(state,v.data.qpos)
        v.step(20)
        self.assertEqual(sum(v.data.warning.number), 0)
        with self.assertRaises(ValueError):
            v.release_cable_attachment("unknown")

    def test_simplification_preserves_endpoint_and_source_corners(self):
        p = [[0,0,0], [1,0,0], [1,1,0]]
        q = cables.simplify(p, .001, .2)
        np.testing.assert_array_equal(q[0], p[0]); np.testing.assert_array_equal(q[-1], p[-1])
        self.assertTrue(any(np.array_equal(row,p[1]) for row in q))
        self.assertLessEqual(max(np.linalg.norm(np.diff(q,axis=0),axis=1)), .2+1e-12)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CableTests))
    report = dict(tests_run=result.testsRun, passed=result.wasSuccessful(),
                  failures=[str(t)+"\n"+msg for t,msg in result.failures], errors=[str(t)+"\n"+msg for t,msg in result.errors])
    report['sources_sha256'] = {name:hashlib.sha256((cables.ROOT/name).read_bytes()).hexdigest()
                                for name in ('test_cables.py','cables.py','live.py','source_cables.json','cable_contact_audit.py')}
    (cables.ROOT/"component_verification.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)
