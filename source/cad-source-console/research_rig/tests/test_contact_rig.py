import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from plant import Plant
from mass_properties import canonical_sha


class ContactRigTest(unittest.TestCase):
    def test_source_binding_mass_and_holes(self):
        p=Plant({'mode':'pin_core_contact','gravity_m_s2':[0,0,0]})
        self.assertEqual(p.model.nv,56)
        self.assertEqual(p.model.ngeom,950)
        self.assertEqual(len(p.contract['bindings']),35)
        self.assertLess(abs(p.snapshot()['minimum_contact_gap_m']),1e-8)
        for c in p.contract['colliders']:
            self.assertIn(c['body_name'],p.body_ids)
        expected=sum(b['mass_kg'] for b in p.contract['bodies'])
        self.assertAlmostEqual(float(sum(p.model.body_mass)),expected,places=10)

    def test_complete_geometry_variant_is_visible_and_identified(self):
        a=Plant({'mode':'pin_core_contact','gravity_m_s2':[0,0,0]})
        b=Plant({'mode':'pin_core_contact','gravity_m_s2':[0,0,0],'pin_geometry_variant':'research_d4p98'})
        self.assertNotEqual(a.identity,b.identity)
        self.assertEqual(len(b.contract['display_geometry_overrides']),2)
        bindings={v['mesh_name']:v for v in b.contract['bindings']}
        for m in b.contract['display_geometry_overrides']:
            witness=canonical_sha({'vertices':m['vertices'],'indices':m['indices']})
            self.assertEqual(bindings[m['name']]['source_geometry_sha256'],witness)
            self.assertEqual(next(c for c in b.contract['colliders'] if c['source_mesh_name']==m['name'])['source_geometry_sha256'],witness)
        with self.assertRaises(ValueError):b.restore(a.checkpoint())

    def test_contact_work_and_complete_replay(self):
        p=Plant({'mode':'pin_core_contact','gravity_m_s2':[0,0,0],'pin_geometry_variant':'research_d4p98'})
        load={'FIT_50DF4_F643_NOMINAL_PIN5_8A2C_CANDIDATE':[0,-1,0]}
        for _ in range(20):
            r=p.step(load)
            self.assertLess(abs(r['contact_generalized_work_closure_error_J']),1e-10)
        cp=p.checkpoint()
        for _ in range(20):p.step(load)
        expected=p.snapshot();p.restore(cp)
        for _ in range(20):p.step(load)
        self.assertEqual(p.snapshot(),expected)

if __name__=='__main__':unittest.main()
