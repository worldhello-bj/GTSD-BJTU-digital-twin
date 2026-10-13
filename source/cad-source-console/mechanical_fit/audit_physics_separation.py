"""Read source/solver contracts without advancing or modifying either solver."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def audit(root=ROOT):
    def read(name):
        return json.loads((root/name).read_text())
    profiles=read('mechanical_fit/profiles.json')
    assert profiles['solver_mapping_unchanged'] is True
    counts={}
    for p in profiles['profiles']:
        rows=[m for g in p['groups'] for m in g['add_meshes']]
        assert all(m['physics_binding'] is False and m['collision_enabled'] is False
                   and m['added_mass_kg']==0 and m['body_name'] is None for m in rows)
        counts[p['id']]=len(rows)
    binding=read('pantograph/candidate/candidate_binding.json')
    review=read('pantograph/binding_review.json')
    xml=ET.parse(root/'pantograph/candidate/candidate.xml').getroot()
    joints=[dict(e.attrib) for e in xml.findall('.//worldbody//joint')]
    assert len(joints)==5 and all(e['axis']=='0 1 0' for e in joints)
    assert xml.find('default/joint').get('limited')=='false'
    assert xml.find('option').get('gravity')=='0 0 0'
    assert binding['contact_enabled'] is False and binding['old_body_pose_reuse'] is False
    paths=['pantograph/candidate/candidate.xml','pantograph/candidate/candidate_binding.json',
           'baseline_console/GTSD_Assembly_Console/app/data/mapping_bundle.json',
           'frame_track/registration.js','frame_track/bogie_registration.js',
           'integration/plant.py','integration/seat_plant.py']
    return {'schema':'gtsd-fit-solver-separation-audit-v1','ok':True,
            'candidate_display_meshes_by_profile':counts,'new_mass_kg':0,'new_collision':False,
            'solver_files_sha256':{p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in paths},
            'existing_pantograph_research':{'joints':joints,'joint_travel_limits':'UNSPECIFIED_NOT_HARDWARE_VALIDATED',
                'gravity_m_s2':[0,0,0],'timestep_s':float(xml.find('option').get('timestep')),
                'motor_control_range_Nm':[-.1,.1],'inertials':binding['test_inertials'],
                'old_reference_geometry_comparison':review['old_comparison'],
                'reference_geometry_moved':binding['reference_geometry_moved'],
                'C_pin_axis_source_mismatch_m':binding['C_pin_axis_source_mismatch_m'],
                'C_axis':'outer plate; not the tie-axis static circle study',
                'D_relation':'equality closure, not a measured bearing joint'},
            'scope':'Contract consistency only. No new hardware fit, full motion-envelope, contact or load validation.'}

if __name__=='__main__':
    print(json.dumps(audit(),indent=2))
