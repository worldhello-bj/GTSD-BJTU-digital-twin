"""Six settled B11 cases: retain B8 thresholds, audit cable geometry and refinement."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
from simulate import CASES
from replay_geometry import contact_geometry


def read_csv(path):
    with path.open(encoding='utf-8', newline='') as f:
        reader = csv.DictReader(f)
        names = reader.fieldnames
        rows = [tuple(float(row[n]) for n in names) for row in reader]
    return np.array(rows, dtype=[(n,'f8') for n in names])


def rotation(q):
    w,x,y,z = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def geometry(replay):
    join = fixed = length = 0.
    for frame in replay['frames']:
        states = frame['body_transforms']
        for route in replay['cable_configuration']['routes']:
            points = np.array(route['reference_centerline_world_m'])
            names = route['body_names']
            fixed = max(fixed, float(np.linalg.norm(np.array(states[names[0]]['position'])-points[0])))
            for i, name in enumerate(names):
                state = states[name]
                a = np.array(state['position'])
                b = a+rotation(state['quaternion_wxyz'])@(points[i+1]-points[i])
                length = max(length, abs(float(np.linalg.norm(b-a)-np.linalg.norm(points[i+1]-points[i]))))
                if i+1 < len(names):
                    join = max(join, float(np.linalg.norm(b-np.array(states[names[i+1]]['position']))))
    return dict(maximum_link_length_error_m=length, maximum_internal_joint_gap_m=join, maximum_fixed_anchor_motion_m=fixed)


def main():
    checks, metrics, arrays, geometries, contact_geometries = [], {}, {}, {}, {}
    def check(name, ok, detail):
        checks.append(dict(name=name, passed=bool(ok), detail=detail))
    def peak(name, values, limit):
        value = float(np.max(np.abs(values)))
        check(name, value < limit, dict(maximum=value, limit=limit))
    for name in ('component', 'regression'):
        path = ROOT/(name+'_verification.json')
        record = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {'passed':False}
        check(name, record['passed'], record)
        check('test_identity_'+name, bool(record.get('sources_sha256')) and
              all(hashlib.sha256((ROOT/file).read_bytes()).hexdigest()==expected
                  for file,expected in record.get('sources_sha256',{}).items()), 'test and plant source bytes')
    for name in CASES:
        folder = ROOT/'results'
        path = folder/(name+'_metrics.json')
        if not path.exists():
            check('complete_'+name, False, 'missing full settled case'); continue
        m = json.loads(path.read_text(encoding='utf-8')); metrics[name] = m
        a = read_csv(folder/(name+'.csv')); arrays[name] = a
        aux = read_csv(folder/(name+'_auxiliaries.csv'))
        spec = m['cable_configuration']
        check('case_configuration_'+name, m['case']==name and m['config']==CASES[name], m['config'])
        expected_dt = CASES[name].get('timestep_s', .00005)
        check('case_timestep_'+name, abs(m['dt_s']-expected_dt)<1e-15 and
              abs(m['parameters']['timestep_s']-expected_dt)<1e-15, dict(actual_s=m['dt_s'], expected_s=expected_dt))
        expected_length = CASES[name].get('cable_maximum_link_length_m', .6)
        check('case_resolution_'+name, spec['parameters']['maximum_link_length_m']==expected_length,
              dict(actual_m=spec['parameters']['maximum_link_length_m'], expected_m=expected_length))
        check('finite_'+name, all(np.isfinite(a[c]).all() for c in a.dtype.names) and all(np.isfinite(aux[c]).all() for c in aux.dtype.names), 'all recorded channels')
        check('full_window_'+name, m['settle_s']==6 and m['duration_s']==10 and abs(a['time_s'][-1]-10)<1e-8, '6 s settling + 10 s observation')
        check('topology_'+name, m['dofs']==77+3*spec['total_links'] and len(spec['routes'])==5 and
              len(m['pneumatic_configuration']['volumes'])==27 and
              m['auxiliary_final_state']['spool_audit']['port_count']==51 and
              not m['pneumatic_configuration']['compressors'], dict(dofs=m['dofs'], links=spec['total_links'], gas_states=27, dynamic_ports=51))
        check('warnings_'+name, not any(m['warning_counts']), m['warning_counts'])
        check('train_mass_'+name, abs(m['total_mass_kg']-265.904)<1e-10, 'B8 vehicle mass unchanged; cables are separate finite masses')
        for label, col, limit in [('mechanical','mechanical_balance_residual_J',.15),('gas','gas_energy_residual_J',1e-5),
                                  ('mass','air_mass_residual_kg',1e-10),('momentum','momentum_balance_residual_Ns',.05)]:
            peak(label+'_'+name, a[col], limit)
        # Preserve raw gas columns; remove only the recorded pump geometry work
        # from the gas work charged to vehicle pneumatic actuators.
        pump_delta = a['pump_geometry_work_J']-a['pump_geometry_work_J'][0]
        peak('vehicle_coupling_'+name, a['coupling_work_discrepancy_J']+pump_delta, .05)
        peak('spool_'+name, aux['spool_balance_residual_J'], 1e-7)
        for obj in ('pump','compressor_fan','bay_fan'):
            for kind in ('mechanical','electrical'):
                peak(kind+'_'+obj+'_'+name, aux[obj+'_'+kind+'_balance_residual_J'], 1e-6)
        peak('pump_gas_geometry_'+name, aux['pump_gas_geometry_work_discrepancy_J'], 1e-6)
        for obj in ('compressor_motor','cabinet'):
            peak('thermal_'+obj+'_'+name, aux[obj+'_balance_residual_J'], 1e-5)
        check('hc_passivity_'+name, max(a['contact_friction_work_J'])<1e-8 and max(a['contact_dissipation_work_J'])<1e-8, 'unchanged B8 passive rail law')
        for route in spec['routes']:
            key = route['key']; mask = a['time_s']<2 if name=='tether_release' and key.startswith('supply_') else np.ones(len(a),dtype=bool)
            # Numerical attachment tolerance is one tenth of the cable radius;
            # it does not assert physical connector compliance or calibration.
            peak('attachment_'+key+'_'+name, a['cable_'+key+'_attachment_error_m'][mask], .1*route['radius_m'])
            check('source_length_'+key+'_'+name, abs(route['source_length_relative_change'])<.001, route['source_length_relative_change'])
        identity = json.loads((folder/(name+'_identity.json')).read_text(encoding='utf-8'))
        check('identity_case_'+name, identity['case']==name and identity['configuration']==spec and
              identity['solver']==m['solver_version']=='3.3.7', 'case, cable parameters and pinned solver')
        for file, expected in identity['sources_sha256'].items():
            check('source_'+name+'_'+file, hashlib.sha256((ROOT.parent/file.replace('\\','/')).read_bytes()).hexdigest()==expected, 'actual executed source')
        for file, expected in identity['outputs_sha256'].items():
            check('output_'+name+'_'+file, hashlib.sha256((folder/file).read_bytes()).hexdigest()==expected, 'actual output association')
        replay = json.loads((folder/(name+'_replay.json')).read_text(encoding='utf-8'))
        check('replay_configuration_'+name, replay['cable_configuration']==spec and replay['parameters']==m['parameters'],
              'same force model and parameters')
        frame_times = np.array([frame['time_s'] for frame in replay['frames']])
        check('replay_window_'+name, len(frame_times)>1 and abs(frame_times[0])<1e-8 and
              abs(frame_times[-1]-10)<1e-8 and np.all(np.diff(frame_times)>0), 'monotonic complete observation')
        positions = np.array([state['position'] for frame in replay['frames'] for state in frame['body_transforms'].values()])
        quaternions = np.array([state['quaternion_wxyz'] for frame in replay['frames'] for state in frame['body_transforms'].values()])
        check('replay_finite_unit_quaternion_'+name, np.isfinite(positions).all() and np.isfinite(quaternions).all() and
              np.max(np.abs(np.linalg.norm(quaternions,axis=1)-1))<1e-10, 'finite poses and unit quaternion')
        steps, stride = round(m['duration_s']/m['dt_s']), max(1,round((1/30)/m['dt_s']))
        expected = steps//stride+1+int(steps%stride!=0)
        check('replay_'+name, len(replay['frames'])==expected and len(replay['reference_zero_qpos_body_transforms'])==44+spec['total_links'], dict(frames=len(replay['frames']), expected=expected, bodies=44+spec['total_links']))
        check('xml_'+name, hashlib.sha256((folder/(name+'.xml')).read_bytes()).hexdigest()==m['model_xml_sha256']==replay['model_xml_sha256'], 'exact XML association')
        g = geometry(replay); geometries[name] = g
        for channel, value in g.items():
            check(channel+'_'+name, value<2e-8, dict(value=value, limit=2e-8))
        contact = contact_geometry(folder/(name+'.xml'),replay,m)
        contact_geometries[name] = contact
        check('native_replay_pose_'+name, contact['maximum_native_pose_position_error_m']<2e-8, contact['maximum_native_pose_position_error_m'])
        # Half the diameter prevents the capsule centerline passing through a
        # support or the smaller contact axis entering the other capsule.
        # It is a numerical proxy domain, not a measured material compliance.
        check('sampled_cable_contact_domain_'+name, contact['maximum_sampled_penetration_radius_ratio']<1.,
              dict(maximum_ratio=contact['maximum_sampled_penetration_radius_ratio'],limit=1.,scope='recorded poses'))
        peak_contact = contact['all_step_global_worst_contact_if_cable']
        if peak_contact is not None:
            check('global_worst_cable_contact_domain_'+name, peak_contact['ratio']<1.,peak_contact)
        integrated=m.get('cable_contact_geometry_all_steps',{})
        for phase,begin,duration in [('settling',0.,m['settle_s']),('observation',m['settle_s'],m['duration_s'])]:
            record=integrated.get(phase,{})
            expected_states=round(duration/m['dt_s'])+1
            covered=(record.get('integrated_position_states')==expected_states and
                     abs(record.get('first_integration_time_s',-100)-begin)<1e-8 and
                     abs(record.get('last_integration_time_s',-100)-begin-duration)<1e-8)
            check('allstep_geometry_coverage_'+phase+'_'+name,covered,
                  dict(actual=record.get('integrated_position_states'),expected=expected_states,
                       first=record.get('first_integration_time_s'),last=record.get('last_integration_time_s')))
        observed=integrated.get('observation',{})
        ratio=observed.get('maximum_penetration_radius_ratio',float('inf'))
        check('allstep_observation_cable_contact_domain_'+name,np.isfinite(ratio) and ratio<1.,observed)
        check('allstep_contains_replay_peak_'+name,np.isfinite(ratio) and
              ratio+1e-10>=contact['maximum_sampled_penetration_radius_ratio'],
              dict(allstep_ratio=ratio,replay_sampled_ratio=contact['maximum_sampled_penetration_radius_ratio']))
        if name=='service_brake':
            check('normal_brake_stop', m['stop_distance_m'] is not None and abs(m['final_speed_mps'])<.01,
                  dict(distance_m=m['stop_distance_m'], final_speed_mps=m['final_speed_mps']))
        if name in ('doors_open_close','doors_half_dt'):
            check('six_doors_'+name, len(m['door_max_opening_rad'])==6, 'all original B8 door leaves retained')
            for key, opened in m['door_max_opening_rad'].items():
                check('door_opens_'+key+'_'+name, opened>.5, opened)
                check('door_relatches_'+key+'_'+name, m['door_final_latched'][key], 'force-solved close and passive latch')
        if name=='tether_release':
            mask = a['time_s']>2.01
            for key in ('supply_0','supply_1','supply_2'):
                peak('released_force_'+key, a['cable_'+key+'_attachment_force_N'][mask], 1e-8)
                check('released_separation_'+key, max(a['cable_'+key+'_attachment_error_m'][mask])>.01,
                      'free end moves independently after equality release')
    if len(metrics)==len(CASES):
        main, half, refined = [metrics[n] for n in ('service_brake','half_dt','spatial_refinement')]
        for name, other, limit in [('halfstep_stop',half,.025), ('spatial_stop',refined,.05)]:
            change = abs(other['stop_distance_m']/main['stop_distance_m']-1) if main['stop_distance_m'] and other['stop_distance_m'] else float('inf')
            check(name, change<limit, dict(relative_change=change, limit=limit))
        collector_change = abs(half['peak_collector_single_contact_N']/main['peak_collector_single_contact_N']-1) if main['peak_collector_single_contact_N'] else float('inf')
        check('halfstep_collector', collector_change<.1, dict(relative_change=collector_change, unchanged_B8_limit=.1))
        for key in main['door_max_opening_rad']:
            opened, fine = metrics['doors_open_close']['door_max_opening_rad'][key], metrics['doors_half_dt']['door_max_opening_rad'][key]
            check('halfstep_door_'+key, abs(fine-opened)<.02, dict(main_rad=opened, half_rad=fine, absolute_limit_rad=.02))
    frozen = json.loads((ROOT.parent/'b8/results/physics_run_identity.json').read_text(encoding='utf-8'))
    for name, expected in frozen['source_files'].items():
        check('frozen_b8_'+name, hashlib.sha256((ROOT.parent/'b8'/name).read_bytes()).hexdigest()==expected, 'frozen source retained')
    writes = []
    for node in ast.walk(ast.parse((ROOT/'simulate.py').read_text(encoding='utf-8'))):
        if isinstance(node,(ast.Assign,ast.AnnAssign,ast.AugAssign)):
            targets = node.targets if isinstance(node,ast.Assign) else [node.target]
            writes += [ast.unparse(t) for t in targets if any(isinstance(part,ast.Attribute) and part.attr in ('qpos','qvel') for part in ast.walk(t))]
    check('no_prescribed_state', not writes, writes)
    passed = len(metrics)==len(CASES) and all(c['passed'] for c in checks)
    report = dict(status='PASS_WITH_MODEL_LIMITATIONS' if passed else 'INCOMPLETE_OR_FAILED', passed=passed,
                  integrated_cases=len(metrics), checks=checks, replay_geometry=geometries,replay_contact_geometry=contact_geometries,
                  scope='six settled source-cable equivalent cases; B8/B9/B10 older full-case acceptance remains historical',
                  limits=['Source centerlines/anchors, inferred density/EI/damping/friction; no hardware calibration.',
                          'Rigid capsule links and isotropic passive ball joints; no axial elasticity.',
                          'Soft endpoint constraints; no literal connector material law.',
                          'No cable electrical continuity, hose pressure-radius coupling, real breakage criterion or internal hardware reconstruction.',
                          'B10 auxiliaries remain fixed external bench; no vehicle mount reaction.'])
    report['limits'].append('Native cable penetration/radius recorded at every integration position, separately for source-pose settling and observation; geometry-domain acceptance applies to the complete settled observation. The intersecting cold source pose is numerical preconditioning, not a validated hardware startup.')
    report['limits'].append('Endpoint attachment errors use 100 Hz CSV samples; replay poses use approximately 30 Hz. Geometry-domain checks do not establish contact-force peak accuracy or measured material compression.')
    (ROOT/'results/verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(status=report['status'], checks=len(checks), failed=[c for c in checks if not c['passed']]),indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
