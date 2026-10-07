"""Reproducible six-door dynamics verification; no prescribed motion trajectories.

Run: python b8/test_access_doors.py (vendored MuJoCo loaded below).
Simulation loops change only bounded torque controls, applied forces and explicit
latch eq_active flags. Each new simulation starts from the compiler's closed rest.
"""
from pathlib import Path
import ast
import csv
import json
import hashlib
import sys
from xml.etree.ElementTree import fromstring
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'vendor'))
import numpy as np
import mujoco
import access_doors as ad
OUT = ROOT / 'door_results'


def compile_model(dt=.0001, params=None, free_car=False):
    xml, spec = ad.standalone_xml(dt, params=params, free_car=free_car)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    bound = spec.bind(model)
    mujoco.mj_forward(model, data)
    return model, data, spec, bound, xml


def step_for(model, data, duration):
    for _ in range(round(duration/model.opt.timestep)):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)


def openings(bound, data):
    return np.array([d.opening_sign*data.qpos[bound.qpos_addresses[d.key]] for d in bound.spec.doors])


def full_cycle(dt=.0001, save=False):
    m,d,s,b,xml = compile_model(dt)
    limits = np.array([x.opening_limit_rad for x in s.doors])
    torques = {x.key: x.torque_limit_Nm for x in s.doors}
    def poses():
        return {mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i):
                {'position':d.xpos[i].tolist(),'quaternion_wxyz':d.xquat[i].tolist()}
                for i in range(1,m.nbody)}
    reference = poses()
    initial_qpos,initial_qvel = d.qpos.tolist(),d.qvel.tolist()
    frames = [{'time_s':float(d.time),'qpos':initial_qpos,'qvel':initial_qvel,
               'body_transforms':poses(),'access_doors':b.telemetry(d),'eq_active':d.eq_active.tolist()}]
    b.set_opening_torques(d, torques)
    step_for(m,d,.5)
    latched_excursion = float(np.max(np.abs(openings(b,d))))
    q_before, v_before = d.qpos.copy(), d.qvel.copy()
    b.release_latch(d)
    assert np.array_equal(q_before,d.qpos) and np.array_equal(v_before,d.qvel)
    rows, stop_constraint = [], None
    max_open_penetration, max_closed_penetration = 0.,0.
    premature_recatch = None
    # 4 s bounded opening, then 12 s passive spring return, then 1 s at the closed stop.
    for i in range(round(17./dt)):
        t = i*dt
        command = torques if t < 4. else ({x.key:-x.torque_limit_Nm for x in s.doors} if t >= 16. else {})
        b.set_opening_torques(d,command)
        mujoco.mj_step(m,d)
        q = openings(b,d)
        max_open_penetration = max(max_open_penetration,float(np.max(q-limits)))
        max_closed_penetration = max(max_closed_penetration,float(np.max(-q)))
        if i == round(2./dt):
            q_before, v_before = d.qpos.copy(), d.qvel.copy()
            premature_recatch = b.try_latch(d)
            assert not any(premature_recatch.values())
            assert np.array_equal(q_before,d.qpos) and np.array_equal(v_before,d.qvel)
        if i == round(3.9/dt):
            mujoco.mj_forward(m,d)
            stop_constraint = {x.key: float(d.qfrc_constraint[b.qvel_addresses[x.key]]) for x in s.doors}
        if i % max(1,round(.01/dt)) == 0:
            rows.append([float(d.time),*q.tolist()])
        if save and i % max(1,round(.05/dt)) == 0:
            mujoco.mj_forward(m,d)
            frames.append({'time_s':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),
                           'body_transforms':poses(),'access_doors':b.telemetry(d),'eq_active':d.eq_active.tolist()})
        if i == round(15.9/dt):
            returned = q.copy()
    mujoco.mj_forward(m,d)
    b.release(d)
    step_for(m,d,.5)
    q_before, v_before = d.qpos.copy(), d.qvel.copy()
    relatched = b.try_latch(d)
    assert all(relatched.values())
    assert np.array_equal(q_before,d.qpos) and np.array_equal(v_before,d.qvel)
    b.set_opening_torques(d,torques)
    step_for(m,d,.5)
    relatch_excursion = float(np.max(np.abs(openings(b,d))))
    a = np.asarray(rows)
    peak = a[:,1:].max(axis=0)
    assert np.all(peak > .99*limits), (peak,limits)
    assert max_open_penetration < .002, max_open_penetration
    assert max_closed_penetration < .002, max_closed_penetration
    assert np.max(np.abs(returned)) < .006, returned
    assert latched_excursion < .002 and relatch_excursion < .002
    assert all(x.opening_sign*stop_constraint[x.key] < 0 for x in s.doors)
    assert np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))
    result = {'dt_s':dt,'peak_opening_rad':dict(zip(ad.DOOR_KEYS,peak.tolist())),
              'passive_return_opening_rad':dict(zip(ad.DOOR_KEYS,returned.tolist())),
              'max_open_stop_penetration_rad':max_open_penetration,
              'max_closed_stop_penetration_rad':max_closed_penetration,
              'open_stop_constraint_torque_Nm':stop_constraint,
              'initial_latch_max_excursion_rad':latched_excursion,
              'relatched_max_excursion_rad':relatch_excursion,
              'premature_relatch_rejected':not any(premature_recatch.values()),
              'closed_relatch_succeeded':all(relatched.values())}
    if save:
        OUT.mkdir(exist_ok=True)
        (OUT/'access_doors_standalone.xml').write_text(xml)
        (OUT/'access_doors_replay.json').write_text(json.dumps({
            'schema':'B8 force driven access doors replay v1',
            'model_xml_sha256':hashlib.sha256(xml.encode()).hexdigest(),
            'units':'SI','solver':'MuJoCo '+mujoco.__version__,
            'scope':'Standalone six-door dynamics with fixed car and cabinet supports; no whole-train claim.',
            'reference_zero_qpos_body_transforms':reference,
            'initial_qpos':initial_qpos,'initial_qvel':initial_qvel,'parameters':s.params,
            'frames':frames},indent=2))
        with (OUT/'access_doors_cycle.csv').open('w') as f:
            w = csv.writer(f);w.writerow(['time_s',*ad.DOOR_KEYS]);w.writerows(rows)
    return result,a


def test_topology_and_mass():
    m,d,s,b,xml = compile_model()
    assert m.njnt == 6 and m.nu == 6 and m.neq == 6
    assert len(s.source_body_mapping()) == 84
    assert sum(len(x.source_parts) for x in s.doors if x.attached_to=='car_A') == 44
    assert len(set(s.source_body_mapping().values())) == 6
    audited = json.loads((OUT/'source_door_closed_rest.json').read_text())
    source_groups = {g['name']:g for g in audited['groups']}
    for leaf in s.doors:
        expected = {o['name'] for o in source_groups[leaf.source_owner]['objects']}
        assert set(leaf.source_parts) == expected
    assert audited['source_sha256'] == ad.SOURCE_HASH
    assert all(b.latched(d).values())
    assert np.all(m.actuator_biastype == mujoco.mjtBias.mjBIAS_NONE)
    assert np.all(m.actuator_dyntype == mujoco.mjtDyn.mjDYN_NONE)
    assert np.all(m.actuator_ctrllimited) and np.all(m.actuator_forcelimited)
    assert np.all(m.jnt_limited)
    for door in s.doors:
        j,body = b.joint_ids[door.key],b.body_ids[door.key]
        assert np.allclose(m.jnt_axis[j],[0,0,1])
        assert np.allclose(d.xpos[body],door.hinge_world_rest_m,atol=1e-10)
        assert np.allclose(m.jnt_range[j],door.joint_range_rad)
        assert m.body_mass[body] > 0 and np.all(m.body_inertia[body]>0)
        assert np.allclose(d.xipos[body],np.asarray(door.com_source_m)+[3.,0,0])
        if door.attached_to == 'world':
            assert m.body_parentid[body] != m.body('A_carbody').id
    allocation = s.car_mass_allocation()
    M = allocation['residual_mass_kg']
    c = np.asarray(allocation['residual_com_m'])
    I = np.asarray(allocation['residual_inertia_com_kgm2'])+ad._parallel_axis(M,c)
    first = M*c
    for leaf in allocation['moving_leaves']:
        M += leaf['mass_kg']
        first += leaf['mass_kg']*np.asarray(leaf['com_car_m'])
        I += np.asarray(leaf['inertia_car_origin_kgm2'])
    assert abs(M-60.) < 1e-12
    assert np.max(abs(first)) < 1e-12
    assert np.allclose(I,np.diag([10.025,36.7625,35.7625]),atol=1e-12)
    root = fromstring(xml);car = root.find("worldbody/body[@name='A_carbody']")
    try:
        s.apply_car_mass_allocation(car)
        raise AssertionError('Double allocation was accepted')
    except ValueError:
        pass
    # Binding state never writes generalized positions/velocities.
    source = ast.parse(Path(ad.__file__).read_text())
    forbidden = []
    for node in ast.walk(source):
        if isinstance(node,(ast.Assign,ast.AugAssign,ast.AnnAssign)):
            targets = node.targets if isinstance(node,ast.Assign) else [node.target]
            for target in targets:
                text = ast.unparse(target)
                if 'qpos[' in text or 'qvel[' in text: forbidden.append(text)
    assert not forbidden,forbidden
    return {'source_part_count':84,'car_leaf_part_count':44,'cabinet_leaf_part_count':40,
            'independent_hinges':6,'bounded_torque_actuators':6,'optional_ideal_latches':6,
            'no_generalized_state_writes':True,'mass_com_inertia_reconstruction_passed':True,
            'mass_allocation':allocation}


def test_zero_input():
    m,d,s,b,_ = compile_model()
    b.release_latch(d)
    largest = 0.
    for _ in range(round(2./m.opt.timestep)):
        mujoco.mj_step(m,d)
        largest = max(largest,float(np.max(openings(b,d))))
    assert largest < 1e-8,largest
    return {'unlatched_zero_input_max_opening_rad':largest}


def test_mass_response():
    results = []
    for scale in (1.,2.):
        overrides = {k:ad.DEFAULTS[k]*scale for k in ('bay_mass_kg','bay_fan_mass_kg','cabinet_mass_kg')}
        m,d,s,b,_ = compile_model(params=overrides)
        b.release_latch(d)
        b.set_opening_torques(d,{x.key:.8 if x.attached_to=='car_A' else 2. for x in s.doors})
        step_for(m,d,.4)
        results.append(openings(b,d))
    assert np.all(results[0]>0) and np.all(results[1]>0)
    assert np.all(results[1]<.7*results[0]),results
    return {'same_torque_same_duration_s':.4,'baseline_opening_rad':dict(zip(ad.DOOR_KEYS,results[0].tolist())),
            'double_mass_and_inertia_opening_rad':dict(zip(ad.DOOR_KEYS,results[1].tolist())),
            'all_heavier_doors_open_more_slowly':True}


def test_control_and_latch_guards():
    m,d,s,b,_ = compile_model()
    q,v = d.qpos.copy(),d.qvel.copy()
    b.set_torques(d,{x.key:1e9 for x in s.doors})
    assert np.array_equal(q,d.qpos) and np.array_equal(v,d.qvel)
    assert all(d.ctrl[b.actuator_ids[x.key]]==x.torque_limit_Nm for x in s.doors)
    # Engine force limits remain effective even if a caller bypasses our input clamp.
    for x in s.doors:
        d.ctrl[b.actuator_ids[x.key]]=1e9
    mujoco.mj_forward(m,d)
    assert all(abs(d.actuator_force[b.actuator_ids[x.key]])<=x.torque_limit_Nm for x in s.doors)
    old = d.ctrl.copy()
    for cmd in ({'bay_L':float('nan')},{'bay_L':float('inf')},{'bogus':1}):
        try:
            b.set_torques(d,cmd)
            raise AssertionError('Bad command was accepted')
        except (ValueError,KeyError): pass
        assert np.array_equal(old,d.ctrl)
    b.release(d)
    b.release_latch(d)
    b.set_opening_torques(d,{'bay_L':1.5})
    step_for(m,d,.04)
    # At 0.04 s this leaf is still geometrically close but moving too rapidly.
    qnow=abs(b.angles(d)['bay_L']);vnow=abs(b.velocities(d)['bay_L'])
    assert qnow<s.params['latch_capture_angle_rad'] and vnow>s.params['latch_capture_velocity_radps'],(qnow,vnow)
    assert not b.try_latch(d,['bay_L'])['bay_L']
    return {'control_inputs_finite_and_bounded':True,'engine_force_limit_independently_verified':True,'invalid_command_atomic':True,
            'fast_near_closed_relatch_rejected':True,'near_closed_angle_rad':qnow,'near_closed_speed_radps':vnow}


def test_action_reaction():
    m,d,s,b,_ = compile_model(free_car=True)
    b.release_latch(d,['bay_L'])
    b.set_opening_torques(d,{'bay_L':1.})
    step_for(m,d,.3)
    mujoco.mj_subtreeVel(m,d)
    root=m.body('A_carbody').id
    angular_momentum = np.asarray(d.subtree_angmom[root]).copy()
    car_dof=int(m.jnt_dofadr[m.joint('A_car_free').id])
    yaw_rate=float(d.qvel[car_dof+5])
    assert abs(yaw_rate)>1e-4 and b.angles(d)['bay_L']<-.01
    assert np.linalg.norm(angular_momentum)<1e-4,angular_momentum
    return {'free_car_yaw_velocity_radps':yaw_rate,'door_angle_rad':b.angles(d)['bay_L'],
            'car_subtree_angular_momentum_kgm2ps':angular_momentum.tolist(),
            'internal_torque_reacts_on_car_body':True}


def main():
    OUT.mkdir(exist_ok=True)
    report={'mujoco_version':mujoco.__version__,'scope':'Standalone physical door mechanisms; not whole train validation',
            'topology_and_mass':test_topology_and_mass(),'zero_input':test_zero_input(),
            'inertial_response':test_mass_response(),'control_guards':test_control_and_latch_guards(),
            'action_reaction':test_action_reaction()}
    report['cycle'],base = full_cycle(save=True)
    report['half_dt_cycle'],half = full_cycle(.00005)
    differences=[]
    for i in range(1,7):
        finer=np.interp(base[:,0],half[:,0],half[:,i])
        differences.append(float(np.max(np.abs(base[:,i]-finer))))
    assert max(differences)<.005,differences
    report['half_dt_comparison']={'max_trace_angle_difference_rad':dict(zip(ad.DOOR_KEYS,differences)),
                                  'acceptance_rad':.005,'passed':True}
    report['passed']=True
    _,_,spec,_,_=compile_model()
    (OUT/'access_doors_spec.json').write_text(json.dumps(spec.to_dict(),indent=2))
    (OUT/'access_doors_results.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
