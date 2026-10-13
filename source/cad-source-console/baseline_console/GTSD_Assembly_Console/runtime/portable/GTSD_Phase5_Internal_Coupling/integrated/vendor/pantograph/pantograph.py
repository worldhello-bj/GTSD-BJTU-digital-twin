"""New-geometry GTSD pantograph: force-driven, explicitly partial MuJoCo model.

This is an OPEN serial surrogate, not the unverified real pantograph linkage.
The geometry's A/E/U/H reference points are retained exactly.  E -> U is a
rigid engineering-equivalent carrier fixed to the lower arm: the physical
E-axis freedom and both slender-link loops remain unmodeled.  The visible
bellows and B32 barrel are NOT actuators in this model.  An explicit head-load
port and root/upper torque ports permit force-driven numerical experiments.
No positions are prescribed during integration.  See README.md and parameters.json.
"""
from __future__ import annotations
import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
DEFAULT_SOURCE = Path(os.environ.get('GTSD_GEOMETRY_ROOT', str(HERE.parent)))
BODY_NAMES = ('panto_base', 'panto_lower', 'panto_upper', 'panto_head_carriage', 'panto_head_float', 'panto_wire')
JOINT_NAMES = ('panto_lower_hinge', 'panto_upper_hinge', 'panto_head_pitch', 'panto_head_slide')

DEFAULTS = {
    'timestep_s': 0.0002,
    'gravity_mps2': 9.81,
    'lower_mass_kg': 1.5,
    'upper_mass_kg': 1.2,
    'head_carriage_mass_kg': 0.2,
    'head_float_mass_kg': 0.4,
    'lower_inertia_scale': 1.0,
    'upper_inertia_scale': 1.0,
    'head_carriage_inertia_scale': 1.0,
    'head_float_inertia_scale': 1.0,
    'lower_stiffness_Nm_per_rad': 60.0,
    'upper_stiffness_Nm_per_rad': 60.0,
    'head_pitch_stiffness_Nm_per_rad': 1.0,
    'head_slide_stiffness_N_per_m': 1500.0,
    'lower_damping_Nms_per_rad': 1.0,
    'upper_damping_Nms_per_rad': 1.0,
    'head_pitch_damping_Nms_per_rad': 0.08,
    'head_slide_damping_Ns_per_m': 4.0,
    'joint_armature_kg_m2': 0.00001,
    'head_slide_armature_kg': 0.00001,
    'contact_friction': 0.12,
    'contact_timeconst_s': 0.004,
    'contact_dampratio': 1.0,
    'contact_impedance_min': 0.95,
    'contact_impedance_max': 0.995,
    'contact_impedance_width_m': 0.001,
    'wire_radius_m': 0.0025,
    'torque_limit_Nm': 100.0,
    'head_load_limit_N': 100.0,
}

PARAM_INFO = {
    'lower_inertia_scale': ([0.2, 5], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Scales all principal moments of analytic lower-arm inertia, independently of mass.'),
    'upper_inertia_scale': ([0.2, 5], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Scales all principal moments of analytic upper-arm inertia, independently of mass.'),
    'head_carriage_inertia_scale': ([0.2, 5], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Scales analytic head carriage moments independently of mass.'),
    'head_float_inertia_scale': ([0.2, 5], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Scales analytic collector moments independently of mass.'),
    'lower_mass_kg': ([0.3, 5], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Whole lower assembly lumped at arm midpoint; not inferred from mesh volume.'),
    'upper_mass_kg': ([0.2, 4], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Upper beam and fork lumped at centerline midpoint.'),
    'head_carriage_mass_kg': ([0.05, 1], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Stationary portion of assumed head guide assembly.'),
    'head_float_mass_kg': ([0.1, 2], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Collector, carbon strips, carriers and assumed floating pins/yokes.'),
    'lower_stiffness_Nm_per_rad': ([0, 200], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Passive joint spring about source reference pose; not a measured real return spring.'),
    'upper_stiffness_Nm_per_rad': ([0, 200], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Passive joint spring, not an inter-stage transmission or closure.'),
    'head_pitch_stiffness_Nm_per_rad': ([0, 20], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Relative pitch compliance at U->H endpoint, no world-leveling law.'),
    'head_slide_stiffness_N_per_m': ([200, 20000], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Assumed vertical guide spring, no measured spring internals.'),
    'lower_damping_Nms_per_rad': ([0.02, 10], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Linear viscous hinge damping.'),
    'upper_damping_Nms_per_rad': ([0.02, 10], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Linear viscous hinge damping.'),
    'head_pitch_damping_Nms_per_rad': ([0.001, 1], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Linear viscous hinge damping.'),
    'head_slide_damping_Ns_per_m': ([0.1, 100], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Linear viscous guide damping.'),
    'contact_friction': ([0, 0.5], 'ENGINEERING_EQUIVALENT_UNMEASURED', 'Solver friction coefficient; carbon/copper materials are not calibrated.'),
    'contact_timeconst_s': ([0.001, 0.02], 'NUMERICAL_CONTACT_PARAMETER', 'MuJoCo positive solref time constant; not directly a measured material stiffness.'),
    'contact_dampratio': ([0.5, 2], 'NUMERICAL_CONTACT_PARAMETER', 'MuJoCo positive solref damping ratio.'),
    'wire_radius_m': ([0.0025, 0.0025], 'CAD_2D_LAYOUT_CANDIDATE', 'CAD design diameter 5 mm, not a physical measurement.'),
    'torque_limit_Nm': ([1, 100], 'ENGINEERING_TEST_PORT_LIMIT', 'Safety envelope for direct numerical joint torque experiments.'),
    'head_load_limit_N': ([1, 100], 'ENGINEERING_TEST_PORT_LIMIT', 'Safety envelope for external laboratory-equivalent head load.'),
}

UNMODELED = [
    'Physical E-axis motion relative to lower arm; E->U carrier is provisionally rigid with lower arm.',
    'Both slender adjustable rods: 3D ball centers, hinge attachment ownership and mechanical closure.',
    'Air bellows effective area, pressure chamber, hysteresis, force path, air circuit and deformation.',
    'B32 black barrel linkage, bore, stroke, piston side, seals, friction and function.',
    'Rigid head linkage leveling/synchronization; no ideal parallelogram or algebraic fold relation is imposed.',
    'Actual head guide internals and which guide pieces translate; present split is a declared equivalent.',
    'Wire tension, sag, transverse dynamics, droppers and speed-dependent catenary effects; wire is fixed here.',
    'Carbon wear, elasticity, arcing, electric current and copper braid flexibility.',
    'Physical masses, inertia tensors, damping, springs and true motion stops; all dynamics are uncalibrated.',
    'Arm-roof, link-link and cover collisions; only collector/wire and numerical joint stops are solved.',
]


def vec(v):
    return ' '.join(f'{float(x):.17g}' for x in v)


def add(parent, tag, **kw):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in kw.items()})


def file_hash(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def source_geometry(source=DEFAULT_SOURCE):
    source = Path(source)
    contract = json.loads((source / 'assembly_contract.json').read_text())
    manifest = json.loads((source / 'components/pantograph_manifest.json').read_text())
    # Read executable parameter definitions without importing Blender or executing
    # source code.  Values have been independently checked against that module.
    l = float(manifest['parameters']['lower_main_center_length']['value'])
    upper_l = float(manifest['parameters']['upper_visual_axis_to_axis_length']['value'])
    roof = contract['roof_top_z_m']
    lo = math.radians(contract['pantograph']['lower_angle_deg'])
    up = math.radians(contract['pantograph']['upper_angle_deg'])
    A = np.array([manifest['parameters']['base_joint_x']['value'], 0, roof + manifest['parameters']['base_joint_height_over_roof']['value']])
    E = A + np.array([l*math.cos(lo), 0, l*math.sin(lo)])
    U = E + np.array(manifest['parameters']['knee_upper_axis_offset_from_lower_xyz']['value'])
    H = U + np.array([-upper_l*math.cos(up), 0, upper_l*math.sin(up)])
    geometry = json.loads((source / 'web/geometry.json').read_text())
    wire = next(m for m in geometry['meshes'] if m['name'] == 'ENV_contact_wire')
    wv = np.asarray(wire['vertices']).reshape(-1, 3)
    return {
        'A': A, 'E': E, 'U': U, 'H': H,
        'roof_z_m': roof, 'lower_length_m': l, 'upper_length_m': upper_l,
        'lower_reference_angle_rad': lo, 'upper_reference_angle_rad': up,
        'wire_z_m': contract['display_contact_wire_center_z_m'],
        'wire_x_bounds_m': [float(wv[:,0].min()),float(wv[:,0].max())],
        'source_hashes': {str(p.relative_to(source)): file_hash(p) for p in [source/'web/geometry.json', source/'assembly_contract.json', source/'components/pantograph_manifest.json', source/'scripts/component_pantograph.py']},
        'meshes': geometry['meshes'],
    }


def beam_inertial(body, delta, mass, width, height, fork_spread=0, inertia_scale=1):
    """Positive-definite engineering inertia, not a material-density estimate."""
    length = np.linalg.norm(delta)
    axis = delta / length
    ilong = mass * (width**2 + height**2 + fork_spread**2) / 12
    itrans = mass * (length**2 + (width**2+height**2)/2 + fork_spread**2/2) / 12
    I = inertia_scale * (itrans * np.eye(3) + (ilong - itrans) * np.outer(axis, axis))
    add(body, 'inertial', mass=mass, pos=vec(delta/2), fullinertia=vec([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]]))


def box_inertial(body, mass, halfsize, pos, inertia_scale=1):
    x,y,z = halfsize
    add(body, 'inertial', mass=mass, pos=vec(pos), diaginertia=vec(inertia_scale*np.array([mass*(y*y+z*z)/3, mass*(x*x+z*z)/3, mass*(x*x+y*y)/3])))


def build_model(source=DEFAULT_SOURCE, params=None, contact=True):
    p = DEFAULTS | (params or {})
    g = source_geometry(source)
    A,E,U,H = (g[k] for k in ('A','E','U','H'))
    root = ET.Element('mujoco', model='gtsd_new_geometry_pantograph_partial_force_model')
    add(root,'compiler',angle='radian',inertiafromgeom='false',autolimits='true')
    opt = add(root,'option',timestep=p['timestep_s'],gravity=vec([0,0,-p['gravity_mps2']]),integrator='implicitfast',solver='Newton',iterations=100,tolerance='1e-10',cone='elliptic',jacobian='dense')
    add(opt,'flag',energy='enable')
    default = add(root,'default')
    add(default,'geom',contype=0,conaffinity=0,rgba='.18 .23 .27 1')
    add(default,'joint',limited='true',solreflimit='.004 1',solimplimit='.95 .995 .001')
    world = add(root,'worldbody')
    add(world,'light',pos='0 -2 4',dir='0 .4 -1',directional='true')
    add(world,'camera',name='side',pos='0 -3 1.7',xyaxes='1 0 0 0 0 1')
    base = add(world,'body',name='panto_base',pos='0 0 0')
    add(base,'geom',name='roof_reference',type='box',size='.65 .4 .012',pos=vec([0,0,g['roof_z_m']-.012]),rgba='.12 .3 .4 1')
    add(base,'site',name='source_A',pos=vec(A),size='.006')
    lower = add(base,'body',name='panto_lower',pos=vec(A))
    add(lower,'joint',name=JOINT_NAMES[0],type='hinge',axis='0 -1 0',range='-0.8 1.1',stiffness=p['lower_stiffness_Nm_per_rad'],damping=p['lower_damping_Nms_per_rad'],armature=p['joint_armature_kg_m2'])
    beam_inertial(lower,E-A,p['lower_mass_kg'],.042,.0405,inertia_scale=p['lower_inertia_scale'])
    add(lower,'geom',name='lower_collision_free_display',type='capsule',fromto=vec(np.r_[np.zeros(3),E-A]),size='.020')
    add(lower,'geom',name='equivalent_rigid_knee_carrier',type='capsule',fromto=vec(np.r_[E-A,U-A]),size='.013',rgba='.8 .42 .10 1')
    add(lower,'site',name='source_E',pos=vec(E-A),size='.006')
    add(lower,'site',name='source_U_on_lower',pos=vec(U-A),size='.006')
    upper = add(lower,'body',name='panto_upper',pos=vec(U-A))
    add(upper,'joint',name=JOINT_NAMES[1],type='hinge',axis='0 -1 0',range='-1.1 0.9',stiffness=p['upper_stiffness_Nm_per_rad'],damping=p['upper_damping_Nms_per_rad'],armature=p['joint_armature_kg_m2'])
    beam_inertial(upper,H-U,p['upper_mass_kg'],.032,.035,fork_spread=.08,inertia_scale=p['upper_inertia_scale'])
    add(upper,'geom',name='upper_collision_free_display',type='capsule',fromto=vec(np.r_[np.zeros(3),H-U]),size='.015')
    add(upper,'site',name='source_U',pos='0 0 0',size='.006')
    carriage = add(upper,'body',name='panto_head_carriage',pos=vec(H-U))
    add(carriage,'joint',name=JOINT_NAMES[2],type='hinge',axis='0 -1 0',range='-0.7 0.7',stiffness=p['head_pitch_stiffness_Nm_per_rad'],damping=p['head_pitch_damping_Nms_per_rad'],armature=p['joint_armature_kg_m2'])
    box_inertial(carriage,p['head_carriage_mass_kg'],[.036,.156,.03],[0,0,.012],inertia_scale=p['head_carriage_inertia_scale'])
    add(carriage,'geom',name='head_carriage_display',type='box',pos='0 0 .003',size='.0355 .156 .010',rgba='.4 .43 .45 1')
    add(carriage,'site',name='source_H',pos='0 0 0',size='.006')
    floating = add(carriage,'body',name='panto_head_float',pos='0 0 0')
    add(floating,'joint',name=JOINT_NAMES[3],type='slide',axis='0 0 1',range='-.018 .018',stiffness=p['head_slide_stiffness_N_per_m'],damping=p['head_slide_damping_Ns_per_m'],armature=p['head_slide_armature_kg'])
    box_inertial(floating,p['head_float_mass_kg'],[.0715,.32,.017],[0,0,.054],inertia_scale=p['head_float_inertia_scale'])
    for k,x in enumerate([-.056,.056],1):
        # Only the flat central 470 mm strip is collision-active. Horns remain visual.
        add(floating,'geom',name=f'collector_contact_{k}',type='box',pos=vec([x,0,.068-.0085/2]),size=vec([.031/2,.235,.0085/2]),contype=8 if contact else 0,conaffinity=8 if contact else 0,condim=3,friction=vec([p['contact_friction'],.001,.0001]),solref=vec([p['contact_timeconst_s'],p['contact_dampratio']]),solimp=vec([p['contact_impedance_min'],p['contact_impedance_max'],p['contact_impedance_width_m']]),rgba='.035 .035 .035 1')
    add(floating,'site',name='head_load_point',pos='0 0 .054',size='.007',rgba='.85 .15 .15 1')
    add(floating,'site',name='carbon_top_center',pos='0 0 .068',size='.004')
    wire = add(world,'body',name='panto_wire',pos='0 0 0')
    add(wire,'geom',name='contact_wire',type='cylinder',fromto=vec([g['wire_x_bounds_m'][0],0,g['wire_z_m'],g['wire_x_bounds_m'][1],0,g['wire_z_m']]),size=p['wire_radius_m'],contype=8 if contact else 0,conaffinity=8 if contact else 0,condim=3,friction=vec([p['contact_friction'],.001,.0001]),solref=vec([p['contact_timeconst_s'],p['contact_dampratio']]),solimp=vec([p['contact_impedance_min'],p['contact_impedance_max'],p['contact_impedance_width_m']]),rgba='.65 .31 .12 1')
    actuator = add(root,'actuator')
    for name, joint in [('root_torque_port',JOINT_NAMES[0]),('upper_torque_port',JOINT_NAMES[1])]:
        add(actuator,'motor',name=name,joint=joint,gear=1,ctrllimited='true',ctrlrange=vec([-p['torque_limit_Nm'],p['torque_limit_Nm']]))
    ET.indent(root)
    xml = ET.tostring(root,encoding='unicode')
    model = mujoco.MjModel.from_xml_string(xml)
    return model, xml, g, p


def pose_map(model, data):
    return {n:{'position':data.xpos[model.body(n).id].tolist(),'quaternion':data.xquat[model.body(n).id].tolist()} for n in BODY_NAMES}


def parameter_ledger(g,p):
    ledger = {key:{'value':value,'exploration_range_not_calibration':PARAM_INFO.get(key,([value,value],'NUMERICAL_CONFIGURATION','Numerical configuration, not a measured physical property.'))[0], 'evidence':PARAM_INFO.get(key,(None,'NUMERICAL_CONFIGURATION',''))[1], 'note':PARAM_INFO.get(key,(None,None,'Numerical configuration, not a measured physical property.'))[2]} for key,value in p.items()}
    return {'schema':'gtsd-pantograph-parameter-ledger-v1','geometry':{**{k:g[k].tolist() for k in ('A','E','U','H')},'lower_length_m':g['lower_length_m'],'lower_length_evidence':'CAD_2D_GEOMETRY_CANDIDATE_NOT_MEASURED_LINK_IDENTITY','upper_length_m':g['upper_length_m'],'upper_length_evidence':'PHOTO_INFERRED_GEOMETRY','reference_lower_angle_rad':g['lower_reference_angle_rad'],'reference_upper_angle_rad':g['upper_reference_angle_rad'],'wire_z_m':g['wire_z_m'],'wire_x_bounds_m':g['wire_x_bounds_m']},'parameters':ledger,'unmodeled':UNMODELED,'inertia_model':'Analytic slender-beam / box tensors from declared masses and equivalent envelopes. Independent positive inertia scales are configurable; tensors are not mesh-derived measurements.','calibration_status':'NUMERICAL_DEMONSTRATION_ONLY_NOT_VALIDATED_HARDWARE'}


def build_binding(model,g):
    d=mujoco.MjData(model);mujoco.mj_forward(model,d)
    rows=[]
    for m in g['meshes']:
        n=m['name']
        if not (n.startswith('Panto_') or n.startswith('DETAIL_SENSOR_panto_') or n=='ENV_contact_wire'):
            continue
        body='panto_base';mode='rigid';evidence='PHOTO_INFERRED_GEOMETRY';note='Fixed support/detail object in partial subsystem; reference mounting is uncalibrated.'
        if n=='ENV_contact_wire':
            body='panto_wire';mode='rigid';evidence='CAD_2D_LAYOUT_CANDIDATE';note='Fixed rigid 5 mm wire surrogate; tension and sag omitted.'
        elif n.startswith('Panto_copper_') or n in ('Panto_base_three_convolution_bellows','Panto_bellows_visible_air_hose') or n.startswith('DETAIL_SENSOR_panto_position_lead_'):
            mode='excluded_deformable';note='Flexible geometry omitted from moving physics view. No rigid deformation mapping is asserted.'
        elif n.startswith(('Panto_lower_link_','Panto_upper_link_')) or n in ('Panto_lower_slender_adjustable_link','Panto_upper_slender_adjustable_link'):
            mode='static_reference';note='Unclosed slender-link geometry. Static reference only; hide during dynamic replay to avoid suggesting a solved loop.'
        elif n.startswith('Panto_B32_') or n.startswith('Panto_bellows_'):
            mode='static_reference';note='Actuator force path/attachment ownership unknown. Static reference only, never driven by the equivalent head load.'
        elif n=='Panto_B29_lower_main_box_arm' or n=='Panto_lower_arm_transverse_pivot' or n.startswith(('Panto_knee_link_cheek_','Panto_knee_lower_','Panto_knee_upper_hex_','Panto_knee_upper_washer_')):
            body='panto_lower';mode='rigid';evidence='ENGINEERING_EQUIVALENT_BODY_ATTACHMENT';note='Lower arm + E/U knee carrier rigid equivalent. The physical E-axis motion is unresolved and not simulated.'
            if n=='Panto_B29_lower_main_box_arm':evidence='CAD_2D_GEOMETRY_CANDIDATE_WITH_UNVALIDATED_BODY_ASSIGNMENT'
        elif n.startswith('Panto_B30_') or n=='Panto_knee_upper_shaft':
            body='panto_upper';mode='rigid';evidence='ENGINEERING_EQUIVALENT_BODY_ATTACHMENT';note='Upper arm pivots at distinct U; shaft attachment to rotating upper is provisional.'
        elif n.startswith('Panto_B31_'):
            is_float = n.startswith(('Panto_B31_carbon_contact_strip_','Panto_B31_collector_metal_carrier_','Panto_B31_head_guide_pin_','Panto_B31_head_top_yoke_','Panto_B31_strip_mount_bolt_'))
            body='panto_head_float' if is_float else 'panto_head_carriage';mode='rigid';evidence='ENGINEERING_EQUIVALENT_GUIDE_PART_SPLIT';note='Assumed floating collector/pins/yokes' if is_float else 'Assumed pitched carriage/hangers/outer guides/collars/pivot washers'
        rows.append({'mesh_name':n,'body_name':body,'mode':mode,'evidence':evidence,'note':note})
    return {'schema':'gtsd-physics-binding-v1','subsystem':'pantograph','source_geometry_sha256':g['source_hashes']['web/geometry.json'],'reference_poses':pose_map(model,d),'meshes':rows,'coordinate_system':'metres; X longitudinal, Y transverse, Z up; quaternion wxyz','transform_contract':'v_dynamic_world = R(body.xquat) @ inv(R(reference.quaternion)) @ (v_source_world - reference.position) + body.xpos','classification_note':'Every source Panto_ mesh plus 8 position-sensor meshes and ENV_contact_wire is explicitly classified. No prefix-based runtime rebinding. static_reference and excluded_deformable should be hidden in motion-only view unless clearly labeled.','unmodeled':UNMODELED}


def contacts(model,data):
    rows=[];normal=0.;world=np.zeros(3);wrench=np.zeros(6)
    for i in range(data.ncon):
        c=data.contact[i];a=model.geom(c.geom1).name;b=model.geom(c.geom2).name
        if 'contact_wire' not in (a,b):continue
        mujoco.mj_contactForce(model,data,i,wrench)
        # The contact normal points from geom1 to geom2. Force convention is on
        # geom2; invert when the collector is geom1, to get force on collector.
        f=np.asarray(c.frame).reshape(3,3).T@wrench[:3]
        if a.startswith('collector_contact_'):f=-f
        normal+=max(0.,float(wrench[0]));world+=f
        rows.append({'geom1':a,'geom2':b,'distance_m':float(c.dist),'position_m':c.pos.tolist(),'normal_force_N':float(wrench[0]),'force_on_collector_world_N':f.tolist(),'local_wrench_N_Nm':wrench.tolist()})
    return normal,world,rows


def simulate(case_id,source=DEFAULT_SOURCE,params=None,duration_s=.6,head_force_fn=None,torque_fn=None,contact=True,sample_hz=100,trace_path=None):
    model,xml,g,p=build_model(source,params,contact)
    d=mujoco.MjData(model)
    mujoco.mj_forward(model,d)
    body_id=model.body('panto_head_float').id
    site_id=model.site('head_load_point').id
    carbon_id=model.site('carbon_top_center').id
    frames=[];warning_log=[]
    trace=defaultdict(list);contact_trace=[]
    accumulated_work=0.;accumulated_passive_work=0.;accumulated_constraint_work=0.;accumulated_viscous_work=0.
    diagnostic=[]
    timestep=p['timestep_s'];count=int(round(duration_s/timestep));stride=max(1,round(1/sample_hz/timestep))
    warnings_before=np.zeros(len(d.warning),dtype=int)
    loads=np.zeros(3)
    applied=np.zeros(model.nv)
    for step in range(count+1):
        # This is a force-only update. mj_forward merely updates derived fields;
        # qpos/qvel are never overwritten after reset/initialization.
        mujoco.mj_forward(model,d)
        force=np.array(head_force_fn(float(d.time)) if head_force_fn else [0,0,0],dtype=float)
        if not np.all(np.isfinite(force)) or np.linalg.norm(force)>p['head_load_limit_N']+1e-12:
            raise ValueError('External head load exceeds declared test-port limit')
        torque=np.array(torque_fn(float(d.time)) if torque_fn else [0,0],dtype=float)
        if not np.all(np.isfinite(torque)) or np.max(np.abs(torque))>p['torque_limit_Nm']+1e-12:
            raise ValueError('Torque exceeds declared test-port limit')
        d.ctrl[:]=torque
        d.qfrc_applied[:]=0
        mujoco.mj_applyFT(model,d,force,np.zeros(3),d.site_xpos[site_id],body_id,d.qfrc_applied)
        mujoco.mj_forward(model,d)
        normal,world_force,contact_rows=contacts(model,d)
        act_power=float(np.dot(d.qfrc_applied+d.qfrc_actuator,d.qvel))
        passive_power=float(np.dot(d.qfrc_passive,d.qvel))
        constraint_power=float(np.dot(d.qfrc_constraint,d.qvel))
        viscous_power=-float(np.dot(model.dof_damping,d.qvel*d.qvel))
        energy=float(sum(d.energy));peak_penetration=max([0]+[-r['distance_m'] for r in contact_rows])
        diagnostic.append([float(d.time),normal,energy,peak_penetration,act_power,float(np.linalg.norm(d.qvel)),float(d.solver_niter[0]),len(contact_rows)])
        if trace_path is not None:
            vals={
                'time_s':float(d.time),'qpos':d.qpos.copy(),'qvel':d.qvel.copy(),'qacc':d.qacc.copy(),
                'ctrl':d.ctrl.copy(),'actuator_force':d.actuator_force.copy(),'head_force_world_N':force.copy(),
                'qfrc_applied':d.qfrc_applied.copy(),'qfrc_actuator':d.qfrc_actuator.copy(),
                'qfrc_passive':d.qfrc_passive.copy(),'qfrc_constraint':d.qfrc_constraint.copy(),
                'body_xpos':np.array([d.xpos[model.body(n).id] for n in BODY_NAMES]),
                'body_xquat':np.array([d.xquat[model.body(n).id] for n in BODY_NAMES]),
                'energy_potential_kinetic_J':d.energy.copy(),
                'contact_normal_force_N':normal,'contact_force_on_collector_world_N':world_force.copy(),
                'contact_count':len(contact_rows),'ncon_total':int(d.ncon),
                'carbon_top_center_world_m':d.site_xpos[carbon_id].copy(),
                'external_passive_constraint_viscous_power_W':[act_power,passive_power,constraint_power,viscous_power],
                'external_passive_constraint_viscous_work_left_J':[accumulated_work,accumulated_passive_work,accumulated_constraint_work,accumulated_viscous_work],
                'solver_iterations':int(d.solver_niter[0]),
                'warning_counts':[int(x.number) for x in d.warning],
                'warning_lastinfo':[int(x.lastinfo) for x in d.warning],
            }
            for k,v in vals.items():trace[k].append(v)
            for c in contact_rows:
                contact_trace.append([step,model.geom(c['geom1']).id,model.geom(c['geom2']).id,c['distance_m'],*c['position_m'],c['normal_force_N'],*c['force_on_collector_world_N'],*c['local_wrench_N_Nm']])
        if step%stride==0 or step==count:
            frames.append({'time_s':float(d.time),'body_poses':pose_map(model,d),'telemetry':{
                'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'qacc':d.qacc.tolist(),
                'head_force_world_N':force.tolist(),'joint_torque_port_Nm':torque.tolist(),
                'qfrc_applied':d.qfrc_applied.tolist(),'qfrc_actuator':d.qfrc_actuator.tolist(),'qfrc_passive':d.qfrc_passive.tolist(),'qfrc_constraint':d.qfrc_constraint.tolist(),
                'contact_normal_force_N':normal,'contact_force_on_collector_world_N':world_force.tolist(),'contacts':contact_rows,
                'carbon_center_top_world_m':d.site_xpos[carbon_id].tolist(),
                'energy_potential_J':float(d.energy[0]),'energy_kinetic_J':float(d.energy[1]),'energy_total_J':energy,
                'external_power_W':act_power,'external_work_left_riemann_J':accumulated_work,
                'passive_power_W':passive_power,'passive_work_left_riemann_J':accumulated_passive_work,
                'constraint_power_W':constraint_power,'constraint_work_left_riemann_J':accumulated_constraint_work,
                'viscous_dissipation_power_W':-viscous_power,'viscous_work_left_riemann_J':accumulated_viscous_work,
                'solver_iterations':int(d.solver_niter[0]),'ncon_total':int(d.ncon),
                'warning_counts':[int(x.number) for x in d.warning],
            }})
        current=np.array([x.number for x in d.warning],dtype=int)
        for i in np.flatnonzero(current>warnings_before):
            warning_log.append({'time_s':float(d.time),'type':mujoco.mjtWarning(i).name,'number':int(current[i]),'lastinfo':int(d.warning[i].lastinfo)})
        warnings_before=current
        if step==count:break
        accumulated_work+=act_power*timestep
        accumulated_passive_work+=passive_power*timestep
        accumulated_constraint_work+=constraint_power*timestep
        accumulated_viscous_work+=viscous_power*timestep
        mujoco.mj_step(model,d)
        if not np.all(np.isfinite(np.r_[d.qpos,d.qvel,d.qacc])):
            raise RuntimeError(f'Nonfinite state in {case_id} at {d.time}')
    a=np.asarray(diagnostic)
    summary={'case_id':case_id,'duration_s':duration_s,'solver_steps':count,'sample_frames':len(frames),'peak_contact_normal_N':float(a[:,1].max()),'final_contact_normal_N':float(a[-1,1]),'peak_penetration_m':float(a[:,3].max()),'energy_initial_J':float(a[0,2]),'energy_final_J':float(a[-1,2]),'external_work_left_riemann_J':accumulated_work,'viscous_work_left_riemann_J':accumulated_viscous_work,'constraint_work_left_riemann_J':accumulated_constraint_work,'energy_work_residual_J':float(a[-1,2]-a[0,2])-accumulated_work-accumulated_viscous_work-accumulated_constraint_work,'minimum_energy_J':float(a[:,2].min()),'maximum_energy_J':float(a[:,2].max()),'maximum_solver_iterations':int(a[:,6].max()),'maximum_qvel_norm_mixed_units':float(a[:,5].max()),'final_qpos':d.qpos.tolist(),'initial_qpos':frames[0]['telemetry']['qpos'],'maximum_contact_pairs':int(a[:,7].max()),'warning_log':warning_log,'finite':True}
    result={'schema':'gtsd-physics-trajectory-v1','subsystem':'pantograph','case_id':case_id,'source_geometry_sha256':g['source_hashes']['web/geometry.json'],'frames':frames,'parameters':p,'simulation_status':'MUJOCO_FORCE_INTEGRATED_PARTIAL_ENGINEERING_EQUIVALENT_NOT_CALIBRATED','mujoco_version':mujoco.__version__,'model_name':model.names[:model.names.find(b'\0')].decode(),'qpos_layout':list(JOINT_NAMES),'quaternion_order':'wxyz','summary':summary,'unmodeled':UNMODELED,'loads_note':'External head force and direct joint torque ports are numerical laboratory-equivalent loads. They are not pneumatic forces or a recovered real actuator linkage.','energy_note':'MuJoCo potential includes gravity and joint spring potentials; soft-contact penalty energy is not exposed as an ordinary potential. Work fields are stepwise quadrature diagnostics, not a calibrated power/efficiency claim.'}
    if trace_path is not None:
        payload={k:np.asarray(v) for k,v in trace.items()}
        payload['contact_rows']=np.array(contact_trace,dtype=float).reshape(-1,17)
        payload['body_names']=np.array(BODY_NAMES)
        payload['joint_names']=np.array(JOINT_NAMES)
        payload['contact_row_columns']=np.array(['step_index','geom1_id','geom2_id','distance_m','x_m','y_m','z_m','normal_force_N','collector_fx_N','collector_fy_N','collector_fz_N','local_fx_N','local_fy_N','local_fz_N','local_tx_Nm','local_ty_Nm','local_tz_Nm'])
        payload['geom_names']=np.array([model.geom(i).name for i in range(model.ngeom)])
        payload['schema']=np.array('gtsd-mujoco-every-step-v1')
        payload['source_geometry_sha256']=np.array(g['source_hashes']['web/geometry.json'])
        payload['timestep_s']=np.array(timestep)
        np.savez_compressed(trace_path,**payload)
        result['every_step_trace']={'file':Path(trace_path).name,'sha256':file_hash(trace_path),'sample_count':count+1,'timestep_s':timestep,'schema':'gtsd-mujoco-every-step-v1','body_order':list(BODY_NAMES),'quaternion_order':'wxyz','contains':'Every solver-step state, actual inputs/generalized forces, all body poses, energy, contact details/wrenches, warning counts and solver iterations. Load with numpy.load(..., allow_pickle=False).'}
    return result,model,xml,g,p,a


def write_json(path,obj):
    Path(path).write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',default=str(DEFAULT_SOURCE));ap.add_argument('--out',default=str(HERE));ap.add_argument('--params',help='JSON dictionary of overrides');args=ap.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    overrides=json.loads(Path(args.params).read_text()) if args.params else {}
    cases=[
        ('reference_zero_gravity',{'gravity_mps2':0},.10,None,None,False),
        ('gravity_release',{},.40,None,None,True),
        ('head_force_contact',{},.60,lambda t:[0,0,30.0 if t<.30-1e-12 else 0.0],None,True),
        ('joint_torque_pulse',{},.30,None,lambda t:[2.0 if t<.12 else 0.0,-2.0 if t<.12 else 0.0],False),
    ]
    summaries=[]
    for case,extra,duration,force,torque,contact in cases:
        result,model,xml,g,p,diag=simulate(case,args.source,overrides|extra,duration,force,torque,contact,trace_path=out/f'steps_{case}.npz')
        write_json(out/f'trajectory_{case}.json',result)
        np.savetxt(out/f'diagnostics_{case}.csv',diag,delimiter=',',header='time_s,contact_normal_N,total_energy_J,penetration_m,external_power_W,qvel_norm_mixed_units,solver_iterations,contact_pairs',comments='')
        summaries.append(result['summary'])
        if case=='head_force_contact':
            write_json(out/'trajectory.json',result)
            write_json(out/'binding.json',build_binding(model,g))
            write_json(out/'parameters.json',parameter_ledger(g,p))
            (out/'model.xml').write_text(xml+'\n')
            write_json(out/'source_hashes.json',g['source_hashes'])
    write_json(out/'simulation_summary.json',{'schema':'gtsd-pantograph-simulation-summary-v1','cases':summaries,'calibration_status':'NUMERICAL_DEMONSTRATION_ONLY','warning_count':sum(len(c['warning_log']) for c in summaries)})
    print(json.dumps(summaries,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
