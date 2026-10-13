"""New-geometry candidate planar bogie. SI, MuJoCo 3.3.7; no old model import.
Geometry is parsed as data from the new component source, never executed.
"""
from pathlib import Path
import ast, json, hashlib, csv, os
from xml.etree.ElementTree import Element, SubElement, tostring, indent
ROOT=Path(__file__).resolve().parent
GEOMETRY=Path(os.environ.get('GTSD_GEOMETRY_ROOT',str(ROOT.parent)))

def source_inputs():
    contract=json.loads((GEOMETRY/'assembly_contract.json').read_text())
    tree=ast.parse((GEOMETRY/'scripts/component_bogie.py').read_text())
    pars=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='PARAMETERS' for t in n.targets))
    return contract,pars

C,G=source_inputs()
DEFAULT={
    'timestep_s':0.0005,'sample_interval_s':0.02,'settle_s':3.0,
    'rolling_radius_m':C['wheel_visual_inner_ring_radius_m'],
    'contact_tread_width_m':0.045,'contact_tread_outward_offset_m':0.0155,
    'frame_mass_kg':24.0,'payload_mass_kg':30.0,'carrier_mass_kg':1.0,'wheelset_mass_kg':4.0,
    'frame_inertia_kgm2':[0.65,2.35,2.85],'payload_inertia_kgm2':[2.36,4.98,5.83],
    'carrier_inertia_kgm2':[0.035,0.008,0.035],'wheelset_inertia_kgm2':[0.1,0.052,0.1],
    'primary_spring_stiffness_Npm':15000.0,'primary_spring_damping_Nspm':150.0,
    'secondary_stiffness_Npm':40000.0,'secondary_damping_Nspm':300.0,
    'swing_hinge_damping_Nmsprad':0.2,'wheel_bearing_damping_Nmsprad':0.002,
    'wheel_rail_friction':0.35,'contact_timeconst_s':0.008,'contact_dampratio':1.0,
    'contact_impedance':0.98,'rail_length_m':12.0,'rail_depth_m':0.04,
    'gravity_mps2':9.81,'traction_torque_each_axle_Nm':0.65,'brake_torque_limit_each_axle_Nm':0.8,
    'brake_velocity_gain_Nmsprad':0.5,'traction_observation_s':12.0,'vertical_pulse_force_N':200.0,'vertical_pulse_start_s':0.10,'vertical_pulse_end_s':0.15,
}

def fmt(x):
    return ' '.join(f'{v:.15g}' for v in x) if isinstance(x,(list,tuple)) else str(x)
def add(p,tag,**a): return SubElement(p,tag,{k:fmt(v) for k,v in a.items()})
def sub(a,b):return [x-y for x,y in zip(a,b)]

def reference_positions(p=None):
    p=DEFAULT|(p or {}); z=C['wheel_axis_height_m']; dx=G['arm_projected_delta_x_m']; dz=G['arm_projected_delta_z_m']
    poses={'frame':[0,0,0.240+C['bogie_vertical_datum_shift_m']], 'payload_secondary':[0,0,C['body_bottom_z_m']+C['body_height_m']/2]}
    for i,s in [(1,-1),(2,1)]:
        x=s*C['axle_distance_m']/2
        poses[f'arm_carrier_{i}']=[x-s*dx,0,z+dz]
        poses[f'wheelset_{i}']=[x,0,z]
    return poses

def make_model(p=None):
    p=DEFAULT|(p or {}); ref=reference_positions(p)
    r=Element('mujoco',model='GTSD_v5_new_geometry_candidate_planar_bogie')
    add(r,'compiler',angle='radian',autolimits='true')
    o=add(r,'option',timestep=p['timestep_s'],gravity=[0,0,-p['gravity_mps2']],integrator='implicitfast',solver='Newton',iterations=80,tolerance='1e-10',cone='elliptic',jacobian='dense')
    add(o,'flag',energy='enable')
    default=add(r,'default');add(default,'geom',contype=0,conaffinity=0,rgba='.3 .4 .45 1')
    w=add(r,'worldbody')
    for side in [-1,1]:
        add(w,'geom',name=f'rail_{side}',type='box',pos=[0,side*C['environment_rail_half_center_m'],-p['rail_depth_m']/2],size=[p['rail_length_m']/2,C['display_rail_head_width_m']/2,p['rail_depth_m']/2],contype=1,conaffinity=2,friction=[p['wheel_rail_friction'],0,0],condim=3,solref=[p['contact_timeconst_s'],p['contact_dampratio']],solimp=[p['contact_impedance'],p['contact_impedance'],0.001],rgba='.7 .7 .75 1')
    f=add(w,'body',name='frame',pos=ref['frame']);add(f,'freejoint',name='frame_free')
    add(f,'inertial',pos=[0,0,0],mass=p['frame_mass_kg'],diaginertia=p['frame_inertia_kgm2'])
    add(f,'geom',name='frame_proxy',type='box',size=[.535,.24,.035],rgba='.15 .25 .28 .5')
    q=add(f,'body',name='payload_secondary',pos=sub(ref['payload_secondary'],ref['frame']))
    add(q,'joint',name='secondary_z',type='slide',axis=[0,0,1],range=[-.03,.03],stiffness=p['secondary_stiffness_Npm'],damping=p['secondary_damping_Nspm'],springref=p['payload_mass_kg']*p['gravity_mps2']/p['secondary_stiffness_Npm'])
    add(q,'inertial',pos=[0,0,0],mass=p['payload_mass_kg'],diaginertia=p['payload_inertia_kgm2'])
    add(q,'geom',name='payload_proxy',type='box',size=[.55,.28,.06],rgba='.45 .6 .7 .25')
    add(f,'site',name='secondary_lower',pos=sub([0,0,.269+C['bogie_vertical_datum_shift_m']],ref['frame']),size='.006')
    add(q,'site',name='secondary_upper',pos=sub([0,0,.355+C['bogie_vertical_datum_shift_m']],ref['payload_secondary']),size='.006')
    t=add(r,'tendon');a=add(r,'actuator')
    zshift=C['bogie_vertical_datum_shift_m'];yshift=C['bogie_transverse_shift_m']
    for i,s in [(1,-1),(2,1)]:
        name=f'arm_carrier_{i}';pivot=ref[name];axle=ref[f'wheelset_{i}'];x=axle[0]
        b=add(f,'body',name=name,pos=sub(pivot,ref['frame']))
        add(b,'joint',name=f'swing_{i}',type='hinge',axis=[0,1,0],range=[-.18,.18],damping=p['swing_hinge_damping_Nmsprad'])
        add(b,'inertial',pos=[s*G['arm_projected_delta_x_m']*.65,0,-G['arm_projected_delta_z_m']*.65],mass=p['carrier_mass_kg'],diaginertia=p['carrier_inertia_kgm2'])
        for side in [-1,1]:
            y=side*(G['axlebox_center_abs_y_m']+yshift)
            add(b,'geom',name=f'arm_proxy_{i}_{side}',type='capsule',fromto=[0,y,0,*sub([x,y,axle[2]],pivot)],size='.012',rgba='.2 .3 .25 1')
            sy=side*(G['primary_spring_center_abs_y_m']+yshift)
            lo=[x,sy,.2225+zshift];up=[x,sy,.306+zshift]
            add(b,'site',name=f'primary_lower_{i}_{side}',pos=sub(lo,pivot),size='.005',rgba='.3 .8 .3 1')
            add(f,'site',name=f'primary_upper_{i}_{side}',pos=sub(up,ref['frame']),size='.005',rgba='.8 .5 .2 1')
            L=up[2]-lo[2]
            preload=(p['frame_mass_kg']+p['payload_mass_kg'])*p['gravity_mps2']/4
            td=add(t,'spatial',name=f'primary_{i}_{side}',stiffness=p['primary_spring_stiffness_Npm'],damping=p['primary_spring_damping_Nspm'],springlength=L+preload/p['primary_spring_stiffness_Npm'],width='.005',rgba='.4 .8 .4 1')
            add(td,'site',site=f'primary_lower_{i}_{side}');add(td,'site',site=f'primary_upper_{i}_{side}')
        ws=add(b,'body',name=f'wheelset_{i}',pos=sub(axle,pivot))
        add(ws,'joint',name=f'wheel_spin_{i}',type='hinge',axis=[0,1,0],limited='false',damping=p['wheel_bearing_damping_Nmsprad'])
        add(ws,'inertial',pos=[0,0,0],mass=p['wheelset_mass_kg'],diaginertia=p['wheelset_inertia_kgm2'])
        for side in [-1,1]:
            add(ws,'geom',name=f'wheel_{i}_{side}',type='cylinder',pos=[0,side*(C['wheel_center_y_m']+p['contact_tread_outward_offset_m']),0],size=[p['rolling_radius_m'],p['contact_tread_width_m']/2],quat=[2**-.5,2**-.5,0,0],contype=2,conaffinity=1,condim=3,friction=[p['wheel_rail_friction'],0,0],solref=[p['contact_timeconst_s'],p['contact_dampratio']],solimp=[p['contact_impedance'],p['contact_impedance'],.001],rgba='.35 .35 .38 1')
        add(a,'motor',name=f'axle_torque_{i}',joint=f'wheel_spin_{i}',gear='1',ctrllimited='false')
    indent(r)
    return tostring(r,encoding='unicode')

def write_parameters(p=None):
    p=DEFAULT|(p or {});rows=[]
    for key,value in p.items():
        level='EXPLICIT_UNCALIBRATED_ENGINEERING_ASSUMPTION';source='No physical measurement supplied; editable numerical demonstration parameter'
        if key=='rolling_radius_m':level='DISPLAY_CONTOUR_SELECTED_AS_UNCALIBRATED_ROLLING_SURROGATE';source='assembly_contract.json wheel_visual_inner_ring_radius_m; contour does NOT identify true rolling circle'
        elif key in ('contact_tread_width_m','contact_tread_outward_offset_m'):level='PHOTO_DISPLAY_GEOMETRY_SURROGATE';source='component_bogie.py section flat band [-0.007,0.038] m, not measured wheel section'
        elif key in ('timestep_s','sample_interval_s','settle_s','contact_timeconst_s','contact_dampratio','contact_impedance'):level='NUMERICAL_SETTING';source='Solver/sample choice, not physical calibration'
        rows.append({'parameter':key,'value':value,'evidence_level':level,'source':source})
    for key in ['axle_distance_m','body_length_m','wheel_axis_height_m','wheel_center_y_m','environment_rail_half_center_m','display_rail_head_width_m','bogie_vertical_datum_shift_m','bogie_transverse_shift_m']:
        rows.append({'parameter':key,'value':C[key],'evidence_level':'CAD_VERIFIED_2D' if key in ('axle_distance_m','body_length_m') else 'DISPLAY_ALIGNMENT_OR_SECTION_ASSUMPTION','source':'assembly_contract.json; '+C['evidence'].get(key,'new geometry shared datum')})
    for key in ['arm_projected_delta_x_m','arm_projected_delta_z_m','arm_projected_center_distance_m']:
        rows.append({'parameter':key,'value':G[key],'evidence_level':'CAD_2D_MOTION_CANDIDATE_NOT_VERIFIED_HINGE','source':'component_bogie.py PARAMETERS / cad_motion_candidates_v1_scale_clarification.json'})
    result={'schema':'gtsd-bogie-parameters-v1','parameters':p,'source_geometry_contract':C,'parameter_provenance':rows,'geometry_source_sha256':{str(x.relative_to(GEOMETRY)):hashlib.sha256(x.read_bytes()).hexdigest() for x in [GEOMETRY/'assembly_contract.json',GEOMETRY/'scripts/component_bogie.py',GEOMETRY/'web/geometry.json']},'mechanism_level':'candidate planar paired swing arms; inferred common transverse hinge, synchronized left/right arms, no established 3D topology'}
    (ROOT/'parameters.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    with (ROOT/'PARAMETER_PROVENANCE.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['parameter','value','evidence_level','source']);writer.writeheader();writer.writerows(rows)
    return result

if __name__=='__main__':
    (ROOT/'model.xml').write_text(make_model());write_parameters();print(ROOT/'model.xml')
