"""Spatial suspension for the NEW 0.8333345658 m wheelbase / 1.3 m side frame.

The generalized bush coordinates are engineering equivalents, not reconstructed
hardware hinges. This module never imports old vehicle dynamics or edits frozen
source modules. All dynamics are native MuJoCo forces; no pose animation.
"""
from pathlib import Path
import copy
import csv
import hashlib
import importlib.util
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent
AXES = {'x': [1,0,0], 'y': [0,1,0], 'z': [0,0,1],
        'roll': [1,0,0], 'pitch': [0,1,0], 'yaw': [0,0,1]}
# Fresh numerical design choices; NONE is represented as a measured calibration.
# z=40 kN/m alone retains the existing NEW bogie's mechanical demonstration value.
DEFAULT = {
    'secondary_x_stiffness_Npm': 26000., 'secondary_y_stiffness_Npm': 34000.,
    'secondary_z_stiffness_Npm': 40000.,
    'secondary_x_damping_Nspm': 210., 'secondary_y_damping_Nspm': 240.,
    'secondary_z_damping_Nspm': 300.,
    'secondary_roll_stiffness_Nmprad': 950.,
    'secondary_pitch_stiffness_Nmprad': 1300.,
    'secondary_yaw_stiffness_Nmprad': 1100.,
    'secondary_roll_damping_Nmsprad': 32.,
    'secondary_pitch_damping_Nmsprad': 45.,
    'secondary_yaw_damping_Nmsprad': 38.,
    'secondary_x_limit_m': .025, 'secondary_y_limit_m': .020,
    'secondary_z_limit_m': .030,
    'secondary_roll_limit_rad': .075, 'secondary_pitch_limit_rad': .070,
    'secondary_yaw_limit_rad': .090,
    'primary_x_stiffness_Npm': 56000., 'primary_y_stiffness_Npm': 48000.,
    'primary_roll_stiffness_Nmprad': 850., 'primary_yaw_stiffness_Nmprad': 1200.,
    'primary_x_damping_Nspm': 180., 'primary_y_damping_Nspm': 160.,
    'primary_roll_damping_Nmsprad': 12., 'primary_yaw_damping_Nmsprad': 14.,
    'primary_x_limit_m': .008, 'primary_y_limit_m': .008,
    'primary_roll_limit_rad': .045, 'primary_yaw_limit_rad': .045,
    'limit_timeconst_s': .006, 'limit_dampratio': 1.0, 'limit_impedance': .99,
    'air_enabled': False,
    'air_spring_half_spacing_m': .18,
    'secondary_backup_stiffness_Npm': 1800.,
    'secondary_backup_damping_Nspm': 60.,
    'air_support_fraction': 1.0,
    'secondary_preload_mass_kg': None,
    'primary_preload_mode': 'supported_subtree_mass',
    'gravity_preload_mps2': 9.81,
}

def sha(data):
    return hashlib.sha256(data.encode() if isinstance(data,str) else data).hexdigest()

def parameters(overrides=None):
    unknown = set(overrides or {}) - set(DEFAULT)
    if unknown: raise ValueError(f'Unknown suspension parameters: {sorted(unknown)}')
    p = DEFAULT | (overrides or {})
    for key,value in p.items():
        if key.endswith(('_Npm','_Nspm','_Nmprad','_Nmsprad','_limit_m','_limit_rad','_s')):
            if not np.isfinite(value) or value <= 0: raise ValueError(f'{key} must be finite and positive')
    if type(p['air_enabled']) is not bool: raise ValueError('air_enabled must be boolean')
    if not np.isfinite(p['air_spring_half_spacing_m']) or p['air_spring_half_spacing_m'] <= 0: raise ValueError('Air half spacing must be positive')
    if not np.isfinite(p['limit_dampratio']) or p['limit_dampratio'] <= 0: raise ValueError('Limit damping ratio must be positive')
    if not 0 <= p['air_support_fraction'] <= 1: raise ValueError('air_support_fraction must lie in [0,1]')
    if not 0 < p['limit_impedance'] < 1: raise ValueError('limit_impedance must lie in (0,1)')
    if not np.isfinite(p['gravity_preload_mps2']) or p['gravity_preload_mps2'] < 0:
        raise ValueError('gravity_preload_mps2 must be finite and nonnegative')
    if p['secondary_preload_mass_kg'] is not None and (not np.isfinite(p['secondary_preload_mass_kg']) or p['secondary_preload_mass_kg'] < 0):
        raise ValueError('secondary_preload_mass_kg must be finite and nonnegative')
    if p['primary_preload_mode'] not in ('supported_subtree_mass','preserve_inherited'):
        raise ValueError('Unsupported primary_preload_mode')
    return p

def _body(root,name):
    out = root.find(f".//body[@name='{name}']")
    if out is None: raise ValueError(f'Missing required source body {name}')
    return out

def _mass(body):
    return sum(float(e.get('mass','0')) for e in body.iter('inertial'))

def preload_allocation(xml, overrides=None):
    """Return explicit support allocation; derived mass is a simulation ledger.

    air_required_force_N is the TOTAL equilibrium axial force the external gas
    subsystem must supply. Merely setting air_enabled does NOT implement gas.
    """
    p = parameters(overrides)
    root = ET.fromstring(xml)
    payload = _body(root,'payload_secondary')
    frame = _body(root,'frame')
    actual_mass = _mass(payload)
    selected_mass = actual_mass if p['secondary_preload_mass_kg'] is None else p['secondary_preload_mass_kg']
    desired = selected_mass*p['gravity_preload_mps2']
    fraction = p['air_support_fraction'] if p['air_enabled'] else 0.
    primary_mass = _mass(frame) - sum(_mass(_body(root,f'arm_carrier_{i}')) for i in (1,2))
    return {'payload_subtree_mass_kg': actual_mass,
            'selected_secondary_preload_mass_kg': selected_mass,
            'secondary_total_preload_N': desired,
            'secondary_mechanical_preload_N': desired*(1-fraction),
            'air_required_force_N': desired*fraction,
            'primary_supported_mass_kg': primary_mass,
            'primary_each_target_preload_N': primary_mass*p['gravity_preload_mps2']/4,
            'primary_preload_mode': p['primary_preload_mode'],
            'mass_override_matches_actual': abs(actual_mass-selected_mass)<1e-9,
            'note': 'Equivalent model masses only. Pressure/force initialization must use the same allocation; external head/wire loads and lever moments are not hidden in preload.'}

def _fmt(value): return ' '.join(format(x,'.16g') for x in value) if isinstance(value,(list,tuple,np.ndarray)) else str(value)

def _joint(parent,name,group,dof,p):
    trans = dof in ('x','y','z')
    su,du,qu = ('Npm','Nspm','m') if trans else ('Nmprad','Nmsprad','rad')
    k,c = p[f'{group}_{dof}_stiffness_{su}'],p[f'{group}_{dof}_damping_{du}']
    if name=='secondary_z' and p['air_enabled']:
        k,c = p['secondary_backup_stiffness_Npm'],p['secondary_backup_damping_Nspm']
    lim = p[f'{group}_{dof}_limit_{qu}']
    return ET.Element('joint', {k:_fmt(v) for k,v in dict(name=name,type='slide' if trans else 'hinge',
        axis=AXES[dof],pos=[0,0,0],limited='true',range=[-lim,lim],stiffness=k,damping=c,
        springref=0,margin=0,solreflimit=[p['limit_timeconst_s'],p['limit_dampratio']],
        solimplimit=[p['limit_impedance'],p['limit_impedance'],.001]).items()})

def augment_xml(xml, parameters=None):
    """Pure XML -> XML, preserve all source bodies/geometries/sites/actuators.

    Secondary order: x,y,z,roll,pitch,yaw, all at payload reference COM.
    Carrier order: x,y,roll,yaw,existing candidate swing. Rotations compose in
    this order; finite rotations are not claimed to be independent world angles.
    No mass is added. 13 scalar coordinates augment either 11-DOF bare bogie or
    15-DOF integrated model; untouched body names keep source mesh ownership.
    """
    p = globals()['parameters'](parameters)
    root = ET.fromstring(xml)
    if root.find(".//joint[@name='secondary_x']") is not None:
        raise ValueError('Suspension 3D is already augmented; refusing duplicate coordinates')
    allocation = preload_allocation(xml,p)
    payload = _body(root,'payload_secondary')
    old = payload.find("joint[@name='secondary_z']")
    if old is None: raise ValueError('Source requires exactly the native secondary_z joint')
    payload.remove(old)
    for n,dof in enumerate(('x','y','z','roll','pitch','yaw')):
        j = _joint(payload,f'secondary_{dof}','secondary',dof,p)
        if dof=='z': j.set('springref',str(allocation['secondary_mechanical_preload_N']/float(j.get('stiffness'))))
        payload.insert(n,j)
    for i in (1,2):
        carrier = _body(root,f'arm_carrier_{i}')
        for n,dof in enumerate(('x','y','roll','yaw')):
            carrier.insert(n,_joint(carrier,f'primary_{i}_{dof}','primary',dof,p))
    if p['primary_preload_mode']=='supported_subtree_mass':
        # The source end points have 83.5 mm separation; measure through native
        # kinematics rather than assuming it remains that way after augmentation.
        m = mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        d = mujoco.MjData(m); mujoco.mj_forward(m,d)
        for i in (1,2):
            for side in (-1,1):
                name=f'primary_{i}_{side}'
                tendon=root.find(f"tendon/spatial[@name='{name}']")
                tid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_TENDON,name)
                if tendon is None or tid < 0: raise ValueError(f'Missing original spatial spring {name}')
                k=float(tendon.get('stiffness','0'))
                if k<=0: raise ValueError('Primary spatial spring requires positive source stiffness')
                tendon.set('springlength',str(d.ten_length[tid]+allocation['primary_each_target_preload_N']/k))
    if p['air_enabled']:
        frame=_body(root,'frame')
        lower=frame.find("site[@name='secondary_lower']")
        upper=payload.find("site[@name='secondary_upper']")
        if lower is None or upper is None: raise ValueError('Air ports require original secondary endpoint references')
        tendon_section=root.find('tendon');actuator_section=root.find('actuator')
        for side,sgn in [('left',-1),('right',1)]:
            name=f'air_spring_{side}'
            for body,source,ending in [(frame,lower,'lower'),(payload,upper,'upper')]:
                xyz=np.fromstring(source.get('pos'),sep=' ');xyz[1]+=sgn*p['air_spring_half_spacing_m']
                ET.SubElement(body,'site',name=f'{name}_{ending}',pos=_fmt(xyz),size='.004',rgba='.2 .7 .8 1')
            tendon=ET.SubElement(tendon_section,'spatial',name=name,width='.008',rgba='.2 .7 .8 1')
            ET.SubElement(tendon,'site',site=name+'_lower');ET.SubElement(tendon,'site',site=name+'_upper')
            ET.SubElement(actuator_section,'motor',name=name+'_force',tendon=name,gear='1',ctrllimited='false')
    custom=root.find('custom')
    if custom is None: custom=ET.SubElement(root,'custom')
    ET.SubElement(custom,'text',name='suspension_3d_boundary',data='Uncalibrated generalized bushing compliance. No photo-established extra hinges. Cylindrical wheel rail proxy has no flange or derailment validation.')
    ET.SubElement(custom,'numeric',name='suspension_3d_preload_N',data=_fmt([allocation['secondary_mechanical_preload_N'],allocation['air_required_force_N'],allocation['primary_each_target_preload_N']]))
    ET.SubElement(custom,'numeric',name='suspension_3d_air_enabled',data='1' if p['air_enabled'] else '0')
    root.set('model','GTSD_new_geometry_spatial_suspension_uncalibrated')
    ET.indent(root)
    return ET.tostring(root,encoding='unicode')

def _import(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module);return module

def build_model(parameters=None,timestep_s=.0004,base='integrated'):
    """Return (MjModel, xml, metadata); runtime factory can use augment_xml alone."""
    if base=='integrated':
        source=_import('suspension_source_integrated',SOURCE/'integrated/integrated_model.py')
        _,xml,*_=source.build_model(dt=timestep_s)
    elif base=='bogie':
        source=_import('suspension_source_bogie',SOURCE/'bogie/model.py')
        xml=source.make_model({'timestep_s':timestep_s})
    else: raise ValueError('base must be integrated or bogie')
    p=globals()['parameters'](parameters)
    augmented=augment_xml(xml,p)
    m=mujoco.MjModel.from_xml_string(augmented)
    return m,augmented,metadata(m,xml,augmented,p,base)

def joint_names():
    return ([f'secondary_{q}' for q in ('x','y','z','roll','pitch','yaw')]
            +[f'primary_{i}_{q}' for i in (1,2) for q in ('x','y','roll','yaw')])

def telemetry(m,d):
    """Real generalized loads and actual changing 3D endpoint positions."""
    joints={}
    for name in joint_names():
        j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)
        if j<0: raise ValueError(f'Missing expected augmented joint {name}')
        qa,va=m.jnt_qposadr[j],m.jnt_dofadr[j]
        q,v=float(d.qpos[qa]),float(d.qvel[va])
        elastic=-m.jnt_stiffness[j]*(q-m.qpos_spring[qa])
        damping=-m.dof_damping[va]*v
        joints[name]={'q':q,'q_unit':'m' if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else 'rad',
            'qvel':v,'elastic_generalized_load':float(elastic),'damping_generalized_load':float(damping),
            'load_unit':'N' if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else 'N m',
            'passive_generalized_load_with_spatial_springs':float(d.qfrc_passive[va]),
            'constraint_generalized_load':float(d.qfrc_constraint[va]),
            'limit_interval':m.jnt_range[j].tolist()}
    springs={}
    for i in (1,2):
        for side in (-1,1):
            name=f'primary_{i}_{side}';tid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_TENDON,name)
            lo=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,f'primary_lower_{i}_{side}')
            up=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,f'primary_upper_{i}_{side}')
            L=float(d.ten_length[tid]);v=float(d.ten_velocity[tid]);L0=float(m.tendon_lengthspring[tid,0])
            springs[name]={'length_m':L,'length_velocity_mps':v,
                'compression_force_N':float(m.tendon_stiffness[tid]*(L0-L)-m.tendon_damping[tid]*v),
                'lower_world_m':d.site_xpos[lo].tolist(),'upper_world_m':d.site_xpos[up].tolist(),
                'endpoint_distance_error_m':float(abs(np.linalg.norm(d.site_xpos[up]-d.site_xpos[lo])-L))}
    return {'joints':joints,'primary_spatial_springs':springs,
            'potential_energy_J':float(d.energy[0]),'kinetic_energy_J':float(d.energy[1]),
            'finite_state':bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())}

def poses(m,d):
    return {mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i):
        {'position':d.xpos[i].tolist(),'quaternion':d.xquat[i].tolist()} for i in range(m.nbody)}

def binding(m,source_binding=None):
    source_binding = source_binding or json.loads((SOURCE/'integrated/binding.json').read_text())
    result=copy.deepcopy(source_binding)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    result['subsystem']='suspension_3d_independent_extension'
    result['reference_poses']=poses(m,d)
    result['extension_boundary']='Existing exact source-mesh body ownership retained for all composite 3D poses. Unresolved attachments remain hidden. Generalized compliance does not establish real additional joints.'
    for row in result['meshes']:
        if row.get('body_name') is not None and row.get('body_name') not in result['reference_poses']:
            raise ValueError('Source mesh binding references missing body: '+row['body_name'])
    return result

def provenance(p):
    rows=[]
    units={'Npm':'N/m','Nspm':'N s/m','Nmprad':'N m/rad','Nmsprad':'N m s/rad','mps2':'m/s^2','kg':'kg','rad':'rad','m':'m','s':'s'}
    for key,value in p.items():
        unit=next((v for k,v in units.items() if key.endswith('_'+k)),'dimensionless or selection')
        evidence='EXPLICIT_UNCALIBRATED_NEW_GEOMETRY_ENGINEERING_EQUIVALENT'
        source='suspension_3d/model.py independent numerical design; no hardware measurement'
        if key in ('secondary_z_stiffness_Npm','secondary_z_damping_Nspm'):
            evidence='INHERITED_NEW_BOGIE_UNCALIBRATED_EQUIVALENT';source='bogie/model.py DEFAULT; not the old B8 calibration'
        if key.startswith('limit_'): evidence='NUMERICAL_SOFT_STOP_SETTING';source='MuJoCo soft joint constraint; time constant is not a measured bumper stiffness'
        if 'preload' in key or 'support_fraction' in key:
            evidence='EXPLICIT_PRELOAD_ACCOUNTING_ASSUMPTION';source='mass ledger and declared gravity; force allocation, not pressure calibration'
        rows.append({'parameter':key,'value':value,'unit':unit,'evidence_level':evidence,'source':source})
    return rows


def compiled_provenance(m):
    """Every native suspension spring, damper, preload and numerical stop."""
    d=mujoco.MjData(m);mujoco.mj_forward(m,d);rows=[]
    names=joint_names()+['swing_1','swing_2']
    for name in names:
        j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)
        q=m.jnt_qposadr[j];v=m.jnt_dofadr[j];trans=m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE
        qu,fu=('m','N') if trans else ('rad','N m')
        values={'stiffness':(float(m.jnt_stiffness[j]),fu+'/'+qu),
                'damping':(float(m.dof_damping[v]),fu+' s/'+qu),
                'spring_reference':(float(m.qpos_spring[q]),qu),
                'reference_elastic_preload':(float(m.jnt_stiffness[j]*(m.qpos_spring[q]-m.qpos0[q])),fu),
                'soft_stop_range':(m.jnt_range[j].tolist(),qu),
                'soft_stop_solref':(m.jnt_solref[j].tolist(),'[s, dimensionless]'),
                'soft_stop_solimp':(m.jnt_solimp[j].tolist(),'MuJoCo solver parameters')}
        for key,(value,unit) in values.items():
            rows.append({'parameter':name+'.'+key,'value':value,'unit':unit,
                'evidence_level':'NATIVE_COMPILED_UNCALIBRATED_EQUIVALENT',
                'source':'suspension_3d/model.py; candidate swing retained from new bogie, never a measured hinge assertion'})
    for name in [f'primary_{i}_{s}' for i in (1,2) for s in (-1,1)]:
        t=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_TENDON,name)
        L=float(d.ten_length[t]);rest=float(m.tendon_lengthspring[t,0]);k=float(m.tendon_stiffness[t])
        for key,value,unit in [('stiffness',k,'N/m'),('damping',float(m.tendon_damping[t]),'N s/m'),
                               ('reference_endpoint_length',L,'m'),('unloaded_length',rest,'m'),
                               ('reference_compression_preload',k*(rest-L),'N')]:
            rows.append({'parameter':name+'.'+key,'value':value,'unit':unit,
                'evidence_level':'NATIVE_SPATIAL_SPRING_UNCALIBRATED_EQUIVALENT',
                'source':'Source new-bogie four spatial springs; rest length from explicit supported-mass allocation, endpoints from MuJoCo'})
    return rows

def metadata(m,source_xml,xml,p,base):
    source_model=mujoco.MjModel.from_xml_string(source_xml)
    mass={mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i):float(m.body_mass[i]) for i in range(1,m.nbody) if m.body_mass[i]>0}
    source_joint_names={mujoco.mj_id2name(source_model,mujoco.mjtObj.mjOBJ_JOINT,i) for i in range(source_model.njnt)}
    return {'schema':'gtsd-suspension-3d-v1','base_model':base,
        'source_model_sha256':sha(source_xml),'model_sha256':sha(xml),
        'source_geometry_sha256':sha((SOURCE/'web/geometry.json').read_bytes()),
        'parameters':p,'parameter_provenance':provenance(p)+compiled_provenance(m),
        'preload_allocation':preload_allocation(source_xml,p),
        'mass_ledger_kg':mass,'total_dynamic_mass_kg':float(sum(m.body_mass)),
        'added_mass_kg':float(sum(m.body_mass)-sum(source_model.body_mass)),
        'source_nv':source_model.nv,'nv':m.nv,'nq':m.nq,
        'added_coordinates':[x for x in joint_names() if x not in source_joint_names],
        'kinematic_order':{'secondary':['x','y','z','roll','pitch','yaw'],'primary':['x','y','roll','yaw','candidate_swing']},
        'source_geometry_preserved':{'axle_distance_m':.8333345657712634,'body_side_frame_length_m':1.3,
            'rail_contour_center_distance_m':.51783333333332,'rail_spacing_boundary':'2D contour center distance, NOT inside rail gauge'},
        'mechanism_status':'Uncalibrated small-displacement generalized bush; candidate paired swing retained; real spatial primary spring endpoints depend on native composed transforms.',
        'air_boundary':'air_enabled replaces 40 kN/m spring with backup. It requires an external finite-volume gas force implementation. air_required_force_N is allocation, never applied by this module.',
        'wheel_rail_boundary':'Unchanged cylindrical tread/rectangular rail proxies. No measured wheel profile, flange-contact, derailment or high-speed stability claim.',
        'old_model_boundary':'Old B8 inspected for feature coverage only. No geometry, suspension constants, trajectories or mass scaling imported.'}

def write_artifacts(m,xml,meta,directory=ROOT):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    (directory/'model.xml').write_text(xml)
    (directory/'parameters.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
    with (directory/'PARAMETER_PROVENANCE.csv').open('w',newline='') as stream:
        rows=meta['parameter_provenance'];writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    (directory/'initial_state.json').write_text(json.dumps({'model_sha256':sha(xml),'time_s':d.time,'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'body_poses':poses(m,d),'telemetry':telemetry(m,d)},indent=2))
    if meta['base_model']=='integrated': (directory/'binding.json').write_text(json.dumps(binding(m),ensure_ascii=False,indent=2))

if __name__=='__main__':
    m,xml,meta=build_model();write_artifacts(m,xml,meta)
    print(json.dumps({'nv':m.nv,'mass_kg':sum(m.body_mass),'model_sha256':meta['model_sha256'],'preload_allocation':meta['preload_allocation']}))
