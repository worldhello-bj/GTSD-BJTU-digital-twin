"""Frozen-source, new-geometry integrated engineering equivalent; SI only.
No qpos playback, hardware, network or imported historical vehicle mechanics.
"""
from pathlib import Path
import copy, csv, hashlib, importlib.util, json, os, sys
from dataclasses import asdict
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent
os.environ['GTSD_GEOMETRY_ROOT']=str(SOURCE)
sys.path.insert(0,str(ROOT/'vendor/bogie'))
sys.path.insert(0,str(ROOT/'vendor/drive_brake'))
import model as bogie
import coupling

def import_path(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    out=importlib.util.module_from_spec(spec);sys.modules[name]=out;spec.loader.exec_module(out);return out
panto=import_path('frozen_panto',ROOT/'vendor/pantograph/pantograph.py')
helpers=import_path('frozen_bogie_helpers',ROOT/'vendor/bogie/run_suite.py')
FROZEN_GEOMETRY_SHA='7d925267cbfb18d39e64041ea851fc13db2e3a13f6d403eab386235f741d8f43'
DEFAULT_DT=.0004

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def freeze_manifest():
    files=list((ROOT/'vendor').rglob('*'))
    files=[p for p in files if p.is_file() and '__pycache__' not in str(p)]
    return {str(p.relative_to(ROOT)):sha(p) for p in sorted(files)}

def build_model(dt=DEFAULT_DT):
    assert sha(SOURCE/'web/geometry.json')==FROZEN_GEOMETRY_SHA
    bp=bogie.DEFAULT|{'timestep_s':dt}
    pp=panto.DEFAULTS|{'timestep_s':dt}
    mp=coupling.DriveParameters()
    root=ET.fromstring(bogie.make_model(bp))
    root.set('model','GTSD_v5_integrated_uncalibrated_new_geometry_equivalent')
    root.find('compiler').set('inertiafromgeom','false')
    root.find('option').set('iterations','100')
    _,pxml,g,_=panto.build_model(SOURCE,pp)
    proot=ET.fromstring(pxml)
    payload=root.find(".//body[@name='payload_secondary']")
    world=root.find('worldbody')
    base=copy.deepcopy(proot.find(".//body[@name='panto_base']"))
    # Source base origin is world zero. The payload's reference transform has
    # identity rotation, so this local translation preserves EVERY panto anchor.
    base.set('pos',panto.vec(-np.array(bogie.reference_positions(bp)['payload_secondary'])))
    # Stamp pantograph-only defaults; never leak joint limits into bogie/free DOFs.
    joint_defaults=proot.find('default/joint').attrib
    for j in base.iter('joint'):
        for key,value in joint_defaults.items():
            if key not in j.attrib:j.set(key,value)
    # All visual proxies inherit zero collision bits. Only 1/2 wheel/rail and
    # 8/8 collector/wire pairs are active; no blind defaults merge.
    payload.append(base)
    world.append(copy.deepcopy(proot.find(".//body[@name='panto_wire']")))
    for a in proot.find('actuator'):root.find('actuator').append(copy.deepcopy(a))
    for i in (1,2):
        j=root.find(f".//joint[@name='wheel_spin_{i}']")
        j.set('armature',str(mp.rotor_inertia_kgm2*mp.gear_ratio_motor_per_axle**2))
        j.set('damping',str(bp['wheel_bearing_damping_Nmsprad']+mp.rotor_viscous_friction_Nms_rad*mp.gear_ratio_motor_per_axle**2))
    ET.indent(root)
    xml=ET.tostring(root,encoding='unicode')
    model=mujoco.MjModel.from_xml_string(xml)
    return model,xml,bp,pp,mp,g

def poses(m,d):
    out=helpers.poses(m,d)
    out['world']={'position':[0.,0.,0.],'quaternion':[1.,0.,0.,0.]}
    return out

def binding(m,g):
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    a=json.loads((ROOT/'vendor/bogie/binding.json').read_text())
    b=json.loads((ROOT/'vendor/pantograph/binding.json').read_text())
    rows=copy.deepcopy(a['meshes']+b['meshes']);mapped={r['mesh_name'] for r in rows}
    for mesh in g['meshes']:
        n=mesh['name']
        if n in mapped:continue
        group=mesh.get('group')
        if group=='body':
            flex=any(s in n.lower() for s in ('wire','lead','hose','tube'))
            mode='excluded_deformable' if flex else 'rigid'
            note=('Flexible cable/lead/tube geometry has no deformation model; hide in dynamic view.' if flex else 'Rigidly fixed cabinet/body mounting candidate. Included in 30 kg payload equivalent; no per-mesh added mass.')
            if n.startswith(('SHELL_access_door','SHELL_door_handle','SHELL_hinge')):
                note+=' Door fixed at supplied reference pose. No hinge coordinate or door actuation is asserted.'
            if 'fan_rotor' in n:note+=' Fan rotation is not simulated.'
            body='payload_secondary';evidence='EXPLICIT_UNCALIBRATED_RIGID_BODY_MOUNTING_ASSUMPTION'
        elif group=='environment':
            body='world';mode='rigid';evidence='FIXED_SOURCE_ENVIRONMENT_REFERENCE';note='Fixed world scenery; visual geometry does not imply collision-active hardware.'
        else:
            body='frame';mode='static_reference';evidence='UNRESOLVED_ATTACHMENT';note='No established physical ownership or mechanism; hidden during dynamic replay.'
        rows.append({'mesh_name':n,'body_name':body,'mode':mode,'evidence':evidence,'note':note})
    for row in rows:
        row['visible_in_dynamic_view']=row['mode']=='rigid'
    assert len(rows)==len(g['meshes'])==len({r['mesh_name'] for r in rows})
    return {'schema':'gtsd-physics-binding-v1','subsystem':'integrated','source_geometry_sha256':FROZEN_GEOMETRY_SHA,
       'reference_poses':poses(m,d),'meshes':rows,'coordinate_system':'metres; X longitudinal, Y transverse, Z up; quaternion wxyz; no scale adjustment',
       'transform_contract':b['transform_contract'],'classification_note':'Exact mesh names only. static_reference and excluded_deformable hidden in moving view. Fixed doors and cabinet attachments are assumptions.',
       'unmodeled':panto.UNMODELED+['All independently moving door/fan/internal cabinet mechanics.', 'Unresolved bogie linkage/drive-shaft details remain hidden.', 'Actual whole-vehicle mass, COM, inertia and suspension calibration.']}

def metadata(m,xml,bp,pp,mp,g):
    mass={'frame_kg':bp['frame_mass_kg'],'body_cabinet_equivalent_excluding_pantograph_kg':bp['payload_mass_kg'],
          'two_arm_carriers_kg':2*bp['carrier_mass_kg'],'two_wheelsets_kg':2*bp['wheelset_mass_kg'],
          'pantograph_lower_kg':pp['lower_mass_kg'],'pantograph_upper_kg':pp['upper_mass_kg'],
          'pantograph_head_carriage_kg':pp['head_carriage_mass_kg'],'pantograph_head_float_kg':pp['head_float_mass_kg']}
    assert abs(sum(mass.values())-sum(m.body_mass))<1e-10
    sources={str(p.relative_to(SOURCE)):sha(p) for p in [SOURCE/'web/geometry.json',SOURCE/'assembly_contract.json',SOURCE/'scripts/component_bogie.py',SOURCE/'scripts/component_pantograph.py',SOURCE/'components/pantograph_manifest.json']}
    params={'schema':'gtsd-integrated-parameters-v1','calibration_status':'UNCALIBRATED_ENGINEERING_INTEGRATION_NOT_VALIDATED_WHOLE_VEHICLE',
       'bogie':bp,'pantograph':pp,'drive':asdict(mp),'brake':asdict(coupling.BrakeParameters(gas_substep_s=min(.0005,bp['timestep_s']))),
       'mass_ledger':mass,'total_dynamic_mass_kg':float(sum(m.body_mass)),
       'mass_boundary':'30 kg is defined here as body/cabinets/mounted static supports excluding all 3.3 kg moving pantograph bodies. Panto_base has zero separately added mass, fixed roof/base supports are included in 30 kg. No mesh-volume inferred mass. Motor rotational inertia is reflected through joint armature only; physical motor case mass is included in frame equivalent, rotor energy must not be added twice.',
       'preload_boundary':'Original primary and secondary spring reference settings preserved (54 kg primary frame+payload preload; 30 kg secondary preload). Added panto mass causes actual sag/reaction; no hidden preload re-tuning.',
       'mounting_assumption':'panto_base is welded as a fixed child of payload_secondary; source world anchors preserved exactly. body-group rigid cabinets/doors fixed to payload at source reference pose.',
       'force_port_boundary':'head load is an explicit external world-vertical laboratory test force at head_load_point. It is not gas pressure, bellows force, or B32 cylinder force.',
       'source_sha256':sources,'vendor_sha256':freeze_manifest(),
       'discrete_acceptance_tolerances':{'midpoint_feedback_rad_s':1e-7,'per_axle_positive_brake_work_J':1e-12},
       'integration_sources_sha256':{name:sha(ROOT/name) for name in ('integrated_model.py','run_integrated.py','run_exploratory.py','verify_integrated.py','split_traces.py')},
       'collision_boundary':'wheel bits 2 against rail bits 1; panto collector bits 8 against wire bits 8; all other proxy geoms inactive. Wire fixed to world.',
       'inheritance':'B8/B9 used for scalar finite-volume gas/spool kernels only; no historical vehicle geometry or mechanical factory imported.',
       'unmodeled':panto.UNMODELED}
    rows=[]
    for section in ('bogie','pantograph','drive','brake'):
        for k,v in params[section].items():rows.append({'parameter':section+'.'+k,'value':json.dumps(v),'evidence':'INHERITED_UNCALIBRATED_EQUIVALENT_OR_NUMERICAL_SETTING','source':f'vendor/{section if section in ("bogie","pantograph") else "drive_brake"}/parameters.json'})
    for k,v in mass.items():rows.append({'parameter':'mass_boundary.'+k,'value':v,'evidence':'EXPLICIT_EQUIVALENT_MASS_ACCOUNTING_NOT_MEASUREMENT','source':'integrated/integrated_model.py'})
    (ROOT/'model.xml').write_text(xml)
    (ROOT/'parameters.json').write_text(json.dumps(params,ensure_ascii=False,indent=2))
    (ROOT/'source_hashes.json').write_text(json.dumps({'geometry':sources,'frozen_vendor':freeze_manifest(),'integration':params['integration_sources_sha256']},indent=2))
    (ROOT/'binding.json').write_text(json.dumps(binding(m,g),ensure_ascii=False,indent=2))
    with (ROOT/'PARAMETER_PROVENANCE.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['parameter','value','evidence','source']);w.writeheader();w.writerows(rows)
    return params

if __name__=='__main__':
    a=build_model();metadata(*a);print('mass',sum(a[0].body_mass),'dof',a[0].nv)
