"""Stage 2: exact pin/plate/slot contact on the shared finite-mount rig.

Only two named station pin interfaces are admitted. Other rail, balise, ring,
wire and jaw contacts remain explicitly outside this stage. No hidden welds.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as X
import numpy as np
from build_rig import build as build_mount, ROOT, fmt, source_meshes
from mass_properties import integrate, canonical_sha
from contact_adapter import add_partition


def build(parameters, root=ROOT):
    cfg=dict(parameters)
    if cfg.pop('mode',None)!='pin_core_contact':raise ValueError('Explicit pin_core_contact mode required')
    pin_variant=cfg.pop('pin_geometry_variant','nominal_d5')
    if pin_variant not in ('nominal_d5','research_d4p98'):
        raise ValueError('Choose nominal D5 or explicit D4.98 complete-geometry research variant')
    overrides={}
    variant_path=Path(__file__).parent/'contact_assets/pin_geometry_variants.json'
    variants=json.loads(variant_path.read_text())['variants']
    original_rows,_=source_meshes(root)
    originals={m['name']:m for m in original_rows}
    if pin_variant=='research_d4p98':
        for station in ('50DF4','51409'):
            name='FIT_'+station+'_F643_NOMINAL_PIN5_8A2C_CANDIDATE'
            mesh=next(m for m in variants if m['name']=='RESEARCH_D4p98_'+name)
            old=originals[name]
            if mesh['source_parent_name']!=name or mesh['source_parent_geometry_sha256']!=canonical_sha({'name':old['name'],'vertices':old['vertices'],'indices':old['indices']}):
                raise ValueError('Pin variant source-parent witness failed')
            if mesh['geometry_content_sha256']!=canonical_sha({'name':mesh['name'],'vertices':mesh['vertices'],'indices':mesh['indices']}):
                raise ValueError('Pin variant complete-geometry witness failed')
            if mesh['nominal_research_diameter_mm']!=4.98 or not mesh['not_measured_hardware']:
                raise ValueError('Unexpected or unlabelled pin variant')
            overrides[name]=mesh
    xml,contract=build_mount(cfg,root,mesh_overrides=overrides)
    partition_path=Path(__file__).parent/'contact_assets/core_partition.json'
    raw=partition_path.read_bytes();partition=json.loads(raw)
    for rel,expected in partition['source_files_sha256'].items():
        if hashlib.sha256((root/rel).read_bytes()).hexdigest()!=expected:raise ValueError('Partition source changed '+rel)
    tree=X.fromstring(xml);asset=X.SubElement(tree,'asset');rig=tree.find('./worldbody/body')
    opt=tree.find('option');opt.set('cone','elliptic');opt.set('ccd_tolerance','1e-10');opt.set('ccd_iterations','100')
    flag=opt.find('flag');flag.set('nativeccd','enable');flag.set('multiccd','enable')
    default=X.SubElement(tree,'default');X.SubElement(default,'geom',friction='0.3 0.005 0.0001',
         solref='0.002 1',solimp='0.95 0.99 0.0001',margin='0',gap='0')
    retained=json.loads((root/'mechanical_fit/components/clamps/published_clamp_meshes.json').read_text())['meshes']
    if isinstance(retained,dict):retained=list(retained.values())
    candidates,_=source_meshes(root)
    all_meshes={m['name']:m for m in [*retained,*candidates]}
    colliders=[];force_coverage=[]
    for station,bit in [('50DF4',1),('51409',2)]:
        part_names=['FIT_'+station+'_B535_OUTLINE_8A2C_HOLE25_VERSION_CANDIDATE','CAD_'+station+'_F643_profile_0']
        for name in part_names:
            part=next(p for p in partition['parts'] if p['source_name']==name)
            mesh=all_meshes[name]
            if canonical_sha({'name':mesh['name'],'vertices':mesh['vertices'],'indices':mesh['indices']})!=part['source_geometry_sha256']:
                raise ValueError('Source mesh/partition geometry identity mismatch '+name)
            body=next((b for b in rig.findall('body') if b.get('name')==name),None)
            if body is None:
                prop=integrate(mesh,density_kg_m3=contract['parameters']['source_density_kg_m3'])
                center=prop['center_of_mass_world_m'];tensor=np.array(prop['inertia_com_world_kg_m2'])
                body=X.SubElement(rig,'body',name=name,pos=fmt(center))
                full=[tensor[0,0],tensor[1,1],tensor[2,2],tensor[0,1],tensor[0,2],tensor[1,2]]
                X.SubElement(body,'inertial',pos='0 0 0',mass=fmt(prop['mass_kg']),fullinertia=fmt(full))
                for i,axis in enumerate(np.eye(3)):
                    k=100000.;c=2*contract['parameters']['damping_ratio']*np.sqrt(k*prop['mass_kg'])
                    jname=name+'_carrier_translation_'+str(i)
                    X.SubElement(body,'joint',name=jname,type='slide',axis=fmt(axis),stiffness=fmt(k),damping=fmt(c))
                    contract['joint_parameters'].append({'name':jname,'kind':'slide','stiffness':k,'damping':float(c),'source':'UNCALIBRATED_CARRIER_RESEARCH_INPUT'})
                contract['inertial_properties'][name]=prop
                contract['bodies'].append({'name':name,'branch':'clamps','reference_position_m':center,'mass_kg':prop['mass_kg']})
                contract['bindings'].append({'mesh_name':name,'body_name':name,'branch':'clamps',
                    'source_geometry_sha256':prop['geometry_sha256'],'body_reference_position_m':center,
                    'body_reference_quaternion_wxyz':[1.,0.,0.,0.], 'rigid_group_hypothesis':'SINGLE_SOURCE_CARRIER'})
            center=next(b['reference_position_m'] for b in contract['bodies'] if b['name']==name)
            names=add_partition(asset,body,part,body_reference_world_m=center,contype=0,conaffinity=bit)
            colliders.append({'source_mesh_name':name,'body_name':name,'geom_names':names,'partition_piece_count':len(names),
                'source_geometry_sha256':canonical_sha({'vertices':mesh['vertices'],'indices':mesh['indices']}),
                'partition_named_source_sha256':part['source_geometry_sha256']})
        pin_name='FIT_'+station+'_F643_NOMINAL_PIN5_8A2C_CANDIDATE'
        pin=overrides.get(pin_name,all_meshes[pin_name]);body=next(b for b in rig.findall('body') if b.get('name')==pin_name)
        center=np.array(next(b['reference_position_m'] for b in contract['bodies'] if b['name']==pin_name))
        vertices=np.array(pin['vertices']).reshape(-1,3)-center
        asset_name='pin_collision_'+station
        X.SubElement(asset,'mesh',name=asset_name,inertia='shell',vertex=fmt(vertices.ravel()),face=' '.join(map(str,pin['indices'])))
        X.SubElement(body,'geom',name=asset_name,type='mesh',mesh=asset_name,mass='0',density='0',contype=str(bit),conaffinity='0')
        colliders.append({'source_mesh_name':pin['name'],'body_name':pin_name,'geom_names':[asset_name],
                          'partition_piece_count':1,'source_geometry_sha256':canonical_sha({'vertices':pin['vertices'],'indices':pin['indices']})})
        force_coverage.append({'interface_id':'pin_core_'+station,'source_meshes':[pin_name,*part_names],
                              'owner':'native_contact','boundary':'finite carrier and body springs return reaction to common rig'})
    contract.pop('identity_sha256')
    contract.update(scope='PIN_CORE_CONTACT_AND_FINITE_MOUNTS',collision_enabled=True,contact_acceptance=False,
                    collision_scope='two pins against their B535 bores and F643 slots only',
                    excluded_unadmitted_contacts=['rail/support','balise/environment','4079/wire','tube/ring','lower jaw/wire'],
                    partition_sha256=hashlib.sha256(raw).hexdigest(),colliders=colliders,native_contact_interfaces=force_coverage,
                    contact_parameters={'friction':[0.3,0.005,0.0001],'solref':[0.002,1.],
                     'solimp':[0.95,0.99,0.0001],'maximum_signed_penetration_m':50e-6,'status':'UNCALIBRATED_NUMERICAL_PARAMETERS'})
    contract['parameters']['mode']='pin_core_contact'
    contract['parameters']['pin_geometry_variant']=pin_variant
    contract['pin_geometry_choice']={'choice':pin_variant,'nominal_is_zero_clearance':True,
        'hardware_diameter_confirmed':False,'display_and_collision_share_same_vertices':True,
        'variant_file_sha256':hashlib.sha256(variant_path.read_bytes()).hexdigest(),
        'reason':'Nominal D5 candidate has exact slot tangency and unqualified transient friction; D4.98 is a separate openly parameterized geometric clearance study.'}
    contract['display_geometry_overrides']=list(overrides.values())
    for name in ('build_contact_rig.py','contact_adapter.py'):
        contract['source_code_sha256'][name]=hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
    xml=X.tostring(tree,encoding='unicode');contract['xml_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    contract['identity_sha256']=canonical_sha(contract)
    return xml,contract
