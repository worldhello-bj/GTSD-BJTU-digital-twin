"""Replaceable opposed-piston engineering assembly, native contacts on both axles.
No source CAD part has been identified as the disc/caliper mounting below.
"""
from dataclasses import asdict
from xml.etree import ElementTree as ET
from brake_contact_bench.model import BrakeParameters

PISTONS=tuple(f'brake_axle_{i}_{side}' for i in (1,2) for side in ('minus','plus'))
PROVENANCE='REPLACEABLE_UNCALIBRATED_ENGINEERING_MOUNT_NO_CONFIRMED_CAD'

def augment_xml(xml, parameters):
    p=parameters;root=ET.fromstring(xml)
    opt=root.find('option');opt.set('cone','elliptic');opt.set('impratio','10');opt.set('tolerance','1e-12')
    flag=opt.find('flag')
    if flag is None:flag=ET.SubElement(opt,'flag')
    flag.set('multiccd','enable')
    contact=root.find('contact')
    if contact is None:contact=ET.SubElement(root,'contact')
    metadata={'provenance':PROVENANCE,'confirmed_CAD_mapping':False,'parameters':asdict(p),
      'mass_added_kg':2*p.disc_mass_kg+4*p.pad_mass_kg,'discs':{},'pistons':{},
      'mass_boundary':'Both discs and four piston/pad assemblies are ADDITIONAL to the 67.3 kg core equivalent. No inferred mesh mass. Caliper fixed support/mount is covered by existing carrier equivalent; actual mounting/parts require measurement.',
      'parameter_boundary':'Contact bench geometry/contact/return parameters retained; each axle has a native disc rigidly welded to wheelset. Each opposed piston is guided relative to arm_carrier. One reservoir/auxiliary plant supplies all four chambers.'}
    def add(parent,tag,**kwargs):return ET.SubElement(parent,tag,{k:str(v) for k,v in kwargs.items()})
    for i in (1,2):
        wheel=root.find(f".//body[@name='wheelset_{i}']");carrier=root.find(f".//body[@name='arm_carrier_{i}']")
        if wheel is None or carrier is None:raise ValueError('Required axle/carrier is absent')
        origin=list(map(float,wheel.get('pos','0 0 0').split()))
        disc=f'brake_disc_{i}';surface=disc+'_surface'
        d=add(wheel,'body',name=disc,pos='0 0 0')
        j=p.disc_spin_inertia_kgm2;jt=p.disc_mass_kg*(3*p.disc_radius_m**2+p.disc_thickness_m**2)/12
        add(d,'inertial',pos='0 0 0',mass=p.disc_mass_kg,diaginertia=f'{jt} {j} {jt}')
        add(d,'geom',name=surface,type='cylinder',size=f'{p.disc_radius_m} {p.disc_thickness_m/2}',quat='.7071067811865476 .7071067811865476 0 0',contype=0,conaffinity=0,rgba='.60 .66 .72 1')
        metadata['discs'][disc]={'body':disc,'surface':surface,'wheel_body':f'wheelset_{i}','axis_joint':f'wheel_spin_{i}','mass_kg':p.disc_mass_kg,'mount':'wheelset-centred Y-axis fixed child; placeholder engineering assumption'}
        for side,sign in (('minus',-1),('plus',1)):
            name=f'brake_axle_{i}_{side}';xyz=[origin[0]+p.pad_radial_x_m,origin[1]+sign*(p.disc_thickness_m/2+p.pad_halfthickness_m+p.pad_release_gap_m),origin[2]+p.pad_radial_z_m]
            b=add(carrier,'body',name=name,pos=' '.join(map(str,xyz)))
            add(b,'joint',name=name+'_slide',type='slide',axis=f'0 {-sign} 0',stiffness=p.pad_return_stiffness_N_m,damping=p.pad_guide_damping_Ns_m,limited='true',range=f'{p.piston_stroke_min_m} {p.piston_stroke_max_m}',springref=0)
            add(b,'inertial',pos='0 0 0',mass=p.pad_mass_kg,diaginertia='.000026 .000037 .000012')
            add(b,'geom',name=name+'_visual',type='box',size=f'{p.pad_halfwidth_x_m} {p.pad_halfthickness_m} {p.pad_halfheight_z_m}',contype=0,conaffinity=0,rgba='.90 .32 .10 1')
            nodes=[]
            for k,(x,z) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
                node=name+'_contact_'+str(k);nodes.append(node)
                add(b,'geom',name=node,type='sphere',size=p.contact_node_radius_m,pos=f'{x*p.contact_node_offset_x_m} 0 {z*p.contact_node_offset_z_m}',contype=0,conaffinity=0,rgba='1 .2 .1 0',group=3)
                add(contact,'pair',name=node+'_pair',geom1=node,geom2=surface,condim=(1 if p.pad_disc_mu==0 else 3),friction=f'{p.pad_disc_mu} {p.pad_disc_mu} 0 0 0',solref=f'{p.contact_timeconstant_s} {p.contact_damping_ratio}',solimp=f'{p.contact_impedance} {p.contact_impedance} .0001')
            metadata['pistons'][name]={'body':name,'joint':name+'_slide','carrier':f'arm_carrier_{i}','disc':disc,'gas_node':name,'nodes':nodes,'mass_kg':p.pad_mass_kg,'axis_in_carrier':[0,-sign,0]}
    ET.indent(root)
    return ET.tostring(root,encoding='unicode'),metadata
