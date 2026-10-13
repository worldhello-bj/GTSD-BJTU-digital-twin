"""Three-DOF isolated opposed-piston brake, with native MuJoCo pad/disc contact.
All geometry is an uncalibrated numerical proxy, NOT new CAD or measured GTSD.
"""
from dataclasses import dataclass,asdict
from pathlib import Path
import math
from xml.etree.ElementTree import Element,SubElement,tostring,indent
ROOT=Path(__file__).resolve().parent
PROVENANCE='UNCALIBRATED_LEGACY_ENGINEERING_PROXY_NO_NEW_CAD_MAPPING'
BODIES=('disc','piston_minus','piston_plus')
@dataclass(frozen=True)
class BrakeParameters:
    timestep_s:float=.0002
    disc_radius_m:float=.112
    disc_thickness_m:float=.015
    disc_mass_kg:float=1.
    bearing_damping_Nms_rad:float=.002
    pad_mass_kg:float=.1
    pad_return_stiffness_N_m:float=1000.
    pad_guide_damping_Ns_m:float=50.
    pad_release_gap_m:float=.0015
    pad_radial_x_m:float=.075
    pad_radial_z_m:float=.014
    pad_halfwidth_x_m:float=.0185
    pad_halfthickness_m:float=.0025
    pad_halfheight_z_m:float=.0275
    contact_node_radius_m:float=.0025
    contact_node_offset_x_m:float=.012
    contact_node_offset_z_m:float=.018
    pad_disc_mu:float=.35
    piston_stroke_min_m:float=-.0001
    piston_stroke_max_m:float=.004
    piston_area_m2:float=6e-5
    chamber_dead_volume_m3:float=2e-6
    reservoir_volume_m3:float=.002
    reservoir_initial_gauge_pressure_Pa:float=500000.
    supply_orifice_area_m2:float=8e-9
    exhaust_orifice_area_m2:float=1.6e-8
    chamber_leak_orifice_area_m2:float=3e-9
    reservoir_leak_orifice_area_m2:float=2e-6
    contact_timeconstant_s:float=.003
    contact_damping_ratio:float=1.
    contact_impedance:float=.99
    def __post_init__(self):
        zeros={'pad_disc_mu','reservoir_initial_gauge_pressure_Pa','bearing_damping_Nms_rad'}
        for k,v in asdict(self).items():
            if type(v) not in (int,float) or not math.isfinite(v):raise ValueError(k+' must be finite numeric')
            if k=='piston_stroke_min_m':continue
            if v<0 or (v==0 and k not in zeros):raise ValueError(k+' must be positive')
        if not 1e-5<=self.timestep_s<=.0004:raise ValueError('timestep outside independently supported 1e-5..4e-4 s')
        if not 0<self.contact_impedance<1:raise ValueError('contact impedance must be in (0,1)')
        if not self.piston_stroke_min_m<=0<self.pad_release_gap_m<self.piston_stroke_max_m:raise ValueError('invalid stroke/gap')
        if self.chamber_dead_volume_m3+self.piston_area_m2*self.piston_stroke_min_m<=0:raise ValueError('invalid minimum chamber volume')
        if self.contact_node_radius_m!=self.pad_halfthickness_m:raise ValueError('pad/node surface reference mismatch')
        rmax=math.hypot(abs(self.pad_radial_x_m)+self.contact_node_offset_x_m,abs(self.pad_radial_z_m)+self.contact_node_offset_z_m)
        if rmax+self.contact_node_radius_m>=self.disc_radius_m:raise ValueError('contact nodes must remain strictly on disc flat face')
    @property
    def disc_spin_inertia_kgm2(self):return .5*self.disc_mass_kg*self.disc_radius_m**2

def model(parameters=None):
    p=parameters or BrakeParameters()
    if not isinstance(p,BrakeParameters):raise TypeError('BrakeParameters required')
    def add(parent,tag,**attrs):return SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
    m=Element('mujoco',model='GTSD_independent_piston_disc_research_bench')
    add(m,'compiler',angle='radian',autolimits='true',inertiafromgeom='false')
    opt=add(m,'option',timestep=p.timestep_s,gravity='0 0 0',integrator='implicitfast',solver='Newton',iterations=100,tolerance='1e-12',cone='elliptic',impratio=10)
    add(opt,'flag',energy='enable',multiccd='enable')
    default=add(m,'default');add(default,'geom',contype=0,conaffinity=0);add(default,'joint',limited='false')
    w=add(m,'worldbody');disc=add(w,'body',name='disc')
    add(disc,'joint',name='disc_rotation',type='hinge',axis='0 1 0',damping=p.bearing_damping_Nms_rad)
    j=p.disc_spin_inertia_kgm2; jt=p.disc_mass_kg*(3*p.disc_radius_m**2+p.disc_thickness_m**2)/12
    add(disc,'inertial',pos='0 0 0',mass=p.disc_mass_kg,diaginertia=f'{jt} {j} {jt}')
    add(disc,'geom',name='disc_surface',type='cylinder',size=f'{p.disc_radius_m} {p.disc_thickness_m/2}',quat='.7071067811865476 .7071067811865476 0 0',rgba='.60 .66 .72 1')
    add(disc,'geom',name='disc_marker',type='box',pos=f'{p.disc_radius_m*.6} {-p.disc_thickness_m/2-.00015} 0',size=f'{p.disc_radius_m*.3} .00015 .004',rgba='1 .70 .1 1')
    pairs=[]
    for side,sign in [('minus',-1),('plus',1)]:
        name='piston_'+side;y=sign*(p.disc_thickness_m/2+p.pad_halfthickness_m+p.pad_release_gap_m)
        body=add(w,'body',name=name,pos=f'{p.pad_radial_x_m} {y} {p.pad_radial_z_m}')
        add(body,'joint',name=name+'_slide',type='slide',axis=f'0 {-sign} 0',stiffness=p.pad_return_stiffness_N_m,damping=p.pad_guide_damping_Ns_m,limited='true',range=f'{p.piston_stroke_min_m} {p.piston_stroke_max_m}')
        add(body,'inertial',pos='0 0 0',mass=p.pad_mass_kg,diaginertia='.000026 .000037 .000012')
        add(body,'geom',name=name+'_visual',type='box',size=f'{p.pad_halfwidth_x_m} {p.pad_halfthickness_m} {p.pad_halfheight_z_m}',rgba='.90 .32 .10 1')
        for k,(x,z) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
            geom=name+'_contact_'+str(k)
            add(body,'geom',name=geom,type='sphere',size=p.contact_node_radius_m,pos=f'{x*p.contact_node_offset_x_m} 0 {z*p.contact_node_offset_z_m}',rgba='1 .2 .1 0',group=3)
            pairs.append(geom)
    contact=add(m,'contact')
    for geom in pairs:add(contact,'pair',geom1=geom,geom2='disc_surface',condim=(1 if p.pad_disc_mu==0 else 3),friction=f'{p.pad_disc_mu} {p.pad_disc_mu} 0 0 0',solref=f'{p.contact_timeconstant_s} {p.contact_damping_ratio}',solimp=f'{p.contact_impedance} {p.contact_impedance} .0001')
    indent(m)
    return tostring(m,encoding='unicode'),p
