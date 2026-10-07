"""SI-unit, reduced straight-track bogie. All mechanical values are assumptions.
No prescribed trajectory, velocity servo, wheel-distance constraint, or brake torque.
"""
from pathlib import Path
import json
from xml.etree.ElementTree import Element,SubElement,tostring,indent
ROOT=Path(__file__).resolve().parent
PARAMS=dict(wheel_radius_m=.14,wheel_tread_centers_m=.478,wheelbase_m=1.10,
 rail_length_m=10,rail_top_m=.04,frame_mass_kg=40,payload_mass_kg=60,
 axle_carrier_mass_kg=3,wheelset_mass_kg=12,motor_rotor_mass_kg=4,
 primary_stiffness_Npm=15000,primary_damping_Nspm=250,
 secondary_stiffness_Npm=25000,secondary_damping_Nspm=400,
 gear_ratio=2,motor_torque_cap_Nm=4,motor_power_cap_W=250,
 wheel_rail_mu=.30,pad_disc_mu=.35,pad_force_N=30,
 disc_radius_m=.112,disc_thickness_m=.015,pad_release_gap_m=.0015,
 pad_radial_x_m=.075,pad_radial_z_m=.014,timestep_s=.0001,
 frame_inertia_kgm2=[1.5,7,8],payload_inertia_per_kg_m2=[.05,.2,13/60],
 carrier_inertia_kgm2=[.04,.04,.04],wheelset_inertia_kgm2=[.65,.14,.65],
 rotor_inertia_kgm2=[.007,.007,.009],pad_mass_kg=.1,pad_inertia_kgm2=[.000026,.000037,.000012],
 pad_return_stiffness_Npm=1000,pad_guide_damping_Nspm=50,axle_bearing_damping_Nmsprad=.002,rotor_bearing_damping_Nmsprad=.0002)
def add(parent,tag,**attrs):
 return SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
def model(params=None):
 p=PARAMS| (params or {})
 m=Element('mujoco',model='B7_force_driven_reduced_bogie')
 add(m,'compiler',angle='radian',autolimits='true',inertiafromgeom='false')
 opt=add(m,'option',timestep=p['timestep_s'],gravity='0 0 -9.81',integrator='implicitfast',solver='Newton',iterations=100,tolerance='1e-10',cone='elliptic',impratio=10)
 add(opt,'flag',energy='enable',multiccd='enable')
 vis=add(m,'visual');add(vis,'global',offwidth=1280,offheight=720);add(vis,'headlight',ambient='.5 .5 .5',diffuse='.7 .7 .7')
 d=add(m,'default');add(d,'geom',contype=0,conaffinity=0,rgba='.3 .4 .5 1')
 add(d,'joint',limited='false')
 w=add(m,'worldbody');add(w,'light',pos='0 -3 5',dir='.2 .3 -1',directional='true')
 add(w,'geom',name='ground_visual_only',type='plane',size='15 4 .1',pos='5 0 -.07',rgba='.13 .16 .2 1')
 for side,y in [('L',-.239),('R',.239)]:
  add(w,'geom',name=f'rail_{side}',type='box',pos=f'5 {y} .02',size='5 .018 .02',rgba='.65 .7 .75 1')
 for k in range(21):
  add(w,'geom',name=f'sleeper_{k}',type='box',pos=f'{k*.5} 0 -.02',size='.035 .34 .025',rgba='.25 .21 .17 1')
 frame=add(w,'body',name='frame',pos='1.3 0 .28')
 add(frame,'joint',name='travel_x',type='slide',axis='1 0 0')
 add(frame,'joint',name='heave_z',type='slide',axis='0 0 1')
 add(frame,'joint',name='pitch_y',type='hinge',axis='0 1 0')
 add(frame,'inertial',pos='0 0 0',mass=p['frame_mass_kg'],diaginertia=' '.join(map(str,p['frame_inertia_kgm2'])))
 for y in [-.27,.27]:add(frame,'geom',type='box',pos=f'0 {y} 0',size='.65 .035 .035',rgba='.16 .40 .65 1')
 add(frame,'geom',type='box',size='.05 .27 .035',rgba='.16 .40 .65 1')
 payload=add(frame,'body',name='payload',pos='0 0 .20')
 add(payload,'joint',name='secondary_z',type='slide',axis='0 0 1',stiffness=p['secondary_stiffness_Npm'],damping=p['secondary_damping_Nspm'],limited='true',range='-.09 .03')
 mm=p['payload_mass_kg'];add(payload,'inertial',pos='0 0 0',mass=mm,diaginertia=' '.join(str(mm*a) for a in p['payload_inertia_per_kg_m2']))
 add(payload,'geom',type='box',size='.40 .22 .06',rgba='.2 .65 .55 .9')
 add(payload,'site',name='secondary_top',pos='0 0 -.06',size='.01')
 add(frame,'site',name='secondary_bottom',pos='0 0 .04',size='.01')
 pads=[]
 for a,x in [('front',-.55),('rear',.55)]:
  c=add(frame,'body',name=f'{a}_carrier',pos=f'{x} 0 -.10')
  add(c,'joint',name=f'{a}_primary_z',type='slide',axis='0 0 1',stiffness=p['primary_stiffness_Npm'],damping=p['primary_damping_Nspm'],limited='true',range='-.03 .08')
  add(c,'inertial',pos='0 0 0',mass=p['axle_carrier_mass_kg'],diaginertia=' '.join(map(str,p['carrier_inertia_kgm2'])))
  for side,y in [('L',-.29),('R',.29)]:
   add(c,'geom',type='box',pos=f'0 {y} 0',size='.055 .025 .04',rgba='.65 .70 .75 1')
   add(c,'site',name=f'{a}_{side}_spring_lower',pos=f'0 {y} .035',size='.006')
   add(frame,'site',name=f'{a}_{side}_spring_upper',pos=f'{x} {y} .05',size='.006')
  ax=add(c,'body',name=f'{a}_wheelset')
  add(ax,'joint',name=f'{a}_axle_y',type='hinge',axis='0 1 0',damping=p['axle_bearing_damping_Nmsprad'])
  add(ax,'inertial',pos='0 0 0',mass=p['wheelset_mass_kg'],diaginertia=' '.join(map(str,p['wheelset_inertia_kgm2'])))
  add(ax,'geom',type='cylinder',size='.020 .30',quat='.7071067812 .7071067812 0 0',rgba='.65 .68 .72 1')
  for side,y in [('L',-.239),('R',.239)]:
   add(ax,'geom',name=f'{a}_wheel_visual_{side}',type='cylinder',pos=f'0 {y} 0',size=f'{p["wheel_radius_m"]} .018',quat='.7071067812 .7071067812 0 0',rgba='.32 .35 .4 1')
   add(ax,'geom',name=f'{a}_wheel_{side}',type='sphere',pos=f'0 {y} 0',size=p['wheel_radius_m'],group=3,rgba='.3 .3 .3 0')
   add(ax,'geom',name=f'{a}_wheel_marker_{side}',type='box',pos=f'.06 {y+(-.019 if side=="L" else .019)} 0',size='.055 .0015 .009',rgba='1 .72 .1 1')
  for ds,dy in [('L',-.179),('R',.179)]:
   disc=f'{a}_disc_{ds}'
   add(ax,'geom',name=disc,type='cylinder',pos=f'0 {dy} 0',size=f'{p["disc_radius_m"]} {p["disc_thickness_m"]/2}',quat='.7071067812 .7071067812 0 0',rgba='.65 .7 .74 1')
   # Fixed caliper bridge is visual geometry attached to the bearing carrier.
   add(c,'geom',type='box',pos=f'.125 {dy} .014',size='.012 .035 .045',rgba='.85 .3 .12 1')
   for side,sgn in [('minus',-1),('plus',1)]:
    name=f'{a}_{ds}_{side}_pad'
    y=dy+sgn*(p['disc_thickness_m']/2+.0025+p['pad_release_gap_m'])
    b=add(c,'body',name=name,pos=f'{p["pad_radial_x_m"]} {y} {p["pad_radial_z_m"]}')
    add(b,'joint',name=name+'_slide',type='slide',axis=f'0 {-sgn} 0',stiffness=p['pad_return_stiffness_Npm'],damping=p['pad_guide_damping_Nspm'],limited='true',range='-.0001 .004')
    add(b,'inertial',pos='0 0 0',mass=p['pad_mass_kg'],diaginertia=' '.join(map(str,p['pad_inertia_kgm2'])))
    add(b,'geom',name=name+'_visual',type='box',size='.0185 .0025 .0275',rgba='.9 .35 .1 1')
    for k,(xx,zz) in enumerate([(-.012,-.018),(-.012,.018),(.012,-.018),(.012,.018)]):
     node=name+f'_contact_{k}'
     add(b,'geom',name=node,type='sphere',size='.0025',pos=f'{xx} 0 {zz}',group=3,rgba='1 .3 .1 0')
     pads.append((node,disc))
  if a=='front':
   rotor=add(c,'body',name='motor_rotor',pos='0 .088 .075')
   add(rotor,'joint',name='motor_z',type='hinge',axis='0 0 1',damping=p['rotor_bearing_damping_Nmsprad'])
   add(rotor,'inertial',pos='0 0 0',mass=p['motor_rotor_mass_kg'],diaginertia=' '.join(map(str,p['rotor_inertia_kgm2'])))
   add(rotor,'geom',type='cylinder',size='.043 .045',rgba='.90 .67 .12 1')
   add(rotor,'geom',type='box',pos='.022 0 .046',size='.022 .006 .002',rgba='.1 .1 .12 1')
 con=add(m,'contact')
 for a in ['front','rear']:
  for side in ['L','R']:
   add(con,'pair',geom1=f'{a}_wheel_{side}',geom2=f'rail_{side}',condim=3,friction=f'{p["wheel_rail_mu"]} {p["wheel_rail_mu"]} 0 0 0',solref='.005 1',solimp='.95 .95 .001')
 for name,disc in pads:
  add(con,'pair',geom1=name,geom2=disc,condim=3,friction=f'{p["pad_disc_mu"]} {p["pad_disc_mu"]} 0 0 0',solref='.003 1',solimp='.99 .99 .0001')
 eq=add(m,'equality');add(eq,'joint',name='ideal_bevel_gear',joint1='motor_z',joint2='front_axle_y',polycoef=f'0 {-p["gear_ratio"]} 0 0 0',solref='.002 1',solimp='.99 .9999 .00001')
 act=add(m,'actuator');add(act,'motor',name='motor_torque',joint='motor_z',gear='1',ctrllimited='true',ctrlrange='-30 30')
 for name in dict.fromkeys(n.rsplit('_contact_',1)[0] for n,_ in pads):add(act,'motor',name=name+'_force',joint=name+'_slide',gear=1,ctrllimited='true',ctrlrange='0 200')
 tendon=add(m,'tendon')
 ts=add(tendon,'spatial',name='secondary_visual',width='.02',rgba='.18 .18 .2 1');add(ts,'site',site='secondary_bottom');add(ts,'site',site='secondary_top')
 for a in ['front','rear']:
  for s in ['L','R']:
   ts=add(tendon,'spatial',name=f'{a}_{s}_primary_visual',width='.009',rgba='.95 .8 .2 1');add(ts,'site',site=f'{a}_{s}_spring_upper');add(ts,'site',site=f'{a}_{s}_spring_lower')
 indent(m);return tostring(m,encoding='unicode')
if __name__=='__main__':
 (ROOT/'bogie.xml').write_text(model());(ROOT/'parameters.json').write_text(json.dumps(PARAMS,indent=2))
