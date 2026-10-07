"""B8 source-layout two-car 3D vehicle: finite spring/damper suspension in all six relative DOFs.
B7 source is imported read-only to preserve its validated checkpoint. All SI.
Wheel treads use capsule central cylindrical bands; flange discs add clearance
and unilateral rail-side constraints, not a derailment-certified wheel profile.
"""
from pathlib import Path
import sys,json,copy
from xml.etree.ElementTree import Element,SubElement,fromstring,tostring,indent
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent))
from build_model import model as b7_model,PARAMS as B7

def add(parent,tag,**attrs):return SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
PARAMS=B7|dict(car_mass_kg=60.,bogie_spacing_m=2.6,car_inertia_kgm2=[10.025,36.7625,35.7625],
 secondary_x_stiffness_Npm=20000.,secondary_y_stiffness_Npm=40000.,secondary_z_stiffness_Npm=50000.,
 secondary_x_damping_Nspm=700.,secondary_y_damping_Nspm=700.,secondary_z_damping_Nspm=650.,
 secondary_roll_stiffness_Nmprad=3000.,secondary_pitch_stiffness_Nmprad=1500.,secondary_yaw_stiffness_Nmprad=1500.,
 secondary_roll_damping_Nmsprad=120.,secondary_pitch_damping_Nmsprad=180.,secondary_yaw_damping_Nmsprad=120.,
 primary_x_stiffness_Npm=40000.,primary_y_stiffness_Npm=30000.,primary_roll_stiffness_Nmprad=1500.,primary_yaw_stiffness_Nmprad=2000.,
 primary_x_damping_Nspm=300.,primary_y_damping_Nspm=300.,primary_roll_damping_Nmsprad=25.,primary_yaw_damping_Nmsprad=35.,
 flange_radius_m=.149,flange_halfwidth_m=.004,flange_center_m=.213,rail_length_m=40.,
 brake_piston_area_m2=0.00006,brake_dead_volume_m3=2e-6,include_pantograph=True,include_air_springs=True,
 airspring_area_m2=.0015,airspring_dead_volume_m3=.000125,airspring_initial_pressure_Pa=200000.,
 secondary_backup_stiffness_Npm=2000.,powered_bogies=['A'],car_center_height_m=1.095,
 bogie_frame_height_m=.37,car_length_m=2.5,car_width_m=.95,car_height_m=1.05,
 panto_mount_local_z_m=.609,panto_mount_local_x_m=-.85,overhead_height_m=1.93,
 coupler_stiffness_Npm=30000.,coupler_damping_Nspm=400.,
 include_access_doors=True,compliant_wheel_rail=True,contact_stiffness_Npm=2e6,
 contact_dissipation_spm=40.,contact_regularization_mps=.005,contact_distance_tolerance=1e-12,timestep_s=.00005,flange_contact_timeconst_s=.010,flange_contact_dampratio=1.,flange_contact_impedance=.95)

def model(params=None, return_spec=False, return_all_specs=False):
 p=PARAMS|(params or {})
 if p['include_air_springs']:p['secondary_z_stiffness_Npm']=p['secondary_backup_stiffness_Npm']
 base=fromstring(b7_model(p));base.set('model','B8_3D_two_bogie_force_driven_vehicle')
 w=base.find('worldbody');old_frame=w.find("body[@name='frame']");w.remove(old_frame)
 old_frame.remove(old_frame.find("body[@name='payload']"))
 for s in list(old_frame.findall("site[@name='secondary_bottom']")):old_frame.remove(s)
 for j in list(old_frame.findall('joint')):old_frame.remove(j)
 # Extend rails, retaining separate rectangular primitive rails.
 for g in w.findall('geom'):
  if (g.get('name') or '').startswith('rail_'):
   side=-1 if g.get('name')=='rail_L' else 1;g.set('pos',f'{p["rail_length_m"]/2} {side*.239} .02');g.set('size',f'{p["rail_length_m"]/2} .018 .02')
  if (g.get('name') or '').startswith('sleeper_'):w.remove(g)
 for k in range(int(p['rail_length_m']*2)+1):add(w,'geom',type='box',pos=f'{k*.5} 0 -.02',size='.035 .34 .025',rgba='.25 .21 .17 1')
 oldsections={n:base.find(n) for n in ['contact','equality','actuator','tendon']}
 for n,e in oldsections.items():base.remove(e)
 sections={n:add(base,n) for n in oldsections}
 cars={}
 for bi,bx in [('A',3.+p['bogie_spacing_m']/2),('B',3.-p['bogie_spacing_m']/2)]:
  car=add(w,'body',name=f'{bi}_carbody',pos=f'{bx} 0 {p["car_center_height_m"]}')
  cars[bi]=car
  add(car,'freejoint',name=f'{bi}_car_free')
  add(car,'inertial',pos='0 0 0',mass=p['car_mass_kg'],diaginertia=' '.join(map(str,p['car_inertia_kgm2'])))
  add(car,'geom',name=f'{bi}_carbody_deck',type='box',pos='0 0 -.465',size='1.25 .475 .06',rgba='.80 .85 .87 .7')
  for x in [-1.2,1.2]:
   for y in [-.455,.455]:add(car,'geom',type='capsule',fromto=f'{x} {y} -.405 {x} {y} .50',size='.012',rgba='.22 .52 .56 1')
  add(car,'geom',name=f'{bi}_car_roof',type='box',pos='0 0 .51',size='1.25 .475 .015',rgba='.56 .66 .72 .45')
  f=copy.deepcopy(old_frame);f.set('pos',f'0 0 {p["bogie_frame_height_m"]-p["car_center_height_m"]}')
  # Prefix all private identifiers/references, leaving world rails unchanged.
  for el in f.iter():
   if el.get('name'):el.set('name',bi+'_'+el.get('name'))
   if el.tag=='site' and '_spring_' in el.get('name',''):
    xyz=el.get('pos').split();xyz[1]=str(-.318 if float(xyz[1])<0 else .318);xyz[2]='.065' if el.get('name').endswith('_lower') else '-.045';el.set('pos',' '.join(xyz))
  for dof,axis,typ,rng in [('x','1 0 0','slide','-.08 .08'),('y','0 1 0','slide','-.06 .06'),('z','0 0 1','slide','-.08 .08'),('roll','1 0 0','hinge','-.18 .18'),('pitch','0 1 0','hinge','-.15 .15'),('yaw','0 0 1','hinge','-.20 .20')]:
   unit='Npm' if typ=='slide' else 'Nmprad';dunit='Nspm' if typ=='slide' else 'Nmsprad'
   add(f,'joint',name=f'{bi}_secondary_{dof}',type=typ,axis=axis,limited='true',range=rng,stiffness=p[f'secondary_{dof}_stiffness_{unit}'],damping=p[f'secondary_{dof}_damping_{dunit}'])
  for ax in ['front','rear']:
   carrier=f.find(f"body[@name='{bi}_{ax}_carrier']")
   carrier.set('pos',f'{-.55 if ax=="front" else .55} 0 {p["rail_top_m"]+p["wheel_radius_m"]-p["bogie_frame_height_m"]}')
   for dof,axis,typ,rng in [('x','1 0 0','slide','-.025 .025'),('y','0 1 0','slide','-.025 .025'),('roll','1 0 0','hinge','-.12 .12'),('yaw','0 0 1','hinge','-.10 .10')]:
    unit='Npm' if typ=='slide' else 'Nmprad';dunit='Nspm' if typ=='slide' else 'Nmsprad'
    add(carrier,'joint',name=f'{bi}_{ax}_primary_{dof}',type=typ,axis=axis,limited='true',range=rng,stiffness=p[f'primary_{dof}_stiffness_{unit}'],damping=p[f'primary_{dof}_damping_{dunit}'])
   wheel=carrier.find(f"body[@name='{bi}_{ax}_wheelset']")
   for side in ['L','R']:
    tread=wheel.find(f"geom[@name='{bi}_{ax}_wheel_{side}']")
    tread.set('type','capsule');tread.set('size',f'{p["wheel_radius_m"]} .018');tread.set('quat','.7071067812 .7071067812 0 0')
   for side,sgn in [('L',-1),('R',1)]:
    flange=f'{bi}_{ax}_flange_{side}'
    add(wheel,'geom',name=flange,type='cylinder',pos=f'0 {sgn*p["flange_center_m"]} 0',size=f'{p["flange_radius_m"]} {p["flange_halfwidth_m"]}',quat='.7071067812 .7071067812 0 0',rgba='.45 .46 .50 1')
    add(sections['contact'],'pair',geom1=flange,geom2=f'rail_{side}',condim=3,friction=f'{p["wheel_rail_mu"]} {p["wheel_rail_mu"]} 0 0 0',solref=f'{p["flange_contact_timeconst_s"]} {p["flange_contact_dampratio"]}',solimp=f'{p["flange_contact_impedance"]} {p["flange_contact_impedance"]} .001')
  if bi not in p['powered_bogies']:
   c=f.find(f"body[@name='{bi}_front_carrier']")
   c.remove(c.find(f"body[@name='{bi}_motor_rotor']"))
  car.append(f)
  if p['include_air_springs']:
   for side,x,y in [('FL',-.14,-.154),('FR',-.14,.154),('RL',.14,-.154),('RR',.14,.154)]:
    air=f'air_{bi}_{side}'
    add(car,'site',name=air+'_top',pos=f'{x} {y} -.525',size='.009',rgba='.1 .9 .7 1')
    add(f,'site',name=air+'_bottom',pos=f'{x} {y} .10',size='.009',rgba='.1 .9 .7 1')
    td=add(sections['tendon'],'spatial',name=air,width='.035',rgba='.12 .22 .25 1')
    add(td,'site',site=air+'_bottom');add(td,'site',site=air+'_top')
    add(sections['actuator'],'motor',name=air+'_force',tendon=air,gear='1',ctrllimited='false')
  for sn,old in oldsections.items():
   for elem in old:
    if sn=='tendon' and elem.get('name')=='secondary_visual':continue
    if bi not in p['powered_bogies'] and elem.get('name') in ['motor_torque','ideal_bevel_gear']:continue
    e=copy.deepcopy(elem)
    for el in e.iter():
     for attr in ['name','joint','joint1','joint2','site','geom1','geom2','body1','body2']:
      v=el.get(attr)
      if v and not v.startswith('rail_'):el.set(attr,bi+'_'+v)
    if sn=='actuator' and '_pad_force' in e.get('name',''):e.set('ctrlrange','-200 200')
    sections[sn].append(e)
 # Compliant drawbar transmits only actual endpoint spring/damper forces.
 # It connects the two source car bodies; neither body's pose is prescribed.
 add(cars['A'],'site',name='coupler_A',pos='-1.25 0 -.425',size='.025')
 add(cars['B'],'site',name='coupler_B',pos='1.25 0 -.425',size='.025')
 ct=add(sections['tendon'],'spatial',name='coupler',stiffness=p['coupler_stiffness_Npm'],damping=p['coupler_damping_Nspm'],springlength='.1',limited='true',range='.06 .14',width='.024',rgba='.28 .30 .33 1')
 add(ct,'site',site='coupler_A');add(ct,'site',site='coupler_B')
 # Source single-arm pantograph mounted on motor car A only.
 pspec=None
 if p.get('include_pantograph'):
  import pantograph
  pspec=pantograph.add_pantograph(cars['A'],w,sections['equality'],sections['tendon'],sections['actuator'],prefix='panto',base_z=p['panto_mount_local_z_m'])
  cars['A'].find("body[@name='panto_mount']").set('pos',f'{p["panto_mount_local_x_m"]} 0 {p["panto_mount_local_z_m"]}')
  pantograph.add_overhead_rail(w,p['overhead_height_m'],center_x=p['rail_length_m']/2,half_length=p['rail_length_m']/2)
 door_spec=None
 if p.get('include_access_doors',True):
  from access_doors import add_access_doors
  door_spec=add_access_doors(cars['A'],w,sections['actuator'],source_x_shift=3.,equality=sections['equality'])
  door_spec.apply_car_mass_allocation(cars['A'],original_mass_kg=p['car_mass_kg'],original_inertia_kgm2=p['car_inertia_kgm2'])
 if p.get('compliant_wheel_rail'):
  for pair in list(sections['contact']):
   geoms=(pair.get('geom1',''),pair.get('geom2',''))
   if any(g.startswith('rail_') for g in geoms) and any('_wheel_' in g or '_flange_' in g for g in geoms):sections['contact'].remove(pair)
  custom=add(base,'custom');add(custom,'text',name='external_wheel_rail_law',data='HuntCrossley compliant normal force and regularized Coulomb; Python coupled solver required')
  add(custom,'numeric',name='wheel_rail_k_alpha_vreg_mu',data=f'{p["contact_stiffness_Npm"]} {p["contact_dissipation_spm"]} {p["contact_regularization_mps"]} {p["wheel_rail_mu"]}')
 indent(base);xml=tostring(base,encoding='unicode')
 if return_all_specs:return xml,pspec,door_spec
 return (xml,pspec) if return_spec else xml
if __name__=='__main__':
 (ROOT/'full_vehicle.xml').write_text(model());(ROOT/'parameters.json').write_text(json.dumps(PARAMS|{'secondary_z_stiffness_Npm':PARAMS['secondary_backup_stiffness_Npm']},indent=2))
